// Input shaper integration with hosth4 runtime
//
// Copyright (C) 2019-2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020-2025 Dmitry Butyugin <dmbutyugin@google.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"strconv"
	"strings"

	"klipper-go-migration/pkg/inputshaper"
)

// InputShaperIntegration manages input shaping for the runtime.
type InputShaperIntegration struct {
	rt      *runtime
	shapers []*inputshaper.AxisInputShaper // x, y, z
	enabled bool
}

// InputShaperConfig holds configuration for input shaping.
type InputShaperConfig struct {
	ShaperTypeX  inputshaper.ShaperType
	ShaperTypeY  inputshaper.ShaperType
	ShaperTypeZ  inputshaper.ShaperType
	ShaperFreqX  float64
	ShaperFreqY  float64
	ShaperFreqZ  float64
	DampingRatio float64
}

// DefaultInputShaperConfig returns default input shaper configuration.
func DefaultInputShaperConfig() InputShaperConfig {
	return InputShaperConfig{
		ShaperTypeX:  inputshaper.ShaperMZV,
		ShaperTypeY:  inputshaper.ShaperMZV,
		ShaperTypeZ:  "",
		ShaperFreqX:  0,
		ShaperFreqY:  0,
		ShaperFreqZ:  0,
		DampingRatio: inputshaper.DefaultDampingRatio,
	}
}

// newInputShaperIntegration creates a new input shaper integration.
func newInputShaperIntegration(rt *runtime, cfg InputShaperConfig) (*InputShaperIntegration, error) {
	is := &InputShaperIntegration{
		rt:      rt,
		enabled: false,
	}

	// Create shapers for each axis
	xShaper, err := inputshaper.NewAxisInputShaper("x", cfg.ShaperTypeX, cfg.DampingRatio, cfg.ShaperFreqX)
	if err != nil {
		return nil, fmt.Errorf("x axis: %w", err)
	}

	yShaper, err := inputshaper.NewAxisInputShaper("y", cfg.ShaperTypeY, cfg.DampingRatio, cfg.ShaperFreqY)
	if err != nil {
		return nil, fmt.Errorf("y axis: %w", err)
	}

	zShaper, err := inputshaper.NewAxisInputShaper("z", cfg.ShaperTypeZ, cfg.DampingRatio, cfg.ShaperFreqZ)
	if err != nil {
		return nil, fmt.Errorf("z axis: %w", err)
	}

	is.shapers = []*inputshaper.AxisInputShaper{xShaper, yShaper, zShaper}

	// Enable if any shaper has a non-zero frequency
	is.enabled = cfg.ShaperFreqX > 0 || cfg.ShaperFreqY > 0 || cfg.ShaperFreqZ > 0

	return is, nil
}

// GetShapers returns all axis shapers.
func (is *InputShaperIntegration) GetShapers() []*inputshaper.AxisInputShaper {
	return is.shapers
}

// IsEnabled returns true if input shaping is active.
func (is *InputShaperIntegration) IsEnabled() bool {
	return is.enabled
}

// DisableShaping temporarily disables all shaping.
func (is *InputShaperIntegration) DisableShaping() {
	for _, shaper := range is.shapers {
		shaper.DisableShaping()
	}
}

// EnableShaping re-enables all shaping.
func (is *InputShaperIntegration) EnableShaping() {
	for _, shaper := range is.shapers {
		shaper.EnableShaping()
	}
}

// GetStatus returns status for all shapers.
func (is *InputShaperIntegration) GetStatus() map[string]any {
	result := make(map[string]any)
	for _, shaper := range is.shapers {
		for k, v := range shaper.GetStatus() {
			result[k+"_"+shaper.Axis] = v
		}
	}
	return result
}

// cmdSetInputShaper handles the SET_INPUT_SHAPER command.
func (is *InputShaperIntegration) cmdSetInputShaper(args map[string]string) error {
	// Parse parameters
	for _, shaper := range is.shapers {
		axis := strings.ToUpper(shaper.Axis)

		// Check for shaper type
		var shaperType *inputshaper.ShaperType
		if st, ok := args["SHAPER_TYPE"]; ok {
			t := inputshaper.ShaperType(strings.ToLower(st))
			shaperType = &t
		}
		if st, ok := args["SHAPER_TYPE_"+axis]; ok {
			t := inputshaper.ShaperType(strings.ToLower(st))
			shaperType = &t
		}

		// Check for damping ratio
		var dampingRatio *float64
		if dr, ok := args["DAMPING_RATIO_"+axis]; ok {
			v, err := strconv.ParseFloat(dr, 64)
			if err != nil {
				return fmt.Errorf("invalid DAMPING_RATIO_%s: %w", axis, err)
			}
			dampingRatio = &v
		}

		// Check for shaper frequency
		var shaperFreq *float64
		if sf, ok := args["SHAPER_FREQ_"+axis]; ok {
			v, err := strconv.ParseFloat(sf, 64)
			if err != nil {
				return fmt.Errorf("invalid SHAPER_FREQ_%s: %w", axis, err)
			}
			shaperFreq = &v
		}

		// Update the shaper if any parameters were provided
		if shaperType != nil || dampingRatio != nil || shaperFreq != nil {
			if err := shaper.Update(shaperType, dampingRatio, shaperFreq); err != nil {
				return err
			}
		}
	}

	// Update enabled state
	is.enabled = false
	for _, shaper := range is.shapers {
		if shaper.IsEnabled() {
			is.enabled = true
			break
		}
	}

	return nil
}

// GetShaperCoefficients returns the shaper coefficients for an axis.
// This can be used by the motion system to apply input shaping.
func (is *InputShaperIntegration) GetShaperCoefficients(axis int) (n int, A, T []float64) {
	if axis < 0 || axis >= len(is.shapers) {
		return 0, nil, nil
	}
	return is.shapers[axis].GetShaper()
}

// ApplyShaping applies input shaping to a position trajectory.
// This is a simplified implementation that convolves the shaper pulses.
func (is *InputShaperIntegration) ApplyShaping(axis int, times, positions []float64) ([]float64, []float64) {
	n, A, T := is.GetShaperCoefficients(axis)
	if n == 0 || len(times) == 0 {
		return times, positions
	}

	// Normalize amplitudes
	sumA := 0.0
	for _, a := range A {
		sumA += a
	}
	normA := make([]float64, len(A))
	for i, a := range A {
		normA[i] = a / sumA
	}

	// Apply convolution
	outputLen := len(times) + n - 1
	outTimes := make([]float64, 0, outputLen)
	outPositions := make([]float64, 0, outputLen)

	// Create time-shifted copies for each shaper pulse
	for pulseIdx := 0; pulseIdx < n; pulseIdx++ {
		timeOffset := T[pulseIdx]
		amplitude := normA[pulseIdx]

		for i := 0; i < len(times); i++ {
			t := times[i] + timeOffset
			p := positions[i] * amplitude

			// Find insertion point to keep output sorted by time
			insertIdx := len(outTimes)
			for j := len(outTimes) - 1; j >= 0; j-- {
				if outTimes[j] <= t {
					insertIdx = j + 1
					break
				}
				if j == 0 {
					insertIdx = 0
				}
			}

			// Insert or add to existing point
			if insertIdx < len(outTimes) && outTimes[insertIdx] == t {
				outPositions[insertIdx] += p
			} else {
				outTimes = append(outTimes[:insertIdx], append([]float64{t}, outTimes[insertIdx:]...)...)
				outPositions = append(outPositions[:insertIdx], append([]float64{p}, outPositions[insertIdx:]...)...)
			}
		}
	}

	return outTimes, outPositions
}
