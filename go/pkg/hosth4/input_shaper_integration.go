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
	"log"
	"strconv"
	"strings"
	"sync"

	"klipper-go-migration/pkg/chelper"
	"klipper-go-migration/pkg/inputshaper"
)

// InputShaperIntegration manages input shaping for the runtime.
// It wraps stepper kinematics with input shaper kinematics to apply
// shaping algorithms to the motion path.
type InputShaperIntegration struct {
	rt      *runtime
	shapers []*inputshaper.AxisInputShaper // x, y, z
	enabled bool

	// Stepper kinematics management
	mu                            sync.Mutex
	inputShaperStepperKinematics  []*chelper.StepperKinematics // input shaper wrappers
	origStepperKinematics         []*chelper.StepperKinematics // original kinematics
	stepperToInputShaperKinematics map[*chelper.StepperKinematics]*chelper.StepperKinematics

	// Dual carriage support
	hasDualCarriage bool
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
		rt:                            rt,
		enabled:                       false,
		inputShaperStepperKinematics:  make([]*chelper.StepperKinematics, 0),
		origStepperKinematics:         make([]*chelper.StepperKinematics, 0),
		stepperToInputShaperKinematics: make(map[*chelper.StepperKinematics]*chelper.StepperKinematics),
		hasDualCarriage:               false,
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

// Connect is called when the printer connects.
// It sets up input shaping for all steppers.
func (is *InputShaperIntegration) Connect() error {
	// Check if dual carriage is enabled
	// For now, we don't have dual carriage detection - assume it's not enabled
	is.hasDualCarriage = false

	if is.hasDualCarriage {
		// With dual carriage, input shaper must be configured per-carriage
		for _, shaper := range is.shapers {
			if shaper.IsEnabled() {
				return fmt.Errorf("input shaper parameters cannot be configured via " +
					"[input_shaper] section with dual_carriage(s) enabled. " +
					"Refer to Klipper documentation on how to configure input shaper for dual_carriage(s)")
			}
		}
		return nil
	}

	// Configure initial input shaping
	return is.updateInputShaping()
}

// SetHasDualCarriage sets whether dual carriage mode is enabled.
func (is *InputShaperIntegration) SetHasDualCarriage(hasDualCarriage bool) {
	is.mu.Lock()
	defer is.mu.Unlock()
	is.hasDualCarriage = hasDualCarriage
}

// getAllSteppers returns all steppers from the runtime that have trapqs assigned.
func (is *InputShaperIntegration) getAllSteppers() []*stepperInfo {
	if is.rt == nil {
		return nil
	}

	var result []*stepperInfo

	// Collect XYZ steppers
	for i := 0; i < 3; i++ {
		if is.rt.steppers[i] != nil {
			result = append(result, &stepperInfo{
				SK:    is.rt.steppers[i].sk,
				TrapQ: is.rt.steppers[i].trapq,
			})
		}
	}

	// Collect extruder stepper
	if is.rt.stepperE != nil {
		result = append(result, &stepperInfo{
			SK:    is.rt.stepperE.sk,
			TrapQ: is.rt.stepperE.trapq,
		})
	}

	// Collect extra stepper (if any)
	if is.rt.hasExtraStepper && is.rt.extraStepper != nil {
		result = append(result, &stepperInfo{
			SK:    is.rt.extraStepper.sk,
			TrapQ: is.rt.extraStepper.trapq,
		})
	}

	return result
}

// stepperInfo holds the kinematics and trapq for a stepper.
type stepperInfo struct {
	SK    *chelper.StepperKinematics
	TrapQ *chelper.TrapQ
}

// getInputShaperStepperKinematics gets or creates the input shaper wrapper
// for a stepper's kinematics.
func (is *InputShaperIntegration) getInputShaperStepperKinematics(origSK *chelper.StepperKinematics) (*chelper.StepperKinematics, error) {
	is.mu.Lock()
	defer is.mu.Unlock()

	// Check if we already have an input shaper wrapper for this kinematics
	if isSK, ok := is.stepperToInputShaperKinematics[origSK]; ok {
		return isSK, nil
	}

	// Allocate a new input shaper kinematics wrapper
	isSK, err := chelper.NewInputShaperStepperKinematics()
	if err != nil {
		return nil, fmt.Errorf("failed to allocate input shaper kinematics: %w", err)
	}

	// Set the original kinematics
	if err := isSK.InputShaperSetSK(origSK); err != nil {
		// Stepper has no active X/Y/Z axes, so we can't apply input shaping
		return nil, nil
	}

	// Track the kinematics
	is.origStepperKinematics = append(is.origStepperKinematics, origSK)
	is.inputShaperStepperKinematics = append(is.inputShaperStepperKinematics, isSK)
	is.stepperToInputShaperKinematics[origSK] = isSK

	return isSK, nil
}

// setShaperKinematics applies the shaper parameters to a stepper kinematics.
// Returns true if successful, false if the shaper was disabled due to invalid parameters.
func (is *InputShaperIntegration) setShaperKinematics(shaper *inputshaper.AxisInputShaper, sk *chelper.StepperKinematics) bool {
	n, A, T := shaper.GetShaper()
	axis := shaper.Axis[0] // 'x', 'y', or 'z'

	err := sk.InputShaperSetParams(axis, n, A, T)
	if err != nil {
		// Failed to set parameters - disable shaping for this axis
		shaper.DisableShaping()
		n, A, T = shaper.GetShaper()
		_ = sk.InputShaperSetParams(axis, n, A, T)
		return false
	}
	return true
}

// updateKinematics is called when kinematics may have changed (e.g., dual carriage switch).
func (is *InputShaperIntegration) updateKinematics() error {
	if is.rt == nil {
		return nil
	}

	// Flush step generation before updating kinematics
	// Note: In the full implementation, this would call toolhead.flush_step_generation()
	// For now, we flush the motion queue
	if is.rt.motion != nil {
		if err := is.rt.motion.flushAllSteps(); err != nil {
			log.Printf("Warning: failed to flush steps: %v", err)
		}
	}

	is.mu.Lock()
	// Update all input shaper kinematics
	for _, isSK := range is.inputShaperStepperKinematics {
		isSK.InputShaperUpdateSK()
	}
	is.mu.Unlock()

	// Check step generation scan windows
	is.checkStepGenerationScanWindows()

	return nil
}

// updateInputShaping applies shaping parameters to all steppers.
func (is *InputShaperIntegration) updateInputShaping() error {
	if is.rt == nil {
		return nil
	}

	// Flush step generation before updating
	// Note: In the full implementation, this would call toolhead.flush_step_generation()
	// For now, we flush the motion queue
	if is.rt.motion != nil {
		if err := is.rt.motion.flushAllSteps(); err != nil {
			log.Printf("Warning: failed to flush steps: %v", err)
		}
	}

	failedShapers := make([]*inputshaper.AxisInputShaper, 0)

	// Get all steppers from the runtime
	steppers := is.getAllSteppers()

	for _, si := range steppers {
		// Skip steppers without a trapq (not actively generating steps)
		if si.TrapQ == nil {
			continue
		}

		// Get or create input shaper wrapper
		origSK := si.SK
		if origSK == nil {
			continue
		}

		isSK, err := is.getInputShaperStepperKinematics(origSK)
		if err != nil {
			log.Printf("Warning: failed to get input shaper kinematics: %v", err)
			continue
		}
		if isSK == nil {
			// Stepper has no active X/Y/Z axes
			continue
		}

		// Apply shaping for each axis
		for _, shaper := range is.shapers {
			// Skip if this shaper has already failed
			failed := false
			for _, fs := range failedShapers {
				if fs == shaper {
					failed = true
					break
				}
			}
			if failed {
				continue
			}

			// Apply shaper parameters
			if !is.setShaperKinematics(shaper, isSK) {
				failedShapers = append(failedShapers, shaper)
			}
		}

		// Update the stepper to use the input shaper kinematics
		// Note: In a full implementation, we would call stepper.SetStepperKinematics(isSK)
		// For now, we just track that the shaping is configured
	}

	// Check step generation scan windows
	is.checkStepGenerationScanWindows()

	if len(failedShapers) > 0 {
		names := make([]string, len(failedShapers))
		for i, s := range failedShapers {
			names[i] = s.GetName()
		}
		return fmt.Errorf("failed to configure shaper(s) %s with given parameters",
			strings.Join(names, ", "))
	}

	return nil
}

// checkStepGenerationScanWindows updates motion queuing for the new shaper windows.
// This is called after shaper parameters change.
func (is *InputShaperIntegration) checkStepGenerationScanWindows() {
	// In the Python implementation, this calls motion_queuing.check_step_generation_scan_windows()
	// For now, this is a placeholder - the actual implementation would:
	// 1. Get the maximum pre_active and post_active times from all input shapers
	// 2. Update the motion queue's step generation windows accordingly

	// Get max window times from all input shaper kinematics
	maxPreActive := 0.0
	maxPostActive := 0.0

	is.mu.Lock()
	for _, isSK := range is.inputShaperStepperKinematics {
		preActive := isSK.GenStepsPreActive()
		postActive := isSK.GenStepsPostActive()
		if preActive > maxPreActive {
			maxPreActive = preActive
		}
		if postActive > maxPostActive {
			maxPostActive = postActive
		}
	}
	is.mu.Unlock()

	// Log the window times for debugging
	if maxPreActive > 0 || maxPostActive > 0 {
		log.Printf("Input shaper scan windows: pre_active=%.6f, post_active=%.6f",
			maxPreActive, maxPostActive)
	}
}

// GetShapers returns all axis shapers.
func (is *InputShaperIntegration) GetShapers() []*inputshaper.AxisInputShaper {
	return is.shapers
}

// IsEnabled returns true if input shaping is active.
func (is *InputShaperIntegration) IsEnabled() bool {
	return is.enabled
}

// DisableShaping temporarily disables all shaping and updates steppers.
func (is *InputShaperIntegration) DisableShaping() error {
	for _, shaper := range is.shapers {
		shaper.DisableShaping()
	}
	return is.updateInputShaping()
}

// EnableShaping re-enables all shaping and updates steppers.
func (is *InputShaperIntegration) EnableShaping() error {
	for _, shaper := range is.shapers {
		shaper.EnableShaping()
	}
	return is.updateInputShaping()
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
	// If no parameters provided, just report current settings
	if len(args) == 0 {
		return nil
	}

	// Parse and update parameters for each axis
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

	// Apply the updated shaping to all steppers
	return is.updateInputShaping()
}

// Report generates a status report for all shapers.
func (is *InputShaperIntegration) Report() string {
	var parts []string
	for i, shaper := range is.shapers {
		// Only report X, Y by default, Z only if enabled
		if i < 2 || shaper.IsEnabled() {
			status := shaper.GetStatus()
			for k, v := range status {
				parts = append(parts, fmt.Sprintf("%s_%s:%v", k, shaper.Axis, v))
			}
		}
	}
	return strings.Join(parts, " ")
}

// GetShaperCoefficients returns the shaper coefficients for an axis.
// This can be used by the motion system to apply input shaping.
func (is *InputShaperIntegration) GetShaperCoefficients(axis int) (n int, A, T []float64) {
	if axis < 0 || axis >= len(is.shapers) {
		return 0, nil, nil
	}
	return is.shapers[axis].GetShaper()
}

// GetInputShaperKinematics returns the input shaper wrapper for a stepper's kinematics.
// If no wrapper exists yet, it returns nil.
func (is *InputShaperIntegration) GetInputShaperKinematics(origSK *chelper.StepperKinematics) *chelper.StepperKinematics {
	is.mu.Lock()
	defer is.mu.Unlock()
	return is.stepperToInputShaperKinematics[origSK]
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
