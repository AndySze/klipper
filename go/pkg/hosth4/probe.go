// Probe system - port of klippy/extras/probe.py
//
// Copyright (C) 2017-2024  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025  Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"math"
	"sort"
)

// ProbeParams holds probing parameters.
type ProbeParams struct {
	ProbeSpeed            float64
	LiftSpeed             float64
	Samples               int
	SampleRetractDist     float64
	SamplesTolerance      float64
	SamplesToleranceRetry int
	SamplesResult         string // "average" or "median"
}

// ProbeConfig holds probe configuration from config file.
type ProbeConfig struct {
	XOffset               float64
	YOffset               float64
	ZOffset               float64
	Speed                 float64
	LiftSpeed             float64
	Samples               int
	SampleRetractDist     float64
	SamplesTolerance      float64
	SamplesToleranceRetry int
	SamplesResult         string
}

// Probe represents a Z probe device.
type Probe struct {
	rt         *runtime
	config     *ProbeConfig
	lastZResult float64
	lastState  bool
	results    [][]float64
}

// newProbe creates a new probe from configuration.
func newProbe(rt *runtime, cfg *config) (*Probe, error) {
	sec, ok := cfg.section("probe")
	if !ok {
		// Try bltouch section
		sec, ok = cfg.section("bltouch")
		if !ok {
			return nil, nil // No probe configured
		}
	}

	defSpeed := 5.0
	speed, err := parseFloat(sec, "speed", &defSpeed)
	if err != nil {
		return nil, err
	}

	liftSpeed, err := parseFloat(sec, "lift_speed", &speed)
	if err != nil {
		return nil, err
	}

	defSamples := 1.0
	samplesFloat, err := parseFloat(sec, "samples", &defSamples)
	if err != nil {
		return nil, err
	}
	samples := int(samplesFloat)

	defRetract := 2.0
	sampleRetract, err := parseFloat(sec, "sample_retract_dist", &defRetract)
	if err != nil {
		return nil, err
	}

	defTolerance := 0.1
	tolerance, err := parseFloat(sec, "samples_tolerance", &defTolerance)
	if err != nil {
		return nil, err
	}

	defRetries := 0.0
	retriesFloat, err := parseFloat(sec, "samples_tolerance_retries", &defRetries)
	if err != nil {
		return nil, err
	}
	retries := int(retriesFloat)

	samplesResult := "average"
	if sr, ok := sec["samples_result"]; ok {
		samplesResult = sr
	}

	defXOff := 0.0
	xOffset, err := parseFloat(sec, "x_offset", &defXOff)
	if err != nil {
		return nil, err
	}

	defYOff := 0.0
	yOffset, err := parseFloat(sec, "y_offset", &defYOff)
	if err != nil {
		return nil, err
	}

	defZOff := 0.0
	zOffset, err := parseFloat(sec, "z_offset", &defZOff)
	if err != nil {
		return nil, err
	}

	return &Probe{
		rt: rt,
		config: &ProbeConfig{
			XOffset:               xOffset,
			YOffset:               yOffset,
			ZOffset:               zOffset,
			Speed:                 speed,
			LiftSpeed:             liftSpeed,
			Samples:               samples,
			SampleRetractDist:     sampleRetract,
			SamplesTolerance:      tolerance,
			SamplesToleranceRetry: retries,
			SamplesResult:         samplesResult,
		},
		results: make([][]float64, 0),
	}, nil
}

// GetOffsets returns probe offsets (x, y, z).
func (p *Probe) GetOffsets() (float64, float64, float64) {
	return p.config.XOffset, p.config.YOffset, p.config.ZOffset
}

// GetProbeParams returns current probe parameters, optionally overridden by gcmd args.
func (p *Probe) GetProbeParams(args map[string]string) *ProbeParams {
	params := &ProbeParams{
		ProbeSpeed:            p.config.Speed,
		LiftSpeed:             p.config.LiftSpeed,
		Samples:               p.config.Samples,
		SampleRetractDist:     p.config.SampleRetractDist,
		SamplesTolerance:      p.config.SamplesTolerance,
		SamplesToleranceRetry: p.config.SamplesToleranceRetry,
		SamplesResult:         p.config.SamplesResult,
	}

	// Override from args if provided
	if v, err := floatArg(args, "PROBE_SPEED", params.ProbeSpeed); err == nil {
		params.ProbeSpeed = v
	}
	if v, err := floatArg(args, "LIFT_SPEED", params.LiftSpeed); err == nil {
		params.LiftSpeed = v
	}
	if v, err := floatArg(args, "SAMPLES", float64(params.Samples)); err == nil {
		params.Samples = int(v)
	}
	if v, err := floatArg(args, "SAMPLE_RETRACT_DIST", params.SampleRetractDist); err == nil {
		params.SampleRetractDist = v
	}
	if v, err := floatArg(args, "SAMPLES_TOLERANCE", params.SamplesTolerance); err == nil {
		params.SamplesTolerance = v
	}
	if v, err := floatArg(args, "SAMPLES_TOLERANCE_RETRIES", float64(params.SamplesToleranceRetry)); err == nil {
		params.SamplesToleranceRetry = int(v)
	}
	if sr, ok := args["SAMPLES_RESULT"]; ok && sr != "" {
		params.SamplesResult = sr
	}

	return params
}

// StartProbeSession starts a multi-probe session.
func (p *Probe) StartProbeSession() {
	p.results = make([][]float64, 0)
}

// EndProbeSession ends the probe session.
func (p *Probe) EndProbeSession() {
	p.results = nil
}

// RunProbe performs a single probe and records the result.
func (p *Probe) RunProbe(params *ProbeParams) ([]float64, error) {
	// Get current position
	pos := p.rt.toolhead.commandedPos

	// Move to minimum Z (probe down)
	targetZ := p.rt.rails[2].positionMin
	probePos := []float64{pos[0], pos[1], targetZ}

	// Perform probing move
	if err := p.rt.homingMove(2, probePos, params.ProbeSpeed); err != nil {
		return nil, fmt.Errorf("probe move failed: %w", err)
	}

	// Record result
	result := make([]float64, 3)
	copy(result, p.rt.toolhead.commandedPos[:3])
	p.results = append(p.results, result)
	p.lastZResult = result[2]

	return result, nil
}

// PullProbedResults returns all probed positions.
func (p *Probe) PullProbedResults() [][]float64 {
	results := p.results
	p.results = make([][]float64, 0)
	return results
}

// RunMultiProbe performs multiple probes with tolerance checking.
func (p *Probe) RunMultiProbe(params *ProbeParams) ([]float64, error) {
	p.StartProbeSession()
	defer p.EndProbeSession()

	retries := 0
	for {
		// Clear previous results for this attempt
		p.results = make([][]float64, 0)

		// Perform required number of samples
		for i := 0; i < params.Samples; i++ {
			if _, err := p.RunProbe(params); err != nil {
				return nil, err
			}

			// Retract between samples (except last)
			if i < params.Samples-1 {
				pos := p.rt.toolhead.commandedPos
				liftPos := []float64{pos[0], pos[1], pos[2] + params.SampleRetractDist}
				if err := p.rt.toolhead.move(liftPos, params.LiftSpeed); err != nil {
					return nil, fmt.Errorf("retract move failed: %w", err)
				}
			}
		}

		// Calculate result based on method
		positions := p.PullProbedResults()
		avgPos := calcProbeZAverage(positions, params.SamplesResult)

		// Check tolerance
		zValues := make([]float64, len(positions))
		for i, pos := range positions {
			zValues[i] = pos[2]
		}
		zRange := maxFloat(zValues) - minFloat(zValues)

		if zRange <= params.SamplesTolerance {
			return avgPos, nil
		}

		// Tolerance exceeded
		retries++
		if retries > params.SamplesToleranceRetry {
			return nil, fmt.Errorf("probe samples exceed tolerance (range=%.6f, tolerance=%.6f)",
				zRange, params.SamplesTolerance)
		}

		p.rt.tracef("Probe tolerance exceeded (range=%.6f), retrying...\n", zRange)

		// Retract before retry
		pos := p.rt.toolhead.commandedPos
		liftPos := []float64{pos[0], pos[1], pos[2] + params.SampleRetractDist}
		if err := p.rt.toolhead.move(liftPos, params.LiftSpeed); err != nil {
			return nil, fmt.Errorf("retract move failed: %w", err)
		}
	}
}

// GetStatus returns probe status.
func (p *Probe) GetStatus() map[string]any {
	return map[string]any{
		"last_query":    p.lastState,
		"last_z_result": p.lastZResult,
	}
}

// calcProbeZAverage calculates the average Z from probe positions.
func calcProbeZAverage(positions [][]float64, method string) []float64 {
	if len(positions) == 0 {
		return []float64{0, 0, 0}
	}

	if method == "median" {
		// Sort by Z value
		sorted := make([][]float64, len(positions))
		copy(sorted, positions)
		sort.Slice(sorted, func(i, j int) bool {
			return sorted[i][2] < sorted[j][2]
		})

		middle := len(sorted) / 2
		if len(sorted)%2 == 1 {
			// Odd number of samples
			return sorted[middle]
		}
		// Even number - average the two middle values
		return calcProbeZAverage(sorted[middle-1:middle+1], "average")
	}

	// Default: average
	count := float64(len(positions))
	result := make([]float64, 3)
	for _, pos := range positions {
		result[0] += pos[0]
		result[1] += pos[1]
		result[2] += pos[2]
	}
	result[0] /= count
	result[1] /= count
	result[2] /= count
	return result
}

// maxFloat and minFloat are defined in z_tilt.go

// ProbeAccuracyResult holds the results of a probe accuracy test.
type ProbeAccuracyResult struct {
	Maximum  float64
	Minimum  float64
	Range    float64
	Average  float64
	Median   float64
	StdDev   float64
}

// RunProbeAccuracy performs a probe accuracy test.
func (p *Probe) RunProbeAccuracy(params *ProbeParams, sampleCount int) (*ProbeAccuracyResult, error) {
	p.StartProbeSession()
	defer p.EndProbeSession()

	// Perform samples
	for i := 0; i < sampleCount; i++ {
		if _, err := p.RunProbe(params); err != nil {
			return nil, err
		}

		// Retract between samples (except last)
		if i < sampleCount-1 {
			pos := p.rt.toolhead.commandedPos
			liftPos := []float64{pos[0], pos[1], pos[2] + params.SampleRetractDist}
			if err := p.rt.toolhead.move(liftPos, params.LiftSpeed); err != nil {
				return nil, fmt.Errorf("retract move failed: %w", err)
			}
		}
	}

	positions := p.PullProbedResults()
	if len(positions) == 0 {
		return nil, fmt.Errorf("no probe results")
	}

	// Extract Z values
	zValues := make([]float64, len(positions))
	for i, pos := range positions {
		zValues[i] = pos[2]
	}

	// Calculate statistics
	max := maxFloat(zValues)
	min := minFloat(zValues)
	rangeVal := max - min

	// Average
	sum := 0.0
	for _, z := range zValues {
		sum += z
	}
	avg := sum / float64(len(zValues))

	// Median
	sorted := make([]float64, len(zValues))
	copy(sorted, zValues)
	sort.Float64s(sorted)
	var median float64
	mid := len(sorted) / 2
	if len(sorted)%2 == 1 {
		median = sorted[mid]
	} else {
		median = (sorted[mid-1] + sorted[mid]) / 2
	}

	// Standard deviation
	var deviationSum float64
	for _, z := range zValues {
		deviationSum += math.Pow(z-avg, 2)
	}
	stdDev := math.Sqrt(deviationSum / float64(len(zValues)))

	return &ProbeAccuracyResult{
		Maximum: max,
		Minimum: min,
		Range:   rangeVal,
		Average: avg,
		Median:  median,
		StdDev:  stdDev,
	}, nil
}
