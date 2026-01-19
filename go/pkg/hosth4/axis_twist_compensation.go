// Axis Twist Compensation - port of klippy/extras/axis_twist_compensation.py
//
// Compensate for axis twist/tilt in Z probing
//
// Copyright (C) 2022 Jeremy Tan <tan.jeremy@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// AxisTwistCompensation compensates for axis twist.
type AxisTwistCompensation struct {
	rt              *runtime
	calibrateAxis   string // "x" or "y"
	calibrateStart  float64
	calibrateEnd    float64
	calibrateCount  int
	compensations   []float64 // Z compensation values at calibration points
	enabled         bool
	mu              sync.Mutex
}

// AxisTwistCompensationConfig holds configuration.
type AxisTwistCompensationConfig struct {
	CalibrateAxis  string
	CalibrateStart float64
	CalibrateEnd   float64
	CalibrateCount int
	Compensations  []float64 // Pre-loaded compensation values
}

// DefaultAxisTwistCompensationConfig returns default configuration.
func DefaultAxisTwistCompensationConfig() AxisTwistCompensationConfig {
	return AxisTwistCompensationConfig{
		CalibrateAxis:  "x",
		CalibrateCount: 3,
	}
}

// newAxisTwistCompensation creates a new axis twist compensation handler.
func newAxisTwistCompensation(rt *runtime, cfg AxisTwistCompensationConfig) (*AxisTwistCompensation, error) {
	if cfg.CalibrateAxis != "x" && cfg.CalibrateAxis != "y" {
		return nil, fmt.Errorf("axis_twist_compensation: calibrate_axis must be 'x' or 'y'")
	}
	if cfg.CalibrateCount < 2 {
		cfg.CalibrateCount = 2
	}

	atc := &AxisTwistCompensation{
		rt:             rt,
		calibrateAxis:  cfg.CalibrateAxis,
		calibrateStart: cfg.CalibrateStart,
		calibrateEnd:   cfg.CalibrateEnd,
		calibrateCount: cfg.CalibrateCount,
		compensations:  cfg.Compensations,
		enabled:        len(cfg.Compensations) > 0,
	}

	log.Printf("axis_twist_compensation: initialized axis=%s points=%d",
		cfg.CalibrateAxis, cfg.CalibrateCount)
	return atc, nil
}

// Enable enables compensation.
func (atc *AxisTwistCompensation) Enable() {
	atc.mu.Lock()
	defer atc.mu.Unlock()
	atc.enabled = true
}

// Disable disables compensation.
func (atc *AxisTwistCompensation) Disable() {
	atc.mu.Lock()
	defer atc.mu.Unlock()
	atc.enabled = false
}

// IsEnabled returns whether compensation is enabled.
func (atc *AxisTwistCompensation) IsEnabled() bool {
	atc.mu.Lock()
	defer atc.mu.Unlock()
	return atc.enabled
}

// SetCompensations sets the compensation values.
func (atc *AxisTwistCompensation) SetCompensations(values []float64) {
	atc.mu.Lock()
	defer atc.mu.Unlock()
	atc.compensations = values
	atc.enabled = len(values) > 0
}

// GetCompensation returns the Z compensation for a given position.
func (atc *AxisTwistCompensation) GetCompensation(x, y float64) float64 {
	atc.mu.Lock()
	defer atc.mu.Unlock()

	if !atc.enabled || len(atc.compensations) < 2 {
		return 0
	}

	// Get the position along the calibration axis
	var pos float64
	if atc.calibrateAxis == "x" {
		pos = x
	} else {
		pos = y
	}

	// Find interpolation position
	axisRange := atc.calibrateEnd - atc.calibrateStart
	if axisRange == 0 {
		return atc.compensations[0]
	}

	// Normalize position to 0-1 range
	t := (pos - atc.calibrateStart) / axisRange

	// Clamp to valid range
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}

	// Linear interpolation between calibration points
	numPoints := len(atc.compensations)
	segmentLen := 1.0 / float64(numPoints-1)
	segmentIdx := int(t / segmentLen)

	if segmentIdx >= numPoints-1 {
		return atc.compensations[numPoints-1]
	}

	// Interpolate within segment
	segmentT := (t - float64(segmentIdx)*segmentLen) / segmentLen
	comp1 := atc.compensations[segmentIdx]
	comp2 := atc.compensations[segmentIdx+1]

	return comp1 + (comp2-comp1)*segmentT
}

// GetCalibrationPoints returns the calibration point positions.
func (atc *AxisTwistCompensation) GetCalibrationPoints() []float64 {
	atc.mu.Lock()
	defer atc.mu.Unlock()

	points := make([]float64, atc.calibrateCount)
	step := (atc.calibrateEnd - atc.calibrateStart) / float64(atc.calibrateCount-1)

	for i := range points {
		points[i] = atc.calibrateStart + float64(i)*step
	}

	return points
}

// ClearCalibration clears calibration data.
func (atc *AxisTwistCompensation) ClearCalibration() {
	atc.mu.Lock()
	defer atc.mu.Unlock()
	atc.compensations = nil
	atc.enabled = false
}

// GetStatus returns the compensation status.
func (atc *AxisTwistCompensation) GetStatus() map[string]any {
	atc.mu.Lock()
	defer atc.mu.Unlock()

	return map[string]any{
		"enabled":         atc.enabled,
		"calibrate_axis":  atc.calibrateAxis,
		"calibrate_start": atc.calibrateStart,
		"calibrate_end":   atc.calibrateEnd,
		"calibrate_count": atc.calibrateCount,
		"compensations":   atc.compensations,
	}
}
