// Delta Calibrate - port of klippy/extras/delta_calibrate.py
//
// Delta calibration support
//
// Copyright (C) 2017-2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"math"
	"sync"
)

// Calibration object measurements (from docs/prints/calibrate_size.stl)
var (
	measureAngles      = []float64{210, 270, 330, 30, 90, 150}
	measureOuterRadius = 65.0
	measureRidgeRadius = 5.0 - 0.5
	measureWeight      = 0.5
)

// StablePosition is a 3-tuple of steps since hitting endstop on each tower.
type StablePosition [3]float64

// DeltaCalibrate provides delta printer calibration.
type DeltaCalibrate struct {
	rt                  *runtime
	radius              float64
	lastProbePositions  []ProbePosition
	manualHeights       []ManualHeight
	lastDistances       []DistanceMeasurement
	deltaAnalyzeEntry   map[string][]float64
	mu                  sync.Mutex
}

// ProbePosition holds a probe position and its Z offset.
type ProbePosition struct {
	ZOffset       float64
	StablePos     StablePosition
}

// ManualHeight holds a manually entered height measurement.
type ManualHeight struct {
	Height    float64
	StablePos StablePosition
}

// DistanceMeasurement holds a distance measurement between two positions.
type DistanceMeasurement struct {
	Distance   float64
	StablePos1 StablePosition
	StablePos2 StablePosition
}

// DeltaCalibrateConfig holds configuration for delta calibration.
type DeltaCalibrateConfig struct {
	Radius float64
}

// newDeltaCalibrate creates a new delta calibrator.
func newDeltaCalibrate(rt *runtime, cfg DeltaCalibrateConfig) *DeltaCalibrate {
	dc := &DeltaCalibrate{
		rt:                rt,
		radius:            cfg.Radius,
		lastProbePositions: make([]ProbePosition, 0),
		manualHeights:     make([]ManualHeight, 0),
		lastDistances:     make([]DistanceMeasurement, 0),
		deltaAnalyzeEntry: map[string][]float64{"SCALE": {1.0}},
	}

	log.Printf("delta_calibrate: initialized with radius=%.2f", cfg.Radius)
	return dc
}

// GetDefaultProbePoints returns the default probe points for calibration.
func (dc *DeltaCalibrate) GetDefaultProbePoints() [][2]float64 {
	points := make([][2]float64, 7)
	points[0] = [2]float64{0, 0}

	scatter := []float64{0.95, 0.90, 0.85, 0.70, 0.75, 0.80}
	for i := 0; i < 6; i++ {
		r := (90.0 + 60.0*float64(i)) * math.Pi / 180.0
		dist := dc.radius * scatter[i]
		points[i+1] = [2]float64{
			math.Cos(r) * dist,
			math.Sin(r) * dist,
		}
	}

	return points
}

// AddManualHeight adds a manually measured height.
func (dc *DeltaCalibrate) AddManualHeight(height float64, stablePos StablePosition) {
	dc.mu.Lock()
	defer dc.mu.Unlock()

	dc.manualHeights = append(dc.manualHeights, ManualHeight{
		Height:    height,
		StablePos: stablePos,
	})

	log.Printf("delta_calibrate: added manual height %.3f", height)
}

// AddProbePosition adds a probe position.
func (dc *DeltaCalibrate) AddProbePosition(zOffset float64, stablePos StablePosition) {
	dc.mu.Lock()
	defer dc.mu.Unlock()

	dc.lastProbePositions = append(dc.lastProbePositions, ProbePosition{
		ZOffset:   zOffset,
		StablePos: stablePos,
	})
}

// AddDistance adds a distance measurement.
func (dc *DeltaCalibrate) AddDistance(dist float64, pos1, pos2 StablePosition) {
	dc.mu.Lock()
	defer dc.mu.Unlock()

	dc.lastDistances = append(dc.lastDistances, DistanceMeasurement{
		Distance:   dist,
		StablePos1: pos1,
		StablePos2: pos2,
	})
}

// SetAnalyzeEntry sets a delta analyze measurement entry.
func (dc *DeltaCalibrate) SetAnalyzeEntry(name string, values []float64) error {
	dc.mu.Lock()
	defer dc.mu.Unlock()

	expectedCounts := map[string]int{
		"CENTER_DISTS":         6,
		"CENTER_PILLAR_WIDTHS": 3,
		"OUTER_DISTS":          6,
		"OUTER_PILLAR_WIDTHS":  6,
		"SCALE":                1,
	}

	expected, ok := expectedCounts[name]
	if !ok {
		return fmt.Errorf("unknown parameter '%s'", name)
	}
	if len(values) != expected {
		return fmt.Errorf("parameter '%s' must have %d values", name, expected)
	}

	dc.deltaAnalyzeEntry[name] = values
	log.Printf("delta_calibrate: set %s = %v", name, values)
	return nil
}

// CalculateError calculates the calibration error for given parameters.
func (dc *DeltaCalibrate) CalculateError(
	heightPositions []ManualHeight,
	distances []DistanceMeasurement,
	getPosition func(StablePosition) (float64, float64, float64),
) float64 {
	dc.mu.Lock()
	defer dc.mu.Unlock()

	// Calculate z weight
	zWeight := 1.0
	if len(distances) > 0 && len(heightPositions) > 0 {
		zWeight = float64(len(distances)) / (measureWeight * float64(len(heightPositions)))
	}

	totalError := 0.0

	// Height errors
	for _, hp := range heightPositions {
		_, _, z := getPosition(hp.StablePos)
		diff := z - hp.Height
		totalError += diff * diff
	}
	totalError *= zWeight

	// Distance errors
	for _, d := range distances {
		x1, y1, z1 := getPosition(d.StablePos1)
		x2, y2, z2 := getPosition(d.StablePos2)
		dist := math.Sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2))
		diff := dist - d.Distance
		totalError += diff * diff
	}

	return totalError
}

// GetProbePositions returns the stored probe positions.
func (dc *DeltaCalibrate) GetProbePositions() []ProbePosition {
	dc.mu.Lock()
	defer dc.mu.Unlock()

	result := make([]ProbePosition, len(dc.lastProbePositions))
	copy(result, dc.lastProbePositions)
	return result
}

// GetManualHeights returns the stored manual heights.
func (dc *DeltaCalibrate) GetManualHeights() []ManualHeight {
	dc.mu.Lock()
	defer dc.mu.Unlock()

	result := make([]ManualHeight, len(dc.manualHeights))
	copy(result, dc.manualHeights)
	return result
}

// GetDistances returns the stored distance measurements.
func (dc *DeltaCalibrate) GetDistances() []DistanceMeasurement {
	dc.mu.Lock()
	defer dc.mu.Unlock()

	result := make([]DistanceMeasurement, len(dc.lastDistances))
	copy(result, dc.lastDistances)
	return result
}

// ClearCalibrationData clears all stored calibration data.
func (dc *DeltaCalibrate) ClearCalibrationData() {
	dc.mu.Lock()
	defer dc.mu.Unlock()

	dc.lastProbePositions = make([]ProbePosition, 0)
	dc.manualHeights = make([]ManualHeight, 0)
	dc.lastDistances = make([]DistanceMeasurement, 0)
	dc.deltaAnalyzeEntry = map[string][]float64{"SCALE": {1.0}}
}

// GetStatus returns the calibration status.
func (dc *DeltaCalibrate) GetStatus() map[string]any {
	dc.mu.Lock()
	defer dc.mu.Unlock()

	return map[string]any{
		"radius":          dc.radius,
		"probe_positions": len(dc.lastProbePositions),
		"manual_heights":  len(dc.manualHeights),
		"distances":       len(dc.lastDistances),
	}
}
