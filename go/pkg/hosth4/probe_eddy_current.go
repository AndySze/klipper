// Probe Eddy Current - port of klippy/extras/probe_eddy_current.py
//
// Support for eddy current based Z probes
//
// Copyright (C) 2021-2024 Kevin O'Connor <kevin@koconnor.net>
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

// EddyCurrentProbe represents an eddy current based Z probe.
type EddyCurrentProbe struct {
	rt              *runtime
	name            string
	sensor          *LDC1612
	zOffset         float64
	speed           float64
	liftSpeed       float64
	sampleCount     int
	sampleRetract   float64
	calibration     *EddyCalibration
	triggerDistance float64
	triggerDive     float64
	triggered       bool
	lastFreq        float64
	lastZ           float64
	mu              sync.Mutex
}

// EddyCalibration holds calibration data for eddy current probe.
// Uses piecewise linear interpolation between calibration points (matching Python).
type EddyCalibration struct {
	frequencies []float64 // Sorted in ascending order
	zPositions  []float64 // Corresponding Z positions
	calibrated  bool      // Whether calibration data has been loaded
}

// EddyCurrentProbeConfig holds configuration for eddy current probe.
type EddyCurrentProbeConfig struct {
	Name            string
	Sensor          *LDC1612
	ZOffset         float64
	Speed           float64
	LiftSpeed       float64
	SampleCount     int
	SampleRetract   float64
	TriggerDistance float64
	TriggerDive     float64
}

// DefaultEddyCurrentProbeConfig returns default configuration.
func DefaultEddyCurrentProbeConfig() EddyCurrentProbeConfig {
	return EddyCurrentProbeConfig{
		ZOffset:         0.0,
		Speed:           5.0,
		LiftSpeed:       5.0,
		SampleCount:     1,
		SampleRetract:   2.0,
		TriggerDistance: 2.0,
		TriggerDive:     5.0,
	}
}

// newEddyCurrentProbe creates a new eddy current probe.
func newEddyCurrentProbe(rt *runtime, cfg EddyCurrentProbeConfig) (*EddyCurrentProbe, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("probe_eddy_current: name is required")
	}
	if cfg.Sensor == nil {
		return nil, fmt.Errorf("probe_eddy_current: sensor is required")
	}

	probe := &EddyCurrentProbe{
		rt:              rt,
		name:            cfg.Name,
		sensor:          cfg.Sensor,
		zOffset:         cfg.ZOffset,
		speed:           cfg.Speed,
		liftSpeed:       cfg.LiftSpeed,
		sampleCount:     cfg.SampleCount,
		sampleRetract:   cfg.SampleRetract,
		triggerDistance: cfg.TriggerDistance,
		triggerDive:     cfg.TriggerDive,
		calibration:     &EddyCalibration{},
	}

	log.Printf("probe_eddy_current: initialized '%s'", cfg.Name)
	return probe, nil
}

// GetName returns the probe name.
func (p *EddyCurrentProbe) GetName() string {
	return p.name
}

// GetZOffset returns the Z offset.
func (p *EddyCurrentProbe) GetZOffset() float64 {
	return p.zOffset
}

// SetZOffset sets the Z offset.
func (p *EddyCurrentProbe) SetZOffset(offset float64) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.zOffset = offset
}

const outOfRange = 99.9

// FrequencyToZ converts frequency to Z position using calibration.
// Uses piecewise linear interpolation matching Python's apply_calibration.
func (p *EddyCurrentProbe) FrequencyToZ(freq float64) (float64, error) {
	p.mu.Lock()
	defer p.mu.Unlock()

	if !p.calibration.calibrated || len(p.calibration.frequencies) < 2 {
		return 0, fmt.Errorf("probe_eddy_current: not calibrated")
	}

	// Binary search to find position in sorted frequency list
	pos := bisectRight(p.calibration.frequencies, freq)

	// Handle out of range cases
	if pos >= len(p.calibration.zPositions) {
		return -outOfRange, nil
	}
	if pos == 0 {
		return outOfRange, nil
	}

	// Linear interpolation between adjacent points
	thisFreq := p.calibration.frequencies[pos]
	prevFreq := p.calibration.frequencies[pos-1]
	thisZPos := p.calibration.zPositions[pos]
	prevZPos := p.calibration.zPositions[pos-1]

	// Calculate gain and offset for linear interpolation
	gain := (thisZPos - prevZPos) / (thisFreq - prevFreq)
	offset := prevZPos - prevFreq*gain
	z := freq*gain + offset

	return math.Round(z*1000000) / 1000000, nil // Round to 6 decimal places
}

// bisectRight returns the insertion point for freq in sorted list a.
func bisectRight(a []float64, freq float64) int {
	lo, hi := 0, len(a)
	for lo < hi {
		mid := (lo + hi) / 2
		if freq < a[mid] {
			hi = mid
		} else {
			lo = mid + 1
		}
	}
	return lo
}

// AddCalibrationPoint adds a calibration point.
func (p *EddyCurrentProbe) AddCalibrationPoint(freq, z float64) {
	p.mu.Lock()
	defer p.mu.Unlock()

	p.calibration.frequencies = append(p.calibration.frequencies, freq)
	p.calibration.zPositions = append(p.calibration.zPositions, z)
}

// ClearCalibration clears calibration data.
func (p *EddyCurrentProbe) ClearCalibration() {
	p.mu.Lock()
	defer p.mu.Unlock()

	p.calibration.frequencies = nil
	p.calibration.zPositions = nil
	p.calibration.calibrated = false
}

// LoadCalibration loads calibration data from frequency:z pairs.
// The data is sorted by frequency for efficient lookup.
func (p *EddyCurrentProbe) LoadCalibration(cal [][2]float64) error {
	p.mu.Lock()
	defer p.mu.Unlock()

	if len(cal) < 2 {
		return fmt.Errorf("probe_eddy_current: need at least 2 calibration points")
	}

	// Sort by Z position (which maps to frequency inversely) and extract freq/z
	// Python sorts by (z, freq) then extracts freq and z separately
	type calPoint struct {
		freq float64
		z    float64
	}
	points := make([]calPoint, len(cal))
	for i, c := range cal {
		points[i] = calPoint{freq: c[0], z: c[1]}
	}

	// Sort by frequency (ascending)
	for i := 0; i < len(points)-1; i++ {
		for j := i + 1; j < len(points); j++ {
			if points[j].freq < points[i].freq {
				points[i], points[j] = points[j], points[i]
			}
		}
	}

	p.calibration.frequencies = make([]float64, len(points))
	p.calibration.zPositions = make([]float64, len(points))
	for i, pt := range points {
		p.calibration.frequencies[i] = pt.freq
		p.calibration.zPositions[i] = pt.z
	}
	p.calibration.calibrated = true

	log.Printf("probe_eddy_current: calibration loaded with %d points", len(cal))
	return nil
}

// UpdateMeasurement updates the probe with new frequency reading.
func (p *EddyCurrentProbe) UpdateMeasurement(freq float64) {
	p.mu.Lock()
	defer p.mu.Unlock()

	p.lastFreq = freq

	if p.calibration.calibrated && len(p.calibration.frequencies) >= 2 {
		// Convert frequency to Z using linear interpolation
		pos := bisectRight(p.calibration.frequencies, freq)

		var z float64
		if pos >= len(p.calibration.zPositions) {
			z = -outOfRange
		} else if pos == 0 {
			z = outOfRange
		} else {
			thisFreq := p.calibration.frequencies[pos]
			prevFreq := p.calibration.frequencies[pos-1]
			thisZPos := p.calibration.zPositions[pos]
			prevZPos := p.calibration.zPositions[pos-1]
			gain := (thisZPos - prevZPos) / (thisFreq - prevFreq)
			offset := prevZPos - prevFreq*gain
			z = freq*gain + offset
		}
		p.lastZ = z

		// Check trigger
		p.triggered = z <= p.triggerDistance
	}
}

// IsTriggered returns whether the probe is triggered.
func (p *EddyCurrentProbe) IsTriggered() bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.triggered
}

// GetLastZ returns the last calculated Z position.
func (p *EddyCurrentProbe) GetLastZ() float64 {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.lastZ
}

// GetStatus returns probe status.
func (p *EddyCurrentProbe) GetStatus() map[string]any {
	p.mu.Lock()
	defer p.mu.Unlock()

	return map[string]any{
		"z_offset":    p.zOffset,
		"last_freq":   p.lastFreq,
		"last_z":      p.lastZ,
		"triggered":   p.triggered,
		"calibrated":  p.calibration.calibrated,
	}
}
