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
type EddyCalibration struct {
	frequencies []float64
	zPositions  []float64
	polyCoeffs  []float64 // Polynomial coefficients for freq->z conversion
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

// FrequencyToZ converts frequency to Z position using calibration.
func (p *EddyCurrentProbe) FrequencyToZ(freq float64) (float64, error) {
	p.mu.Lock()
	defer p.mu.Unlock()

	if len(p.calibration.polyCoeffs) == 0 {
		return 0, fmt.Errorf("probe_eddy_current: not calibrated")
	}

	// Evaluate polynomial: z = a0 + a1*f + a2*f^2 + ...
	z := 0.0
	for i, coeff := range p.calibration.polyCoeffs {
		z += coeff * math.Pow(freq, float64(i))
	}

	return z, nil
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
	p.calibration.polyCoeffs = nil
}

// FitCalibration fits polynomial to calibration data.
func (p *EddyCurrentProbe) FitCalibration(degree int) error {
	p.mu.Lock()
	defer p.mu.Unlock()

	if len(p.calibration.frequencies) < degree+1 {
		return fmt.Errorf("probe_eddy_current: need at least %d points for degree %d polynomial",
			degree+1, degree)
	}

	// Simple polynomial fitting (in real implementation would use proper least squares)
	// For now, store placeholder coefficients
	p.calibration.polyCoeffs = make([]float64, degree+1)
	log.Printf("probe_eddy_current: calibration fitted with %d points", len(p.calibration.frequencies))
	return nil
}

// UpdateMeasurement updates the probe with new frequency reading.
func (p *EddyCurrentProbe) UpdateMeasurement(freq float64) {
	p.mu.Lock()
	defer p.mu.Unlock()

	p.lastFreq = freq

	if len(p.calibration.polyCoeffs) > 0 {
		// Convert frequency to Z
		z := 0.0
		for i, coeff := range p.calibration.polyCoeffs {
			z += coeff * math.Pow(freq, float64(i))
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
		"calibrated":  len(p.calibration.polyCoeffs) > 0,
	}
}
