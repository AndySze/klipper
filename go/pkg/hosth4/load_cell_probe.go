// Load Cell Probe - port of klippy/extras/load_cell_probe.py
//
// Support for load cell based Z probes
//
// Copyright (C) 2022-2024 Kevin O'Connor <kevin@koconnor.net>
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

// LoadCellProbe represents a load cell based Z probe.
type LoadCellProbe struct {
	rt            *runtime
	name          string
	loadCell      *LoadCell
	zOffset       float64
	speed         float64
	liftSpeed     float64
	sampleCount   int
	sampleRetract float64
	triggerForce  float64
	tareForce     float64
	triggered     bool
	lastForce     float64
	mu            sync.Mutex
}

// LoadCellProbeConfig holds configuration for load cell probe.
type LoadCellProbeConfig struct {
	Name          string
	LoadCell      *LoadCell
	ZOffset       float64
	Speed         float64
	LiftSpeed     float64
	SampleCount   int
	SampleRetract float64
	TriggerForce  float64 // Force threshold to trigger (in grams or configured unit)
}

// DefaultLoadCellProbeConfig returns default configuration.
func DefaultLoadCellProbeConfig() LoadCellProbeConfig {
	return LoadCellProbeConfig{
		ZOffset:       0.0,
		Speed:         5.0,
		LiftSpeed:     5.0,
		SampleCount:   1,
		SampleRetract: 2.0,
		TriggerForce:  50.0, // 50 grams default
	}
}

// newLoadCellProbe creates a new load cell probe.
func newLoadCellProbe(rt *runtime, cfg LoadCellProbeConfig) (*LoadCellProbe, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("load_cell_probe: name is required")
	}
	if cfg.LoadCell == nil {
		return nil, fmt.Errorf("load_cell_probe: load_cell is required")
	}

	probe := &LoadCellProbe{
		rt:            rt,
		name:          cfg.Name,
		loadCell:      cfg.LoadCell,
		zOffset:       cfg.ZOffset,
		speed:         cfg.Speed,
		liftSpeed:     cfg.LiftSpeed,
		sampleCount:   cfg.SampleCount,
		sampleRetract: cfg.SampleRetract,
		triggerForce:  cfg.TriggerForce,
	}

	log.Printf("load_cell_probe: initialized '%s' trigger_force=%.1f", cfg.Name, cfg.TriggerForce)
	return probe, nil
}

// GetName returns the probe name.
func (p *LoadCellProbe) GetName() string {
	return p.name
}

// GetZOffset returns the Z offset.
func (p *LoadCellProbe) GetZOffset() float64 {
	return p.zOffset
}

// SetZOffset sets the Z offset.
func (p *LoadCellProbe) SetZOffset(offset float64) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.zOffset = offset
}

// SetTriggerForce sets the trigger force threshold.
func (p *LoadCellProbe) SetTriggerForce(force float64) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.triggerForce = math.Abs(force)
}

// Tare zeros the load cell.
func (p *LoadCellProbe) Tare() {
	p.mu.Lock()
	defer p.mu.Unlock()

	p.loadCell.Tare()
	p.tareForce = p.loadCell.GetWeight()
	log.Printf("load_cell_probe '%s': tared at %.2f", p.name, p.tareForce)
}

// UpdateMeasurement updates the probe with new force reading.
func (p *LoadCellProbe) UpdateMeasurement(force float64) {
	p.mu.Lock()
	defer p.mu.Unlock()

	p.lastForce = force - p.tareForce

	// Trigger when force exceeds threshold (either direction)
	p.triggered = math.Abs(p.lastForce) >= p.triggerForce
}

// CheckTriggered reads the load cell and checks if triggered.
func (p *LoadCellProbe) CheckTriggered() bool {
	force := p.loadCell.GetWeight()
	p.UpdateMeasurement(force)
	return p.IsTriggered()
}

// IsTriggered returns whether the probe is triggered.
func (p *LoadCellProbe) IsTriggered() bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.triggered
}

// GetLastForce returns the last measured force.
func (p *LoadCellProbe) GetLastForce() float64 {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.lastForce
}

// GetStatus returns probe status.
func (p *LoadCellProbe) GetStatus() map[string]any {
	p.mu.Lock()
	defer p.mu.Unlock()

	return map[string]any{
		"z_offset":      p.zOffset,
		"trigger_force": p.triggerForce,
		"last_force":    p.lastForce,
		"triggered":     p.triggered,
	}
}
