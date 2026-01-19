// Load Cell - port of klippy/extras/load_cell.py
//
// Support for load cell sensors (HX711, HX717, ADS1220)
//
// Copyright (C) 2022 Kevin O'Connor <kevin@koconnor.net>
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

// Load cell types.
const (
	LoadCellHX711  = "HX711"
	LoadCellHX717  = "HX717"
	LoadCellADS1220 = "ADS1220"
)

// HX711/HX717 channels and gains.
const (
	hxChannelAGain128 = 25 // Channel A, gain 128 (25 clocks)
	hxChannelBGain32  = 26 // Channel B, gain 32 (26 clocks)
	hxChannelAGain64  = 27 // Channel A, gain 64 (27 clocks)
)

// LoadCell represents a load cell sensor.
type LoadCell struct {
	rt              *runtime
	name            string
	cellType        string
	sclkPin         string
	doutPin         string
	gain            int
	scale           float64
	offset          float64
	weight          float64
	rawValue        int32
	averageSamples  int
	samples         []int32
	callback        func(readTime, weight float64)
	mu              sync.Mutex
}

// LoadCellConfig holds configuration for load cell.
type LoadCellConfig struct {
	Name           string
	CellType       string
	SclkPin        string
	DoutPin        string
	Gain           int
	Scale          float64 // Scale factor (counts per unit)
	Offset         float64 // Tare offset
	AverageSamples int
}

// DefaultLoadCellConfig returns default load cell configuration.
func DefaultLoadCellConfig() LoadCellConfig {
	return LoadCellConfig{
		CellType:       LoadCellHX711,
		Gain:           hxChannelAGain128,
		Scale:          1.0,
		Offset:         0.0,
		AverageSamples: 4,
	}
}

// newLoadCell creates a new load cell sensor.
func newLoadCell(rt *runtime, cfg LoadCellConfig) (*LoadCell, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("load_cell: name is required")
	}
	if cfg.SclkPin == "" || cfg.DoutPin == "" {
		return nil, fmt.Errorf("load_cell: sclk_pin and dout_pin are required")
	}
	if cfg.AverageSamples < 1 {
		cfg.AverageSamples = 1
	}

	lc := &LoadCell{
		rt:             rt,
		name:           cfg.Name,
		cellType:       cfg.CellType,
		sclkPin:        cfg.SclkPin,
		doutPin:        cfg.DoutPin,
		gain:           cfg.Gain,
		scale:          cfg.Scale,
		offset:         cfg.Offset,
		averageSamples: cfg.AverageSamples,
		samples:        make([]int32, 0, cfg.AverageSamples),
	}

	log.Printf("load_cell: initialized '%s' type=%s gain=%d", cfg.Name, cfg.CellType, cfg.Gain)
	return lc, nil
}

// GetName returns the sensor name.
func (lc *LoadCell) GetName() string {
	return lc.name
}

// SetCallback sets the measurement callback.
func (lc *LoadCell) SetCallback(cb func(readTime, weight float64)) {
	lc.mu.Lock()
	defer lc.mu.Unlock()
	lc.callback = cb
}

// ProcessRawValue processes raw ADC value (24-bit two's complement).
func (lc *LoadCell) ProcessRawValue(data []byte) (int32, error) {
	if len(data) < 3 {
		return 0, fmt.Errorf("load_cell: insufficient data")
	}

	// HX711/HX717 output is 24-bit two's complement, MSB first
	raw := int32(data[0])<<16 | int32(data[1])<<8 | int32(data[2])

	// Sign extend from 24 bits to 32 bits
	if raw&0x800000 != 0 {
		raw |= -0x1000000
	}

	return raw, nil
}

// UpdateMeasurement updates the weight measurement.
func (lc *LoadCell) UpdateMeasurement(readTime float64, rawValue int32) {
	lc.mu.Lock()
	defer lc.mu.Unlock()

	lc.rawValue = rawValue

	// Add to rolling average
	lc.samples = append(lc.samples, rawValue)
	if len(lc.samples) > lc.averageSamples {
		lc.samples = lc.samples[1:]
	}

	// Calculate average
	var sum int64
	for _, s := range lc.samples {
		sum += int64(s)
	}
	avg := float64(sum) / float64(len(lc.samples))

	// Apply scale and offset
	lc.weight = (avg - lc.offset) / lc.scale

	if lc.callback != nil {
		lc.callback(readTime, lc.weight)
	}
}

// Tare sets the current reading as zero.
func (lc *LoadCell) Tare() {
	lc.mu.Lock()
	defer lc.mu.Unlock()

	if len(lc.samples) > 0 {
		var sum int64
		for _, s := range lc.samples {
			sum += int64(s)
		}
		lc.offset = float64(sum) / float64(len(lc.samples))
	}

	log.Printf("load_cell '%s': tare offset=%.1f", lc.name, lc.offset)
}

// Calibrate sets the scale factor using a known weight.
func (lc *LoadCell) Calibrate(knownWeight float64) error {
	lc.mu.Lock()
	defer lc.mu.Unlock()

	if len(lc.samples) == 0 {
		return fmt.Errorf("load_cell: no samples available for calibration")
	}
	if knownWeight == 0 {
		return fmt.Errorf("load_cell: known weight cannot be zero")
	}

	var sum int64
	for _, s := range lc.samples {
		sum += int64(s)
	}
	avg := float64(sum) / float64(len(lc.samples))

	lc.scale = (avg - lc.offset) / knownWeight

	log.Printf("load_cell '%s': calibrated scale=%.6f", lc.name, lc.scale)
	return nil
}

// GetWeight returns the current weight.
func (lc *LoadCell) GetWeight() float64 {
	lc.mu.Lock()
	defer lc.mu.Unlock()
	return lc.weight
}

// GetRawValue returns the current raw ADC value.
func (lc *LoadCell) GetRawValue() int32 {
	lc.mu.Lock()
	defer lc.mu.Unlock()
	return lc.rawValue
}

// GetStatus returns the sensor status.
func (lc *LoadCell) GetStatus() map[string]any {
	lc.mu.Lock()
	defer lc.mu.Unlock()

	return map[string]any{
		"weight":    lc.weight,
		"raw_value": lc.rawValue,
		"scale":     lc.scale,
		"offset":    lc.offset,
	}
}

// LoadCellEndstop provides endstop functionality based on load cell.
type LoadCellEndstop struct {
	loadCell     *LoadCell
	triggerForce float64
	triggered    bool
	mu           sync.Mutex
}

// newLoadCellEndstop creates a load cell based endstop.
func newLoadCellEndstop(loadCell *LoadCell, triggerForce float64) *LoadCellEndstop {
	return &LoadCellEndstop{
		loadCell:     loadCell,
		triggerForce: math.Abs(triggerForce),
	}
}

// CheckTriggered checks if the endstop is triggered.
func (lce *LoadCellEndstop) CheckTriggered() bool {
	lce.mu.Lock()
	defer lce.mu.Unlock()

	weight := lce.loadCell.GetWeight()
	lce.triggered = math.Abs(weight) >= lce.triggerForce
	return lce.triggered
}

// IsTriggered returns the triggered state.
func (lce *LoadCellEndstop) IsTriggered() bool {
	lce.mu.Lock()
	defer lce.mu.Unlock()
	return lce.triggered
}
