// LDC1612 - port of klippy/extras/ldc1612.py
//
// Support for LDC1612 inductive sensor
//
// Copyright (C) 2023 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// LDC1612 constants.
const (
	ldc1612I2CAddr        = 0x2A
	ldc1612RegData0Msb    = 0x00
	ldc1612RegData0Lsb    = 0x01
	ldc1612RegData1Msb    = 0x02
	ldc1612RegData1Lsb    = 0x03
	ldc1612RegRCount0     = 0x08
	ldc1612RegRCount1     = 0x09
	ldc1612RegSettleCount0 = 0x10
	ldc1612RegSettleCount1 = 0x11
	ldc1612RegClockDividers0 = 0x14
	ldc1612RegClockDividers1 = 0x15
	ldc1612RegStatus      = 0x18
	ldc1612RegErrorConfig = 0x19
	ldc1612RegConfig      = 0x1A
	ldc1612RegMuxConfig   = 0x1B
	ldc1612RegResetDev    = 0x1C
	ldc1612RegDriveCurrentCh0 = 0x1E
	ldc1612RegDriveCurrentCh1 = 0x1F
	ldc1612RegManuID      = 0x7E
	ldc1612RegDeviceID    = 0x7F
)

// LDC1612 expected IDs.
const (
	ldc1612ManuID   = 0x5449
	ldc1612DevID    = 0x3055
)

// LDC1612 represents an LDC1612 inductive sensor.
type LDC1612 struct {
	rt           *runtime
	name         string
	i2cAddr      int
	channel      int
	refFreq      float64
	frequency    float64
	rawValue     uint32
	callback     func(readTime, frequency float64)
	mu           sync.Mutex
}

// LDC1612Config holds configuration for LDC1612.
type LDC1612Config struct {
	Name    string
	I2CAddr int
	Channel int     // 0 or 1
	RefFreq float64 // Reference clock frequency (typically 40MHz)
}

// DefaultLDC1612Config returns default LDC1612 configuration.
func DefaultLDC1612Config() LDC1612Config {
	return LDC1612Config{
		I2CAddr: ldc1612I2CAddr,
		Channel: 0,
		RefFreq: 40000000.0, // 40 MHz
	}
}

// newLDC1612 creates a new LDC1612 sensor.
func newLDC1612(rt *runtime, cfg LDC1612Config) (*LDC1612, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("ldc1612: name is required")
	}
	if cfg.Channel < 0 || cfg.Channel > 1 {
		return nil, fmt.Errorf("ldc1612: channel must be 0 or 1")
	}
	if cfg.I2CAddr <= 0 {
		cfg.I2CAddr = ldc1612I2CAddr
	}
	if cfg.RefFreq <= 0 {
		cfg.RefFreq = 40000000.0
	}

	ldc := &LDC1612{
		rt:      rt,
		name:    cfg.Name,
		i2cAddr: cfg.I2CAddr,
		channel: cfg.Channel,
		refFreq: cfg.RefFreq,
	}

	log.Printf("ldc1612: initialized '%s' addr=0x%02x channel=%d", cfg.Name, cfg.I2CAddr, cfg.Channel)
	return ldc, nil
}

// GetName returns the sensor name.
func (ldc *LDC1612) GetName() string {
	return ldc.name
}

// SetCallback sets the measurement callback.
func (ldc *LDC1612) SetCallback(cb func(readTime, frequency float64)) {
	ldc.mu.Lock()
	defer ldc.mu.Unlock()
	ldc.callback = cb
}

// ProcessData processes raw LDC1612 data.
// data should be 4 bytes: [DATA_MSB, DATA_LSB] for MSB and LSB registers.
func (ldc *LDC1612) ProcessData(dataMsb, dataLsb uint16) (float64, error) {
	// 28-bit data: MSB[11:0] + LSB[15:0]
	rawValue := uint32(dataMsb&0x0FFF)<<16 | uint32(dataLsb)

	// Check for errors in MSB upper bits
	if dataMsb&0xF000 != 0 {
		errBits := dataMsb >> 12
		if errBits&0x8 != 0 {
			return 0, fmt.Errorf("ldc1612: conversion underflow")
		}
		if errBits&0x4 != 0 {
			return 0, fmt.Errorf("ldc1612: conversion overflow")
		}
		if errBits&0x2 != 0 {
			return 0, fmt.Errorf("ldc1612: watchdog timeout")
		}
		if errBits&0x1 != 0 {
			return 0, fmt.Errorf("ldc1612: amplitude error")
		}
	}

	// Calculate frequency
	// f_sensor = (f_ref * DATA) / 2^28
	frequency := ldc.refFreq * float64(rawValue) / 268435456.0 // 2^28

	return frequency, nil
}

// UpdateMeasurement updates the frequency measurement.
func (ldc *LDC1612) UpdateMeasurement(readTime float64, dataMsb, dataLsb uint16) error {
	frequency, err := ldc.ProcessData(dataMsb, dataLsb)
	if err != nil {
		return err
	}

	ldc.mu.Lock()
	defer ldc.mu.Unlock()

	ldc.frequency = frequency
	ldc.rawValue = uint32(dataMsb&0x0FFF)<<16 | uint32(dataLsb)

	if ldc.callback != nil {
		ldc.callback(readTime, frequency)
	}

	return nil
}

// GetFrequency returns the measured frequency.
func (ldc *LDC1612) GetFrequency() float64 {
	ldc.mu.Lock()
	defer ldc.mu.Unlock()
	return ldc.frequency
}

// GetRawValue returns the raw sensor value.
func (ldc *LDC1612) GetRawValue() uint32 {
	ldc.mu.Lock()
	defer ldc.mu.Unlock()
	return ldc.rawValue
}

// GetStatus returns the sensor status.
func (ldc *LDC1612) GetStatus() map[string]any {
	ldc.mu.Lock()
	defer ldc.mu.Unlock()

	return map[string]any{
		"frequency": ldc.frequency,
		"raw_value": ldc.rawValue,
		"channel":   ldc.channel,
	}
}

// LDC1612Probe provides probing functionality using LDC1612.
type LDC1612Probe struct {
	sensor       *LDC1612
	baseFreq     float64
	triggerDelta float64
	triggered    bool
	mu           sync.Mutex
}

// newLDC1612Probe creates an LDC1612-based probe.
func newLDC1612Probe(sensor *LDC1612, triggerDelta float64) *LDC1612Probe {
	return &LDC1612Probe{
		sensor:       sensor,
		triggerDelta: triggerDelta,
	}
}

// SetBaseline sets the baseline frequency.
func (probe *LDC1612Probe) SetBaseline() {
	probe.mu.Lock()
	defer probe.mu.Unlock()
	probe.baseFreq = probe.sensor.GetFrequency()
	log.Printf("ldc1612_probe: baseline frequency=%.1f", probe.baseFreq)
}

// CheckTriggered checks if the probe is triggered.
func (probe *LDC1612Probe) CheckTriggered() bool {
	probe.mu.Lock()
	defer probe.mu.Unlock()

	currentFreq := probe.sensor.GetFrequency()
	delta := currentFreq - probe.baseFreq

	// Trigger when frequency change exceeds threshold
	probe.triggered = delta > probe.triggerDelta || delta < -probe.triggerDelta
	return probe.triggered
}

// IsTriggered returns the triggered state.
func (probe *LDC1612Probe) IsTriggered() bool {
	probe.mu.Lock()
	defer probe.mu.Unlock()
	return probe.triggered
}
