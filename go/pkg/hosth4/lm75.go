// LM75 - port of klippy/extras/lm75.py
//
// Support for LM75/LM75A temperature sensors
//
// Copyright (C) 2021 Jared Smith <jaredsmith4@pm.me>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// LM75 constants.
const (
	lm75I2CAddr     = 0x48
	lm75ReportTime  = 0.8
	lm75RegTemp     = 0x00
	lm75RegConfig   = 0x01
	lm75RegTHyst    = 0x02
	lm75RegTOS      = 0x03
)

// LM75 represents an LM75/LM75A temperature sensor.
type LM75 struct {
	rt           *runtime
	name         string
	i2cAddr      int
	reportTime   float64
	temperature  float64
	lastReadTime float64
	callback     func(readTime, temp float64)
	mu           sync.Mutex
}

// LM75Config holds configuration for LM75.
type LM75Config struct {
	Name       string
	I2CAddr    int
	ReportTime float64
}

// DefaultLM75Config returns default LM75 configuration.
func DefaultLM75Config() LM75Config {
	return LM75Config{
		I2CAddr:    lm75I2CAddr,
		ReportTime: lm75ReportTime,
	}
}

// newLM75 creates a new LM75 sensor.
func newLM75(rt *runtime, cfg LM75Config) (*LM75, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("lm75: name is required")
	}
	if cfg.I2CAddr <= 0 {
		cfg.I2CAddr = lm75I2CAddr
	}
	if cfg.ReportTime <= 0 {
		cfg.ReportTime = lm75ReportTime
	}

	lm := &LM75{
		rt:         rt,
		name:       cfg.Name,
		i2cAddr:    cfg.I2CAddr,
		reportTime: cfg.ReportTime,
	}

	log.Printf("lm75: initialized '%s' addr=0x%02x", cfg.Name, cfg.I2CAddr)
	return lm, nil
}

// GetName returns the sensor name.
func (lm *LM75) GetName() string {
	return lm.name
}

// SetCallback sets the measurement callback.
func (lm *LM75) SetCallback(cb func(readTime, temp float64)) {
	lm.mu.Lock()
	defer lm.mu.Unlock()
	lm.callback = cb
}

// UpdateMeasurement updates the measurement values.
func (lm *LM75) UpdateMeasurement(readTime, temp float64) {
	lm.mu.Lock()
	defer lm.mu.Unlock()

	lm.temperature = temp
	lm.lastReadTime = readTime

	if lm.callback != nil {
		lm.callback(readTime, temp)
	}
}

// ProcessData processes raw temperature data (2 bytes).
func (lm *LM75) ProcessData(data []byte) (float64, error) {
	if len(data) < 2 {
		return 0, fmt.Errorf("lm75: insufficient data")
	}

	// LM75 returns 16-bit signed value, with 0.5°C resolution (9 bits)
	raw := (int16(data[0]) << 8) | int16(data[1])
	temp := float64(raw) / 256.0

	return temp, nil
}

// GetTemperature returns the last temperature reading.
func (lm *LM75) GetTemperature() float64 {
	lm.mu.Lock()
	defer lm.mu.Unlock()
	return lm.temperature
}

// GetStatus returns the sensor status.
func (lm *LM75) GetStatus() map[string]any {
	lm.mu.Lock()
	defer lm.mu.Unlock()

	return map[string]any{
		"temperature": lm.temperature,
	}
}

// GetMinReportTime returns the minimum report time.
func (lm *LM75) GetMinReportTime() float64 {
	return lm.reportTime
}
