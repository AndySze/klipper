// HTU21D - port of klippy/extras/htu21d.py
//
// Support for HTU21D temperature/humidity sensor
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

// HTU21D constants.
const (
	htu21dI2CAddr          = 0x40
	htu21dReportTime       = 0.8
	htu21dCmdTriggerTempHold   = 0xE3
	htu21dCmdTriggerHumHold    = 0xE5
	htu21dCmdTriggerTempNoHold = 0xF3
	htu21dCmdTriggerHumNoHold  = 0xF5
	htu21dCmdSoftReset     = 0xFE
	htu21dCmdReadUserReg   = 0xE7
	htu21dCmdWriteUserReg  = 0xE6
)

// HTU21D represents an HTU21D temperature/humidity sensor.
type HTU21D struct {
	rt           *runtime
	name         string
	i2cAddr      int
	reportTime   float64
	temperature  float64
	humidity     float64
	lastReadTime float64
	callback     func(readTime, temp, humidity float64)
	mu           sync.Mutex
}

// HTU21DConfig holds configuration for HTU21D.
type HTU21DConfig struct {
	Name       string
	I2CAddr    int
	ReportTime float64
}

// DefaultHTU21DConfig returns default HTU21D configuration.
func DefaultHTU21DConfig() HTU21DConfig {
	return HTU21DConfig{
		I2CAddr:    htu21dI2CAddr,
		ReportTime: htu21dReportTime,
	}
}

// newHTU21D creates a new HTU21D sensor.
func newHTU21D(rt *runtime, cfg HTU21DConfig) (*HTU21D, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("htu21d: name is required")
	}
	if cfg.I2CAddr <= 0 {
		cfg.I2CAddr = htu21dI2CAddr
	}
	if cfg.ReportTime <= 0 {
		cfg.ReportTime = htu21dReportTime
	}

	htu := &HTU21D{
		rt:         rt,
		name:       cfg.Name,
		i2cAddr:    cfg.I2CAddr,
		reportTime: cfg.ReportTime,
	}

	log.Printf("htu21d: initialized '%s' addr=0x%02x", cfg.Name, cfg.I2CAddr)
	return htu, nil
}

// GetName returns the sensor name.
func (htu *HTU21D) GetName() string {
	return htu.name
}

// SetCallback sets the measurement callback.
func (htu *HTU21D) SetCallback(cb func(readTime, temp, humidity float64)) {
	htu.mu.Lock()
	defer htu.mu.Unlock()
	htu.callback = cb
}

// UpdateMeasurement updates the measurement values.
func (htu *HTU21D) UpdateMeasurement(readTime, temp, humidity float64) {
	htu.mu.Lock()
	defer htu.mu.Unlock()

	htu.temperature = temp
	htu.humidity = humidity
	htu.lastReadTime = readTime

	if htu.callback != nil {
		htu.callback(readTime, temp, humidity)
	}
}

// ProcessTemperature processes raw temperature data.
func (htu *HTU21D) ProcessTemperature(data []byte) (float64, error) {
	if len(data) < 2 {
		return 0, fmt.Errorf("htu21d: insufficient temperature data")
	}

	raw := (uint16(data[0]) << 8) | uint16(data[1])
	raw &= 0xFFFC // Clear status bits

	temp := -46.85 + 175.72*float64(raw)/65536.0
	return temp, nil
}

// ProcessHumidity processes raw humidity data.
func (htu *HTU21D) ProcessHumidity(data []byte) (float64, error) {
	if len(data) < 2 {
		return 0, fmt.Errorf("htu21d: insufficient humidity data")
	}

	raw := (uint16(data[0]) << 8) | uint16(data[1])
	raw &= 0xFFFC // Clear status bits

	humidity := -6.0 + 125.0*float64(raw)/65536.0

	// Clamp to valid range
	if humidity < 0 {
		humidity = 0
	} else if humidity > 100 {
		humidity = 100
	}

	return humidity, nil
}

// GetTemperature returns the last temperature reading.
func (htu *HTU21D) GetTemperature() float64 {
	htu.mu.Lock()
	defer htu.mu.Unlock()
	return htu.temperature
}

// GetHumidity returns the last humidity reading.
func (htu *HTU21D) GetHumidity() float64 {
	htu.mu.Lock()
	defer htu.mu.Unlock()
	return htu.humidity
}

// GetStatus returns the sensor status.
func (htu *HTU21D) GetStatus() map[string]any {
	htu.mu.Lock()
	defer htu.mu.Unlock()

	return map[string]any{
		"temperature": htu.temperature,
		"humidity":    htu.humidity,
	}
}

// GetMinReportTime returns the minimum report time.
func (htu *HTU21D) GetMinReportTime() float64 {
	return htu.reportTime
}
