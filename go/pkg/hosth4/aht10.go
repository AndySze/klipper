// AHT10 - port of klippy/extras/aht10.py
//
// Support for AHT10/AHT20/AHT21 temperature/humidity sensors
//
// Copyright (C) 2021 MUSIC Code <music@github.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// AHT10 constants.
const (
	aht10I2CAddr      = 0x38
	aht10ReportTime   = 0.8
	aht10CmdInit      = 0xE1
	aht10CmdMeasure   = 0xAC
	aht10CmdSoftReset = 0xBA
)

// AHT chip types.
const (
	AHT10Type = "AHT10"
	AHT20Type = "AHT20"
	AHT21Type = "AHT21"
)

// AHT10 represents an AHT10/AHT20/AHT21 temperature/humidity sensor.
type AHT10 struct {
	rt           *runtime
	name         string
	ahtType      string
	i2cAddr      int
	reportTime   float64
	temperature  float64
	humidity     float64
	lastReadTime float64
	callback     func(readTime, temp, humidity float64)
	mu           sync.Mutex
}

// AHT10Config holds configuration for AHT10.
type AHT10Config struct {
	Name       string
	AhtType    string
	I2CAddr    int
	ReportTime float64
}

// DefaultAHT10Config returns default AHT10 configuration.
func DefaultAHT10Config() AHT10Config {
	return AHT10Config{
		AhtType:    AHT10Type,
		I2CAddr:    aht10I2CAddr,
		ReportTime: aht10ReportTime,
	}
}

// newAHT10 creates a new AHT10 sensor.
func newAHT10(rt *runtime, cfg AHT10Config) (*AHT10, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("aht10: name is required")
	}
	if cfg.I2CAddr <= 0 {
		cfg.I2CAddr = aht10I2CAddr
	}
	if cfg.ReportTime <= 0 {
		cfg.ReportTime = aht10ReportTime
	}

	aht := &AHT10{
		rt:         rt,
		name:       cfg.Name,
		ahtType:    cfg.AhtType,
		i2cAddr:    cfg.I2CAddr,
		reportTime: cfg.ReportTime,
	}

	log.Printf("aht10: initialized '%s' type=%s addr=0x%02x",
		cfg.Name, cfg.AhtType, cfg.I2CAddr)
	return aht, nil
}

// GetName returns the sensor name.
func (aht *AHT10) GetName() string {
	return aht.name
}

// SetCallback sets the measurement callback.
func (aht *AHT10) SetCallback(cb func(readTime, temp, humidity float64)) {
	aht.mu.Lock()
	defer aht.mu.Unlock()
	aht.callback = cb
}

// UpdateMeasurement updates the measurement values.
func (aht *AHT10) UpdateMeasurement(readTime, temp, humidity float64) {
	aht.mu.Lock()
	defer aht.mu.Unlock()

	aht.temperature = temp
	aht.humidity = humidity
	aht.lastReadTime = readTime

	if aht.callback != nil {
		aht.callback(readTime, temp, humidity)
	}
}

// ProcessData processes raw measurement data from sensor.
func (aht *AHT10) ProcessData(data []byte) (temp, humidity float64, err error) {
	if len(data) < 6 {
		return 0, 0, fmt.Errorf("aht10: insufficient data")
	}

	// Check status byte
	if data[0]&0x80 != 0 {
		return 0, 0, fmt.Errorf("aht10: sensor busy")
	}

	// Extract humidity (20 bits)
	humRaw := (uint32(data[1]) << 12) | (uint32(data[2]) << 4) | (uint32(data[3]) >> 4)
	humidity = float64(humRaw) * 100.0 / 1048576.0

	// Extract temperature (20 bits)
	tempRaw := ((uint32(data[3]) & 0x0F) << 16) | (uint32(data[4]) << 8) | uint32(data[5])
	temp = float64(tempRaw)*200.0/1048576.0 - 50.0

	return temp, humidity, nil
}

// GetTemperature returns the last temperature reading.
func (aht *AHT10) GetTemperature() float64 {
	aht.mu.Lock()
	defer aht.mu.Unlock()
	return aht.temperature
}

// GetHumidity returns the last humidity reading.
func (aht *AHT10) GetHumidity() float64 {
	aht.mu.Lock()
	defer aht.mu.Unlock()
	return aht.humidity
}

// GetStatus returns the sensor status.
func (aht *AHT10) GetStatus() map[string]any {
	aht.mu.Lock()
	defer aht.mu.Unlock()

	return map[string]any{
		"temperature": aht.temperature,
		"humidity":    aht.humidity,
	}
}

// GetMinReportTime returns the minimum report time.
func (aht *AHT10) GetMinReportTime() float64 {
	return aht.reportTime
}
