// SHT3x - port of klippy/extras/sht3x.py
//
// Support for SHT3x temperature/humidity sensors
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

// SHT3x constants.
const (
	sht3xI2CAddr       = 0x44
	sht3xReportTime    = 0.8
	sht3xCmdMeasure    = 0x2400 // High repeatability measurement
	sht3xCmdSoftReset  = 0x30A2
	sht3xCmdHeaterOn   = 0x306D
	sht3xCmdHeaterOff  = 0x3066
	sht3xCmdReadStatus = 0xF32D
)

// SHT3x represents an SHT3x temperature/humidity sensor.
type SHT3x struct {
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

// SHT3xConfig holds configuration for SHT3x.
type SHT3xConfig struct {
	Name       string
	I2CAddr    int
	ReportTime float64
}

// DefaultSHT3xConfig returns default SHT3x configuration.
func DefaultSHT3xConfig() SHT3xConfig {
	return SHT3xConfig{
		I2CAddr:    sht3xI2CAddr,
		ReportTime: sht3xReportTime,
	}
}

// newSHT3x creates a new SHT3x sensor.
func newSHT3x(rt *runtime, cfg SHT3xConfig) (*SHT3x, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("sht3x: name is required")
	}
	if cfg.I2CAddr <= 0 {
		cfg.I2CAddr = sht3xI2CAddr
	}
	if cfg.ReportTime <= 0 {
		cfg.ReportTime = sht3xReportTime
	}

	sht := &SHT3x{
		rt:         rt,
		name:       cfg.Name,
		i2cAddr:    cfg.I2CAddr,
		reportTime: cfg.ReportTime,
	}

	log.Printf("sht3x: initialized '%s' addr=0x%02x", cfg.Name, cfg.I2CAddr)
	return sht, nil
}

// GetName returns the sensor name.
func (sht *SHT3x) GetName() string {
	return sht.name
}

// SetCallback sets the measurement callback.
func (sht *SHT3x) SetCallback(cb func(readTime, temp, humidity float64)) {
	sht.mu.Lock()
	defer sht.mu.Unlock()
	sht.callback = cb
}

// UpdateMeasurement updates the measurement values.
func (sht *SHT3x) UpdateMeasurement(readTime, temp, humidity float64) {
	sht.mu.Lock()
	defer sht.mu.Unlock()

	sht.temperature = temp
	sht.humidity = humidity
	sht.lastReadTime = readTime

	if sht.callback != nil {
		sht.callback(readTime, temp, humidity)
	}
}

// ProcessData processes raw measurement data (6 bytes: temp MSB, temp LSB, temp CRC, hum MSB, hum LSB, hum CRC).
func (sht *SHT3x) ProcessData(data []byte) (temp, humidity float64, err error) {
	if len(data) < 6 {
		return 0, 0, fmt.Errorf("sht3x: insufficient data")
	}

	// Temperature
	tempRaw := (uint16(data[0]) << 8) | uint16(data[1])
	temp = -45.0 + 175.0*float64(tempRaw)/65535.0

	// Humidity
	humRaw := (uint16(data[3]) << 8) | uint16(data[4])
	humidity = 100.0 * float64(humRaw) / 65535.0

	return temp, humidity, nil
}

// GetTemperature returns the last temperature reading.
func (sht *SHT3x) GetTemperature() float64 {
	sht.mu.Lock()
	defer sht.mu.Unlock()
	return sht.temperature
}

// GetHumidity returns the last humidity reading.
func (sht *SHT3x) GetHumidity() float64 {
	sht.mu.Lock()
	defer sht.mu.Unlock()
	return sht.humidity
}

// GetStatus returns the sensor status.
func (sht *SHT3x) GetStatus() map[string]any {
	sht.mu.Lock()
	defer sht.mu.Unlock()

	return map[string]any{
		"temperature": sht.temperature,
		"humidity":    sht.humidity,
	}
}

// GetMinReportTime returns the minimum report time.
func (sht *SHT3x) GetMinReportTime() float64 {
	return sht.reportTime
}
