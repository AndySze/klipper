// DS18B20 - port of klippy/extras/ds18b20.py
//
// Support for 1-wire based temperature sensors
//
// Copyright (C) 2020 Alan Lord <alanslists@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

const (
	ds18ReportTime             = 3.0
	ds18MinReportTime          = 1.0
	ds18MaxConsecutiveErrors   = 4
)

// DS18B20 represents a DS18B20 1-wire temperature sensor.
type DS18B20 struct {
	rt          *runtime
	name        string
	serialNo    string
	temp        float64
	minTemp     float64
	maxTemp     float64
	reportTime  float64
	callback    func(readTime, temp float64)
	faultCount  int
	mu          sync.RWMutex
}

// DS18B20Config holds configuration for a DS18B20 sensor.
type DS18B20Config struct {
	Name       string
	SerialNo   string  // Sensor serial number (ROM code)
	SensorMCU  string  // MCU handling the sensor
	ReportTime float64 // Report interval in seconds
}

// DefaultDS18B20Config returns default DS18B20 configuration.
func DefaultDS18B20Config() DS18B20Config {
	return DS18B20Config{
		ReportTime: ds18ReportTime,
	}
}

// newDS18B20 creates a new DS18B20 temperature sensor.
func newDS18B20(rt *runtime, cfg DS18B20Config) (*DS18B20, error) {
	if cfg.SerialNo == "" {
		return nil, fmt.Errorf("ds18b20: serial_no is required")
	}

	reportTime := cfg.ReportTime
	if reportTime < ds18MinReportTime {
		reportTime = ds18MinReportTime
	}

	ds := &DS18B20{
		rt:         rt,
		name:       cfg.Name,
		serialNo:   cfg.SerialNo,
		reportTime: reportTime,
	}

	log.Printf("ds18b20: initialized '%s' with serial %s", cfg.Name, cfg.SerialNo)
	return ds, nil
}

// HandleResponse handles a temperature response from the MCU.
func (ds *DS18B20) HandleResponse(temp float64, fault int) {
	ds.mu.Lock()
	defer ds.mu.Unlock()

	if fault != 0 {
		ds.faultCount++
		log.Printf("ds18b20 '%s': fault %d (temp=%.1f), count=%d",
			ds.name, fault, temp, ds.faultCount)

		if ds.faultCount >= ds18MaxConsecutiveErrors {
			log.Printf("ds18b20 '%s': too many consecutive errors", ds.name)
		}
		return
	}

	ds.faultCount = 0
	ds.temp = temp

	// Check temperature limits
	if ds.minTemp > 0 && temp < ds.minTemp {
		log.Printf("ds18b20 '%s': temperature %.1f below minimum %.1f",
			ds.name, temp, ds.minTemp)
	}
	if ds.maxTemp > 0 && temp > ds.maxTemp {
		log.Printf("ds18b20 '%s': temperature %.1f above maximum %.1f",
			ds.name, temp, ds.maxTemp)
	}

	if ds.callback != nil {
		readTime := 0.0 // Would be calculated from MCU clock
		ds.callback(readTime, temp)
	}
}

// SetupMinMax sets the temperature limits.
func (ds *DS18B20) SetupMinMax(minTemp, maxTemp float64) {
	ds.mu.Lock()
	defer ds.mu.Unlock()
	ds.minTemp = minTemp
	ds.maxTemp = maxTemp
}

// SetupCallback registers a temperature callback.
func (ds *DS18B20) SetupCallback(cb func(readTime, temp float64)) {
	ds.mu.Lock()
	defer ds.mu.Unlock()
	ds.callback = cb
}

// GetReportTimeDelta returns the report time interval.
func (ds *DS18B20) GetReportTimeDelta() float64 {
	return ds.reportTime
}

// GetTemp returns the last temperature reading.
func (ds *DS18B20) GetTemp(eventtime float64) (float64, float64) {
	ds.mu.RLock()
	defer ds.mu.RUnlock()
	return ds.temp, 0
}

// GetStatus returns the sensor status.
func (ds *DS18B20) GetStatus() map[string]any {
	ds.mu.RLock()
	defer ds.mu.RUnlock()

	return map[string]any{
		"temperature": round2(ds.temp),
	}
}

// GetName returns the sensor name.
func (ds *DS18B20) GetName() string {
	return ds.name
}

// GetSerialNo returns the sensor serial number.
func (ds *DS18B20) GetSerialNo() string {
	return ds.serialNo
}

// Fault triggers an async shutdown due to sensor fault.
func (ds *DS18B20) Fault(msg string) {
	log.Printf("ds18b20 '%s': FAULT - %s", ds.name, msg)
	// In full implementation, this would trigger printer shutdown
}
