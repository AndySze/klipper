// Temperature Sensor - port of klippy/extras/temperature_sensor.py
//
// Support generic temperature sensors
//
// Copyright (C) 2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"

	"klipper-go-migration/pkg/temperature"
)

// KelvinToCelsius is defined in thermistor.go

// PrinterSensorGeneric is a generic temperature sensor wrapper.
type PrinterSensorGeneric struct {
	rt           *runtime
	name         string
	sensor       temperature.Sensor
	minTemp      float64
	maxTemp      float64
	lastTemp     float64
	measuredMin  float64
	measuredMax  float64
	mu           sync.RWMutex
}

// SensorGenericConfig holds configuration for a generic temperature sensor.
type SensorGenericConfig struct {
	Name    string
	MinTemp float64
	MaxTemp float64
}

// DefaultSensorGenericConfig returns default sensor configuration.
func DefaultSensorGenericConfig() SensorGenericConfig {
	return SensorGenericConfig{
		MinTemp: KelvinToCelsius,
		MaxTemp: 99999999.9,
	}
}

// newPrinterSensorGeneric creates a new generic temperature sensor.
func newPrinterSensorGeneric(rt *runtime, sensor temperature.Sensor, cfg SensorGenericConfig) (*PrinterSensorGeneric, error) {
	if cfg.MinTemp >= cfg.MaxTemp {
		return nil, fmt.Errorf("min_temp must be less than max_temp")
	}

	psg := &PrinterSensorGeneric{
		rt:          rt,
		name:        cfg.Name,
		sensor:      sensor,
		minTemp:     cfg.MinTemp,
		maxTemp:     cfg.MaxTemp,
		lastTemp:    0,
		measuredMin: 99999999.0,
		measuredMax: 0,
	}

	log.Printf("temperature_sensor: initialized '%s' (min=%.1f, max=%.1f)", cfg.Name, cfg.MinTemp, cfg.MaxTemp)
	return psg, nil
}

// TemperatureCallback is called when a new temperature reading is available.
func (psg *PrinterSensorGeneric) TemperatureCallback(readTime, temp float64) {
	psg.mu.Lock()
	defer psg.mu.Unlock()

	psg.lastTemp = temp
	if temp != 0 {
		if temp < psg.measuredMin {
			psg.measuredMin = temp
		}
		if temp > psg.measuredMax {
			psg.measuredMax = temp
		}
	}
}

// GetTemp returns the last temperature reading.
func (psg *PrinterSensorGeneric) GetTemp(eventtime float64) (float64, float64) {
	psg.mu.RLock()
	defer psg.mu.RUnlock()
	return psg.lastTemp, 0
}

// Stats returns sensor statistics.
func (psg *PrinterSensorGeneric) Stats(eventtime float64) (bool, string) {
	psg.mu.RLock()
	defer psg.mu.RUnlock()
	return false, fmt.Sprintf("%s: temp=%.1f", psg.name, psg.lastTemp)
}

// GetStatus returns the sensor status.
func (psg *PrinterSensorGeneric) GetStatus() map[string]any {
	psg.mu.RLock()
	defer psg.mu.RUnlock()

	return map[string]any{
		"temperature":       round2(psg.lastTemp),
		"measured_min_temp": round2(psg.measuredMin),
		"measured_max_temp": round2(psg.measuredMax),
	}
}

// GetName returns the sensor name.
func (psg *PrinterSensorGeneric) GetName() string {
	return psg.name
}

// GetMinTemp returns the minimum temperature limit.
func (psg *PrinterSensorGeneric) GetMinTemp() float64 {
	return psg.minTemp
}

// GetMaxTemp returns the maximum temperature limit.
func (psg *PrinterSensorGeneric) GetMaxTemp() float64 {
	return psg.maxTemp
}

// round2 rounds a float to 2 decimal places.
func round2(v float64) float64 {
	return float64(int(v*100+0.5)) / 100
}

// SensorManager manages temperature sensors.
type SensorManager struct {
	rt      *runtime
	sensors map[string]*PrinterSensorGeneric
	mu      sync.RWMutex
}

// newSensorManager creates a new sensor manager.
func newSensorManager(rt *runtime) *SensorManager {
	return &SensorManager{
		rt:      rt,
		sensors: make(map[string]*PrinterSensorGeneric),
	}
}

// Register registers a new generic temperature sensor.
func (sm *SensorManager) Register(sensor temperature.Sensor, cfg SensorGenericConfig) (*PrinterSensorGeneric, error) {
	sm.mu.Lock()
	defer sm.mu.Unlock()

	if _, exists := sm.sensors[cfg.Name]; exists {
		return nil, fmt.Errorf("temperature sensor '%s' already registered", cfg.Name)
	}

	psg, err := newPrinterSensorGeneric(sm.rt, sensor, cfg)
	if err != nil {
		return nil, err
	}

	sm.sensors[cfg.Name] = psg
	return psg, nil
}

// Get returns a sensor by name.
func (sm *SensorManager) Get(name string) *PrinterSensorGeneric {
	sm.mu.RLock()
	defer sm.mu.RUnlock()
	return sm.sensors[name]
}

// GetAll returns all sensor names.
func (sm *SensorManager) GetAll() []string {
	sm.mu.RLock()
	defer sm.mu.RUnlock()

	names := make([]string, 0, len(sm.sensors))
	for name := range sm.sensors {
		names = append(names, name)
	}
	return names
}
