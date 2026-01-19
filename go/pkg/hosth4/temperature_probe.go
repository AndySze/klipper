// Temperature Probe - port of klippy/extras/temperature_probe.py
//
// Support for temperature-compensated Z probes
//
// Copyright (C) 2024 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// TemperatureProbe handles temperature compensation for Z probes.
type TemperatureProbe struct {
	rt             *runtime
	name           string
	sensorName     string
	calibTemp      float64 // Temperature at which probe was calibrated
	tempCoeff      float64 // Z offset change per degree C
	smoothTime     float64
	currentTemp    float64
	smoothedTemp   float64
	zAdjustment    float64
	enabled        bool
	lastUpdateTime float64
	mu             sync.Mutex
}

// TemperatureProbeConfig holds configuration for temperature probe.
type TemperatureProbeConfig struct {
	Name       string
	SensorName string
	CalibTemp  float64 // Calibration temperature
	TempCoeff  float64 // Z offset coefficient (mm/°C)
	SmoothTime float64 // Temperature smoothing time
}

// DefaultTemperatureProbeConfig returns default configuration.
func DefaultTemperatureProbeConfig() TemperatureProbeConfig {
	return TemperatureProbeConfig{
		CalibTemp:  25.0,
		TempCoeff:  0.0, // No compensation by default
		SmoothTime: 2.0,
	}
}

// newTemperatureProbe creates a new temperature probe.
func newTemperatureProbe(rt *runtime, cfg TemperatureProbeConfig) (*TemperatureProbe, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("temperature_probe: name is required")
	}
	if cfg.SensorName == "" {
		return nil, fmt.Errorf("temperature_probe: sensor_name is required")
	}

	tp := &TemperatureProbe{
		rt:           rt,
		name:         cfg.Name,
		sensorName:   cfg.SensorName,
		calibTemp:    cfg.CalibTemp,
		tempCoeff:    cfg.TempCoeff,
		smoothTime:   cfg.SmoothTime,
		smoothedTemp: cfg.CalibTemp,
		enabled:      true,
	}

	log.Printf("temperature_probe: initialized '%s' sensor=%s coeff=%.6f",
		cfg.Name, cfg.SensorName, cfg.TempCoeff)
	return tp, nil
}

// GetName returns the probe name.
func (tp *TemperatureProbe) GetName() string {
	return tp.name
}

// Enable enables temperature compensation.
func (tp *TemperatureProbe) Enable() {
	tp.mu.Lock()
	defer tp.mu.Unlock()
	tp.enabled = true
	log.Printf("temperature_probe '%s': enabled", tp.name)
}

// Disable disables temperature compensation.
func (tp *TemperatureProbe) Disable() {
	tp.mu.Lock()
	defer tp.mu.Unlock()
	tp.enabled = false
	tp.zAdjustment = 0
	log.Printf("temperature_probe '%s': disabled", tp.name)
}

// IsEnabled returns whether compensation is enabled.
func (tp *TemperatureProbe) IsEnabled() bool {
	tp.mu.Lock()
	defer tp.mu.Unlock()
	return tp.enabled
}

// SetCalibTemp sets the calibration temperature.
func (tp *TemperatureProbe) SetCalibTemp(temp float64) {
	tp.mu.Lock()
	defer tp.mu.Unlock()
	tp.calibTemp = temp
	tp.updateAdjustment()
}

// SetTempCoeff sets the temperature coefficient.
func (tp *TemperatureProbe) SetTempCoeff(coeff float64) {
	tp.mu.Lock()
	defer tp.mu.Unlock()
	tp.tempCoeff = coeff
	tp.updateAdjustment()
}

// UpdateTemperature updates the current temperature.
func (tp *TemperatureProbe) UpdateTemperature(eventTime, temp float64) {
	tp.mu.Lock()
	defer tp.mu.Unlock()

	tp.currentTemp = temp

	// Apply exponential smoothing
	if tp.lastUpdateTime > 0 && tp.smoothTime > 0 {
		dt := eventTime - tp.lastUpdateTime
		alpha := 1.0 - 0.5 // Simplified smoothing
		if dt > 0 && dt < tp.smoothTime {
			alpha = dt / tp.smoothTime
		}
		tp.smoothedTemp = tp.smoothedTemp + alpha*(temp-tp.smoothedTemp)
	} else {
		tp.smoothedTemp = temp
	}

	tp.lastUpdateTime = eventTime
	tp.updateAdjustment()
}

// updateAdjustment recalculates the Z adjustment (must be called with lock held).
func (tp *TemperatureProbe) updateAdjustment() {
	if !tp.enabled {
		tp.zAdjustment = 0
		return
	}

	// Z adjustment = coefficient * (current_temp - calib_temp)
	tempDelta := tp.smoothedTemp - tp.calibTemp
	tp.zAdjustment = tp.tempCoeff * tempDelta
}

// GetZAdjustment returns the current Z adjustment.
func (tp *TemperatureProbe) GetZAdjustment() float64 {
	tp.mu.Lock()
	defer tp.mu.Unlock()
	return tp.zAdjustment
}

// GetCurrentTemp returns the current temperature.
func (tp *TemperatureProbe) GetCurrentTemp() float64 {
	tp.mu.Lock()
	defer tp.mu.Unlock()
	return tp.currentTemp
}

// GetSmoothedTemp returns the smoothed temperature.
func (tp *TemperatureProbe) GetSmoothedTemp() float64 {
	tp.mu.Lock()
	defer tp.mu.Unlock()
	return tp.smoothedTemp
}

// GetStatus returns probe status.
func (tp *TemperatureProbe) GetStatus() map[string]any {
	tp.mu.Lock()
	defer tp.mu.Unlock()

	return map[string]any{
		"enabled":       tp.enabled,
		"current_temp":  tp.currentTemp,
		"smoothed_temp": tp.smoothedTemp,
		"calib_temp":    tp.calibTemp,
		"temp_coeff":    tp.tempCoeff,
		"z_adjustment":  tp.zAdjustment,
	}
}
