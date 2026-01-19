// Heater Generic - port of klippy/extras/heater_generic.py
//
// Support for generic heaters (non-extruder/bed)
//
// Copyright (C) 2019-2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// GenericHeater represents a generic controllable heater.
type GenericHeater struct {
	rt            *runtime
	name          string
	heaterPin     string
	sensorName    string
	control       string // "watermark" or "pid"
	pidKp         float64
	pidKi         float64
	pidKd         float64
	maxPower      float64
	pwmCycleTime  float64
	minTemp       float64
	maxTemp       float64
	targetTemp    float64
	currentTemp   float64
	currentPower  float64
	enabled       bool
	smoothTime    float64
	callback      func(readTime, temp float64)
	mu            sync.Mutex
}

// GenericHeaterConfig holds configuration for generic heater.
type GenericHeaterConfig struct {
	Name         string
	HeaterPin    string
	SensorName   string
	Control      string
	PidKp        float64
	PidKi        float64
	PidKd        float64
	MaxPower     float64
	PwmCycleTime float64
	MinTemp      float64
	MaxTemp      float64
	SmoothTime   float64
}

// DefaultGenericHeaterConfig returns default generic heater configuration.
func DefaultGenericHeaterConfig() GenericHeaterConfig {
	return GenericHeaterConfig{
		Control:      "watermark",
		MaxPower:     1.0,
		PwmCycleTime: 0.100,
		MinTemp:      0,
		MaxTemp:      100,
		SmoothTime:   1.0,
	}
}

// newGenericHeater creates a new generic heater.
func newGenericHeater(rt *runtime, cfg GenericHeaterConfig) (*GenericHeater, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("heater_generic: name is required")
	}
	if cfg.HeaterPin == "" {
		return nil, fmt.Errorf("heater_generic: heater_pin is required")
	}
	if cfg.SensorName == "" {
		return nil, fmt.Errorf("heater_generic: sensor is required")
	}

	heater := &GenericHeater{
		rt:           rt,
		name:         cfg.Name,
		heaterPin:    cfg.HeaterPin,
		sensorName:   cfg.SensorName,
		control:      cfg.Control,
		pidKp:        cfg.PidKp,
		pidKi:        cfg.PidKi,
		pidKd:        cfg.PidKd,
		maxPower:     cfg.MaxPower,
		pwmCycleTime: cfg.PwmCycleTime,
		minTemp:      cfg.MinTemp,
		maxTemp:      cfg.MaxTemp,
		smoothTime:   cfg.SmoothTime,
		enabled:      true,
	}

	log.Printf("heater_generic: initialized '%s' pin=%s sensor=%s control=%s",
		cfg.Name, cfg.HeaterPin, cfg.SensorName, cfg.Control)
	return heater, nil
}

// GetName returns the heater name.
func (h *GenericHeater) GetName() string {
	return h.name
}

// SetCallback sets the temperature update callback.
func (h *GenericHeater) SetCallback(cb func(readTime, temp float64)) {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.callback = cb
}

// SetTarget sets the target temperature.
func (h *GenericHeater) SetTarget(target float64) error {
	h.mu.Lock()
	defer h.mu.Unlock()

	if target < h.minTemp || target > h.maxTemp {
		return fmt.Errorf("heater_generic: target %.1f out of range [%.1f, %.1f]",
			target, h.minTemp, h.maxTemp)
	}

	h.targetTemp = target
	log.Printf("heater_generic '%s': target=%.1f", h.name, target)
	return nil
}

// GetTarget returns the target temperature.
func (h *GenericHeater) GetTarget() float64 {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.targetTemp
}

// UpdateTemperature updates the current temperature.
func (h *GenericHeater) UpdateTemperature(readTime, temp float64) {
	h.mu.Lock()
	defer h.mu.Unlock()

	h.currentTemp = temp

	if h.callback != nil {
		h.callback(readTime, temp)
	}

	if !h.enabled || h.targetTemp == 0 {
		h.currentPower = 0
		return
	}

	// Simple control algorithm
	if h.control == "watermark" {
		// Bang-bang control
		if temp < h.targetTemp-1 {
			h.currentPower = h.maxPower
		} else if temp > h.targetTemp+1 {
			h.currentPower = 0
		}
	}
	// PID would be implemented here
}

// GetCurrentTemp returns the current temperature.
func (h *GenericHeater) GetCurrentTemp() float64 {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.currentTemp
}

// GetCurrentPower returns the current power level.
func (h *GenericHeater) GetCurrentPower() float64 {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.currentPower
}

// TurnOff turns off the heater.
func (h *GenericHeater) TurnOff() {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.targetTemp = 0
	h.currentPower = 0
}

// Enable enables the heater.
func (h *GenericHeater) Enable() {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.enabled = true
}

// Disable disables the heater.
func (h *GenericHeater) Disable() {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.enabled = false
	h.currentPower = 0
}

// IsEnabled returns whether the heater is enabled.
func (h *GenericHeater) IsEnabled() bool {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.enabled
}

// GetStatus returns the heater status.
func (h *GenericHeater) GetStatus() map[string]any {
	h.mu.Lock()
	defer h.mu.Unlock()

	return map[string]any{
		"temperature": h.currentTemp,
		"target":      h.targetTemp,
		"power":       h.currentPower,
		"enabled":     h.enabled,
	}
}
