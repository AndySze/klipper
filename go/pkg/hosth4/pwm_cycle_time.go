// PWM Cycle Time - port of klippy/extras/pwm_cycle_time.py
//
// Support for variable PWM cycle time outputs
//
// Copyright (C) 2017-2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// PWMCycleTime represents a PWM output with variable cycle time.
type PWMCycleTime struct {
	rt            *runtime
	name          string
	pin           string
	cycleTime     float64
	hardwarePwm   bool
	value         float64
	shutdownValue float64
	scale         float64
	lastPrintTime float64
	mu            sync.Mutex
}

// PWMCycleTimeConfig holds configuration for PWM cycle time.
type PWMCycleTimeConfig struct {
	Name          string
	Pin           string
	CycleTime     float64
	HardwarePwm   bool
	Value         float64 // Initial value
	ShutdownValue float64
	Scale         float64
}

// DefaultPWMCycleTimeConfig returns default configuration.
func DefaultPWMCycleTimeConfig() PWMCycleTimeConfig {
	return PWMCycleTimeConfig{
		CycleTime:     0.100, // 10 Hz default
		HardwarePwm:   false,
		Value:         0.0,
		ShutdownValue: 0.0,
		Scale:         1.0,
	}
}

// newPWMCycleTime creates a new PWM cycle time output.
func newPWMCycleTime(rt *runtime, cfg PWMCycleTimeConfig) (*PWMCycleTime, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("pwm_cycle_time: name is required")
	}
	if cfg.Pin == "" {
		return nil, fmt.Errorf("pwm_cycle_time: pin is required")
	}

	pwm := &PWMCycleTime{
		rt:            rt,
		name:          cfg.Name,
		pin:           cfg.Pin,
		cycleTime:     cfg.CycleTime,
		hardwarePwm:   cfg.HardwarePwm,
		value:         cfg.Value,
		shutdownValue: cfg.ShutdownValue,
		scale:         cfg.Scale,
	}

	log.Printf("pwm_cycle_time: initialized '%s' pin=%s cycle_time=%.4f",
		cfg.Name, cfg.Pin, cfg.CycleTime)
	return pwm, nil
}

// GetName returns the PWM name.
func (pwm *PWMCycleTime) GetName() string {
	return pwm.name
}

// SetValue sets the PWM value (0.0-1.0 or scaled).
func (pwm *PWMCycleTime) SetValue(value float64, printTime float64) {
	pwm.mu.Lock()
	defer pwm.mu.Unlock()

	// Apply scale
	if pwm.scale != 0 && pwm.scale != 1 {
		value /= pwm.scale
	}

	// Clamp
	if value < 0 {
		value = 0
	} else if value > 1 {
		value = 1
	}

	pwm.value = value
	pwm.lastPrintTime = printTime

	// In real implementation, this would set MCU PWM
	log.Printf("pwm_cycle_time '%s': value=%.4f", pwm.name, value)
}

// GetValue returns the current PWM value.
func (pwm *PWMCycleTime) GetValue() float64 {
	pwm.mu.Lock()
	defer pwm.mu.Unlock()
	return pwm.value * pwm.scale
}

// SetCycleTime changes the PWM cycle time.
func (pwm *PWMCycleTime) SetCycleTime(cycleTime float64) error {
	pwm.mu.Lock()
	defer pwm.mu.Unlock()

	if cycleTime <= 0 {
		return fmt.Errorf("pwm_cycle_time: cycle_time must be positive")
	}

	pwm.cycleTime = cycleTime
	log.Printf("pwm_cycle_time '%s': cycle_time=%.4f", pwm.name, cycleTime)
	return nil
}

// GetCycleTime returns the current cycle time.
func (pwm *PWMCycleTime) GetCycleTime() float64 {
	pwm.mu.Lock()
	defer pwm.mu.Unlock()
	return pwm.cycleTime
}

// GetStatus returns the PWM status.
func (pwm *PWMCycleTime) GetStatus() map[string]any {
	pwm.mu.Lock()
	defer pwm.mu.Unlock()

	return map[string]any{
		"value":      pwm.value * pwm.scale,
		"cycle_time": pwm.cycleTime,
	}
}
