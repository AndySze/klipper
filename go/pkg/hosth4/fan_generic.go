// Fan Generic - port of klippy/extras/fan_generic.py
//
// Support for generic fans (non-heater controlled)
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

// FanGeneric represents a generic controllable fan.
type FanGeneric struct {
	rt           *runtime
	name         string
	pin          string
	maxPower     float64
	shutdownPower float64
	cycleTime    float64
	hardwarePwm  bool
	kickStartTime float64
	kickStartPower float64
	offBelow     float64
	speed        float64
	lastSpeed    float64
	enabled      bool
	mu           sync.Mutex
}

// FanGenericConfig holds configuration for generic fan.
type FanGenericConfig struct {
	Name           string
	Pin            string
	MaxPower       float64
	ShutdownPower  float64
	CycleTime      float64
	HardwarePwm    bool
	KickStartTime  float64
	KickStartPower float64
	OffBelow       float64
}

// DefaultFanGenericConfig returns default generic fan configuration.
func DefaultFanGenericConfig() FanGenericConfig {
	return FanGenericConfig{
		MaxPower:       1.0,
		ShutdownPower:  0.0,
		CycleTime:      0.010,
		HardwarePwm:    false,
		KickStartTime:  0.1,
		KickStartPower: 1.0,
		OffBelow:       0.0,
	}
}

// newFanGeneric creates a new generic fan.
func newFanGeneric(rt *runtime, cfg FanGenericConfig) (*FanGeneric, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("fan_generic: name is required")
	}
	if cfg.Pin == "" {
		return nil, fmt.Errorf("fan_generic: pin is required")
	}

	fan := &FanGeneric{
		rt:            rt,
		name:          cfg.Name,
		pin:           cfg.Pin,
		maxPower:      cfg.MaxPower,
		shutdownPower: cfg.ShutdownPower,
		cycleTime:     cfg.CycleTime,
		hardwarePwm:   cfg.HardwarePwm,
		kickStartTime: cfg.KickStartTime,
		kickStartPower: cfg.KickStartPower,
		offBelow:      cfg.OffBelow,
		enabled:       true,
	}

	log.Printf("fan_generic: initialized '%s' pin=%s max_power=%.2f", cfg.Name, cfg.Pin, cfg.MaxPower)
	return fan, nil
}

// GetName returns the fan name.
func (f *FanGeneric) GetName() string {
	return f.name
}

// SetSpeed sets the fan speed (0.0-1.0).
func (f *FanGeneric) SetSpeed(speed float64, printTime float64) {
	f.mu.Lock()
	defer f.mu.Unlock()

	if !f.enabled {
		return
	}

	// Clamp speed
	if speed < 0 {
		speed = 0
	} else if speed > 1 {
		speed = 1
	}

	// Apply off_below threshold
	if speed > 0 && speed < f.offBelow {
		speed = 0
	}

	// Apply max_power
	speed *= f.maxPower

	f.lastSpeed = f.speed
	f.speed = speed

	// In real implementation, this would set PWM
	log.Printf("fan_generic '%s': speed=%.2f", f.name, speed)
}

// GetSpeed returns the current fan speed.
func (f *FanGeneric) GetSpeed() float64 {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.speed
}

// TurnOff turns off the fan.
func (f *FanGeneric) TurnOff(printTime float64) {
	f.SetSpeed(0, printTime)
}

// Enable enables the fan.
func (f *FanGeneric) Enable() {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.enabled = true
}

// Disable disables the fan.
func (f *FanGeneric) Disable() {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.enabled = false
	f.speed = 0
}

// IsEnabled returns whether the fan is enabled.
func (f *FanGeneric) IsEnabled() bool {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.enabled
}

// GetStatus returns the fan status.
func (f *FanGeneric) GetStatus() map[string]any {
	f.mu.Lock()
	defer f.mu.Unlock()

	return map[string]any{
		"speed":   f.speed,
		"rpm":     0, // Would need tachometer
		"enabled": f.enabled,
	}
}

// FanGenericManager manages multiple generic fans.
type FanGenericManager struct {
	rt   *runtime
	fans map[string]*FanGeneric
	mu   sync.RWMutex
}

// newFanGenericManager creates a new fan manager.
func newFanGenericManager(rt *runtime) *FanGenericManager {
	return &FanGenericManager{
		rt:   rt,
		fans: make(map[string]*FanGeneric),
	}
}

// AddFan adds a fan to the manager.
func (mgr *FanGenericManager) AddFan(fan *FanGeneric) {
	mgr.mu.Lock()
	defer mgr.mu.Unlock()
	mgr.fans[fan.name] = fan
}

// GetFan returns a fan by name.
func (mgr *FanGenericManager) GetFan(name string) (*FanGeneric, bool) {
	mgr.mu.RLock()
	defer mgr.mu.RUnlock()
	fan, ok := mgr.fans[name]
	return fan, ok
}

// SetFanSpeed sets speed for a named fan.
func (mgr *FanGenericManager) SetFanSpeed(name string, speed float64, printTime float64) error {
	fan, ok := mgr.GetFan(name)
	if !ok {
		return fmt.Errorf("fan_generic: fan '%s' not found", name)
	}
	fan.SetSpeed(speed, printTime)
	return nil
}

// GetStatus returns status of all fans.
func (mgr *FanGenericManager) GetStatus() map[string]any {
	mgr.mu.RLock()
	defer mgr.mu.RUnlock()

	status := make(map[string]any)
	for name, fan := range mgr.fans {
		status[name] = fan.GetStatus()
	}
	return status
}
