// PWM Tool - port of klippy/extras/pwm_tool.py
//
// Support for PWM controlled tools (lasers, spindles, etc.)
//
// Copyright (C) 2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// PWMTool represents a PWM controlled tool.
type PWMTool struct {
	rt            *runtime
	name          string
	pin           string
	cycleTime     float64
	hardwarePwm   bool
	scale         float64
	offBelow      float64
	maxPower      float64
	shutdownValue float64
	value         float64
	enabled       bool
	lastPrintTime float64
	callback      func(value float64)
	mu            sync.Mutex
}

// PWMToolConfig holds configuration for PWM tool.
type PWMToolConfig struct {
	Name          string
	Pin           string
	CycleTime     float64
	HardwarePwm   bool
	Scale         float64
	OffBelow      float64
	MaxPower      float64
	ShutdownValue float64
}

// DefaultPWMToolConfig returns default configuration.
func DefaultPWMToolConfig() PWMToolConfig {
	return PWMToolConfig{
		CycleTime:     0.100,
		HardwarePwm:   false,
		Scale:         1.0,
		OffBelow:      0.0,
		MaxPower:      1.0,
		ShutdownValue: 0.0,
	}
}

// newPWMTool creates a new PWM tool.
func newPWMTool(rt *runtime, cfg PWMToolConfig) (*PWMTool, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("pwm_tool: name is required")
	}
	if cfg.Pin == "" {
		return nil, fmt.Errorf("pwm_tool: pin is required")
	}

	tool := &PWMTool{
		rt:            rt,
		name:          cfg.Name,
		pin:           cfg.Pin,
		cycleTime:     cfg.CycleTime,
		hardwarePwm:   cfg.HardwarePwm,
		scale:         cfg.Scale,
		offBelow:      cfg.OffBelow,
		maxPower:      cfg.MaxPower,
		shutdownValue: cfg.ShutdownValue,
		enabled:       true,
	}

	log.Printf("pwm_tool: initialized '%s' pin=%s max_power=%.2f",
		cfg.Name, cfg.Pin, cfg.MaxPower)
	return tool, nil
}

// GetName returns the tool name.
func (t *PWMTool) GetName() string {
	return t.name
}

// SetCallback sets the value change callback.
func (t *PWMTool) SetCallback(cb func(value float64)) {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.callback = cb
}

// SetValue sets the tool power value.
func (t *PWMTool) SetValue(value float64, printTime float64) {
	t.mu.Lock()
	defer t.mu.Unlock()

	if !t.enabled {
		return
	}

	// Apply scale
	if t.scale != 0 && t.scale != 1 {
		value /= t.scale
	}

	// Clamp to max power
	if value > t.maxPower {
		value = t.maxPower
	}

	// Apply off_below threshold
	if value > 0 && value < t.offBelow {
		value = 0
	}

	t.value = value
	t.lastPrintTime = printTime

	if t.callback != nil {
		t.callback(value)
	}

	// In real implementation, this would set MCU PWM
	log.Printf("pwm_tool '%s': value=%.4f", t.name, value)
}

// GetValue returns the current power value.
func (t *PWMTool) GetValue() float64 {
	t.mu.Lock()
	defer t.mu.Unlock()
	return t.value * t.scale
}

// TurnOff turns off the tool.
func (t *PWMTool) TurnOff(printTime float64) {
	t.SetValue(0, printTime)
}

// Enable enables the tool.
func (t *PWMTool) Enable() {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.enabled = true
}

// Disable disables the tool.
func (t *PWMTool) Disable() {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.enabled = false
	t.value = 0
}

// IsEnabled returns whether the tool is enabled.
func (t *PWMTool) IsEnabled() bool {
	t.mu.Lock()
	defer t.mu.Unlock()
	return t.enabled
}

// GetStatus returns the tool status.
func (t *PWMTool) GetStatus() map[string]any {
	t.mu.Lock()
	defer t.mu.Unlock()

	return map[string]any{
		"value":     t.value * t.scale,
		"enabled":   t.enabled,
		"max_power": t.maxPower,
	}
}

// PWMToolManager manages multiple PWM tools.
type PWMToolManager struct {
	rt    *runtime
	tools map[string]*PWMTool
	mu    sync.RWMutex
}

// newPWMToolManager creates a new PWM tool manager.
func newPWMToolManager(rt *runtime) *PWMToolManager {
	return &PWMToolManager{
		rt:    rt,
		tools: make(map[string]*PWMTool),
	}
}

// AddTool adds a tool to the manager.
func (mgr *PWMToolManager) AddTool(tool *PWMTool) {
	mgr.mu.Lock()
	defer mgr.mu.Unlock()
	mgr.tools[tool.name] = tool
}

// GetTool returns a tool by name.
func (mgr *PWMToolManager) GetTool(name string) (*PWMTool, bool) {
	mgr.mu.RLock()
	defer mgr.mu.RUnlock()
	tool, ok := mgr.tools[name]
	return tool, ok
}

// SetToolValue sets value for a named tool.
func (mgr *PWMToolManager) SetToolValue(name string, value float64, printTime float64) error {
	tool, ok := mgr.GetTool(name)
	if !ok {
		return fmt.Errorf("pwm_tool: tool '%s' not found", name)
	}
	tool.SetValue(value, printTime)
	return nil
}

// GetStatus returns status of all tools.
func (mgr *PWMToolManager) GetStatus() map[string]any {
	mgr.mu.RLock()
	defer mgr.mu.RUnlock()

	status := make(map[string]any)
	for name, tool := range mgr.tools {
		status[name] = tool.GetStatus()
	}
	return status
}
