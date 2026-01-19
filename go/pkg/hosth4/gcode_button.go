// G-code Button - port of klippy/extras/gcode_button.py
//
// Support for G-code triggered buttons/sensors
//
// Copyright (C) 2019 Florian Heilmann <Florian.Heilmann@gmx.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// GCodeButton represents a button that triggers G-code on state change.
type GCodeButton struct {
	rt           *runtime
	name         string
	pin          string
	pullUp       bool
	analogRange  [2]float64
	pressGCode   string
	releaseGCode string
	state        bool
	lastState    bool
	callback     func(pressed bool)
	mu           sync.Mutex
}

// GCodeButtonConfig holds configuration for GCodeButton.
type GCodeButtonConfig struct {
	Name         string
	Pin          string
	PullUp       bool
	AnalogRange  [2]float64 // [min, max] for analog input, or [0,0] for digital
	PressGCode   string
	ReleaseGCode string
}

// newGCodeButton creates a new G-code button.
func newGCodeButton(rt *runtime, cfg GCodeButtonConfig) (*GCodeButton, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("gcode_button: name is required")
	}
	if cfg.Pin == "" {
		return nil, fmt.Errorf("gcode_button: pin is required")
	}

	btn := &GCodeButton{
		rt:           rt,
		name:         cfg.Name,
		pin:          cfg.Pin,
		pullUp:       cfg.PullUp,
		analogRange:  cfg.AnalogRange,
		pressGCode:   cfg.PressGCode,
		releaseGCode: cfg.ReleaseGCode,
	}

	log.Printf("gcode_button: initialized '%s' pin=%s", cfg.Name, cfg.Pin)
	return btn, nil
}

// GetName returns the button name.
func (btn *GCodeButton) GetName() string {
	return btn.name
}

// SetCallback sets the state change callback.
func (btn *GCodeButton) SetCallback(cb func(pressed bool)) {
	btn.mu.Lock()
	defer btn.mu.Unlock()
	btn.callback = cb
}

// IsAnalog returns whether this is an analog button.
func (btn *GCodeButton) IsAnalog() bool {
	return btn.analogRange[0] != 0 || btn.analogRange[1] != 0
}

// ProcessDigitalState processes digital input state.
func (btn *GCodeButton) ProcessDigitalState(pressed bool) {
	btn.mu.Lock()
	defer btn.mu.Unlock()

	if btn.pullUp {
		pressed = !pressed
	}

	btn.lastState = btn.state
	btn.state = pressed

	if btn.state != btn.lastState {
		if btn.callback != nil {
			btn.callback(btn.state)
		}

		if btn.state && btn.pressGCode != "" {
			log.Printf("gcode_button '%s': running press gcode", btn.name)
		} else if !btn.state && btn.releaseGCode != "" {
			log.Printf("gcode_button '%s': running release gcode", btn.name)
		}
	}
}

// ProcessAnalogValue processes analog input value.
func (btn *GCodeButton) ProcessAnalogValue(value float64) {
	pressed := value >= btn.analogRange[0] && value <= btn.analogRange[1]
	btn.ProcessDigitalState(pressed)
}

// GetState returns the current button state.
func (btn *GCodeButton) GetState() bool {
	btn.mu.Lock()
	defer btn.mu.Unlock()
	return btn.state
}

// GetPressGCode returns the press G-code.
func (btn *GCodeButton) GetPressGCode() string {
	return btn.pressGCode
}

// GetReleaseGCode returns the release G-code.
func (btn *GCodeButton) GetReleaseGCode() string {
	return btn.releaseGCode
}

// GetStatus returns the button status.
func (btn *GCodeButton) GetStatus() map[string]any {
	btn.mu.Lock()
	defer btn.mu.Unlock()

	return map[string]any{
		"state": btn.state,
		"pin":   btn.pin,
	}
}

// GCodeButtonManager manages multiple G-code buttons.
type GCodeButtonManager struct {
	rt      *runtime
	buttons map[string]*GCodeButton
	mu      sync.RWMutex
}

// newGCodeButtonManager creates a new button manager.
func newGCodeButtonManager(rt *runtime) *GCodeButtonManager {
	return &GCodeButtonManager{
		rt:      rt,
		buttons: make(map[string]*GCodeButton),
	}
}

// AddButton adds a button to the manager.
func (mgr *GCodeButtonManager) AddButton(btn *GCodeButton) {
	mgr.mu.Lock()
	defer mgr.mu.Unlock()
	mgr.buttons[btn.name] = btn
}

// GetButton returns a button by name.
func (mgr *GCodeButtonManager) GetButton(name string) (*GCodeButton, bool) {
	mgr.mu.RLock()
	defer mgr.mu.RUnlock()
	btn, ok := mgr.buttons[name]
	return btn, ok
}

// GetStatus returns status of all buttons.
func (mgr *GCodeButtonManager) GetStatus() map[string]any {
	mgr.mu.RLock()
	defer mgr.mu.RUnlock()

	status := make(map[string]any)
	for name, btn := range mgr.buttons {
		status[name] = btn.GetStatus()
	}
	return status
}
