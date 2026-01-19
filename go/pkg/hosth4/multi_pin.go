// Multi Pin - port of klippy/extras/multi_pin.py
//
// Virtual pin that propagates its changes to multiple output pins
//
// Copyright (C) 2017-2021 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// MultiPin represents a virtual pin that controls multiple physical pins.
type MultiPin struct {
	rt       *runtime
	name     string
	pinList  []string // List of pin descriptions
	pinType  string   // "digital" or "pwm"
	invert   bool
	mu       sync.Mutex
}

// MultiPinConfig holds configuration for a multi-pin.
type MultiPinConfig struct {
	Name   string   // Pin name
	Pins   []string // List of physical pins
	Invert bool     // Whether to invert the output
}

// newMultiPin creates a new multi-pin controller.
func newMultiPin(rt *runtime, cfg MultiPinConfig) (*MultiPin, error) {
	if len(cfg.Pins) == 0 {
		return nil, fmt.Errorf("multi_pin '%s' requires at least one pin", cfg.Name)
	}

	mp := &MultiPin{
		rt:      rt,
		name:    cfg.Name,
		pinList: cfg.Pins,
		invert:  cfg.Invert,
	}

	log.Printf("multi_pin: initialized '%s' with %d pins", cfg.Name, len(cfg.Pins))
	return mp, nil
}

// SetupDigital sets up the multi-pin for digital output.
func (mp *MultiPin) SetupDigital() error {
	mp.mu.Lock()
	defer mp.mu.Unlock()

	if mp.pinType != "" && mp.pinType != "digital" {
		return fmt.Errorf("multi_pin '%s' already setup as %s", mp.name, mp.pinType)
	}
	mp.pinType = "digital"

	// Note: Actual pin setup would require OID allocation and command registration
	// This is a simplified implementation
	log.Printf("multi_pin: setup digital output for '%s'", mp.name)
	return nil
}

// SetupPWM sets up the multi-pin for PWM output.
func (mp *MultiPin) SetupPWM() error {
	mp.mu.Lock()
	defer mp.mu.Unlock()

	if mp.pinType != "" && mp.pinType != "pwm" {
		return fmt.Errorf("multi_pin '%s' already setup as %s", mp.name, mp.pinType)
	}
	mp.pinType = "pwm"

	// Note: Actual pin setup would require OID allocation and command registration
	// This is a simplified implementation
	log.Printf("multi_pin: setup PWM output for '%s'", mp.name)
	return nil
}

// SetDigital sets the digital output value.
func (mp *MultiPin) SetDigital(printTime float64, value bool) error {
	mp.mu.Lock()
	defer mp.mu.Unlock()

	if mp.pinType != "digital" {
		return fmt.Errorf("multi_pin '%s' not setup for digital output", mp.name)
	}

	if mp.invert {
		value = !value
	}

	// Note: Would send commands to each physical pin
	log.Printf("multi_pin '%s': setDigital time=%.4f value=%v", mp.name, printTime, value)
	return nil
}

// SetPWM sets the PWM output value.
func (mp *MultiPin) SetPWM(printTime float64, value float64) error {
	mp.mu.Lock()
	defer mp.mu.Unlock()

	if mp.pinType != "pwm" {
		return fmt.Errorf("multi_pin '%s' not setup for PWM output", mp.name)
	}

	if mp.invert {
		value = 1.0 - value
	}

	// Note: Would send commands to each physical pin
	log.Printf("multi_pin '%s': setPWM time=%.4f value=%.4f", mp.name, printTime, value)
	return nil
}

// GetName returns the multi-pin name.
func (mp *MultiPin) GetName() string {
	return mp.name
}

// GetPins returns the list of physical pins.
func (mp *MultiPin) GetPins() []string {
	return mp.pinList
}

// GetStatus returns the multi-pin status.
func (mp *MultiPin) GetStatus() map[string]any {
	mp.mu.Lock()
	defer mp.mu.Unlock()

	return map[string]any{
		"name":     mp.name,
		"pin_type": mp.pinType,
		"pins":     mp.pinList,
		"invert":   mp.invert,
	}
}

// MultiPinManager manages multiple multi-pin instances.
type MultiPinManager struct {
	rt        *runtime
	multiPins map[string]*MultiPin
	mu        sync.RWMutex
}

// newMultiPinManager creates a new multi-pin manager.
func newMultiPinManager(rt *runtime) *MultiPinManager {
	return &MultiPinManager{
		rt:        rt,
		multiPins: make(map[string]*MultiPin),
	}
}

// Register registers a new multi-pin.
func (mpm *MultiPinManager) Register(cfg MultiPinConfig) (*MultiPin, error) {
	mpm.mu.Lock()
	defer mpm.mu.Unlock()

	if _, exists := mpm.multiPins[cfg.Name]; exists {
		return nil, fmt.Errorf("multi_pin '%s' already registered", cfg.Name)
	}

	mp, err := newMultiPin(mpm.rt, cfg)
	if err != nil {
		return nil, err
	}

	mpm.multiPins[cfg.Name] = mp
	return mp, nil
}

// Get returns a multi-pin by name.
func (mpm *MultiPinManager) Get(name string) *MultiPin {
	mpm.mu.RLock()
	defer mpm.mu.RUnlock()
	return mpm.multiPins[name]
}

// GetAll returns all multi-pin names.
func (mpm *MultiPinManager) GetAll() []string {
	mpm.mu.RLock()
	defer mpm.mu.RUnlock()

	names := make([]string, 0, len(mpm.multiPins))
	for name := range mpm.multiPins {
		names = append(names, name)
	}
	return names
}
