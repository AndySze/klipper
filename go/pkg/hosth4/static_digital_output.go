// Static Digital Output - port of klippy/extras/static_digital_output.py
//
// Set the state of a list of digital output pins at startup
//
// Copyright (C) 2017-2018 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"strings"
	"sync"
)

// StaticDigitalOutput represents a set of pins that are set at startup.
type StaticDigitalOutput struct {
	rt   *runtime
	name string
	pins []staticDigitalPin
}

// staticDigitalPin holds info about a single static digital output pin.
type staticDigitalPin struct {
	pin    string
	invert bool
	value  bool // The actual value to set (taking invert into account)
}

// StaticDigitalOutputConfig holds configuration for static digital outputs.
type StaticDigitalOutputConfig struct {
	Name string
	Pins []string // List of pin descriptors, e.g., "PA1", "!PA2"
}

// newStaticDigitalOutput creates a new static digital output configuration.
func newStaticDigitalOutput(rt *runtime, cfg StaticDigitalOutputConfig) (*StaticDigitalOutput, error) {
	if len(cfg.Pins) == 0 {
		return nil, fmt.Errorf("static_digital_output: pins list is required")
	}

	sdo := &StaticDigitalOutput{
		rt:   rt,
		name: cfg.Name,
		pins: make([]staticDigitalPin, 0, len(cfg.Pins)),
	}

	for _, pinDesc := range cfg.Pins {
		pinName, invert := parseStaticPinDesc(pinDesc)
		sdo.pins = append(sdo.pins, staticDigitalPin{
			pin:    pinName,
			invert: invert,
			value:  !invert, // Default is HIGH unless inverted
		})
	}

	log.Printf("static_digital_output: initialized '%s' with %d pins", cfg.Name, len(sdo.pins))
	return sdo, nil
}

// parseStaticPinDesc parses a pin descriptor, extracting the pin name and invert flag.
// Pin format: "PIN" or "!PIN" (inverted)
func parseStaticPinDesc(desc string) (pinName string, invert bool) {
	desc = strings.TrimSpace(desc)
	if strings.HasPrefix(desc, "!") {
		return strings.TrimPrefix(desc, "!"), true
	}
	return desc, false
}

// GetName returns the static digital output name.
func (sdo *StaticDigitalOutput) GetName() string {
	return sdo.name
}

// GetPins returns the configured pins.
func (sdo *StaticDigitalOutput) GetPins() []string {
	result := make([]string, len(sdo.pins))
	for i, p := range sdo.pins {
		if p.invert {
			result[i] = "!" + p.pin
		} else {
			result[i] = p.pin
		}
	}
	return result
}

// GetConfigCommands returns the MCU commands needed to configure these pins.
func (sdo *StaticDigitalOutput) GetConfigCommands() []string {
	commands := make([]string, len(sdo.pins))
	for i, p := range sdo.pins {
		value := 1
		if !p.value {
			value = 0
		}
		commands[i] = fmt.Sprintf("set_digital_out pin=%s value=%d", p.pin, value)
	}
	return commands
}

// GetStatus returns the status of the static digital output.
func (sdo *StaticDigitalOutput) GetStatus() map[string]any {
	pinStatus := make([]map[string]any, len(sdo.pins))
	for i, p := range sdo.pins {
		pinStatus[i] = map[string]any{
			"pin":    p.pin,
			"invert": p.invert,
			"value":  p.value,
		}
	}
	return map[string]any{
		"name": sdo.name,
		"pins": pinStatus,
	}
}

// StaticDigitalOutputManager manages multiple static digital output configurations.
type StaticDigitalOutputManager struct {
	rt      *runtime
	outputs map[string]*StaticDigitalOutput
	mu      sync.RWMutex
}

// newStaticDigitalOutputManager creates a new static digital output manager.
func newStaticDigitalOutputManager(rt *runtime) *StaticDigitalOutputManager {
	return &StaticDigitalOutputManager{
		rt:      rt,
		outputs: make(map[string]*StaticDigitalOutput),
	}
}

// Register registers a new static digital output configuration.
func (m *StaticDigitalOutputManager) Register(cfg StaticDigitalOutputConfig) (*StaticDigitalOutput, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	sdo, err := newStaticDigitalOutput(m.rt, cfg)
	if err != nil {
		return nil, err
	}
	m.outputs[cfg.Name] = sdo
	return sdo, nil
}

// Get returns a static digital output by name.
func (m *StaticDigitalOutputManager) Get(name string) *StaticDigitalOutput {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.outputs[name]
}

// GetAll returns all registered static digital outputs.
func (m *StaticDigitalOutputManager) GetAll() []*StaticDigitalOutput {
	m.mu.RLock()
	defer m.mu.RUnlock()

	result := make([]*StaticDigitalOutput, 0, len(m.outputs))
	for _, sdo := range m.outputs {
		result = append(result, sdo)
	}
	return result
}

// GetAllConfigCommands returns all MCU config commands for all outputs.
func (m *StaticDigitalOutputManager) GetAllConfigCommands() []string {
	m.mu.RLock()
	defer m.mu.RUnlock()

	var commands []string
	for _, sdo := range m.outputs {
		commands = append(commands, sdo.GetConfigCommands()...)
	}
	return commands
}
