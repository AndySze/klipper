// DAC084S085 - port of klippy/extras/dac084S085.py
//
// Support for DAC084S085 SPI DAC
//
// Copyright (C) 2018 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// DAC084S085 represents a DAC084S085 SPI DAC.
type DAC084S085 struct {
	rt       *runtime
	name     string
	channels [4]float64
	mu       sync.Mutex
}

// DAC084S085Config holds configuration for DAC084S085.
type DAC084S085Config struct {
	Name string
}

// newDAC084S085 creates a new DAC084S085 DAC.
func newDAC084S085(rt *runtime, cfg DAC084S085Config) (*DAC084S085, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("dac084s085: name is required")
	}

	dac := &DAC084S085{
		rt:   rt,
		name: cfg.Name,
	}

	log.Printf("dac084s085: initialized '%s'", cfg.Name)
	return dac, nil
}

// GetName returns the DAC name.
func (dac *DAC084S085) GetName() string {
	return dac.name
}

// SetChannel sets the output voltage of a channel (0-3).
// Value is 0.0-1.0 representing 0% to 100% of Vref.
func (dac *DAC084S085) SetChannel(channel int, value float64) error {
	dac.mu.Lock()
	defer dac.mu.Unlock()

	if channel < 0 || channel > 3 {
		return fmt.Errorf("dac084s085: channel %d out of range (0-3)", channel)
	}
	if value < 0 {
		value = 0
	} else if value > 1 {
		value = 1
	}

	dac.channels[channel] = value

	// Convert to 8-bit value (0-255)
	dacVal := byte(value * 255.0)

	// In real implementation, this would send SPI command
	// DAC084S085 uses 16-bit words: [A1,A0,X,X,D7-D0,X,X,X,X]
	log.Printf("dac084s085 '%s': channel %d = %.4f (dac=%d)", dac.name, channel, value, dacVal)
	return nil
}

// GetChannel returns the current value of a channel.
func (dac *DAC084S085) GetChannel(channel int) (float64, error) {
	dac.mu.Lock()
	defer dac.mu.Unlock()

	if channel < 0 || channel > 3 {
		return 0, fmt.Errorf("dac084s085: channel %d out of range (0-3)", channel)
	}

	return dac.channels[channel], nil
}

// GetStatus returns the DAC status.
func (dac *DAC084S085) GetStatus() map[string]any {
	dac.mu.Lock()
	defer dac.mu.Unlock()

	return map[string]any{
		"channels": dac.channels,
	}
}
