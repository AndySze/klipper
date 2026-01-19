// AD5206 - port of klippy/extras/ad5206.py
//
// Support for AD5206 digital potentiometers (SPI)
//
// Copyright (C) 2017-2018 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// AD5206 represents an AD5206 digital potentiometer.
type AD5206 struct {
	rt        *runtime
	name      string
	scale     float64
	channels  [6]float64
	mu        sync.Mutex
}

// AD5206Config holds configuration for AD5206.
type AD5206Config struct {
	Name  string
	Scale float64 // Resistance scale (e.g., 10000 for 10k pot)
}

// DefaultAD5206Config returns default AD5206 configuration.
func DefaultAD5206Config() AD5206Config {
	return AD5206Config{
		Scale: 10000.0,
	}
}

// newAD5206 creates a new AD5206 digital potentiometer.
func newAD5206(rt *runtime, cfg AD5206Config) (*AD5206, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("ad5206: name is required")
	}
	if cfg.Scale <= 0 {
		cfg.Scale = 10000.0
	}

	ad := &AD5206{
		rt:    rt,
		name:  cfg.Name,
		scale: cfg.Scale,
	}

	log.Printf("ad5206: initialized '%s' scale=%.1f", cfg.Name, cfg.Scale)
	return ad, nil
}

// GetName returns the digipot name.
func (ad *AD5206) GetName() string {
	return ad.name
}

// SetChannel sets the resistance of a channel (0-5).
func (ad *AD5206) SetChannel(channel int, value float64) error {
	ad.mu.Lock()
	defer ad.mu.Unlock()

	if channel < 0 || channel > 5 {
		return fmt.Errorf("ad5206: channel %d out of range (0-5)", channel)
	}
	if value < 0 || value > ad.scale {
		return fmt.Errorf("ad5206: value %.2f out of range (0-%.1f)", value, ad.scale)
	}

	ad.channels[channel] = value

	// Convert to 8-bit value (0-255)
	wiperVal := byte(value / ad.scale * 255.0)

	// In real implementation, this would send SPI command
	log.Printf("ad5206 '%s': channel %d = %.2f (wiper=%d)", ad.name, channel, value, wiperVal)
	return nil
}

// GetChannel returns the current resistance of a channel.
func (ad *AD5206) GetChannel(channel int) (float64, error) {
	ad.mu.Lock()
	defer ad.mu.Unlock()

	if channel < 0 || channel > 5 {
		return 0, fmt.Errorf("ad5206: channel %d out of range (0-5)", channel)
	}

	return ad.channels[channel], nil
}

// GetStatus returns the digipot status.
func (ad *AD5206) GetStatus() map[string]any {
	ad.mu.Lock()
	defer ad.mu.Unlock()

	return map[string]any{
		"channels": ad.channels,
		"scale":    ad.scale,
	}
}
