// MCP4018 - port of klippy/extras/mcp4018.py
//
// Support for MCP4018 I2C digital potentiometer
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

// MCP4018 constants.
const (
	mcp4018I2CAddr  = 0x2F
	mcp4018MaxWiper = 127
)

// MCP4018 represents an MCP4018 I2C digital potentiometer.
type MCP4018 struct {
	rt      *runtime
	name    string
	i2cAddr int
	scale   float64
	wiper   int
	mu      sync.Mutex
}

// MCP4018Config holds configuration for MCP4018.
type MCP4018Config struct {
	Name    string
	I2CAddr int
	Scale   float64
}

// DefaultMCP4018Config returns default MCP4018 configuration.
func DefaultMCP4018Config() MCP4018Config {
	return MCP4018Config{
		I2CAddr: mcp4018I2CAddr,
		Scale:   10000.0,
	}
}

// newMCP4018 creates a new MCP4018 digital potentiometer.
func newMCP4018(rt *runtime, cfg MCP4018Config) (*MCP4018, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("mcp4018: name is required")
	}
	if cfg.I2CAddr <= 0 {
		cfg.I2CAddr = mcp4018I2CAddr
	}
	if cfg.Scale <= 0 {
		cfg.Scale = 10000.0
	}

	mcp := &MCP4018{
		rt:      rt,
		name:    cfg.Name,
		i2cAddr: cfg.I2CAddr,
		scale:   cfg.Scale,
	}

	log.Printf("mcp4018: initialized '%s' addr=0x%02x scale=%.1f", cfg.Name, cfg.I2CAddr, cfg.Scale)
	return mcp, nil
}

// GetName returns the digipot name.
func (mcp *MCP4018) GetName() string {
	return mcp.name
}

// SetWiper sets the wiper position (0-127 or resistance value).
func (mcp *MCP4018) SetWiper(value float64) error {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()

	var wiperVal int
	if value <= float64(mcp4018MaxWiper) {
		// Treat as direct wiper value
		wiperVal = int(value + 0.5)
	} else {
		// Treat as resistance value
		wiperVal = int(value / mcp.scale * float64(mcp4018MaxWiper))
	}

	if wiperVal < 0 {
		wiperVal = 0
	} else if wiperVal > mcp4018MaxWiper {
		wiperVal = mcp4018MaxWiper
	}

	mcp.wiper = wiperVal

	// In real implementation, this would send I2C command
	log.Printf("mcp4018 '%s': wiper = %d", mcp.name, wiperVal)
	return nil
}

// GetWiper returns the current wiper position.
func (mcp *MCP4018) GetWiper() int {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()
	return mcp.wiper
}

// GetResistance returns the current resistance value.
func (mcp *MCP4018) GetResistance() float64 {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()
	return float64(mcp.wiper) / float64(mcp4018MaxWiper) * mcp.scale
}

// GetStatus returns the digipot status.
func (mcp *MCP4018) GetStatus() map[string]any {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()

	return map[string]any{
		"wiper":      mcp.wiper,
		"resistance": float64(mcp.wiper) / float64(mcp4018MaxWiper) * mcp.scale,
	}
}
