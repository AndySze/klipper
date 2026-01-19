// MCP4451 - port of klippy/extras/mcp4451.py
//
// Support for MCP4451 I2C digital potentiometer
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

// MCP4451 constants.
const (
	mcp4451I2CAddr  = 0x2C
	mcp4451MaxWiper = 256
)

// MCP4451 register addresses.
const (
	mcp4451RegWiper0 = 0x00
	mcp4451RegWiper1 = 0x01
	mcp4451RegWiper2 = 0x06
	mcp4451RegWiper3 = 0x07
)

// MCP4451 represents an MCP4451 I2C digital potentiometer.
type MCP4451 struct {
	rt      *runtime
	name    string
	i2cAddr int
	scale   float64
	wipers  [4]int
	mu      sync.Mutex
}

// MCP4451Config holds configuration for MCP4451.
type MCP4451Config struct {
	Name    string
	I2CAddr int
	Scale   float64
}

// DefaultMCP4451Config returns default MCP4451 configuration.
func DefaultMCP4451Config() MCP4451Config {
	return MCP4451Config{
		I2CAddr: mcp4451I2CAddr,
		Scale:   10000.0,
	}
}

// newMCP4451 creates a new MCP4451 digital potentiometer.
func newMCP4451(rt *runtime, cfg MCP4451Config) (*MCP4451, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("mcp4451: name is required")
	}
	if cfg.I2CAddr <= 0 {
		cfg.I2CAddr = mcp4451I2CAddr
	}
	if cfg.Scale <= 0 {
		cfg.Scale = 10000.0
	}

	mcp := &MCP4451{
		rt:      rt,
		name:    cfg.Name,
		i2cAddr: cfg.I2CAddr,
		scale:   cfg.Scale,
	}

	log.Printf("mcp4451: initialized '%s' addr=0x%02x scale=%.1f", cfg.Name, cfg.I2CAddr, cfg.Scale)
	return mcp, nil
}

// GetName returns the digipot name.
func (mcp *MCP4451) GetName() string {
	return mcp.name
}

// SetWiper sets the wiper position for a channel (0-3).
func (mcp *MCP4451) SetWiper(channel int, value float64) error {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()

	if channel < 0 || channel > 3 {
		return fmt.Errorf("mcp4451: channel %d out of range (0-3)", channel)
	}

	var wiperVal int
	if value <= float64(mcp4451MaxWiper) {
		wiperVal = int(value + 0.5)
	} else {
		wiperVal = int(value / mcp.scale * float64(mcp4451MaxWiper))
	}

	if wiperVal < 0 {
		wiperVal = 0
	} else if wiperVal > mcp4451MaxWiper {
		wiperVal = mcp4451MaxWiper
	}

	mcp.wipers[channel] = wiperVal

	// In real implementation, this would send I2C command
	log.Printf("mcp4451 '%s': wiper[%d] = %d", mcp.name, channel, wiperVal)
	return nil
}

// GetWiper returns the current wiper position for a channel.
func (mcp *MCP4451) GetWiper(channel int) (int, error) {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()

	if channel < 0 || channel > 3 {
		return 0, fmt.Errorf("mcp4451: channel %d out of range (0-3)", channel)
	}

	return mcp.wipers[channel], nil
}

// GetResistance returns the current resistance for a channel.
func (mcp *MCP4451) GetResistance(channel int) (float64, error) {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()

	if channel < 0 || channel > 3 {
		return 0, fmt.Errorf("mcp4451: channel %d out of range (0-3)", channel)
	}

	return float64(mcp.wipers[channel]) / float64(mcp4451MaxWiper) * mcp.scale, nil
}

// GetStatus returns the digipot status.
func (mcp *MCP4451) GetStatus() map[string]any {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()

	resistances := [4]float64{}
	for i, w := range mcp.wipers {
		resistances[i] = float64(w) / float64(mcp4451MaxWiper) * mcp.scale
	}

	return map[string]any{
		"wipers":      mcp.wipers,
		"resistances": resistances,
	}
}
