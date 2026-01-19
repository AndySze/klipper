// MCP4728 - port of klippy/extras/mcp4728.py
//
// Support for MCP4728 I2C DAC
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

// MCP4728 constants.
const (
	mcp4728I2CAddr = 0x60
	mcp4728MaxVal  = 4095 // 12-bit DAC
)

// MCP4728 represents an MCP4728 I2C DAC.
type MCP4728 struct {
	rt       *runtime
	name     string
	i2cAddr  int
	channels [4]uint16
	mu       sync.Mutex
}

// MCP4728Config holds configuration for MCP4728.
type MCP4728Config struct {
	Name    string
	I2CAddr int
}

// DefaultMCP4728Config returns default MCP4728 configuration.
func DefaultMCP4728Config() MCP4728Config {
	return MCP4728Config{
		I2CAddr: mcp4728I2CAddr,
	}
}

// newMCP4728 creates a new MCP4728 DAC.
func newMCP4728(rt *runtime, cfg MCP4728Config) (*MCP4728, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("mcp4728: name is required")
	}
	if cfg.I2CAddr <= 0 {
		cfg.I2CAddr = mcp4728I2CAddr
	}

	mcp := &MCP4728{
		rt:      rt,
		name:    cfg.Name,
		i2cAddr: cfg.I2CAddr,
	}

	log.Printf("mcp4728: initialized '%s' addr=0x%02x", cfg.Name, cfg.I2CAddr)
	return mcp, nil
}

// GetName returns the DAC name.
func (mcp *MCP4728) GetName() string {
	return mcp.name
}

// SetChannel sets the output value of a channel (0-3).
// Value is 0.0-1.0 representing 0% to 100% of Vref, or 0-4095 direct value.
func (mcp *MCP4728) SetChannel(channel int, value float64) error {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()

	if channel < 0 || channel > 3 {
		return fmt.Errorf("mcp4728: channel %d out of range (0-3)", channel)
	}

	var dacVal uint16
	if value <= 1.0 {
		// Treat as ratio
		if value < 0 {
			value = 0
		}
		dacVal = uint16(value * float64(mcp4728MaxVal))
	} else {
		// Treat as direct value
		dacVal = uint16(value)
	}

	if dacVal > mcp4728MaxVal {
		dacVal = mcp4728MaxVal
	}

	mcp.channels[channel] = dacVal

	// In real implementation, this would send I2C command
	log.Printf("mcp4728 '%s': channel %d = %d (%.4f)", mcp.name, channel, dacVal, float64(dacVal)/float64(mcp4728MaxVal))
	return nil
}

// GetChannel returns the current value of a channel.
func (mcp *MCP4728) GetChannel(channel int) (uint16, error) {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()

	if channel < 0 || channel > 3 {
		return 0, fmt.Errorf("mcp4728: channel %d out of range (0-3)", channel)
	}

	return mcp.channels[channel], nil
}

// GetChannelVoltage returns the current value as a ratio (0.0-1.0).
func (mcp *MCP4728) GetChannelVoltage(channel int) (float64, error) {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()

	if channel < 0 || channel > 3 {
		return 0, fmt.Errorf("mcp4728: channel %d out of range (0-3)", channel)
	}

	return float64(mcp.channels[channel]) / float64(mcp4728MaxVal), nil
}

// SetAllChannels sets all channels at once.
func (mcp *MCP4728) SetAllChannels(a, b, c, d float64) error {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()

	values := []float64{a, b, c, d}
	for i, v := range values {
		var dacVal uint16
		if v <= 1.0 {
			if v < 0 {
				v = 0
			}
			dacVal = uint16(v * float64(mcp4728MaxVal))
		} else {
			dacVal = uint16(v)
		}
		if dacVal > mcp4728MaxVal {
			dacVal = mcp4728MaxVal
		}
		mcp.channels[i] = dacVal
	}

	log.Printf("mcp4728 '%s': all channels = [%d,%d,%d,%d]",
		mcp.name, mcp.channels[0], mcp.channels[1], mcp.channels[2], mcp.channels[3])
	return nil
}

// GetStatus returns the DAC status.
func (mcp *MCP4728) GetStatus() map[string]any {
	mcp.mu.Lock()
	defer mcp.mu.Unlock()

	voltages := [4]float64{}
	for i, c := range mcp.channels {
		voltages[i] = float64(c) / float64(mcp4728MaxVal)
	}

	return map[string]any{
		"channels": mcp.channels,
		"voltages": voltages,
	}
}
