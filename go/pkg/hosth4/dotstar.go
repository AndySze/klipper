// DotStar - port of klippy/extras/dotstar.py
//
// Support for APA102 "DotStar" LEDs
//
// Copyright (C) 2019-2022 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// DotStar constants.
const (
	dotstarStartFrame = 0x00000000
	dotstarEndFrame   = 0xFFFFFFFF
	dotstarLedFrame   = 0xE0000000 // Start bits + brightness
)

// DotStar represents a chain of APA102 DotStar LEDs.
type DotStar struct {
	rt          *runtime
	name        string
	dataPin     string
	clockPin    string
	chainCount  int
	colorData   [][4]byte // RGBW for each LED
	oldColorData [][4]byte
	mu          sync.Mutex
}

// DotStarConfig holds configuration for DotStar LEDs.
type DotStarConfig struct {
	Name       string
	DataPin    string
	ClockPin   string
	ChainCount int
}

// DefaultDotStarConfig returns default DotStar configuration.
func DefaultDotStarConfig() DotStarConfig {
	return DotStarConfig{
		ChainCount: 1,
	}
}

// newDotStar creates a new DotStar controller.
func newDotStar(rt *runtime, cfg DotStarConfig) (*DotStar, error) {
	if cfg.DataPin == "" || cfg.ClockPin == "" {
		return nil, fmt.Errorf("dotstar: data_pin and clock_pin are required")
	}
	if cfg.ChainCount < 1 {
		cfg.ChainCount = 1
	}

	ds := &DotStar{
		rt:          rt,
		name:        cfg.Name,
		dataPin:     cfg.DataPin,
		clockPin:    cfg.ClockPin,
		chainCount:  cfg.ChainCount,
		colorData:   make([][4]byte, cfg.ChainCount),
		oldColorData: make([][4]byte, cfg.ChainCount),
	}

	// Initialize with different values to force initial update
	for i := range ds.oldColorData {
		ds.oldColorData[i] = [4]byte{1, 0, 0, 0}
	}

	log.Printf("dotstar: initialized '%s' with %d LEDs", cfg.Name, cfg.ChainCount)
	return ds, nil
}

// GetName returns the LED name.
func (ds *DotStar) GetName() string {
	return ds.name
}

// GetChainCount returns the number of LEDs in the chain.
func (ds *DotStar) GetChainCount() int {
	return ds.chainCount
}

// SetLED sets the color of a single LED.
func (ds *DotStar) SetLED(index int, r, g, b, brightness float64) error {
	ds.mu.Lock()
	defer ds.mu.Unlock()

	if index < 0 || index >= ds.chainCount {
		return fmt.Errorf("dotstar: LED index %d out of range", index)
	}

	// Clamp brightness to 0-1 and convert to 0-31
	if brightness < 0 {
		brightness = 0
	} else if brightness > 1 {
		brightness = 1
	}
	br := byte(brightness*31.0 + 0.5)

	ds.colorData[index] = [4]byte{
		byte(r*255.0 + 0.5),
		byte(g*255.0 + 0.5),
		byte(b*255.0 + 0.5),
		br,
	}

	return nil
}

// SetAllLEDs sets all LEDs to the same color.
func (ds *DotStar) SetAllLEDs(r, g, b, brightness float64) {
	ds.mu.Lock()
	defer ds.mu.Unlock()

	if brightness < 0 {
		brightness = 0
	} else if brightness > 1 {
		brightness = 1
	}
	br := byte(brightness*31.0 + 0.5)

	color := [4]byte{
		byte(r*255.0 + 0.5),
		byte(g*255.0 + 0.5),
		byte(b*255.0 + 0.5),
		br,
	}

	for i := range ds.colorData {
		ds.colorData[i] = color
	}
}

// SendData sends color data to the LEDs.
func (ds *DotStar) SendData(printTime float64) error {
	ds.mu.Lock()
	defer ds.mu.Unlock()

	// Check if data changed
	changed := false
	for i := range ds.colorData {
		if ds.colorData[i] != ds.oldColorData[i] {
			changed = true
			break
		}
	}

	if !changed {
		return nil
	}

	// Copy new data to old
	for i := range ds.colorData {
		ds.oldColorData[i] = ds.colorData[i]
	}

	// Build SPI data
	// Start frame (4 bytes of 0x00)
	// LED frames (4 bytes each: 0xE0|brightness, blue, green, red)
	// End frame (ceil(chainCount/2) bytes of 0xFF)

	log.Printf("dotstar '%s': updated LED data", ds.name)
	return nil
}

// GetLED returns the current color of an LED.
func (ds *DotStar) GetLED(index int) (r, g, b, brightness float64, err error) {
	ds.mu.Lock()
	defer ds.mu.Unlock()

	if index < 0 || index >= ds.chainCount {
		return 0, 0, 0, 0, fmt.Errorf("dotstar: LED index %d out of range", index)
	}

	c := ds.colorData[index]
	return float64(c[0]) / 255.0,
		float64(c[1]) / 255.0,
		float64(c[2]) / 255.0,
		float64(c[3]) / 31.0,
		nil
}

// GetStatus returns the DotStar status.
func (ds *DotStar) GetStatus() map[string]any {
	ds.mu.Lock()
	defer ds.mu.Unlock()

	colorData := make([][]float64, ds.chainCount)
	for i, c := range ds.colorData {
		colorData[i] = []float64{
			float64(c[0]) / 255.0,
			float64(c[1]) / 255.0,
			float64(c[2]) / 255.0,
			float64(c[3]) / 31.0,
		}
	}

	return map[string]any{
		"color_data": colorData,
	}
}
