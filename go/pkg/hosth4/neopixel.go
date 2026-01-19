// NeoPixel - port of klippy/extras/neopixel.py
//
// Support for "neopixel" leds
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

const (
	neopixelBitMaxTime   = 0.000004
	neopixelResetMinTime = 0.000050
	neopixelMaxChainSize = 500
)

// NeoPixel represents a chain of NeoPixel LEDs.
type NeoPixel struct {
	rt          *runtime
	name        string
	pin         string
	chainCount  int
	colorOrder  []string // Color order for each LED (e.g., "GRB", "RGBW")
	colorMap    []struct {
		ledIdx   int
		colorIdx int
	}
	colorData    []byte
	oldColorData []byte
	mu           sync.Mutex
}

// NeoPixelConfig holds configuration for NeoPixel LEDs.
type NeoPixelConfig struct {
	Name       string
	Pin        string
	ChainCount int      // Number of LEDs in chain
	ColorOrder []string // Color order for each LED (or single value for all)
}

// DefaultNeoPixelConfig returns default NeoPixel configuration.
func DefaultNeoPixelConfig() NeoPixelConfig {
	return NeoPixelConfig{
		ChainCount: 1,
		ColorOrder: []string{"GRB"},
	}
}

// newNeoPixel creates a new NeoPixel controller.
func newNeoPixel(rt *runtime, cfg NeoPixelConfig) (*NeoPixel, error) {
	if cfg.ChainCount < 1 {
		cfg.ChainCount = 1
	}
	if cfg.Pin == "" {
		return nil, fmt.Errorf("neopixel: pin is required")
	}

	// Expand color order to match chain count
	colorOrder := cfg.ColorOrder
	if len(colorOrder) == 0 {
		colorOrder = []string{"GRB"}
	}
	if len(colorOrder) == 1 {
		expanded := make([]string, cfg.ChainCount)
		for i := range expanded {
			expanded[i] = colorOrder[0]
		}
		colorOrder = expanded
	}
	if len(colorOrder) != cfg.ChainCount {
		return nil, fmt.Errorf("neopixel: color_order count doesn't match chain_count")
	}

	// Validate color orders and build color map
	colorMap := make([]struct{ ledIdx, colorIdx int }, 0)
	for lidx, co := range colorOrder {
		if !isValidColorOrder(co) {
			return nil, fmt.Errorf("neopixel: invalid color_order '%s'", co)
		}
		for _, c := range co {
			colorIdx := colorCharToIndex(byte(c))
			colorMap = append(colorMap, struct{ ledIdx, colorIdx int }{lidx, colorIdx})
		}
	}

	if len(colorMap) > neopixelMaxChainSize {
		return nil, fmt.Errorf("neopixel: chain too long")
	}

	np := &NeoPixel{
		rt:          rt,
		name:        cfg.Name,
		pin:         cfg.Pin,
		chainCount:  cfg.ChainCount,
		colorOrder:  colorOrder,
		colorMap:    colorMap,
		colorData:   make([]byte, len(colorMap)),
		oldColorData: make([]byte, len(colorMap)),
	}

	// Initialize old data to something different
	for i := range np.oldColorData {
		np.oldColorData[i] = np.colorData[i] ^ 1
	}

	log.Printf("neopixel: initialized '%s' with %d LEDs", cfg.Name, cfg.ChainCount)
	return np, nil
}

// isValidColorOrder checks if a color order string is valid.
func isValidColorOrder(co string) bool {
	sorted := sortString(co)
	return sorted == "BGR" || sorted == "BGRW"
}

// sortString sorts a string's characters.
func sortString(s string) string {
	bytes := []byte(s)
	for i := 0; i < len(bytes)-1; i++ {
		for j := i + 1; j < len(bytes); j++ {
			if bytes[i] > bytes[j] {
				bytes[i], bytes[j] = bytes[j], bytes[i]
			}
		}
	}
	return string(bytes)
}

// colorCharToIndex converts a color character to index (R=0, G=1, B=2, W=3).
func colorCharToIndex(c byte) int {
	switch c {
	case 'R':
		return 0
	case 'G':
		return 1
	case 'B':
		return 2
	case 'W':
		return 3
	default:
		return 0
	}
}

// SetLED sets the color of a single LED.
func (np *NeoPixel) SetLED(index int, r, g, b, w float64) error {
	np.mu.Lock()
	defer np.mu.Unlock()

	if index < 0 || index >= np.chainCount {
		return fmt.Errorf("neopixel: LED index %d out of range", index)
	}

	colors := [4]float64{r, g, b, w}

	for cdidx, cm := range np.colorMap {
		if cm.ledIdx == index {
			np.colorData[cdidx] = byte(colors[cm.colorIdx]*255.0 + 0.5)
		}
	}

	return nil
}

// SetAllLEDs sets all LEDs to the same color.
func (np *NeoPixel) SetAllLEDs(r, g, b, w float64) {
	np.mu.Lock()
	defer np.mu.Unlock()

	colors := [4]float64{r, g, b, w}

	for cdidx, cm := range np.colorMap {
		np.colorData[cdidx] = byte(colors[cm.colorIdx]*255.0 + 0.5)
	}
}

// SendData sends color data to the LEDs.
func (np *NeoPixel) SendData(printTime float64) error {
	np.mu.Lock()
	defer np.mu.Unlock()

	// Check if data changed
	changed := false
	for i, b := range np.colorData {
		if b != np.oldColorData[i] {
			changed = true
			break
		}
	}

	if !changed {
		return nil
	}

	// Copy new data to old
	copy(np.oldColorData, np.colorData)

	// Note: Actual MCU command sending would happen here
	log.Printf("neopixel '%s': updated LED data", np.name)
	return nil
}

// GetLED returns the current color of a LED.
func (np *NeoPixel) GetLED(index int) (r, g, b, w float64, err error) {
	np.mu.Lock()
	defer np.mu.Unlock()

	if index < 0 || index >= np.chainCount {
		return 0, 0, 0, 0, fmt.Errorf("neopixel: LED index %d out of range", index)
	}

	colors := [4]float64{}
	for cdidx, cm := range np.colorMap {
		if cm.ledIdx == index {
			colors[cm.colorIdx] = float64(np.colorData[cdidx]) / 255.0
		}
	}

	return colors[0], colors[1], colors[2], colors[3], nil
}

// GetStatus returns the NeoPixel status.
func (np *NeoPixel) GetStatus() map[string]any {
	np.mu.Lock()
	defer np.mu.Unlock()

	// Build color data as array of RGBW values
	colorData := make([][]float64, np.chainCount)
	for i := range colorData {
		colorData[i] = []float64{0, 0, 0, 0}
	}

	for cdidx, cm := range np.colorMap {
		colorData[cm.ledIdx][cm.colorIdx] = float64(np.colorData[cdidx]) / 255.0
	}

	return map[string]any{
		"color_data": colorData,
	}
}

// GetName returns the NeoPixel name.
func (np *NeoPixel) GetName() string {
	return np.name
}

// GetChainCount returns the number of LEDs in the chain.
func (np *NeoPixel) GetChainCount() int {
	return np.chainCount
}
