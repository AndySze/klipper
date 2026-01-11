// led implements LED control for 3D printers.
// Supports addressable LEDs (Neopixel, dotstar) and simple PWM LEDs.
package hosth4

import (
	"fmt"
	"strconv"
	"strings"
)

// ledColor represents an RGBW color value.
type ledColor struct {
	R float64 // Red (0.0-1.0)
	G float64 // Green (0.0-1.0)
	B float64 // Blue (0.0-1.0)
	W float64 // White (0.0-1.0)
}

// led represents an LED or LED strip.
type led struct {
	rt          *runtime
	name        string
	ledType     string // "neopixel", "dotstar", "pca9533", "pca9632", "led"
	pin         string
	chainCount  int        // Number of LEDs in chain
	colorOrder  string     // e.g., "RGB", "GRB", "RGBW"
	initialRed  float64
	initialGreen float64
	initialBlue float64
	initialWhite float64
	currentColors []ledColor
}

// newLED creates a new LED instance from configuration.
func newLED(rt *runtime, cfg *config, sectionName string) (*led, error) {
	sec, ok := cfg.section(sectionName)
	if !ok {
		return nil, nil
	}

	// Determine LED type from section name
	ledType := "led"
	if strings.HasPrefix(sectionName, "neopixel ") {
		ledType = "neopixel"
	} else if strings.HasPrefix(sectionName, "dotstar ") {
		ledType = "dotstar"
	} else if strings.HasPrefix(sectionName, "pca9533 ") {
		ledType = "pca9533"
	} else if strings.HasPrefix(sectionName, "pca9632 ") {
		ledType = "pca9632"
	}

	// Parse pin (required)
	pin, ok := sec["pin"]
	if !ok {
		return nil, fmt.Errorf("%s: missing pin", sectionName)
	}

	// Parse chain_count (for addressable LEDs)
	chainCount := 1
	if s, ok := sec["chain_count"]; ok {
		var err error
		chainCount, err = strconv.Atoi(s)
		if err != nil {
			return nil, fmt.Errorf("%s: invalid chain_count: %s", sectionName, s)
		}
		if chainCount < 1 {
			return nil, fmt.Errorf("%s: chain_count must be at least 1", sectionName)
		}
	}

	// Parse color_order (for addressable LEDs)
	colorOrder := "GRB" // Default for Neopixel
	if s, ok := sec["color_order"]; ok {
		colorOrder = strings.ToUpper(s)
	}

	// Parse initial colors
	initialRed := 0.0
	if s, ok := sec["initial_red"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &initialRed); err != nil {
			return nil, fmt.Errorf("%s: invalid initial_red: %s", sectionName, s)
		}
	}

	initialGreen := 0.0
	if s, ok := sec["initial_green"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &initialGreen); err != nil {
			return nil, fmt.Errorf("%s: invalid initial_green: %s", sectionName, s)
		}
	}

	initialBlue := 0.0
	if s, ok := sec["initial_blue"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &initialBlue); err != nil {
			return nil, fmt.Errorf("%s: invalid initial_blue: %s", sectionName, s)
		}
	}

	initialWhite := 0.0
	if s, ok := sec["initial_white"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &initialWhite); err != nil {
			return nil, fmt.Errorf("%s: invalid initial_white: %s", sectionName, s)
		}
	}

	// Initialize colors
	colors := make([]ledColor, chainCount)
	for i := range colors {
		colors[i] = ledColor{
			R: initialRed,
			G: initialGreen,
			B: initialBlue,
			W: initialWhite,
		}
	}

	return &led{
		rt:            rt,
		name:          sectionName,
		ledType:       ledType,
		pin:           pin,
		chainCount:    chainCount,
		colorOrder:    colorOrder,
		initialRed:    initialRed,
		initialGreen:  initialGreen,
		initialBlue:   initialBlue,
		initialWhite:  initialWhite,
		currentColors: colors,
	}, nil
}

// setColor sets the color of a specific LED or all LEDs.
func (l *led) setColor(index int, color ledColor, transmit bool) error {
	if index < 0 {
		// Set all LEDs
		for i := range l.currentColors {
			l.currentColors[i] = color
		}
	} else if index < len(l.currentColors) {
		l.currentColors[index] = color
	} else {
		return fmt.Errorf("LED index %d out of range (0-%d)", index, len(l.currentColors)-1)
	}

	if transmit {
		return l.transmit()
	}
	return nil
}

// transmit sends the current colors to the LED hardware.
func (l *led) transmit() error {
	// In real implementation, would send data to MCU
	l.rt.tracef("LED %s: transmitting %d colors\n", l.name, len(l.currentColors))
	return nil
}

// getStatus returns the current status of the LED.
func (l *led) getStatus() map[string]any {
	colors := make([]map[string]float64, len(l.currentColors))
	for i, c := range l.currentColors {
		colors[i] = map[string]float64{
			"r": c.R,
			"g": c.G,
			"b": c.B,
			"w": c.W,
		}
	}
	return map[string]any{
		"color_data": colors,
	}
}

// ledManager manages all LEDs in the system.
type ledManager struct {
	rt   *runtime
	leds map[string]*led
}

// newLEDManager creates a LED manager from configuration.
func newLEDManager(rt *runtime, cfg *config) (*ledManager, error) {
	lm := &ledManager{
		rt:   rt,
		leds: make(map[string]*led),
	}

	// Load [neopixel xxx] sections
	for _, name := range cfg.sectionsByPrefix("neopixel ") {
		l, err := newLED(rt, cfg, name)
		if err != nil {
			return nil, err
		}
		if l != nil {
			ledName := name[len("neopixel "):]
			lm.leds[ledName] = l
		}
	}

	// Load [dotstar xxx] sections
	for _, name := range cfg.sectionsByPrefix("dotstar ") {
		l, err := newLED(rt, cfg, name)
		if err != nil {
			return nil, err
		}
		if l != nil {
			ledName := name[len("dotstar "):]
			lm.leds[ledName] = l
		}
	}

	// Load [led xxx] sections
	for _, name := range cfg.sectionsByPrefix("led ") {
		l, err := newLED(rt, cfg, name)
		if err != nil {
			return nil, err
		}
		if l != nil {
			ledName := name[len("led "):]
			lm.leds[ledName] = l
		}
	}

	return lm, nil
}

// cmdSetLED handles the SET_LED command.
func (lm *ledManager) cmdSetLED(args map[string]string) error {
	// Get LED name
	ledName, ok := args["LED"]
	if !ok {
		return fmt.Errorf("missing LED parameter")
	}

	// Find the LED
	l, ok := lm.leds[ledName]
	if !ok {
		return fmt.Errorf("unknown LED: %s", ledName)
	}

	// Parse color values
	color := ledColor{}
	if s, ok := args["RED"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &color.R); err != nil {
			return fmt.Errorf("invalid RED parameter: %s", s)
		}
	}
	if s, ok := args["GREEN"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &color.G); err != nil {
			return fmt.Errorf("invalid GREEN parameter: %s", s)
		}
	}
	if s, ok := args["BLUE"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &color.B); err != nil {
			return fmt.Errorf("invalid BLUE parameter: %s", s)
		}
	}
	if s, ok := args["WHITE"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &color.W); err != nil {
			return fmt.Errorf("invalid WHITE parameter: %s", s)
		}
	}

	// Get index (default -1 means all LEDs)
	index := -1
	if s, ok := args["INDEX"]; ok {
		var err error
		index, err = strconv.Atoi(s)
		if err != nil {
			return fmt.Errorf("invalid INDEX parameter: %s", s)
		}
		index-- // Convert from 1-based to 0-based
	}

	// Get transmit flag (default true)
	transmit := true
	if s, ok := args["TRANSMIT"]; ok {
		transmit = s == "1" || s == "true" || s == "True"
	}

	return l.setColor(index, color, transmit)
}