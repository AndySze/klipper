// StaticPWMClock - port of klippy/extras/static_pwm_clock.py
//
// Define GPIO as clock output using PWM
//
// Copyright (C) 2025 Timofey Titovets <nefelim4ag@gmail.com>
// Copyright (C) 2026 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// StaticPWMClock outputs a fixed frequency clock signal on a GPIO pin using PWM.
// The PWM is configured with 50% duty cycle to produce a clock signal.
type StaticPWMClock struct {
	name      string
	pin       string
	frequency float64
	mcuName   string
	mu        sync.Mutex
}

// StaticPWMClockConfig holds configuration for static PWM clock output
type StaticPWMClockConfig struct {
	Name      string
	Pin       string
	Frequency float64 // Hz, must be > 3.33 and <= 520MHz
	MCUName   string  // MCU name (default "mcu")
}

// NewStaticPWMClock creates a new static PWM clock output
func NewStaticPWMClock(cfg StaticPWMClockConfig) (*StaticPWMClock, error) {
	if cfg.Pin == "" {
		return nil, fmt.Errorf("static_pwm_clock: pin must be specified")
	}

	// Validate frequency range (1/0.3 = ~3.33 Hz minimum, 520MHz maximum)
	minFreq := 1.0 / 0.3
	maxFreq := 520000000.0
	if cfg.Frequency <= minFreq || cfg.Frequency > maxFreq {
		return nil, fmt.Errorf("static_pwm_clock: frequency must be between %.2f and %.0f Hz",
			minFreq, maxFreq)
	}

	mcuName := cfg.MCUName
	if mcuName == "" {
		mcuName = "mcu"
	}

	clk := &StaticPWMClock{
		name:      cfg.Name,
		pin:       cfg.Pin,
		frequency: cfg.Frequency,
		mcuName:   mcuName,
	}

	log.Printf("static_pwm_clock: initialized %s pin=%s frequency=%.2f Hz mcu=%s",
		cfg.Name, cfg.Pin, cfg.Frequency, mcuName)

	return clk, nil
}

// ValidateFrequency validates that the frequency can be achieved with the given MCU frequency
func (clk *StaticPWMClock) ValidateFrequency(mcuFreq float64) error {
	clk.mu.Lock()
	defer clk.mu.Unlock()

	cycleTicks := int(mcuFreq / clk.frequency)
	// Validate frequency produces integer cycle ticks
	mcuFreqRev := cycleTicks * int(clk.frequency)
	if mcuFreqRev != int(mcuFreq) {
		return fmt.Errorf(
			`static_pwm_clock: frequency output must be without remainder, %d != %d
[%s]
frequency = %f`, int(mcuFreq), mcuFreqRev, clk.name, clk.frequency)
	}

	// Check pulse width is within acceptable range (40-60%)
	value := int(0.5 * float64(cycleTicks))
	pulseWidth := float64(value) / float64(cycleTicks)
	if pulseWidth < 0.4 {
		log.Printf("static_pwm_clock: [%s] pulse width < 40%%", clk.name)
	}
	if pulseWidth > 0.6 {
		log.Printf("static_pwm_clock: [%s] pulse width > 60%%", clk.name)
	}

	return nil
}

// GetCycleTime returns the cycle time in seconds
func (clk *StaticPWMClock) GetCycleTime() float64 {
	return 1.0 / clk.frequency
}

// GetDutyCycle returns the duty cycle (always 0.5 for clock output)
func (clk *StaticPWMClock) GetDutyCycle() float64 {
	return 0.5
}

// GetName returns the clock name
func (clk *StaticPWMClock) GetName() string {
	return clk.name
}

// GetPin returns the pin name
func (clk *StaticPWMClock) GetPin() string {
	return clk.pin
}

// GetFrequency returns the frequency in Hz
func (clk *StaticPWMClock) GetFrequency() float64 {
	return clk.frequency
}

// GetMCUName returns the MCU name
func (clk *StaticPWMClock) GetMCUName() string {
	return clk.mcuName
}

// GetStatus returns the clock status
func (clk *StaticPWMClock) GetStatus() map[string]interface{} {
	return map[string]interface{}{
		"name":       clk.name,
		"pin":        clk.pin,
		"frequency":  clk.frequency,
		"cycle_time": clk.GetCycleTime(),
		"duty_cycle": clk.GetDutyCycle(),
		"mcu_name":   clk.mcuName,
	}
}
