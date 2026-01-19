// PCA9533 - port of klippy/extras/pca9533.py
//
// Support for PCA9533 LED driver
//
// Copyright (C) 2021 Jared Smith <jaredsmith4@pm.me>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// PCA9533 registers and constants.
const (
	pca9533I2CAddr      = 0x62
	pca9533RegInput     = 0x00
	pca9533RegPsc0      = 0x01
	pca9533RegPwm0      = 0x02
	pca9533RegPsc1      = 0x03
	pca9533RegPwm1      = 0x04
	pca9533RegLs0       = 0x05
	pca9533AutoIncrement = 0x10
)

// LED states.
const (
	pca9533LedOff  = 0
	pca9533LedOn   = 1
	pca9533LedPwm0 = 2
	pca9533LedPwm1 = 3
)

// PCA9533 represents a PCA9533 LED driver.
type PCA9533 struct {
	rt          *runtime
	name        string
	i2cAddr     int
	ledStates   [4]int
	pwm0        byte
	pwm1        byte
	mu          sync.Mutex
}

// PCA9533Config holds configuration for PCA9533.
type PCA9533Config struct {
	Name    string
	I2CAddr int
}

// DefaultPCA9533Config returns default PCA9533 configuration.
func DefaultPCA9533Config() PCA9533Config {
	return PCA9533Config{
		I2CAddr: pca9533I2CAddr,
	}
}

// newPCA9533 creates a new PCA9533 LED driver.
func newPCA9533(rt *runtime, cfg PCA9533Config) (*PCA9533, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("pca9533: name is required")
	}
	if cfg.I2CAddr <= 0 {
		cfg.I2CAddr = pca9533I2CAddr
	}

	pca := &PCA9533{
		rt:      rt,
		name:    cfg.Name,
		i2cAddr: cfg.I2CAddr,
		pwm0:    0,
		pwm1:    0,
	}

	log.Printf("pca9533: initialized '%s' addr=0x%02x", cfg.Name, cfg.I2CAddr)
	return pca, nil
}

// GetName returns the LED driver name.
func (pca *PCA9533) GetName() string {
	return pca.name
}

// SetLED sets the state of an LED.
func (pca *PCA9533) SetLED(led int, state int) error {
	pca.mu.Lock()
	defer pca.mu.Unlock()

	if led < 0 || led > 3 {
		return fmt.Errorf("pca9533: LED index %d out of range (0-3)", led)
	}
	if state < 0 || state > 3 {
		return fmt.Errorf("pca9533: invalid state %d (0-3)", state)
	}

	pca.ledStates[led] = state
	return pca.updateLocked()
}

// SetLEDOn turns an LED on.
func (pca *PCA9533) SetLEDOn(led int) error {
	return pca.SetLED(led, pca9533LedOn)
}

// SetLEDOff turns an LED off.
func (pca *PCA9533) SetLEDOff(led int) error {
	return pca.SetLED(led, pca9533LedOff)
}

// SetLEDPWM sets an LED to PWM mode.
func (pca *PCA9533) SetLEDPWM(led int, pwmChannel int) error {
	if pwmChannel == 0 {
		return pca.SetLED(led, pca9533LedPwm0)
	}
	return pca.SetLED(led, pca9533LedPwm1)
}

// SetPWM0 sets the PWM0 duty cycle (0-255).
func (pca *PCA9533) SetPWM0(duty byte) error {
	pca.mu.Lock()
	defer pca.mu.Unlock()

	pca.pwm0 = duty
	return pca.updateLocked()
}

// SetPWM1 sets the PWM1 duty cycle (0-255).
func (pca *PCA9533) SetPWM1(duty byte) error {
	pca.mu.Lock()
	defer pca.mu.Unlock()

	pca.pwm1 = duty
	return pca.updateLocked()
}

// updateLocked updates the LED driver registers (must be called with lock held).
func (pca *PCA9533) updateLocked() error {
	// Build LS0 register value
	ls0 := byte(0)
	for i := 0; i < 4; i++ {
		ls0 |= byte(pca.ledStates[i]) << (i * 2)
	}

	// In a real implementation, this would write to the I2C device
	log.Printf("pca9533 '%s': LS0=0x%02x PWM0=%d PWM1=%d",
		pca.name, ls0, pca.pwm0, pca.pwm1)
	return nil
}

// GetStatus returns the LED driver status.
func (pca *PCA9533) GetStatus() map[string]any {
	pca.mu.Lock()
	defer pca.mu.Unlock()

	return map[string]any{
		"led_states": pca.ledStates,
		"pwm0":       pca.pwm0,
		"pwm1":       pca.pwm1,
	}
}
