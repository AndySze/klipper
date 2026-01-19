// PCA9632 - port of klippy/extras/pca9632.py
//
// Support for PCA9632 LED driver
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

// PCA9632 registers and constants.
const (
	pca9632I2CAddr       = 0x62
	pca9632RegMode1      = 0x00
	pca9632RegMode2      = 0x01
	pca9632RegPwm0       = 0x02
	pca9632RegPwm1       = 0x03
	pca9632RegPwm2       = 0x04
	pca9632RegPwm3       = 0x05
	pca9632RegGrpPwm     = 0x06
	pca9632RegGrpFreq    = 0x07
	pca9632RegLedout     = 0x08
	pca9632AutoIncrement = 0x80
)

// LED output modes.
const (
	pca9632LedOff      = 0 // LED driver off
	pca9632LedOn       = 1 // LED driver fully on
	pca9632LedPwm      = 2 // LED brightness controlled by PWM
	pca9632LedPwmGroup = 3 // LED brightness controlled by PWM and group
)

// PCA9632 represents a PCA9632 LED driver.
type PCA9632 struct {
	rt          *runtime
	name        string
	i2cAddr     int
	ledModes    [4]int
	pwm         [4]byte
	grpPwm      byte
	mu          sync.Mutex
}

// PCA9632Config holds configuration for PCA9632.
type PCA9632Config struct {
	Name    string
	I2CAddr int
}

// DefaultPCA9632Config returns default PCA9632 configuration.
func DefaultPCA9632Config() PCA9632Config {
	return PCA9632Config{
		I2CAddr: pca9632I2CAddr,
	}
}

// newPCA9632 creates a new PCA9632 LED driver.
func newPCA9632(rt *runtime, cfg PCA9632Config) (*PCA9632, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("pca9632: name is required")
	}
	if cfg.I2CAddr <= 0 {
		cfg.I2CAddr = pca9632I2CAddr
	}

	pca := &PCA9632{
		rt:      rt,
		name:    cfg.Name,
		i2cAddr: cfg.I2CAddr,
	}

	log.Printf("pca9632: initialized '%s' addr=0x%02x", cfg.Name, cfg.I2CAddr)
	return pca, nil
}

// GetName returns the LED driver name.
func (pca *PCA9632) GetName() string {
	return pca.name
}

// SetLEDMode sets the output mode for an LED.
func (pca *PCA9632) SetLEDMode(led int, mode int) error {
	pca.mu.Lock()
	defer pca.mu.Unlock()

	if led < 0 || led > 3 {
		return fmt.Errorf("pca9632: LED index %d out of range (0-3)", led)
	}
	if mode < 0 || mode > 3 {
		return fmt.Errorf("pca9632: invalid mode %d (0-3)", mode)
	}

	pca.ledModes[led] = mode
	return pca.updateLocked()
}

// SetLEDPWM sets the PWM duty cycle for an LED (0-255).
func (pca *PCA9632) SetLEDPWM(led int, duty byte) error {
	pca.mu.Lock()
	defer pca.mu.Unlock()

	if led < 0 || led > 3 {
		return fmt.Errorf("pca9632: LED index %d out of range (0-3)", led)
	}

	pca.pwm[led] = duty
	pca.ledModes[led] = pca9632LedPwm
	return pca.updateLocked()
}

// SetColor sets RGB color using LEDs 0-2.
func (pca *PCA9632) SetColor(r, g, b float64) error {
	pca.mu.Lock()
	defer pca.mu.Unlock()

	pca.pwm[0] = byte(r*255.0 + 0.5)
	pca.pwm[1] = byte(g*255.0 + 0.5)
	pca.pwm[2] = byte(b*255.0 + 0.5)

	pca.ledModes[0] = pca9632LedPwm
	pca.ledModes[1] = pca9632LedPwm
	pca.ledModes[2] = pca9632LedPwm

	return pca.updateLocked()
}

// SetRGBW sets RGBW color using all 4 LEDs.
func (pca *PCA9632) SetRGBW(r, g, b, w float64) error {
	pca.mu.Lock()
	defer pca.mu.Unlock()

	pca.pwm[0] = byte(r*255.0 + 0.5)
	pca.pwm[1] = byte(g*255.0 + 0.5)
	pca.pwm[2] = byte(b*255.0 + 0.5)
	pca.pwm[3] = byte(w*255.0 + 0.5)

	for i := 0; i < 4; i++ {
		pca.ledModes[i] = pca9632LedPwm
	}

	return pca.updateLocked()
}

// SetGroupPWM sets the group PWM duty cycle (0-255).
func (pca *PCA9632) SetGroupPWM(duty byte) error {
	pca.mu.Lock()
	defer pca.mu.Unlock()

	pca.grpPwm = duty
	return pca.updateLocked()
}

// updateLocked updates the LED driver registers (must be called with lock held).
func (pca *PCA9632) updateLocked() error {
	// Build LEDOUT register value
	ledout := byte(0)
	for i := 0; i < 4; i++ {
		ledout |= byte(pca.ledModes[i]) << (i * 2)
	}

	// In a real implementation, this would write to the I2C device
	log.Printf("pca9632 '%s': LEDOUT=0x%02x PWM=[%d,%d,%d,%d] GRPPWM=%d",
		pca.name, ledout, pca.pwm[0], pca.pwm[1], pca.pwm[2], pca.pwm[3], pca.grpPwm)
	return nil
}

// GetStatus returns the LED driver status.
func (pca *PCA9632) GetStatus() map[string]any {
	pca.mu.Lock()
	defer pca.mu.Unlock()

	return map[string]any{
		"led_modes": pca.ledModes,
		"pwm":       pca.pwm,
		"grp_pwm":   pca.grpPwm,
	}
}
