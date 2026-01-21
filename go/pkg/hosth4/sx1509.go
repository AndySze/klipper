// SX1509 - port of klippy/extras/sx1509.py
//
// Support for SX1509 I2C GPIO expander with PWM capability
//
// Copyright (C) 2018 Florian Heilmann <Florian.Heilmann@gmx.net>
// Copyright (C) 2026 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// SX1509 register addresses (word registers)
const (
	sx1509RegReset              = 0x7D
	sx1509RegClock              = 0x1E
	sx1509RegMisc               = 0x1F
	sx1509RegDir                = 0x0E
	sx1509RegData               = 0x10
	sx1509RegPullup             = 0x06
	sx1509RegPulldown           = 0x08
	sx1509RegInputDisable       = 0x00
	sx1509RegAnalogDriverEnable = 0x20
)

// SX1509 I_ON registers (byte registers) for each of the 16 pins
var sx1509RegIOn = []int{
	0x2A, 0x2D, 0x30, 0x33, 0x36, 0x3B, 0x40, 0x45,
	0x4A, 0x4D, 0x50, 0x53, 0x56, 0x5B, 0x5F, 0x65,
}

// SX1509 represents an SX1509 I2C GPIO expander with 16 digital/PWM pins.
// The device supports both digital output and PWM modes on each pin.
type SX1509 struct {
	name    string
	i2cAddr int
	i2cBus  string
	mcuName string
	oid     int

	// Register cache
	regDict    map[int]uint16
	regIOnDict map[int]uint8

	mu sync.Mutex
}

// SX1509Config holds configuration for SX1509
type SX1509Config struct {
	Name    string
	I2CAddr int    // Default 0x3E
	I2CBus  string // I2C bus name
	MCUName string // MCU name (default "mcu")
}

// DefaultSX1509Config returns default configuration
func DefaultSX1509Config() SX1509Config {
	return SX1509Config{
		I2CAddr: 0x3E,
		MCUName: "mcu",
	}
}

// NewSX1509 creates a new SX1509 GPIO expander
func NewSX1509(cfg SX1509Config) *SX1509 {
	if cfg.MCUName == "" {
		cfg.MCUName = "mcu"
	}
	if cfg.I2CAddr == 0 {
		cfg.I2CAddr = 0x3E
	}

	sx := &SX1509{
		name:    cfg.Name,
		i2cAddr: cfg.I2CAddr,
		i2cBus:  cfg.I2CBus,
		mcuName: cfg.MCUName,
		regDict: map[int]uint16{
			sx1509RegDir:                0xFFFF, // All pins as inputs
			sx1509RegData:               0,
			sx1509RegPullup:             0,
			sx1509RegPulldown:           0,
			sx1509RegInputDisable:       0,
			sx1509RegAnalogDriverEnable: 0,
		},
		regIOnDict: make(map[int]uint8),
	}

	// Initialize I_ON registers
	for _, reg := range sx1509RegIOn {
		sx.regIOnDict[reg] = 0
	}

	log.Printf("sx1509: initialized name=%s addr=0x%02x bus=%s mcu=%s",
		cfg.Name, cfg.I2CAddr, cfg.I2CBus, cfg.MCUName)

	return sx
}

// GetInitCommands returns the I2C commands to initialize the SX1509
func (sx *SX1509) GetInitCommands() [][]byte {
	sx.mu.Lock()
	defer sx.mu.Unlock()

	cmds := [][]byte{
		// Reset sequence
		{sx1509RegReset, 0x12},
		{sx1509RegReset, 0x34},
		// Enable oscillator
		{sx1509RegClock, 1 << 6},
		// Setup clock divider
		{sx1509RegMisc, 1 << 4},
	}

	// Add initial register values
	for reg, val := range sx.regDict {
		cmds = append(cmds, []byte{byte(reg), byte((val >> 8) & 0xFF), byte(val & 0xFF)})
	}

	for reg, val := range sx.regIOnDict {
		cmds = append(cmds, []byte{byte(reg), val})
	}

	return cmds
}

// GetI2CAddr returns the I2C address
func (sx *SX1509) GetI2CAddr() int {
	return sx.i2cAddr
}

// GetMCUName returns the MCU name
func (sx *SX1509) GetMCUName() string {
	return sx.mcuName
}

// GetOID returns the OID
func (sx *SX1509) GetOID() int {
	return sx.oid
}

// SetOID sets the OID
func (sx *SX1509) SetOID(oid int) {
	sx.oid = oid
}

// ClearBitsInRegister clears bits in a register
func (sx *SX1509) ClearBitsInRegister(reg int, bitmask uint16) {
	sx.mu.Lock()
	defer sx.mu.Unlock()

	if val, ok := sx.regDict[reg]; ok {
		sx.regDict[reg] = val & ^bitmask
	} else if _, ok := sx.regIOnDict[reg]; ok {
		sx.regIOnDict[reg] &= ^uint8(bitmask)
	}
}

// SetBitsInRegister sets bits in a register
func (sx *SX1509) SetBitsInRegister(reg int, bitmask uint16) {
	sx.mu.Lock()
	defer sx.mu.Unlock()

	if val, ok := sx.regDict[reg]; ok {
		sx.regDict[reg] = val | bitmask
	} else if _, ok := sx.regIOnDict[reg]; ok {
		sx.regIOnDict[reg] |= uint8(bitmask)
	}
}

// SetRegister sets a register value
func (sx *SX1509) SetRegister(reg int, value uint16) {
	sx.mu.Lock()
	defer sx.mu.Unlock()

	if _, ok := sx.regDict[reg]; ok {
		sx.regDict[reg] = value
	} else if _, ok := sx.regIOnDict[reg]; ok {
		sx.regIOnDict[reg] = uint8(value)
	}
}

// GetRegisterValue returns the current register value
func (sx *SX1509) GetRegisterValue(reg int) (uint16, bool) {
	sx.mu.Lock()
	defer sx.mu.Unlock()

	if val, ok := sx.regDict[reg]; ok {
		return val, true
	}
	if val, ok := sx.regIOnDict[reg]; ok {
		return uint16(val), true
	}
	return 0, false
}

// EncodeRegisterWrite encodes a register write command
func (sx *SX1509) EncodeRegisterWrite(reg int) []byte {
	sx.mu.Lock()
	defer sx.mu.Unlock()

	data := []byte{byte(reg & 0xFF)}

	if val, ok := sx.regDict[reg]; ok {
		// Word register
		data = append(data, byte((val>>8)&0xFF), byte(val&0xFF))
	} else if val, ok := sx.regIOnDict[reg]; ok {
		// Byte register
		data = append(data, val)
	}

	return data
}

// GetStatus returns the SX1509 status
func (sx *SX1509) GetStatus() map[string]interface{} {
	sx.mu.Lock()
	defer sx.mu.Unlock()

	return map[string]interface{}{
		"name":     sx.name,
		"i2c_addr": fmt.Sprintf("0x%02x", sx.i2cAddr),
		"i2c_bus":  sx.i2cBus,
		"mcu_name": sx.mcuName,
	}
}

// SX1509DigitalOutConfig holds configuration for a digital output pin
type SX1509DigitalOutConfig struct {
	Pin    int
	Invert bool
}

// SX1509DigitalOut implements a digital output pin on SX1509
type SX1509DigitalOut struct {
	sx1509        *SX1509
	sxPin         int
	bitmask       uint16
	invert        bool
	startValue    bool
	shutdownValue bool
	maxDuration   float64
}

// NewSX1509DigitalOut creates a new SX1509 digital output
func NewSX1509DigitalOut(sx *SX1509, cfg SX1509DigitalOutConfig) *SX1509DigitalOut {
	do := &SX1509DigitalOut{
		sx1509:      sx,
		sxPin:       cfg.Pin,
		bitmask:     1 << cfg.Pin,
		invert:      cfg.Invert,
		startValue:  cfg.Invert,
		maxDuration: 2.0,
	}

	// Set direction to output (clear bit in DIR register)
	sx.ClearBitsInRegister(sx1509RegDir, do.bitmask)

	log.Printf("sx1509: digital_out pin=%d invert=%v", cfg.Pin, cfg.Invert)

	return do
}

// GetMCUName returns the MCU name
func (do *SX1509DigitalOut) GetMCUName() string {
	return do.sx1509.mcuName
}

// SetupMaxDuration sets the max duration
func (do *SX1509DigitalOut) SetupMaxDuration(maxDuration float64) {
	do.maxDuration = maxDuration
}

// SetupStartValue sets start and shutdown values
func (do *SX1509DigitalOut) SetupStartValue(startValue, shutdownValue bool) {
	do.startValue = startValue != do.invert
	do.shutdownValue = do.invert

	if do.startValue {
		do.sx1509.SetBitsInRegister(sx1509RegData, do.bitmask)
	} else {
		do.sx1509.ClearBitsInRegister(sx1509RegData, do.bitmask)
	}
}

// SetDigital returns the register write command for setting the digital output
func (do *SX1509DigitalOut) SetDigital(value bool) []byte {
	if value != do.invert {
		do.sx1509.SetBitsInRegister(sx1509RegData, do.bitmask)
	} else {
		do.sx1509.ClearBitsInRegister(sx1509RegData, do.bitmask)
	}
	return do.sx1509.EncodeRegisterWrite(sx1509RegData)
}

// SX1509PWMConfig holds configuration for a PWM output pin
type SX1509PWMConfig struct {
	Pin    int
	Invert bool
}

// SX1509PWM implements a PWM output pin on SX1509
type SX1509PWM struct {
	sx1509        *SX1509
	sxPin         int
	bitmask       uint16
	iOnReg        int
	invert        bool
	startValue    float64
	shutdownValue float64
	maxDuration   float64
	hardwarePWM   bool
	cycleTime     float64
}

// NewSX1509PWM creates a new SX1509 PWM output
func NewSX1509PWM(sx *SX1509, cfg SX1509PWMConfig) *SX1509PWM {
	pwm := &SX1509PWM{
		sx1509:      sx,
		sxPin:       cfg.Pin,
		bitmask:     1 << cfg.Pin,
		iOnReg:      sx1509RegIOn[cfg.Pin],
		invert:      cfg.Invert,
		maxDuration: 2.0,
	}

	if cfg.Invert {
		pwm.startValue = 1.0
		pwm.shutdownValue = 1.0
	}

	// Set required registers for PWM mode
	sx.SetBitsInRegister(sx1509RegInputDisable, pwm.bitmask)
	sx.ClearBitsInRegister(sx1509RegPullup, pwm.bitmask)
	sx.ClearBitsInRegister(sx1509RegDir, pwm.bitmask)
	sx.SetBitsInRegister(sx1509RegAnalogDriverEnable, pwm.bitmask)
	sx.ClearBitsInRegister(sx1509RegData, pwm.bitmask)

	log.Printf("sx1509: pwm pin=%d invert=%v", cfg.Pin, cfg.Invert)

	return pwm
}

// GetMCUName returns the MCU name
func (pwm *SX1509PWM) GetMCUName() string {
	return pwm.sx1509.mcuName
}

// SetupMaxDuration sets the max duration
func (pwm *SX1509PWM) SetupMaxDuration(maxDuration float64) {
	pwm.maxDuration = maxDuration
}

// SetupCycleTime sets the PWM cycle time
func (pwm *SX1509PWM) SetupCycleTime(cycleTime float64, hardwarePWM bool) {
	pwm.cycleTime = cycleTime
	pwm.hardwarePWM = hardwarePWM
}

// SetupStartValue sets start and shutdown values
func (pwm *SX1509PWM) SetupStartValue(startValue, shutdownValue float64) {
	if pwm.invert {
		startValue = 1.0 - startValue
		shutdownValue = 1.0 - shutdownValue
	}
	pwm.startValue = sx1509ClampFloat(startValue, 0, 1)
	pwm.shutdownValue = sx1509ClampFloat(shutdownValue, 0, 1)

	// Set I_ON register (inverted: 255 = off, 0 = full on)
	pwm.sx1509.SetRegister(pwm.iOnReg, uint16(^uint8(255*pwm.startValue)&0xFF))
}

// SetPWM returns the register write command for setting the PWM value
func (pwm *SX1509PWM) SetPWM(value float64) []byte {
	var regVal uint8
	if !pwm.invert {
		regVal = ^uint8(255*value) & 0xFF
	} else {
		regVal = uint8(255*value) & 0xFF
	}
	pwm.sx1509.SetRegister(pwm.iOnReg, uint16(regVal))
	return pwm.sx1509.EncodeRegisterWrite(pwm.iOnReg)
}

// sx1509ClampFloat clamps a float value between min and max
func sx1509ClampFloat(v, min, max float64) float64 {
	if v < min {
		return min
	}
	if v > max {
		return max
	}
	return v
}
