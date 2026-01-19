// TMC2660 - port of klippy/extras/tmc2660.py
//
// TMC2660 stepper driver configuration
//
// Copyright (C) 2018-2019 Florian Heilmann <Florian.Heilmann@gmx.net>
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
	tmc2660Frequency = 15000000.0
)

// TMC2660 register addresses (20-bit registers, addressed via top 4 bits).
var tmc2660Registers = map[string]uint8{
	"DRVCONF": 0xE,
	"SGCSCONF": 0xC,
	"SMARTEN": 0xA,
	"CHOPCONF": 0x8,
	"DRVCTRL": 0x0, // DRVCTRL has address 0 (no address bits set)
}

// TMC2660 register field definitions.
var tmc2660Fields = map[string]map[string]uint32{
	"DRVCONF": {
		"tst":   0x01 << 0,
		"slph":  0x03 << 1,
		"slpl":  0x03 << 3,
		"diss2g": 0x01 << 5,
		"ts2g":  0x03 << 6,
		"sdoff": 0x01 << 8,
		"vsense": 0x01 << 9,
		"rdsel": 0x03 << 10,
	},
	"SGCSCONF": {
		"cs":      0x1F << 0,
		"sgt":     0x7F << 8,
		"sfilt":   0x01 << 15,
	},
	"SMARTEN": {
		"semin":  0x0F << 0,
		"seup":   0x03 << 5,
		"semax":  0x0F << 8,
		"sedn":   0x03 << 13,
		"seimin": 0x01 << 15,
	},
	"CHOPCONF": {
		"toff":   0x0F << 0,
		"hstrt":  0x07 << 4,
		"hend":   0x0F << 7,
		"hdec":   0x03 << 11,
		"rndtf":  0x01 << 13,
		"chm":    0x01 << 14,
		"tbl":    0x03 << 15,
	},
	"DRVCTRL": {
		"mres":   0x0F << 0,
		"dedge":  0x01 << 8,
		"intpol": 0x01 << 9,
	},
	"READRSP_STALLGUARD": {
		"sg_result": 0x3FF << 10,
		"se":        0x1F << 5,
		"stst":      0x01 << 7,
		"ola":       0x01 << 6,
		"olb":       0x01 << 5,
		"s2ga":      0x01 << 4,
		"s2gb":      0x01 << 3,
		"otpw":      0x01 << 2,
		"ot":        0x01 << 1,
		"sg":        0x01 << 0,
	},
}

// TMC2660 represents a TMC2660 stepper driver.
type TMC2660 struct {
	*TMCBase
	spiMode   int
	spiSpeed  int
	sgthrs    int
	mu        sync.RWMutex
}

// TMC2660Config holds TMC2660 configuration.
type TMC2660Config struct {
	TMCSPIConfig
	StallGuardThreshold int
}

// DefaultTMC2660Config returns default TMC2660 configuration.
func DefaultTMC2660Config() TMC2660Config {
	return TMC2660Config{
		TMCSPIConfig: TMCSPIConfig{
			TMCConfig: DefaultTMCConfig(),
			SPIMode:   3,
			SPISpeed:  4000000,
		},
		StallGuardThreshold: 0,
	}
}

// newTMC2660 creates a new TMC2660 driver.
func newTMC2660(rt *runtime, cfg TMC2660Config) (*TMC2660, error) {
	fields := newTMCFieldHelper(tmc2660Fields, []string{"sgt"})
	currentHelper := NewTMCCurrentHelper(cfg.RunCurrent, cfg.HoldCurrent, cfg.SenseResistor)

	base := NewTMCBase(rt, cfg.Name, fields, currentHelper, cfg.Microsteps)

	tmc := &TMC2660{
		TMCBase:  base,
		spiMode:  cfg.SPIMode,
		spiSpeed: cfg.SPISpeed,
		sgthrs:   cfg.StallGuardThreshold,
	}

	// DRVCONF defaults
	fields.SetField("rdsel", 0) // Read MSTEP
	fields.SetField("vsense", 0) // Low sensitivity

	// CHOPCONF defaults
	fields.SetField("toff", 4)
	fields.SetField("hstrt", 4)
	fields.SetField("hend", 0)
	fields.SetField("hdec", 0)
	fields.SetField("tbl", 2)
	fields.SetField("chm", 0) // SpreadCycle

	// DRVCTRL defaults
	mres, err := tmc2660MicrostepsToMRES(cfg.Microsteps)
	if err != nil {
		return nil, err
	}
	fields.SetField("mres", mres)

	// Interpolation
	if cfg.Interpolate {
		fields.SetField("intpol", 1)
	}

	// SGCSCONF - Current scale and StallGuard
	cs := tmc.calcCurrentScale(cfg.RunCurrent, cfg.SenseResistor)
	fields.SetField("cs", cs)
	fields.SetField("sgt", uint32(cfg.StallGuardThreshold&0x7F))
	fields.SetField("sfilt", 0)

	// SMARTEN defaults (CoolStep disabled)
	fields.SetField("semin", 0)
	fields.SetField("seup", 0)
	fields.SetField("semax", 0)
	fields.SetField("sedn", 0)
	fields.SetField("seimin", 0)

	logTMCInit("TMC2660", cfg.Name, cfg.Microsteps, cfg.RunCurrent)

	return tmc, nil
}

// tmc2660MicrostepsToMRES converts microsteps to TMC2660 MRES value.
// TMC2660 uses different encoding than other TMC chips.
func tmc2660MicrostepsToMRES(microsteps int) (uint32, error) {
	switch microsteps {
	case 256:
		return 0, nil
	case 128:
		return 1, nil
	case 64:
		return 2, nil
	case 32:
		return 3, nil
	case 16:
		return 4, nil
	case 8:
		return 5, nil
	case 4:
		return 6, nil
	case 2:
		return 7, nil
	case 1:
		return 8, nil
	default:
		return 0, fmt.Errorf("invalid microsteps: %d", microsteps)
	}
}

// calcCurrentScale calculates the CS (current scale) value.
func (t *TMC2660) calcCurrentScale(current, senseResistor float64) uint32 {
	// TMC2660 current formula:
	// I_RMS = (CS+1)/32 * V_fs / (R_sense * sqrt(2))
	// where V_fs = 0.165V (vsense=1) or 0.310V (vsense=0)

	// Use low sensitivity by default (vsense=0)
	vfs := 0.310
	cs := int(32.0*current*senseResistor*1.41421356/vfs) - 1

	if cs < 0 {
		cs = 0
	}
	if cs > 31 {
		cs = 31
	}

	return uint32(cs)
}

// GetStatus returns the driver status.
func (t *TMC2660) GetStatus() map[string]any {
	status := t.TMCBase.GetStatus()
	status["driver_type"] = "TMC2660"
	status["spi_mode"] = t.spiMode
	status["spi_speed"] = t.spiSpeed
	status["stallguard_threshold"] = t.sgthrs
	return status
}

// SetStallGuardThreshold sets the StallGuard threshold.
func (t *TMC2660) SetStallGuardThreshold(threshold int) {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.sgthrs = threshold
	t.fields.SetField("sgt", uint32(threshold&0x7F))
	log.Printf("tmc2660 '%s': set StallGuard threshold to %d", t.name, threshold)
}

// SetCurrentScale sets the current scale directly.
func (t *TMC2660) SetCurrentScale(cs uint32) {
	t.mu.Lock()
	defer t.mu.Unlock()
	if cs > 31 {
		cs = 31
	}
	t.fields.SetField("cs", cs)
	log.Printf("tmc2660 '%s': set current scale to %d", t.name, cs)
}

// SetChopperMode sets the chopper mode (spreadCycle or classic).
func (t *TMC2660) SetChopperMode(classicMode bool) {
	t.mu.Lock()
	defer t.mu.Unlock()
	if classicMode {
		t.fields.SetField("chm", 1)
	} else {
		t.fields.SetField("chm", 0)
	}
	log.Printf("tmc2660 '%s': set chopper mode to %v", t.name, classicMode)
}
