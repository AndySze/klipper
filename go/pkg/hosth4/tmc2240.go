// TMC2240 - port of klippy/extras/tmc2240.py
//
// TMC2240 stepper driver configuration
//
// Copyright (C) 2018-2023 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2023 Alex Voinea <voinea.dragos.alexandru@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"log"
	"sync"
)

const (
	tmc2240Frequency = 12500000.0
)

// TMC2240 register addresses.
var tmc2240Registers = map[string]uint8{
	"GCONF":           0x00,
	"GSTAT":           0x01,
	"IFCNT":           0x02,
	"NODECONF":        0x03,
	"IOIN":            0x04,
	"DRV_CONF":        0x0A,
	"GLOBALSCALER":    0x0B,
	"IHOLD_IRUN":      0x10,
	"TPOWERDOWN":      0x11,
	"TSTEP":           0x12,
	"TPWMTHRS":        0x13,
	"TCOOLTHRS":       0x14,
	"THIGH":           0x15,
	"DIRECT_MODE":     0x2D,
	"ENCMODE":         0x38,
	"X_ENC":           0x39,
	"ENC_CONST":       0x3A,
	"ENC_STATUS":      0x3B,
	"ENC_LATCH":       0x3C,
	"ADC_VSUPPLY_AIN": 0x50,
	"ADC_TEMP":        0x51,
	"OTW_OV_VTH":      0x52,
	"MSLUT0":          0x60,
	"MSLUT1":          0x61,
	"MSLUT2":          0x62,
	"MSLUT3":          0x63,
	"MSLUT4":          0x64,
	"MSLUT5":          0x65,
	"MSLUT6":          0x66,
	"MSLUT7":          0x67,
	"MSLUTSEL":        0x68,
	"MSLUTSTART":      0x69,
	"MSCNT":           0x6A,
	"MSCURACT":        0x6B,
	"CHOPCONF":        0x6C,
	"COOLCONF":        0x6D,
	"DRV_STATUS":      0x6F,
	"PWMCONF":         0x70,
	"PWM_SCALE":       0x71,
	"PWM_AUTO":        0x72,
	"SG4_THRS":        0x74,
	"SG4_RESULT":      0x75,
	"SG4_IND":         0x76,
}

// TMC2240 register field definitions.
var tmc2240Fields = map[string]map[string]uint32{
	"GCONF": {
		"faststandstill":   0x01 << 1,
		"en_pwm_mode":      0x01 << 2,
		"multistep_filt":   0x01 << 3,
		"shaft":            0x01 << 4,
		"diag0_error":      0x01 << 5,
		"diag0_otpw":       0x01 << 6,
		"diag0_stall":      0x01 << 7,
		"diag1_stall":      0x01 << 8,
		"diag1_index":      0x01 << 9,
		"diag1_onstate":    0x01 << 10,
		"diag0_pushpull":   0x01 << 12,
		"diag1_pushpull":   0x01 << 13,
		"small_hysteresis": 0x01 << 14,
		"stop_enable":      0x01 << 15,
		"direct_mode":      0x01 << 16,
	},
	"CHOPCONF": {
		"toff":    0x0F << 0,
		"hstrt":   0x07 << 4,
		"hend":    0x0F << 7,
		"fd3":     0x01 << 11,
		"disfdcc": 0x01 << 12,
		"chm":     0x01 << 14,
		"tbl":     0x03 << 15,
		"vhighfs": 0x01 << 18,
		"vhighchm": 0x01 << 19,
		"tpfd":    0x0F << 20,
		"mres":    0x0F << 24,
		"intpol":  0x01 << 28,
		"dedge":   0x01 << 29,
		"diss2g":  0x01 << 30,
		"diss2vs": 0x01 << 31,
	},
	"IHOLD_IRUN": {
		"ihold":      0x1F << 0,
		"irun":       0x1F << 8,
		"iholddelay": 0x0F << 16,
		"irundelay":  0x0F << 24,
	},
	"GLOBALSCALER": {
		"globalscaler": 0xFF << 0,
	},
	"COOLCONF": {
		"semin":  0x0F << 0,
		"seup":   0x03 << 5,
		"semax":  0x0F << 8,
		"sedn":   0x03 << 13,
		"seimin": 0x01 << 15,
		"sgt":    0x7F << 16,
		"sfilt":  0x01 << 24,
	},
	"DRV_STATUS": {
		"sg_result":  0x3FF << 0,
		"s2vsa":      0x01 << 12,
		"s2vsb":      0x01 << 13,
		"stealth":    0x01 << 14,
		"fsactive":   0x01 << 15,
		"cs_actual":  0x1F << 16,
		"stallguard": 0x01 << 24,
		"ot":         0x01 << 25,
		"otpw":       0x01 << 26,
		"s2ga":       0x01 << 27,
		"s2gb":       0x01 << 28,
		"ola":        0x01 << 29,
		"olb":        0x01 << 30,
		"stst":       0x01 << 31,
	},
	"PWMCONF": {
		"pwm_ofs":       0xFF << 0,
		"pwm_grad":      0xFF << 8,
		"pwm_freq":      0x03 << 16,
		"pwm_autoscale": 0x01 << 18,
		"pwm_autograd":  0x01 << 19,
		"freewheel":     0x03 << 20,
		"pwm_meas_sd_enable": 0x01 << 22,
		"pwm_dis_reg_stst": 0x01 << 23,
		"pwm_reg":       0x0F << 24,
		"pwm_lim":       0x0F << 28,
	},
	"SG4_THRS": {
		"sg4_thrs":      0xFF << 0,
		"sg4_filt_en":   0x01 << 8,
		"sg4_angle_offset": 0x01 << 9,
	},
}

// TMC2240 represents a TMC2240 stepper driver.
type TMC2240 struct {
	*TMCBase
	spiMode   int
	mu        sync.RWMutex
}

// TMC2240Config holds TMC2240 configuration.
type TMC2240Config struct {
	TMCSPIConfig
	StallGuardThreshold int
	GlobalScaler        int
}

// DefaultTMC2240Config returns default TMC2240 configuration.
func DefaultTMC2240Config() TMC2240Config {
	return TMC2240Config{
		TMCSPIConfig: TMCSPIConfig{
			TMCConfig: DefaultTMCConfig(),
			SPIMode:   3,
			SPISpeed:  4000000,
		},
		GlobalScaler: 0,
	}
}

// newTMC2240 creates a new TMC2240 driver.
func newTMC2240(rt *runtime, cfg TMC2240Config) (*TMC2240, error) {
	fields := newTMCFieldHelper(tmc2240Fields, []string{"sgt"})
	currentHelper := NewTMCCurrentHelper(cfg.RunCurrent, cfg.HoldCurrent, cfg.SenseResistor)

	base := NewTMCBase(rt, cfg.Name, fields, currentHelper, cfg.Microsteps)

	tmc := &TMC2240{
		TMCBase: base,
		spiMode: cfg.SPIMode,
	}

	// Set default field values
	fields.SetField("multistep_filt", 1)
	fields.SetField("en_pwm_mode", 1) // StealthChop by default

	// CHOPCONF defaults
	fields.SetField("toff", 3)
	fields.SetField("hstrt", 5)
	fields.SetField("hend", 2)
	fields.SetField("tbl", 2)
	fields.SetField("tpfd", 4)

	// Set microsteps
	mres, err := MicrostepsToMRES(cfg.Microsteps)
	if err != nil {
		return nil, err
	}
	fields.SetField("mres", mres)

	// Interpolation
	if cfg.Interpolate {
		fields.SetField("intpol", 1)
	}

	// Global scaler
	if cfg.GlobalScaler > 0 {
		fields.SetField("globalscaler", uint32(cfg.GlobalScaler))
	}

	// PWMCONF defaults
	fields.SetField("pwm_ofs", 29)
	fields.SetField("pwm_grad", 0)
	fields.SetField("pwm_freq", 0)
	fields.SetField("pwm_autoscale", 1)
	fields.SetField("pwm_autograd", 1)
	fields.SetField("freewheel", 0)
	fields.SetField("pwm_reg", 4)
	fields.SetField("pwm_lim", 12)

	// COOLCONF defaults
	fields.SetField("semin", 0)
	fields.SetField("seup", 0)
	fields.SetField("semax", 0)
	fields.SetField("sedn", 0)
	fields.SetField("seimin", 0)

	// StallGuard threshold
	fields.SetField("sg4_thrs", uint32(cfg.StallGuardThreshold))

	logTMCInit("TMC2240", cfg.Name, cfg.Microsteps, cfg.RunCurrent)

	return tmc, nil
}

// GetStatus returns the driver status.
func (t *TMC2240) GetStatus() map[string]any {
	status := t.TMCBase.GetStatus()
	status["driver_type"] = "TMC2240"
	status["spi_mode"] = t.spiMode
	return status
}

// SetStallGuardThreshold sets the SG4 StallGuard threshold.
func (t *TMC2240) SetStallGuardThreshold(threshold int) {
	t.fields.SetField("sg4_thrs", uint32(threshold))
	log.Printf("tmc2240 '%s': set StallGuard4 threshold to %d", t.name, threshold)
}

// SetGlobalScaler sets the global current scaler.
func (t *TMC2240) SetGlobalScaler(scaler int) {
	t.fields.SetField("globalscaler", uint32(scaler))
	log.Printf("tmc2240 '%s': set global scaler to %d", t.name, scaler)
}
