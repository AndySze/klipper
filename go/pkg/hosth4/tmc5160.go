// TMC5160 SPI driver - port of klippy/extras/tmc5160.py
//
// TMC5160 configuration
//
// Copyright (C) 2018-2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"math"
	"sync"
)

const (
	tmc5160Frequency = 12000000.0
)

// TMC5160 register addresses
const (
	tmc5160RegGCONF        = 0x00
	tmc5160RegGSTAT        = 0x01
	tmc5160RegIFCNT        = 0x02
	tmc5160RegSLAVECONF    = 0x03
	tmc5160RegIOIN         = 0x04
	tmc5160RegX_COMPARE    = 0x05
	tmc5160RegOTP_READ     = 0x07
	tmc5160RegFACTORY_CONF = 0x08
	tmc5160RegSHORT_CONF   = 0x09
	tmc5160RegDRV_CONF     = 0x0A
	tmc5160RegGLOBALSCALER = 0x0B
	tmc5160RegOFFSET_READ  = 0x0C
	tmc5160RegIHOLD_IRUN   = 0x10
	tmc5160RegTPOWERDOWN   = 0x11
	tmc5160RegTSTEP        = 0x12
	tmc5160RegTPWMTHRS     = 0x13
	tmc5160RegTCOOLTHRS    = 0x14
	tmc5160RegTHIGH        = 0x15
	tmc5160RegMSLUT0       = 0x60
	tmc5160RegMSLUTSEL     = 0x68
	tmc5160RegMSLUTSTART   = 0x69
	tmc5160RegMSCNT        = 0x6A
	tmc5160RegMSCURACT     = 0x6B
	tmc5160RegCHOPCONF     = 0x6C
	tmc5160RegCOOLCONF     = 0x6D
	tmc5160RegDCCTRL       = 0x6E
	tmc5160RegDRV_STATUS   = 0x6F
	tmc5160RegPWMCONF      = 0x70
	tmc5160RegPWM_SCALE    = 0x71
	tmc5160RegPWM_AUTO     = 0x72
	tmc5160RegLOST_STEPS   = 0x73
)

// TMC5160Driver manages a single TMC5160 stepper driver.
type TMC5160Driver struct {
	rt   *runtime
	name string

	// SPI communication
	spiOID int

	// Driver configuration
	runCurrent     float64
	holdCurrent    float64
	senseResistor  float64
	microsteps     int
	interpolate    bool
	stealthchop    bool

	// TMC5160 specific
	globalScaler uint8 // Global current scaler (0-255)

	// Cached register values
	gconf        uint32
	chopconf     uint32
	iholdIrun    uint32
	pwmconf      uint32
	tpowerdown   uint32
	globalscaler uint32
	shortConf    uint32
	drvConf      uint32

	// State
	initialized bool
	mu          sync.Mutex
}

// TMC5160Config holds configuration for a TMC5160 driver.
type TMC5160Config struct {
	Name          string
	SpiOID        int
	RunCurrent    float64 // RMS run current in amps
	HoldCurrent   float64 // RMS hold current in amps (default: run_current)
	SenseResistor float64 // Sense resistor value in ohms (default: 0.075)
	Microsteps    int     // Microsteps (1, 2, 4, 8, 16, 32, 64, 128, 256)
	Interpolate   bool    // Enable 256 microstep interpolation
	Stealthchop   bool    // Enable StealthChop mode
}

// DefaultTMC5160Config returns default TMC5160 configuration.
func DefaultTMC5160Config() TMC5160Config {
	return TMC5160Config{
		SenseResistor: 0.075,
		Microsteps:    16,
		Interpolate:   true,
		Stealthchop:   true,
	}
}

// newTMC5160Driver creates a new TMC5160 driver instance.
func newTMC5160Driver(rt *runtime, cfg TMC5160Config) (*TMC5160Driver, error) {
	if cfg.RunCurrent <= 0 {
		return nil, fmt.Errorf("run_current must be > 0")
	}
	if cfg.HoldCurrent <= 0 {
		cfg.HoldCurrent = cfg.RunCurrent
	}
	if !isValidMicrosteps(cfg.Microsteps) {
		return nil, fmt.Errorf("invalid microsteps value: %d", cfg.Microsteps)
	}

	d := &TMC5160Driver{
		rt:            rt,
		name:          cfg.Name,
		spiOID:        cfg.SpiOID,
		runCurrent:    cfg.RunCurrent,
		holdCurrent:   cfg.HoldCurrent,
		senseResistor: cfg.SenseResistor,
		microsteps:    cfg.Microsteps,
		interpolate:   cfg.Interpolate,
		stealthchop:   cfg.Stealthchop,
	}

	d.calculateRegisters()
	return d, nil
}

// calculateRegisters computes the register values based on configuration.
func (d *TMC5160Driver) calculateRegisters() {
	// Calculate current settings using TMC5160 formula
	// The TMC5160 uses GlobalScaler for current adjustment
	globalScaler, irun, ihold := d.calcCurrent()
	d.globalScaler = globalScaler

	// Calculate microstep resolution
	mres := microstepsToMres(d.microsteps)

	// Build GCONF
	// en_pwm_mode=1 for StealthChop
	d.gconf = 0
	if d.stealthchop {
		d.gconf |= 0x04 // en_pwm_mode
	}

	// Build CHOPCONF
	// toff=3, hstrt=4, hend=1, tbl=2, tpfd=4
	toff := uint32(3)
	hstrt := uint32(4)
	hend := uint32(1)
	tbl := uint32(2)
	tpfd := uint32(4) // passive fast decay time
	intpolVal := uint32(0)
	if d.interpolate {
		intpolVal = 1
	}

	d.chopconf = toff | (hstrt << 4) | (hend << 7) | (tbl << 15) |
		(tpfd << 20) | (uint32(mres) << 24) | (intpolVal << 28)

	// Build IHOLD_IRUN
	iholddelay := uint32(6)
	d.iholdIrun = uint32(ihold) | (uint32(irun) << 8) | (iholddelay << 16)

	// Build GLOBALSCALER
	d.globalscaler = uint32(globalScaler)

	// Build PWMCONF
	// pwm_ofs=30, pwm_grad=0, pwm_freq=0, pwm_autoscale=true, pwm_autograd=true
	// pwm_reg=4, pwm_lim=12
	pwmOfs := uint32(30)
	pwmGrad := uint32(0)
	pwmFreq := uint32(0)
	pwmAutoscale := uint32(1)
	pwmAutograd := uint32(1)
	freewheel := uint32(0)
	pwmReg := uint32(4)
	pwmLim := uint32(12)

	d.pwmconf = pwmOfs | (pwmGrad << 8) | (pwmFreq << 16) | (pwmAutoscale << 18) |
		(pwmAutograd << 19) | (freewheel << 20) | (pwmReg << 24) | (pwmLim << 28)

	// TPOWERDOWN
	d.tpowerdown = 10

	// SHORT_CONF: Default protection settings
	// s2vs_level=6, s2g_level=6, shortfilter=1, shortdelay=0
	d.shortConf = 6 | (6 << 8) | (1 << 16)

	// DRV_CONF: Driver strength and filter settings
	// bbmtime=0, bbmclks=4, otselect=0, drvstrength=0, filt_isense=0
	d.drvConf = 0 | (4 << 8)
}

// calcCurrent calculates the current settings for TMC5160.
// Returns globalScaler (0-255), irun (0-31), ihold (0-31)
func (d *TMC5160Driver) calcCurrent() (uint8, int, int) {
	// TMC5160 current calculation:
	// I_rms = (GLOBALSCALER/256) * ((IRUN+1)/32) * (V_fs / R_sense) / sqrt(2)
	// V_fs = 0.325V

	vfs := 0.325
	sqrt2 := math.Sqrt(2.0)

	// Calculate required globalScaler for maximum current (IRUN=31)
	// Solve for GLOBALSCALER: GLOBALSCALER = I_rms * 256 * 32 * R_sense * sqrt(2) / (32 * V_fs)
	// Simplified: GLOBALSCALER = I_rms * 256 * R_sense * sqrt(2) / V_fs
	gsFloat := d.runCurrent * 256.0 * d.senseResistor * sqrt2 / vfs

	globalScaler := uint8(255)
	if gsFloat < 256 {
		globalScaler = uint8(gsFloat)
		if globalScaler < 32 {
			globalScaler = 32 // Minimum recommended value
		}
	}

	// Now calculate IRUN with the chosen globalScaler
	// I_rms = (GLOBALSCALER/256) * ((IRUN+1)/32) * (V_fs / R_sense) / sqrt(2)
	// IRUN = 32 * I_rms * 256 * R_sense * sqrt(2) / (GLOBALSCALER * V_fs) - 1
	irunFloat := 32.0 * d.runCurrent * 256.0 * d.senseResistor * sqrt2 / (float64(globalScaler) * vfs) - 1
	irun := int(math.Round(irunFloat))
	if irun < 0 {
		irun = 0
	}
	if irun > 31 {
		irun = 31
	}

	// Calculate ihold proportionally
	ihold := int(float64(irun) * d.holdCurrent / d.runCurrent)
	if ihold < 0 {
		ihold = 0
	}
	if ihold > 31 {
		ihold = 31
	}

	return globalScaler, irun, ihold
}

// GetName returns the driver name.
func (d *TMC5160Driver) GetName() string {
	return d.name
}

// GetStatus returns the driver status.
func (d *TMC5160Driver) GetStatus() map[string]any {
	d.mu.Lock()
	defer d.mu.Unlock()

	return map[string]any{
		"run_current":    d.runCurrent,
		"hold_current":   d.holdCurrent,
		"microsteps":     d.microsteps,
		"stealthchop":    d.stealthchop,
		"interpolate":    d.interpolate,
		"global_scaler":  d.globalScaler,
		"initialized":    d.initialized,
	}
}

// SetCurrent sets the run and hold currents.
func (d *TMC5160Driver) SetCurrent(runCurrent, holdCurrent float64) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if runCurrent <= 0 {
		return fmt.Errorf("run_current must be > 0")
	}
	if holdCurrent <= 0 {
		holdCurrent = runCurrent
	}

	d.runCurrent = runCurrent
	d.holdCurrent = holdCurrent
	d.calculateRegisters()

	// In a full implementation, this would send the updated registers
	return nil
}

// SetMicrosteps sets the microstep resolution.
func (d *TMC5160Driver) SetMicrosteps(microsteps int) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	if !isValidMicrosteps(microsteps) {
		return fmt.Errorf("invalid microsteps value: %d", microsteps)
	}

	d.microsteps = microsteps
	d.calculateRegisters()

	// In a full implementation, this would send the updated CHOPCONF register
	return nil
}

// SetStealthchop enables or disables StealthChop mode.
func (d *TMC5160Driver) SetStealthchop(enable bool) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	d.stealthchop = enable
	d.calculateRegisters()

	// In a full implementation, this would send the updated GCONF register
	return nil
}

// GetGlobalScaler returns the global current scaler value.
func (d *TMC5160Driver) GetGlobalScaler() uint8 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.globalScaler
}
