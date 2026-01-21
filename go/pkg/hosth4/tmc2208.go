// TMC2208/TMC2209 UART driver - port of klippy/extras/tmc2208.py
//
// TMC2208 UART communication and configuration
//
// Copyright (C) 2018-2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"sync"
)

const (
	tmc2208Frequency = 12000000.0
)

// TMC2208 register addresses
const (
	tmc2208RegGCONF       = 0x00
	tmc2208RegGSTAT       = 0x01
	tmc2208RegIFCNT       = 0x02
	tmc2208RegSLAVECONF   = 0x03
	tmc2208RegOTP_PROG    = 0x04
	tmc2208RegOTP_READ    = 0x05
	tmc2208RegIOIN        = 0x06
	tmc2208RegFACTORY_CONF = 0x07
	tmc2208RegIHOLD_IRUN  = 0x10
	tmc2208RegTPOWERDOWN  = 0x11
	tmc2208RegTSTEP       = 0x12
	tmc2208RegTPWMTHRS    = 0x13
	tmc2208RegVACTUAL     = 0x22
	tmc2208RegMSCNT       = 0x6a
	tmc2208RegMSCURACT    = 0x6b
	tmc2208RegCHOPCONF    = 0x6c
	tmc2208RegDRV_STATUS  = 0x6f
	tmc2208RegPWMCONF     = 0x70
	tmc2208RegPWM_SCALE   = 0x71
	tmc2208RegPWM_AUTO    = 0x72
)

// TMC2208 GCONF field bits
const (
	tmc2208GCONFIScaleAnalog   = 0x01
	tmc2208GCONFInternalRsense = 0x01 << 1
	tmc2208GCONFEnSpreadcycle  = 0x01 << 2
	tmc2208GCONFShaft          = 0x01 << 3
	tmc2208GCONFIndexOTPW      = 0x01 << 4
	tmc2208GCONFIndexStep      = 0x01 << 5
	tmc2208GCONFPdnDisable     = 0x01 << 6
	tmc2208GCONFMstepRegSelect = 0x01 << 7
	tmc2208GCONFMultistepFilt  = 0x01 << 8
	tmc2208GCONFTestMode       = 0x01 << 9
)

// TMC2208 DRV_STATUS field bits
const (
	tmc2208DRVStatusOTPW     = 0x01
	tmc2208DRVStatusOT       = 0x01 << 1
	tmc2208DRVStatusS2GA     = 0x01 << 2
	tmc2208DRVStatusS2GB     = 0x01 << 3
	tmc2208DRVStatusS2VSA    = 0x01 << 4
	tmc2208DRVStatusS2VSB    = 0x01 << 5
	tmc2208DRVStatusOLA      = 0x01 << 6
	tmc2208DRVStatusOLB      = 0x01 << 7
	tmc2208DRVStatusT120     = 0x01 << 8
	tmc2208DRVStatusT143     = 0x01 << 9
	tmc2208DRVStatusT150     = 0x01 << 10
	tmc2208DRVStatusT157     = 0x01 << 11
	tmc2208DRVStatusCSActual = 0x1f << 16
	tmc2208DRVStatusStealth  = 0x01 << 30
	tmc2208DRVStatusStst     = 0x01 << 31
)

// TMC2208Driver manages a single TMC2208/TMC2209 stepper driver.
type TMC2208Driver struct {
	rt   *runtime
	name string

	// UART communication
	uartAddr uint8 // UART slave address (0-3)

	// Driver configuration
	runCurrent     float64
	holdCurrent    float64
	senseResistor  float64
	microsteps     int
	interpolate    bool
	stealthchop    bool

	// Cached register values
	gconf       uint32
	chopconf    uint32
	iholdIrun   uint32
	pwmconf     uint32
	tpowerdown  uint32
	tpwmthrs    uint32

	// State
	initialized bool
	mu          sync.Mutex
}

// TMC2208Config holds configuration for a TMC2208 driver.
type TMC2208Config struct {
	Name          string
	UartAddr      uint8   // UART address (0-3), default 0
	RunCurrent    float64 // RMS run current in amps
	HoldCurrent   float64 // RMS hold current in amps (default: run_current)
	SenseResistor float64 // Sense resistor value in ohms (default: 0.110)
	Microsteps    int     // Microsteps (1, 2, 4, 8, 16, 32, 64, 128, 256)
	Interpolate   bool    // Enable 256 microstep interpolation
	Stealthchop   bool    // Enable StealthChop mode
}

// DefaultTMC2208Config returns default TMC2208 configuration.
func DefaultTMC2208Config() TMC2208Config {
	return TMC2208Config{
		UartAddr:      0,
		SenseResistor: 0.110,
		Microsteps:    16,
		Interpolate:   true,
		Stealthchop:   true,
	}
}

// newTMC2208Driver creates a new TMC2208 driver instance.
func newTMC2208Driver(rt *runtime, cfg TMC2208Config) (*TMC2208Driver, error) {
	if cfg.RunCurrent <= 0 {
		return nil, fmt.Errorf("run_current must be > 0")
	}
	if cfg.HoldCurrent <= 0 {
		cfg.HoldCurrent = cfg.RunCurrent
	}
	if !isValidMicrosteps(cfg.Microsteps) {
		return nil, fmt.Errorf("invalid microsteps value: %d", cfg.Microsteps)
	}

	d := &TMC2208Driver{
		rt:            rt,
		name:          cfg.Name,
		uartAddr:      cfg.UartAddr,
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

// isValidMicrosteps checks if microsteps value is valid.
func isValidMicrosteps(ms int) bool {
	valid := []int{1, 2, 4, 8, 16, 32, 64, 128, 256}
	for _, v := range valid {
		if ms == v {
			return true
		}
	}
	return false
}

// calculateRegisters computes the register values based on configuration.
func (d *TMC2208Driver) calculateRegisters() {
	// Calculate current settings
	vsense, irun, ihold := d.calcCurrent()

	// Calculate microstep resolution
	mres := microstepsToMres(d.microsteps)

	// Build GCONF
	d.gconf = tmc2208GCONFPdnDisable | tmc2208GCONFMstepRegSelect | tmc2208GCONFMultistepFilt
	if !d.stealthchop {
		d.gconf |= tmc2208GCONFEnSpreadcycle
	}

	// Build CHOPCONF
	// toff=3, hstrt=5, hend=0, tbl=2
	toff := uint32(3)
	hstrt := uint32(5)
	hend := uint32(0)
	tbl := uint32(2)
	vsenseVal := uint32(0)
	if vsense {
		vsenseVal = 1
	}
	intpolVal := uint32(0)
	if d.interpolate {
		intpolVal = 1
	}

	d.chopconf = toff | (hstrt << 4) | (hend << 7) | (tbl << 15) | (vsenseVal << 17) |
		(uint32(mres) << 24) | (intpolVal << 28)

	// Build IHOLD_IRUN
	iholddelay := uint32(8)
	d.iholdIrun = uint32(ihold) | (uint32(irun) << 8) | (iholddelay << 16)

	// Build PWMCONF
	// pwm_ofs=36, pwm_grad=14, pwm_freq=1, pwm_autoscale=true, pwm_autograd=true
	// freewheel=0, pwm_reg=8, pwm_lim=12
	pwmOfs := uint32(36)
	pwmGrad := uint32(14)
	pwmFreq := uint32(1)
	pwmAutoscale := uint32(1)
	pwmAutograd := uint32(1)
	freewheel := uint32(0)
	pwmReg := uint32(8)
	pwmLim := uint32(12)

	d.pwmconf = pwmOfs | (pwmGrad << 8) | (pwmFreq << 16) | (pwmAutoscale << 18) |
		(pwmAutograd << 19) | (freewheel << 20) | (pwmReg << 24) | (pwmLim << 28)

	// TPOWERDOWN
	d.tpowerdown = 20

	// TPWMTHRS (velocity threshold for StealthChop)
	d.tpwmthrs = 0
}

// calcCurrent calculates the current settings.
// Returns vsense (true for high sensitivity), irun (0-31), ihold (0-31)
func (d *TMC2208Driver) calcCurrent() (bool, int, int) {
	// TMC2208 current calculation
	// I_rms = (IRUN+1)/32 * V_fs / (R_sense * sqrt(2))
	// where V_fs = 0.325V (vsense=1) or 0.180V (vsense=0)

	// Try vsense=1 (high sensitivity) first
	vsense := true
	vfs := 0.325
	irun := int(32.0*d.runCurrent*d.senseResistor*1.41421356/vfs) - 1
	if irun > 31 {
		// Need lower sensitivity
		vsense = false
		vfs = 0.180
		irun = int(32.0*d.runCurrent*d.senseResistor*1.41421356/vfs) - 1
	}
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

	return vsense, irun, ihold
}

// GetName returns the driver name.
func (d *TMC2208Driver) GetName() string {
	return d.name
}

// GetPhaseOffset returns the microstep phase offset and total phases.
// TMC2208 has 1024 microsteps per electrical cycle (256 microsteps * 4 phases).
func (d *TMC2208Driver) GetPhaseOffset() (int, int) {
	d.mu.Lock()
	defer d.mu.Unlock()
	phases := (256 / d.microsteps) * 4 * d.microsteps // = 1024
	return 0, phases
}

// GetStatus returns the driver status.
func (d *TMC2208Driver) GetStatus() map[string]any {
	d.mu.Lock()
	defer d.mu.Unlock()

	return map[string]any{
		"run_current":   d.runCurrent,
		"hold_current":  d.holdCurrent,
		"microsteps":    d.microsteps,
		"stealthchop":   d.stealthchop,
		"interpolate":   d.interpolate,
		"initialized":   d.initialized,
	}
}

// SetCurrent sets the run and hold currents.
func (d *TMC2208Driver) SetCurrent(runCurrent, holdCurrent float64) error {
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

	// In a full implementation, this would send the updated IHOLD_IRUN register
	return nil
}

// SetMicrosteps sets the microstep resolution.
func (d *TMC2208Driver) SetMicrosteps(microsteps int) error {
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
func (d *TMC2208Driver) SetStealthchop(enable bool) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	d.stealthchop = enable
	d.calculateRegisters()

	// In a full implementation, this would send the updated GCONF register
	return nil
}

// TMC2209 extends TMC2208 with additional features
// TMC2209 has the same register layout as TMC2208 plus:
// - SGTHRS (StallGuard threshold)
// - SG_RESULT (StallGuard result)
// - COOLCONF

// TMC2209 additional registers
const (
	tmc2209RegTCOOLTHRS = 0x14 // CoolStep/StallGuard lower velocity threshold
	tmc2209RegSGTHRS    = 0x40 // StallGuard threshold
	tmc2209RegSG_RESULT = 0x41 // StallGuard result (read-only)
	tmc2209RegCOOLCONF  = 0x42 // CoolStep configuration
)

// TMC2209Driver extends TMC2208Driver with StallGuard support.
type TMC2209Driver struct {
	TMC2208Driver

	// StallGuard configuration
	sgthrs    uint8  // StallGuard threshold (0-255)
	tcoolthrs uint32 // CoolStep/StallGuard lower velocity threshold (TSTEP units)
	coolconf  uint32 // COOLCONF register value
}

// TMC2209Config extends TMC2208Config with StallGuard settings.
type TMC2209Config struct {
	TMC2208Config
	StallGuardThreshold uint8  // StallGuard threshold (0-255), driver_SGTHRS
	CoolstepThreshold   uint32 // CoolStep/StallGuard velocity threshold (TSTEP units)
	// COOLCONF fields (semin, seup, semax, sedn, seimin - for adaptive current)
	Semin  uint8 // Minimum StallGuard value for CoolStep (0-15)
	Seup   uint8 // Current increment step (0-3)
	Semax  uint8 // Maximum StallGuard offset for CoolStep (0-15)
	Sedn   uint8 // Current decrement step (0-3)
	Seimin bool  // Minimum current (false=1/2, true=1/4)
}

// DefaultTMC2209Config returns default TMC2209 configuration.
func DefaultTMC2209Config() TMC2209Config {
	return TMC2209Config{
		TMC2208Config:       DefaultTMC2208Config(),
		StallGuardThreshold: 0,
	}
}

// newTMC2209Driver creates a new TMC2209 driver instance.
func newTMC2209Driver(rt *runtime, cfg TMC2209Config) (*TMC2209Driver, error) {
	if cfg.TMC2208Config.RunCurrent <= 0 {
		return nil, fmt.Errorf("run_current must be > 0")
	}
	if cfg.TMC2208Config.HoldCurrent <= 0 {
		cfg.TMC2208Config.HoldCurrent = cfg.TMC2208Config.RunCurrent
	}
	if !isValidMicrosteps(cfg.TMC2208Config.Microsteps) {
		return nil, fmt.Errorf("invalid microsteps value: %d", cfg.TMC2208Config.Microsteps)
	}

	// Build COOLCONF register:
	// semin (bits 0-3), seup (bits 5-6), semax (bits 8-11), sedn (bits 13-14), seimin (bit 15)
	seimin := uint32(0)
	if cfg.Seimin {
		seimin = 1
	}
	coolconf := uint32(cfg.Semin&0x0F) |
		(uint32(cfg.Seup&0x03) << 5) |
		(uint32(cfg.Semax&0x0F) << 8) |
		(uint32(cfg.Sedn&0x03) << 13) |
		(seimin << 15)

	d := &TMC2209Driver{
		TMC2208Driver: TMC2208Driver{
			rt:            rt,
			name:          cfg.TMC2208Config.Name,
			uartAddr:      cfg.TMC2208Config.UartAddr,
			runCurrent:    cfg.TMC2208Config.RunCurrent,
			holdCurrent:   cfg.TMC2208Config.HoldCurrent,
			senseResistor: cfg.TMC2208Config.SenseResistor,
			microsteps:    cfg.TMC2208Config.Microsteps,
			interpolate:   cfg.TMC2208Config.Interpolate,
			stealthchop:   cfg.TMC2208Config.Stealthchop,
		},
		sgthrs:    cfg.StallGuardThreshold,
		tcoolthrs: cfg.CoolstepThreshold,
		coolconf:  coolconf,
	}

	d.calculateRegisters()
	return d, nil
}

// SetStallGuardThreshold sets the StallGuard threshold.
func (d *TMC2209Driver) SetStallGuardThreshold(threshold uint8) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.sgthrs = threshold
	// In a full implementation, this would send the SGTHRS register
}

// GetStallGuardResult returns the current StallGuard result.
// This reads the SG_RESULT register (0x41) which contains a 10-bit value (0-1023).
// Higher values indicate more load margin (less load).
//
// In file output mode (golden tests), this returns 0 since there's no UART
// communication to read the actual register value from the driver.
// In a live system, this would query the MCU to read the SG_RESULT register.
func (d *TMC2209Driver) GetStallGuardResult() uint16 {
	// In file output mode, there's no UART communication, so we return 0.
	// A live implementation would read from UART: mcu_tmc.get_register("SG_RESULT")
	return 0
}

// GetStatus returns the TMC2209 driver status.
func (d *TMC2209Driver) GetStatus() map[string]any {
	status := d.TMC2208Driver.GetStatus()
	d.mu.Lock()
	status["stallguard_threshold"] = d.sgthrs
	status["tcoolthrs"] = d.tcoolthrs
	status["coolconf"] = d.coolconf
	d.mu.Unlock()
	return status
}

// SetTCoolThrs sets the CoolStep/StallGuard velocity threshold.
// StallGuard is only active when TSTEP is between TCOOLTHRS and THIGH.
// Set to 0 to disable CoolStep/StallGuard velocity threshold.
func (d *TMC2209Driver) SetTCoolThrs(tcoolthrs uint32) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.tcoolthrs = tcoolthrs
	// In a full implementation, this would send the TCOOLTHRS register
}

// GetTCoolThrs returns the current TCOOLTHRS value.
func (d *TMC2209Driver) GetTCoolThrs() uint32 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.tcoolthrs
}

// SetCoolConf sets the CoolStep configuration register.
func (d *TMC2209Driver) SetCoolConf(coolconf uint32) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.coolconf = coolconf
	// In a full implementation, this would send the COOLCONF register
}

// GetCoolConf returns the current COOLCONF value.
func (d *TMC2209Driver) GetCoolConf() uint32 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.coolconf
}

// GetStallGuardThreshold returns the current StallGuard threshold.
func (d *TMC2209Driver) GetStallGuardThreshold() uint8 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return d.sgthrs
}
