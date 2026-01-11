// TMC2209 stepper driver support - port of klippy/extras/tmc2209.py
//
// Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025  Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package tmc

import "fmt"

// TMC2209Fields defines the register fields for TMC2209.
var TMC2209Fields = map[string]map[string]uint32{
	"GCONF": {
		"i_scale_analog":   1 << 0,
		"internal_rsense":  1 << 1,
		"en_spreadcycle":   1 << 2,
		"shaft":            1 << 3,
		"index_otpw":       1 << 4,
		"index_step":       1 << 5,
		"pdn_disable":      1 << 6,
		"mstep_reg_select": 1 << 7,
		"multistep_filt":   1 << 8,
		"test_mode":        1 << 9,
	},
	"GSTAT": {
		"reset":    1 << 0,
		"drv_err":  1 << 1,
		"uv_cp":    1 << 2,
	},
	"IFCNT": {
		"ifcnt": 0xff,
	},
	"NODECONF": {
		"nodeconf": 0xff,
	},
	"IOIN": {
		"enn":      1 << 0,
		"ms1":      1 << 2,
		"ms2":      1 << 3,
		"diag":     1 << 4,
		"pdn_uart": 1 << 6,
		"step":     1 << 7,
		"spread_en": 1 << 8,
		"dir":      1 << 9,
		"version":  0xff << 24,
	},
	"IHOLD_IRUN": {
		"ihold":      0x1f << 0,
		"irun":       0x1f << 8,
		"iholddelay": 0x0f << 16,
	},
	"TPOWERDOWN": {
		"tpowerdown": 0xff,
	},
	"TSTEP": {
		"tstep": 0xfffff,
	},
	"TPWMTHRS": {
		"tpwmthrs": 0xfffff,
	},
	"VACTUAL": {
		"vactual": 0xffffff,
	},
	"TCOOLTHRS": {
		"tcoolthrs": 0xfffff,
	},
	"SGTHRS": {
		"sgthrs": 0xff,
	},
	"SG_RESULT": {
		"sg_result": 0x3ff,
	},
	"COOLCONF": {
		"semin":  0x0f << 0,
		"seup":   0x03 << 5,
		"semax":  0x0f << 8,
		"sedn":   0x03 << 13,
		"seimin": 1 << 15,
	},
	"MSCNT": {
		"mscnt": 0x3ff,
	},
	"MSCURACT": {
		"cur_a": 0x1ff << 0,
		"cur_b": 0x1ff << 16,
	},
	"CHOPCONF": {
		"toff":      0x0f << 0,
		"hstrt":     0x07 << 4,
		"hend":      0x0f << 7,
		"tbl":       0x03 << 15,
		"vsense":    1 << 17,
		"mres":      0x0f << 24,
		"intpol":    1 << 28,
		"dedge":     1 << 29,
		"diss2g":    1 << 30,
		"diss2vs":   1 << 31,
	},
	"DRV_STATUS": {
		"otpw":      1 << 0,
		"ot":        1 << 1,
		"s2ga":      1 << 2,
		"s2gb":      1 << 3,
		"s2vsa":     1 << 4,
		"s2vsb":     1 << 5,
		"ola":       1 << 6,
		"olb":       1 << 7,
		"t120":      1 << 8,
		"t143":      1 << 9,
		"t150":      1 << 10,
		"t157":      1 << 11,
		"cs_actual": 0x1f << 16,
		"stealth":   1 << 30,
		"stst":      1 << 31,
	},
	"PWMCONF": {
		"pwm_ofs":       0xff << 0,
		"pwm_grad":      0xff << 8,
		"pwm_freq":      0x03 << 16,
		"pwm_autoscale": 1 << 18,
		"pwm_autograd":  1 << 19,
		"freewheel":     0x03 << 20,
		"pwm_reg":       0x0f << 24,
		"pwm_lim":       0x0f << 28,
	},
	"PWM_SCALE": {
		"pwm_scale_sum":  0xff << 0,
		"pwm_scale_auto": 0x1ff << 16,
	},
	"PWM_AUTO": {
		"pwm_ofs_auto":  0xff << 0,
		"pwm_grad_auto": 0xff << 16,
	},
}

// TMC2209SignedFields lists fields that are signed.
var TMC2209SignedFields = []string{"cur_a", "cur_b", "pwm_scale_auto"}

// TMC2209 represents a TMC2209 stepper driver.
type TMC2209 struct {
	Name             string
	Fields           *FieldHelper
	CurrentCalc      *CurrentCalculator
	Config           *TMCConfig
	ReadRegisterFunc  func(addr uint8) (uint32, error)
	WriteRegisterFunc func(addr uint8, value uint32) error
}

// TMC2209 register addresses
const (
	TMC2209_GCONF       = 0x00
	TMC2209_GSTAT       = 0x01
	TMC2209_IFCNT       = 0x02
	TMC2209_NODECONF    = 0x03
	TMC2209_IOIN        = 0x06
	TMC2209_IHOLD_IRUN  = 0x10
	TMC2209_TPOWERDOWN  = 0x11
	TMC2209_TSTEP       = 0x12
	TMC2209_TPWMTHRS    = 0x13
	TMC2209_VACTUAL     = 0x22
	TMC2209_TCOOLTHRS   = 0x14
	TMC2209_SGTHRS      = 0x40
	TMC2209_SG_RESULT   = 0x41
	TMC2209_COOLCONF    = 0x42
	TMC2209_MSCNT       = 0x6A
	TMC2209_MSCURACT    = 0x6B
	TMC2209_CHOPCONF    = 0x6C
	TMC2209_DRV_STATUS  = 0x6F
	TMC2209_PWMCONF     = 0x70
	TMC2209_PWM_SCALE   = 0x71
	TMC2209_PWM_AUTO    = 0x72
)

// TMC2209RegAddrs maps register names to addresses.
var TMC2209RegAddrs = map[string]uint8{
	"GCONF":       TMC2209_GCONF,
	"GSTAT":       TMC2209_GSTAT,
	"IFCNT":       TMC2209_IFCNT,
	"NODECONF":    TMC2209_NODECONF,
	"IOIN":        TMC2209_IOIN,
	"IHOLD_IRUN":  TMC2209_IHOLD_IRUN,
	"TPOWERDOWN":  TMC2209_TPOWERDOWN,
	"TSTEP":       TMC2209_TSTEP,
	"TPWMTHRS":    TMC2209_TPWMTHRS,
	"VACTUAL":     TMC2209_VACTUAL,
	"TCOOLTHRS":   TMC2209_TCOOLTHRS,
	"SGTHRS":      TMC2209_SGTHRS,
	"SG_RESULT":   TMC2209_SG_RESULT,
	"COOLCONF":    TMC2209_COOLCONF,
	"MSCNT":       TMC2209_MSCNT,
	"MSCURACT":    TMC2209_MSCURACT,
	"CHOPCONF":    TMC2209_CHOPCONF,
	"DRV_STATUS":  TMC2209_DRV_STATUS,
	"PWMCONF":     TMC2209_PWMCONF,
	"PWM_SCALE":   TMC2209_PWM_SCALE,
	"PWM_AUTO":    TMC2209_PWM_AUTO,
}

// NewTMC2209 creates a new TMC2209 driver.
func NewTMC2209(name string, config *TMCConfig) *TMC2209 {
	if config == nil {
		config = DefaultTMCConfig()
	}

	fields := NewFieldHelper(TMC2209Fields, TMC2209SignedFields, nil)

	return &TMC2209{
		Name:        name,
		Fields:      fields,
		CurrentCalc: NewCurrentCalculator(config.SenseResistor),
		Config:      config,
	}
}

// GetName returns the driver name.
func (t *TMC2209) GetName() string {
	return t.Name
}

// GetFields returns the field helper.
func (t *TMC2209) GetFields() *FieldHelper {
	return t.Fields
}

// GetRegister reads a register value.
func (t *TMC2209) GetRegister(regName string) (uint32, error) {
	if t.ReadRegisterFunc == nil {
		return t.Fields.Registers[regName], nil
	}
	addr, ok := TMC2209RegAddrs[regName]
	if !ok {
		return 0, fmt.Errorf("unknown register %s", regName)
	}
	return t.ReadRegisterFunc(addr)
}

// SetRegister writes a register value.
func (t *TMC2209) SetRegister(regName string, value uint32) error {
	t.Fields.Registers[regName] = value
	if t.WriteRegisterFunc == nil {
		return nil
	}
	addr, ok := TMC2209RegAddrs[regName]
	if !ok {
		return fmt.Errorf("unknown register %s", regName)
	}
	return t.WriteRegisterFunc(addr, value)
}

// GetMicrosteps returns the current microstep setting.
func (t *TMC2209) GetMicrosteps() int {
	chopconf := t.Fields.Registers["CHOPCONF"]
	mres := t.Fields.GetField("mres", &chopconf, "CHOPCONF")
	return GetMicrostepsFromMRES(int(mres))
}

// SetMicrosteps sets the microstep resolution.
func (t *TMC2209) SetMicrosteps(ms int) error {
	mres, err := GetMRES(ms)
	if err != nil {
		return err
	}

	val := t.Fields.SetField("mres", int32(mres), nil, "CHOPCONF")
	return t.SetRegister("CHOPCONF", val)
}

// GetCurrent returns the run current in amps.
func (t *TMC2209) GetCurrent() float64 {
	irun := t.Fields.GetField("irun", nil, "IHOLD_IRUN")
	vsense := t.Fields.GetField("vsense", nil, "CHOPCONF") != 0
	return t.CurrentCalc.CalcCurrentFromBits(int(irun), vsense)
}

// SetCurrent sets the run and hold currents.
func (t *TMC2209) SetCurrent(runCurrent, holdMultiplier float64) error {
	irun, vsense := t.CurrentCalc.CalcCurrentBits(runCurrent)
	ihold := int(float64(irun) * holdMultiplier)
	if ihold < 0 {
		ihold = 0
	}
	if ihold > 31 {
		ihold = 31
	}

	// Set VSENSE
	var vsenseVal int32
	if vsense {
		vsenseVal = 1
	}
	chopconf := t.Fields.SetField("vsense", vsenseVal, nil, "CHOPCONF")
	if err := t.SetRegister("CHOPCONF", chopconf); err != nil {
		return err
	}

	// Set IHOLD_IRUN
	t.Fields.SetField("irun", int32(irun), nil, "IHOLD_IRUN")
	val := t.Fields.SetField("ihold", int32(ihold), nil, "IHOLD_IRUN")
	return t.SetRegister("IHOLD_IRUN", val)
}

// SetStealthChop enables or disables StealthChop mode.
func (t *TMC2209) SetStealthChop(enable bool) error {
	var val int32
	if !enable {
		val = 1 // en_spreadcycle disables stealthchop
	}
	gconf := t.Fields.SetField("en_spreadcycle", val, nil, "GCONF")
	return t.SetRegister("GCONF", gconf)
}

// SetStallThreshold sets the stallGuard threshold.
func (t *TMC2209) SetStallThreshold(threshold int) error {
	if threshold < 0 || threshold > 255 {
		return fmt.Errorf("stall threshold must be 0-255")
	}
	val := t.Fields.SetField("sgthrs", int32(threshold), nil, "SGTHRS")
	return t.SetRegister("SGTHRS", val)
}

// GetStallResult reads the stallGuard result.
func (t *TMC2209) GetStallResult() (int, error) {
	val, err := t.GetRegister("SG_RESULT")
	if err != nil {
		return 0, err
	}
	return int(t.Fields.GetField("sg_result", &val, "SG_RESULT")), nil
}

// GetStatus reads the driver status.
func (t *TMC2209) GetStatus() (*TMCStatus, error) {
	val, err := t.GetRegister("DRV_STATUS")
	if err != nil {
		return nil, err
	}
	return ParseDRVStatus(val, t.Fields), nil
}

// Initialize sets up the driver with default configuration.
func (t *TMC2209) Initialize() error {
	// Set default GCONF
	gconf := t.Fields.SetField("pdn_disable", 1, nil, "GCONF")
	gconf = t.Fields.SetField("mstep_reg_select", 1, &gconf, "GCONF")
	gconf = t.Fields.SetField("multistep_filt", 1, &gconf, "GCONF")
	if err := t.SetRegister("GCONF", gconf); err != nil {
		return err
	}

	// Set CHOPCONF defaults
	chopconf := t.Fields.SetField("toff", 3, nil, "CHOPCONF")
	chopconf = t.Fields.SetField("hstrt", 5, &chopconf, "CHOPCONF")
	chopconf = t.Fields.SetField("hend", 0, &chopconf, "CHOPCONF")
	chopconf = t.Fields.SetField("tbl", 2, &chopconf, "CHOPCONF")
	if t.Config.Interpolate {
		chopconf = t.Fields.SetField("intpol", 1, &chopconf, "CHOPCONF")
	}
	if err := t.SetRegister("CHOPCONF", chopconf); err != nil {
		return err
	}

	// Set microsteps
	if err := t.SetMicrosteps(t.Config.Microsteps); err != nil {
		return err
	}

	// Set current
	holdMult := t.Config.HoldCurrent / t.Config.RunCurrent
	if err := t.SetCurrent(t.Config.RunCurrent, holdMult); err != nil {
		return err
	}

	// Set PWMCONF defaults for StealthChop
	pwmconf := t.Fields.SetField("pwm_autoscale", 1, nil, "PWMCONF")
	pwmconf = t.Fields.SetField("pwm_autograd", 1, &pwmconf, "PWMCONF")
	pwmconf = t.Fields.SetField("pwm_ofs", 36, &pwmconf, "PWMCONF")
	pwmconf = t.Fields.SetField("pwm_grad", 14, &pwmconf, "PWMCONF")
	pwmconf = t.Fields.SetField("pwm_freq", 1, &pwmconf, "PWMCONF")
	pwmconf = t.Fields.SetField("pwm_reg", 8, &pwmconf, "PWMCONF")
	pwmconf = t.Fields.SetField("pwm_lim", 12, &pwmconf, "PWMCONF")
	if err := t.SetRegister("PWMCONF", pwmconf); err != nil {
		return err
	}

	// Enable StealthChop if configured
	if err := t.SetStealthChop(t.Config.StealthChop); err != nil {
		return err
	}

	// Set stall threshold
	if t.Config.StallThreshold > 0 {
		if err := t.SetStallThreshold(t.Config.StallThreshold); err != nil {
			return err
		}
	}

	// Set TPOWERDOWN
	tpowerdown := t.Fields.SetField("tpowerdown", 20, nil, "TPOWERDOWN")
	if err := t.SetRegister("TPOWERDOWN", tpowerdown); err != nil {
		return err
	}

	return nil
}

// DumpRegisters returns all register values for debugging.
func (t *TMC2209) DumpRegisters() map[string]uint32 {
	regs := make(map[string]uint32)
	for name := range TMC2209RegAddrs {
		if val, err := t.GetRegister(name); err == nil {
			regs[name] = val
		}
	}
	return regs
}
