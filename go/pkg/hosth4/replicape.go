// Replicape - port of klippy/extras/replicape.py
//
// Support for Replicape board (BeagleBone Black based 3D printer controller)
//
// Copyright (C) 2017-2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2026 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// Replicape constants
const (
	replicapeMaxCurrent        = 3.84
	replicapePCA9685Bus        = 2
	replicapePCA9685Address    = 0x70
	replicapePCA9685CycleTime  = 0.001
	replicapePinMinTime        = 0.100
)

// Replicape stepper microstep modes
var replicapeStepConfig = map[string]*int{
	"disable":   nil,
	"1":         intPtr((1 << 7) | (1 << 5)),
	"2":         intPtr((1 << 7) | (1 << 5) | (1 << 6)),
	"spread2":   intPtr(1 << 5),
	"4":         intPtr((1 << 7) | (1 << 5) | (1 << 4)),
	"16":        intPtr((1 << 7) | (1 << 5) | (1 << 6) | (1 << 4)),
	"spread4":   intPtr((1 << 5) | (1 << 4)),
	"spread16":  intPtr(1 << 7),
	"stealth4":  intPtr((1 << 7) | (1 << 6)),
	"stealth16": intPtr(0),
}

func intPtr(v int) *int {
	return &v
}

// ReplicapeServoPins maps servo names to their PWM and GPIO pins
var replicapeServoPins = map[string][3]string{
	"servo0": {"/pwm0", "gpio0_30", "gpio1_18"}, // P9_11, P9_14
	"servo1": {"/pwm1", "gpio3_17", "gpio1_19"}, // P9_28, P9_16
}

// ReplicapePowerPins maps power pin names to PCA9685 channels
var replicapePowerPins = map[string]int{
	"power_e":      5,
	"power_h":      3,
	"power_hotbed": 4,
	"power_fan0":   7,
	"power_fan1":   8,
	"power_fan2":   9,
	"power_fan3":   10,
}

// Replicape represents a Replicape 3D printer control board.
// The Replicape is a BeagleBone Black cape that provides:
// - 5 stepper motor drivers (TMC2100)
// - Heater/fan PWM via PCA9685
// - Servo support via hardware PWM
// - Shift register for stepper configuration
type Replicape struct {
	name     string
	revision string
	hostMCU  string

	// Enable pin configuration
	enablePin string

	// Stepper configurations
	stepperDacs map[int]float64 // channel -> current ratio

	// Shift register state
	srEnabled  []int
	srDisabled []int

	// PWM enable state
	pwmStartValue    bool
	pwmShutdownValue bool
	enabledChannels  map[int]bool
	lastPWMEnableTime float64

	// Servo pin state
	servoPins map[string]int

	mu sync.Mutex
}

// ReplicapeConfig holds configuration for Replicape
type ReplicapeConfig struct {
	Name      string
	Revision  string // "B3" supported
	HostMCU   string
	EnablePin string // Default "!gpio0_20"

	// Stepper configurations (x, y, z, e, h)
	StepperXMicrostepMode string
	StepperYMicrostepMode string
	StepperZMicrostepMode string
	StepperEMicrostepMode string
	StepperHMicrostepMode string

	StepperXCurrent float64
	StepperYCurrent float64
	StepperZCurrent float64
	StepperECurrent float64
	StepperHCurrent float64

	StepperXChopperOffTimeHigh      bool
	StepperYChopperOffTimeHigh      bool
	StepperZChopperOffTimeHigh      bool
	StepperEChopperOffTimeHigh      bool
	StepperHChopperOffTimeHigh      bool

	StepperXChopperHysteresisHigh   bool
	StepperYChopperHysteresisHigh   bool
	StepperZChopperHysteresisHigh   bool
	StepperEChopperHysteresisHigh   bool
	StepperHChopperHysteresisHigh   bool

	StepperXChopperBlankTimeHigh    bool
	StepperYChopperBlankTimeHigh    bool
	StepperZChopperBlankTimeHigh    bool
	StepperEChopperBlankTimeHigh    bool
	StepperHChopperBlankTimeHigh    bool

	StandstillPowerDown bool
}

// DefaultReplicapeConfig returns default configuration
func DefaultReplicapeConfig() ReplicapeConfig {
	return ReplicapeConfig{
		Revision:  "B3",
		EnablePin: "!gpio0_20",
		StepperXChopperBlankTimeHigh: true,
		StepperYChopperBlankTimeHigh: true,
		StepperZChopperBlankTimeHigh: true,
		StepperEChopperBlankTimeHigh: true,
		StepperHChopperBlankTimeHigh: true,
	}
}

// NewReplicape creates a new Replicape instance
func NewReplicape(cfg ReplicapeConfig) (*Replicape, error) {
	if cfg.Revision != "B3" {
		return nil, fmt.Errorf("replicape: only revision B3 is supported")
	}

	r := &Replicape{
		name:            cfg.Name,
		revision:        cfg.Revision,
		hostMCU:         cfg.HostMCU,
		enablePin:       cfg.EnablePin,
		stepperDacs:     make(map[int]float64),
		enabledChannels: make(map[int]bool),
		servoPins: map[string]int{
			"servo0": 3,
			"servo1": 2,
		},
	}

	// Initialize shift registers
	shiftRegisters := []int{1, 0, 0, 1, 1}

	// Process stepper configurations
	stepperConfigs := []struct {
		port       int
		name       string
		mode       string
		current    float64
		offTime    bool
		hysteresis bool
		blankTime  bool
	}{
		{0, "x", cfg.StepperXMicrostepMode, cfg.StepperXCurrent, cfg.StepperXChopperOffTimeHigh, cfg.StepperXChopperHysteresisHigh, cfg.StepperXChopperBlankTimeHigh},
		{1, "y", cfg.StepperYMicrostepMode, cfg.StepperYCurrent, cfg.StepperYChopperOffTimeHigh, cfg.StepperYChopperHysteresisHigh, cfg.StepperYChopperBlankTimeHigh},
		{2, "z", cfg.StepperZMicrostepMode, cfg.StepperZCurrent, cfg.StepperZChopperOffTimeHigh, cfg.StepperZChopperHysteresisHigh, cfg.StepperZChopperBlankTimeHigh},
		{3, "e", cfg.StepperEMicrostepMode, cfg.StepperECurrent, cfg.StepperEChopperOffTimeHigh, cfg.StepperEChopperHysteresisHigh, cfg.StepperEChopperBlankTimeHigh},
		{4, "h", cfg.StepperHMicrostepMode, cfg.StepperHCurrent, cfg.StepperHChopperOffTimeHigh, cfg.StepperHChopperHysteresisHigh, cfg.StepperHChopperBlankTimeHigh},
	}

	for _, sc := range stepperConfigs {
		if sc.mode == "" || sc.mode == "disable" {
			continue
		}

		stepCfg, ok := replicapeStepConfig[sc.mode]
		if !ok || stepCfg == nil {
			continue
		}

		srVal := *stepCfg | shiftRegisters[sc.port]

		if sc.offTime {
			srVal |= 1 << 3
		}
		if sc.hysteresis {
			srVal |= 1 << 2
		}
		if sc.blankTime {
			srVal |= 1 << 1
		}

		shiftRegisters[sc.port] = srVal

		// Configure DAC channel
		channel := sc.port + 11
		if sc.current > 0 && sc.current <= replicapeMaxCurrent {
			r.stepperDacs[channel] = sc.current / replicapeMaxCurrent
		}
	}

	// Copy to disabled state
	r.srDisabled = make([]int, len(shiftRegisters))
	copy(r.srDisabled, shiftRegisters)

	// Reverse for shift register order
	for i, j := 0, len(r.srDisabled)-1; i < j; i, j = i+1, j-1 {
		r.srDisabled[i], r.srDisabled[j] = r.srDisabled[j], r.srDisabled[i]
	}

	// Calculate enabled state
	r.srEnabled = make([]int, len(r.srDisabled))
	copy(r.srEnabled, r.srDisabled)

	// Check if xyz steppers are used
	hasXYZ := false
	for i := 0; i < 3; i++ {
		if _, ok := r.stepperDacs[11+i]; ok {
			hasXYZ = true
			break
		}
	}
	if hasXYZ {
		// Enable xyz steppers (clear bit)
		r.srEnabled[4] &= ^1
	}

	// Check if eh steppers are used
	hasEH := false
	for i := 3; i < 5; i++ {
		if _, ok := r.stepperDacs[11+i]; ok {
			hasEH = true
			break
		}
	}
	if hasEH {
		// Enable eh steppers (clear bit)
		r.srEnabled[1] &= ^1
	}

	// Standstill power down
	if cfg.StandstillPowerDown && len(r.stepperDacs) > 0 {
		r.srEnabled[0] &= ^1
	}

	// Initialize enabled channels for power pins
	for _, ch := range replicapePowerPins {
		r.enabledChannels[ch] = false
	}
	for ch := range r.stepperDacs {
		r.enabledChannels[ch] = false
	}

	log.Printf("replicape: initialized revision=%s host_mcu=%s enable_pin=%s steppers=%d",
		cfg.Revision, cfg.HostMCU, cfg.EnablePin, len(r.stepperDacs))

	return r, nil
}

// GetPCA9685Config returns PCA9685 configuration
func (r *Replicape) GetPCA9685Config() (bus, address int, cycleTime float64) {
	return replicapePCA9685Bus, replicapePCA9685Address, replicapePCA9685CycleTime
}

// GetStepperDACValue returns the DAC value for a stepper channel
func (r *Replicape) GetStepperDACValue(channel int) (float64, bool) {
	r.mu.Lock()
	defer r.mu.Unlock()
	val, ok := r.stepperDacs[channel]
	return val, ok
}

// GetShiftRegisterEnabled returns the enabled shift register state
func (r *Replicape) GetShiftRegisterEnabled() []int {
	r.mu.Lock()
	defer r.mu.Unlock()
	result := make([]int, len(r.srEnabled))
	copy(result, r.srEnabled)
	return result
}

// GetShiftRegisterDisabled returns the disabled shift register state
func (r *Replicape) GetShiftRegisterDisabled() []int {
	r.mu.Lock()
	defer r.mu.Unlock()
	result := make([]int, len(r.srDisabled))
	copy(result, r.srDisabled)
	return result
}

// NotePWMStartValue records the start/shutdown values for a PWM channel
func (r *Replicape) NotePWMStartValue(channel int, startValue, shutdownValue float64) {
	r.mu.Lock()
	defer r.mu.Unlock()

	r.pwmStartValue = r.pwmStartValue || (startValue != 0)
	r.pwmShutdownValue = r.pwmShutdownValue || (shutdownValue != 0)
	r.enabledChannels[channel] = startValue != 0
}

// NotePWMEnable records when a PWM channel is enabled/disabled
func (r *Replicape) NotePWMEnable(printTime float64, channel int, isEnable bool) (enablePWM bool, sendSR bool, srState []int) {
	r.mu.Lock()
	defer r.mu.Unlock()

	r.enabledChannels[channel] = isEnable

	// Check PCA9685 enable state
	peTime := printTime
	if peTime < r.lastPWMEnableTime+replicapePinMinTime {
		peTime = r.lastPWMEnableTime + replicapePinMinTime
	}

	onCount := 0
	for _, enabled := range r.enabledChannels {
		if enabled {
			onCount++
		}
	}

	if onCount == 0 {
		r.lastPWMEnableTime = peTime
		return false, false, nil
	} else if isEnable && onCount == 1 {
		r.lastPWMEnableTime = peTime
		return true, false, nil
	}

	// Check stepper enable state
	if _, ok := r.stepperDacs[channel]; !ok {
		return false, false, nil
	}

	dacOnCount := 0
	for ch := range r.stepperDacs {
		if r.enabledChannels[ch] {
			dacOnCount++
		}
	}

	if dacOnCount == 0 {
		return false, true, r.srDisabled
	} else if isEnable && dacOnCount == 1 {
		return false, true, r.srEnabled
	}

	return false, false, nil
}

// GetPowerPinChannel returns the PCA9685 channel for a power pin
func (r *Replicape) GetPowerPinChannel(pin string) (int, bool) {
	ch, ok := replicapePowerPins[pin]
	return ch, ok
}

// GetServoPinConfig returns the servo pin configuration
func (r *Replicape) GetServoPinConfig(pin string) (pwmPin, reservePin1, reservePin2 string, ok bool) {
	pins, ok := replicapeServoPins[pin]
	if !ok {
		return "", "", "", false
	}
	return pins[0], pins[1], pins[2], true
}

// EnableServoPin enables a servo pin in the shift registers
func (r *Replicape) EnableServoPin(pin string) bool {
	r.mu.Lock()
	defer r.mu.Unlock()

	index, ok := r.servoPins[pin]
	if !ok {
		return false
	}

	r.srEnabled[index] |= 1
	r.srDisabled[index] |= 1

	return true
}

// GetStatus returns the Replicape status
func (r *Replicape) GetStatus() map[string]interface{} {
	r.mu.Lock()
	defer r.mu.Unlock()

	steppers := make(map[string]float64)
	for ch, val := range r.stepperDacs {
		steppers[fmt.Sprintf("channel_%d", ch)] = val * replicapeMaxCurrent
	}

	return map[string]interface{}{
		"name":          r.name,
		"revision":      r.revision,
		"host_mcu":      r.hostMCU,
		"enable_pin":    r.enablePin,
		"stepper_count": len(r.stepperDacs),
		"steppers":      steppers,
	}
}

// ReplicapePCA9685PWM represents a PWM output on the Replicape's PCA9685.
// This provides heater and fan control via I2C PWM.
type ReplicapePCA9685PWM struct {
	replicape     *Replicape
	channel       int
	mcuName       string
	invert        bool
	startValue    float64
	shutdownValue float64
	maxDuration   float64
	cycleTime     float64
}

// ReplicapePCA9685PWMConfig holds configuration for a PCA9685 PWM channel
type ReplicapePCA9685PWMConfig struct {
	Channel int
	Invert  bool
}

// NewReplicapePCA9685PWM creates a new PCA9685 PWM output on Replicape
func NewReplicapePCA9685PWM(r *Replicape, cfg ReplicapePCA9685PWMConfig) *ReplicapePCA9685PWM {
	pwm := &ReplicapePCA9685PWM{
		replicape:   r,
		channel:     cfg.Channel,
		mcuName:     r.hostMCU,
		invert:      cfg.Invert,
		maxDuration: 2.0,
		cycleTime:   replicapePCA9685CycleTime,
	}

	if cfg.Invert {
		pwm.startValue = 1.0
		pwm.shutdownValue = 1.0
	}

	log.Printf("replicape: pca9685_pwm channel=%d invert=%v", cfg.Channel, cfg.Invert)

	return pwm
}

// GetMCUName returns the MCU name
func (pwm *ReplicapePCA9685PWM) GetMCUName() string {
	return pwm.mcuName
}

// SetupMaxDuration sets the max duration
func (pwm *ReplicapePCA9685PWM) SetupMaxDuration(maxDuration float64) {
	pwm.maxDuration = maxDuration
}

// SetupCycleTime sets the PWM cycle time
func (pwm *ReplicapePCA9685PWM) SetupCycleTime(cycleTime float64, hardwarePWM bool) {
	if hardwarePWM {
		log.Printf("replicape: pca9685 does not support hardware_pwm parameter")
	}
	if cycleTime != pwm.cycleTime {
		log.Printf("replicape: ignoring pca9685 cycle time of %.6f (using %.6f)",
			cycleTime, pwm.cycleTime)
	}
}

// SetupStartValue sets start and shutdown values
func (pwm *ReplicapePCA9685PWM) SetupStartValue(startValue, shutdownValue float64) {
	if pwm.invert {
		startValue = 1.0 - startValue
		shutdownValue = 1.0 - shutdownValue
	}
	pwm.startValue = replicapeClampFloat(startValue, 0, 1)
	pwm.shutdownValue = replicapeClampFloat(shutdownValue, 0, 1)
	pwm.replicape.NotePWMStartValue(pwm.channel, pwm.startValue, pwm.shutdownValue)
}

// ReplicapeDACEnable represents a virtual enable pin that uses the stepper DAC
type ReplicapeDACEnable struct {
	replicape *Replicape
	channel   int
	dacValue  float64
	pwm       *ReplicapePCA9685PWM
}

// NewReplicapeDACEnable creates a new DAC enable pin
func NewReplicapeDACEnable(r *Replicape, channel int, invert bool) (*ReplicapeDACEnable, error) {
	if invert {
		return nil, fmt.Errorf("replicape: virtual enable pin cannot be inverted")
	}

	dacValue, ok := r.GetStepperDACValue(channel)
	if !ok {
		return nil, fmt.Errorf("replicape: no DAC configured for channel %d", channel)
	}

	de := &ReplicapeDACEnable{
		replicape: r,
		channel:   channel,
		dacValue:  dacValue,
		pwm:       NewReplicapePCA9685PWM(r, ReplicapePCA9685PWMConfig{Channel: channel, Invert: false}),
	}

	log.Printf("replicape: dac_enable channel=%d value=%.3f", channel, dacValue)

	return de, nil
}

// GetMCUName returns the MCU name
func (de *ReplicapeDACEnable) GetMCUName() string {
	return de.replicape.hostMCU
}

// SetupMaxDuration sets the max duration
func (de *ReplicapeDACEnable) SetupMaxDuration(maxDuration float64) {
	de.pwm.SetupMaxDuration(maxDuration)
}

// replicapeClampFloat clamps a float value between min and max
func replicapeClampFloat(v, min, max float64) float64 {
	if v < min {
		return min
	}
	if v > max {
		return max
	}
	return v
}
