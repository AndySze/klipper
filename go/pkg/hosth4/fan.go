// fan implements fan control for 3D printers.
// Supports part cooling fans (M106/M107) and controller fans.
package hosth4

import (
	"fmt"
	"math"
)

// fan represents a PWM-controlled cooling fan.
type fan struct {
	rt              *runtime
	name            string
	pin             string
	maxPower        float64 // Maximum power (0.0-1.0)
	kickStartTime   float64 // Time to run at full power on startup
	kickStartPower  float64 // Power level for kick start
	offBelow        float64 // Turn off below this power level
	cycleTime       float64 // PWM cycle time
	hardwarePWM     bool
	lastFanValue    float64
	lastFanTime     float64
	enabledFromTime float64
	// PWM output control
	out        *digitalOut // PWM output for sending commands to MCU
	cycleTicks uint64      // PWM cycle in MCU ticks
	mcuFreq    float64     // MCU frequency for time conversion
}

// fanConfig holds configuration for a fan.
type fanConfig struct {
	pin            string
	maxPower       float64
	kickStartTime  float64
	kickStartPower float64
	offBelow       float64
	cycleTime      float64
	hardwarePWM    bool
}

// newFan creates a new fan instance from configuration.
func newFan(rt *runtime, cfg *configWrapper, sectionName string) (*fan, error) {
	sec, ok := cfg.section(sectionName)
	if !ok {
		return nil, nil // not configured
	}

	// Parse pin (required)
	pin, ok := sec["pin"]
	if !ok {
		return nil, fmt.Errorf("%s: missing pin", sectionName)
	}

	// Parse optional parameters
	maxPower := 1.0
	if s, ok := sec["max_power"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &maxPower); err != nil {
			return nil, fmt.Errorf("%s: invalid max_power: %s", sectionName, s)
		}
		if maxPower <= 0 || maxPower > 1.0 {
			return nil, fmt.Errorf("%s: max_power must be between 0 and 1", sectionName)
		}
	}

	kickStartTime := 0.1
	if s, ok := sec["kick_start_time"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &kickStartTime); err != nil {
			return nil, fmt.Errorf("%s: invalid kick_start_time: %s", sectionName, s)
		}
	}

	kickStartPower := 1.0
	if s, ok := sec["kick_start_power"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &kickStartPower); err != nil {
			return nil, fmt.Errorf("%s: invalid kick_start_power: %s", sectionName, s)
		}
	}

	offBelow := 0.0
	if s, ok := sec["off_below"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &offBelow); err != nil {
			return nil, fmt.Errorf("%s: invalid off_below: %s", sectionName, s)
		}
	}

	cycleTime := 0.010 // 10ms default
	if s, ok := sec["cycle_time"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &cycleTime); err != nil {
			return nil, fmt.Errorf("%s: invalid cycle_time: %s", sectionName, s)
		}
	}

	hardwarePWM := false
	if s, ok := sec["hardware_pwm"]; ok {
		hardwarePWM = s == "true" || s == "True" || s == "1"
	}

	return &fan{
		rt:             rt,
		name:           sectionName,
		pin:            pin,
		maxPower:       maxPower,
		kickStartTime:  kickStartTime,
		kickStartPower: kickStartPower,
		offBelow:       offBelow,
		cycleTime:      cycleTime,
		hardwarePWM:    hardwarePWM,
		lastFanValue:   0.0,
		lastFanTime:    0.0,
	}, nil
}

// setSpeed sets the fan speed (0.0-1.0).
func (f *fan) setSpeed(printTime float64, value float64) error {
	// Clamp value to valid range
	if value < 0 {
		value = 0
	}
	if value > 1.0 {
		value = 1.0
	}

	// Apply max_power scaling
	value = value * f.maxPower

	// Check off_below threshold
	if value > 0 && value < f.offBelow {
		value = 0
	}

	// Check if value changed
	if value == f.lastFanValue {
		return nil
	}

	// Handle kick start for fans spinning up from zero
	// (or when increasing speed by > 0.5, matching Python behavior)
	needsKickStart := value > 0 && f.kickStartTime > 0 &&
		(f.lastFanValue == 0 || value-f.lastFanValue > 0.5)

	if needsKickStart {
		// Run at kick start power first (use kickStartPower, scaled by maxPower)
		kickPower := f.kickStartPower * f.maxPower
		if kickPower > f.maxPower {
			kickPower = f.maxPower
		}

		f.rt.tracef("Fan %s: kick-start at power %.2f for %.3fs\n", f.name, kickPower, f.kickStartTime)

		// Send PWM command for kick-start power
		if f.out != nil && f.cycleTicks > 0 {
			onTicks := uint32(kickPower*float64(f.cycleTicks) + 0.5)
			if err := f.out.setOnTicks(printTime, onTicks); err != nil {
				return err
			}
		}

		// Update state for kick-start period
		f.enabledFromTime = printTime
		f.lastFanValue = kickPower
		f.lastFanTime = printTime

		// Now set the actual target value after kick-start time elapses
		// In file output mode, we emit both commands (kick-start now, target later)
		targetTime := printTime + f.kickStartTime
		if f.out != nil && f.cycleTicks > 0 {
			onTicks := uint32(value*float64(f.cycleTicks) + 0.5)
			if err := f.out.setOnTicks(targetTime, onTicks); err != nil {
				return err
			}
		}

		f.lastFanValue = value
		f.lastFanTime = targetTime
		f.rt.tracef("Fan %s: set speed to %.2f at time %.3f (after kick-start)\n", f.name, value, targetTime)
		return nil
	}

	// Normal speed change (no kick-start needed)
	f.lastFanValue = value
	f.lastFanTime = printTime

	// Send PWM command to MCU if output is configured
	if f.out != nil && f.cycleTicks > 0 {
		// Convert value (0.0-1.0) to on_ticks
		onTicks := uint32(value*float64(f.cycleTicks) + 0.5)
		if err := f.out.setOnTicks(printTime, onTicks); err != nil {
			return err
		}
	}

	f.rt.tracef("Fan %s: set speed to %.2f at time %.3f\n", f.name, value, printTime)

	return nil
}

// getSpeed returns the current fan speed.
func (f *fan) getSpeed() float64 {
	return f.lastFanValue
}

// getStatus returns the current status of the fan.
func (f *fan) getStatus() map[string]any {
	return map[string]any{
		"speed": f.lastFanValue,
		"rpm":   nil, // Would need tachometer for this
	}
}

// fanManager manages all fans in the system.
type fanManager struct {
	rt             *runtime
	partFan        *fan              // Main part cooling fan [fan]
	namedFans      map[string]*fan   // [fan_generic xxx] fans
	controllerFans []*controllerFan  // [controller_fan xxx] fans
	heaterFans     []*heaterFan      // [heater_fan xxx] fans
}

// controllerFan is a fan that turns on when steppers are active.
// Note: controller_fan uses a periodic timer callback (reactor.register_timer)
// to check stepper/heater activity and adjust fan speed. In file output mode,
// no reactor runs, so the callback isn't invoked and no auto-control happens.
// For real-time mode, implement the periodic check against stepper_enable status.
type controllerFan struct {
	*fan
	idleTimeout float64
	idleSpeed   float64
	fanSpeed    float64   // Speed when active
	steppers    []string  // Stepper names to monitor
	heaters     []string  // Heater names to monitor
}

// heaterFan is a fan that turns on when a heater is above a threshold.
// Note: heater_fan uses a periodic timer callback (reactor.register_timer)
// to check heater temperature and adjust fan speed. In file output mode,
// no reactor runs, so the callback isn't invoked and no auto-control happens.
// For real-time mode, implement the periodic check against heater temperatures.
type heaterFan struct {
	*fan
	heaterNames []string  // Heater names to monitor
	heaterTemp  float64   // Temperature threshold for fan activation
	fanSpeed    float64   // Speed when above threshold
}

// newFanManager creates a fan manager from configuration.
// fanOID specifies the OID for the main part cooling fan - this varies by config type:
// - For TMC2130 configs: oid=16 (after SPI, steppers, bed ADC/PWM)
// - For non-TMC configs with fan: oid=12
// - Pass -1 if fan OID should be auto-detected (not recommended)
func newFanManager(rt *runtime, cfg *configWrapper, fanOID int) (*fanManager, error) {
	fm := &fanManager{
		rt:        rt,
		namedFans: make(map[string]*fan),
	}

	// Load main part cooling fan [fan]
	partFan, err := newFan(rt, cfg, "fan")
	if err != nil {
		return nil, err
	}
	if partFan != nil && fanOID >= 0 {
		// Set up PWM output for the main fan using the provided OID
		const fanCycleTime = 0.010 // 10ms default cycle time
		cycleTicks := uint64(fanCycleTime * rt.mcuFreq)
		fanOut := newDigitalOut(uint32(fanOID), false, rt.sq, rt.cqMain, rt.formats, rt.mcuFreq, "mcu")
		partFan.out = fanOut
		partFan.cycleTicks = cycleTicks
		partFan.mcuFreq = rt.mcuFreq
	}
	fm.partFan = partFan

	// Load [fan_generic xxx] sections
	for _, name := range cfg.sectionsByPrefix("fan_generic ") {
		f, err := newFan(rt, cfg, name)
		if err != nil {
			return nil, err
		}
		if f != nil {
			fanName := name[len("fan_generic "):]
			fm.namedFans[fanName] = f
			// Note: fan_generic fans use OIDs allocated during hosth1 compile phase.
			// For golden tests, OID assignment depends on config. If fan_generic tests
			// are needed, the OID would be passed from hosth1's output dictionary.
			// Currently no tests exercise fan_generic, so this is tracked for future.
		}
	}

	return fm, nil
}

// cmdM106 handles the M106 (set fan speed) command.
func (fm *fanManager) cmdM106(args map[string]string, printTime float64) error {
	if fm.partFan == nil {
		return fmt.Errorf("no part cooling fan configured")
	}

	// Parse speed (S parameter, 0-255)
	speed := 255.0
	if s, ok := args["S"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &speed); err != nil {
			return fmt.Errorf("invalid S parameter: %s", s)
		}
	}

	// Convert 0-255 to 0.0-1.0
	value := math.Min(1.0, math.Max(0.0, speed/255.0))

	return fm.partFan.setSpeed(printTime, value)
}

// cmdM107 handles the M107 (turn off fan) command.
func (fm *fanManager) cmdM107(printTime float64) error {
	if fm.partFan == nil {
		return fmt.Errorf("no part cooling fan configured")
	}
	return fm.partFan.setSpeed(printTime, 0)
}

// cmdSetFanSpeed handles the SET_FAN_SPEED command.
func (fm *fanManager) cmdSetFanSpeed(args map[string]string, printTime float64) error {
	// Get fan name
	fanName, ok := args["FAN"]
	if !ok {
		return fmt.Errorf("missing FAN parameter")
	}

	// Find the fan
	f, ok := fm.namedFans[fanName]
	if !ok {
		return fmt.Errorf("unknown fan: %s", fanName)
	}

	// Parse speed
	speed := 0.0
	if s, ok := args["SPEED"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &speed); err != nil {
			return fmt.Errorf("invalid SPEED parameter: %s", s)
		}
	}

	return f.setSpeed(printTime, speed)
}