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
func newFan(rt *runtime, cfg *config, sectionName string) (*fan, error) {
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

	// Handle kick start for fans spinning up
	if value > 0 && f.lastFanValue == 0 && f.kickStartTime > 0 {
		// Run at kick start power first
		f.enabledFromTime = printTime
		// In real implementation, would schedule a callback to reduce power
	}

	f.lastFanValue = value
	f.lastFanTime = printTime

	// In real implementation, would send PWM command to MCU
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
type controllerFan struct {
	*fan
	idleTimeout float64
	idleSpeed   float64
}

// heaterFan is a fan that turns on when a heater is above a threshold.
type heaterFan struct {
	*fan
	heaterName string
	heaterTemp float64
}

// newFanManager creates a fan manager from configuration.
func newFanManager(rt *runtime, cfg *config) (*fanManager, error) {
	fm := &fanManager{
		rt:        rt,
		namedFans: make(map[string]*fan),
	}

	// Load main part cooling fan [fan]
	partFan, err := newFan(rt, cfg, "fan")
	if err != nil {
		return nil, err
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