// output_pin implements PWM and digital output pin control.
// This supports [output_pin], [pwm_cycle_time], and [pwm_tool] sections.
package hosth4

import (
	"fmt"
	"math"
	"strings"
)

// outputPin represents a PWM or digital output pin.
type outputPin struct {
	rt          *runtime
	name        string
	pin         string
	isPWM       bool
	hardwarePWM bool
	cycleTime   float64
	scale       float64
	value       float64 // Current value (0.0-1.0)
	shutdownVal float64
	out         *digitalOut // PWM output for sending commands to MCU
	cycleTicks  uint64      // PWM cycle in MCU ticks
	mcuFreq     float64     // MCU frequency for time conversion
}

// outputPinConfig holds configuration for an output pin.
type outputPinConfig struct {
	pin         string
	isPWM       bool
	hardwarePWM bool
	cycleTime   float64
	scale       float64
	value       float64
	shutdownVal float64
}

// outputPinManager manages all output pins in the system.
type outputPinManager struct {
	rt   *runtime
	pins map[string]*outputPin // keyed by pin name (without "output_pin " prefix)
}

// newOutputPinManager creates an output pin manager from configuration.
func newOutputPinManager(rt *runtime, cfg *configWrapper) (*outputPinManager, error) {
	opm := &outputPinManager{
		rt:   rt,
		pins: make(map[string]*outputPin),
	}

	// Load [output_pin xxx] sections
	for _, secName := range cfg.sectionsByPrefix("output_pin ") {
		pinName := strings.TrimPrefix(secName, "output_pin ")
		pin, err := newOutputPin(rt, cfg, secName, pinName)
		if err != nil {
			return nil, err
		}
		if pin != nil {
			opm.pins[pinName] = pin
		}
	}

	// Load [pwm_cycle_time xxx] sections (similar to output_pin but always PWM)
	for _, secName := range cfg.sectionsByPrefix("pwm_cycle_time ") {
		pinName := strings.TrimPrefix(secName, "pwm_cycle_time ")
		pin, err := newPWMCycleTimePin(rt, cfg, secName, pinName)
		if err != nil {
			return nil, err
		}
		if pin != nil {
			opm.pins[pinName] = pin
		}
	}

	// Load [pwm_tool xxx] sections
	for _, secName := range cfg.sectionsByPrefix("pwm_tool ") {
		pinName := strings.TrimPrefix(secName, "pwm_tool ")
		pin, err := newPWMToolPin(rt, cfg, secName, pinName)
		if err != nil {
			return nil, err
		}
		if pin != nil {
			opm.pins[pinName] = pin
		}
	}

	return opm, nil
}

// newOutputPin creates a new output pin from configuration.
func newOutputPin(rt *runtime, cfg *configWrapper, sectionName, pinName string) (*outputPin, error) {
	sec, ok := cfg.section(sectionName)
	if !ok {
		return nil, nil // not configured
	}

	// Parse pin (required)
	pin, ok := sec["pin"]
	if !ok {
		return nil, fmt.Errorf("%s: missing pin", sectionName)
	}

	// Parse pwm setting
	isPWM := false
	if b, err := parseBool(sec, "pwm", &isPWM); err == nil {
		isPWM = b
	}

	// Parse hardware_pwm
	hardwarePWM := false
	if b, err := parseBool(sec, "hardware_pwm", &hardwarePWM); err == nil {
		hardwarePWM = b
	}

	// Parse cycle_time (default 0.1 for soft PWM, varies for hardware)
	cycleTime := 0.100
	if s, ok := sec["cycle_time"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &cycleTime); err != nil {
			return nil, fmt.Errorf("%s: invalid cycle_time: %s", sectionName, s)
		}
	}

	// Parse scale
	scale := 1.0
	if s, ok := sec["scale"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &scale); err != nil {
			return nil, fmt.Errorf("%s: invalid scale: %s", sectionName, s)
		}
	}

	// Parse value (initial value)
	value := 0.0
	if s, ok := sec["value"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &value); err != nil {
			return nil, fmt.Errorf("%s: invalid value: %s", sectionName, s)
		}
		value /= scale // Normalize to 0-1 range
	}

	// Parse shutdown_value
	shutdownVal := 0.0
	if s, ok := sec["shutdown_value"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &shutdownVal); err != nil {
			return nil, fmt.Errorf("%s: invalid shutdown_value: %s", sectionName, s)
		}
		shutdownVal /= scale // Normalize to 0-1 range
	}

	return &outputPin{
		rt:          rt,
		name:        pinName,
		pin:         pin,
		isPWM:       isPWM,
		hardwarePWM: hardwarePWM,
		cycleTime:   cycleTime,
		scale:       scale,
		value:       value,
		shutdownVal: shutdownVal,
	}, nil
}

// newPWMCycleTimePin creates a pwm_cycle_time pin (always PWM).
func newPWMCycleTimePin(rt *runtime, cfg *configWrapper, sectionName, pinName string) (*outputPin, error) {
	sec, ok := cfg.section(sectionName)
	if !ok {
		return nil, nil
	}

	pin, ok := sec["pin"]
	if !ok {
		return nil, fmt.Errorf("%s: missing pin", sectionName)
	}

	cycleTime := 0.100
	if s, ok := sec["cycle_time"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &cycleTime); err != nil {
			return nil, fmt.Errorf("%s: invalid cycle_time: %s", sectionName, s)
		}
	}

	value := 0.0
	if s, ok := sec["value"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &value); err != nil {
			return nil, fmt.Errorf("%s: invalid value: %s", sectionName, s)
		}
	}

	shutdownVal := 0.0
	if s, ok := sec["shutdown_value"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &shutdownVal); err != nil {
			return nil, fmt.Errorf("%s: invalid shutdown_value: %s", sectionName, s)
		}
	}

	return &outputPin{
		rt:          rt,
		name:        pinName,
		pin:         pin,
		isPWM:       true,
		hardwarePWM: false,
		cycleTime:   cycleTime,
		scale:       1.0,
		value:       value,
		shutdownVal: shutdownVal,
	}, nil
}

// newPWMToolPin creates a pwm_tool pin (always PWM, for tool control).
func newPWMToolPin(rt *runtime, cfg *configWrapper, sectionName, pinName string) (*outputPin, error) {
	sec, ok := cfg.section(sectionName)
	if !ok {
		return nil, nil
	}

	pin, ok := sec["pin"]
	if !ok {
		return nil, fmt.Errorf("%s: missing pin", sectionName)
	}

	cycleTime := 0.100
	if s, ok := sec["cycle_time"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &cycleTime); err != nil {
			return nil, fmt.Errorf("%s: invalid cycle_time: %s", sectionName, s)
		}
	}

	value := 0.0
	if s, ok := sec["value"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &value); err != nil {
			return nil, fmt.Errorf("%s: invalid value: %s", sectionName, s)
		}
	}

	shutdownVal := 0.0
	if s, ok := sec["shutdown_value"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &shutdownVal); err != nil {
			return nil, fmt.Errorf("%s: invalid shutdown_value: %s", sectionName, s)
		}
	}

	return &outputPin{
		rt:          rt,
		name:        pinName,
		pin:         pin,
		isPWM:       true,
		hardwarePWM: false,
		cycleTime:   cycleTime,
		scale:       1.0,
		value:       value,
		shutdownVal: shutdownVal,
	}, nil
}

// setValue sets the pin value (0.0-scale).
func (op *outputPin) setValue(printTime float64, value float64) error {
	// Normalize value to 0-1 range
	normalizedValue := value / op.scale
	normalizedValue = math.Max(0.0, math.Min(1.0, normalizedValue))

	// For digital pins, only allow 0 or 1
	if !op.isPWM && normalizedValue != 0.0 && normalizedValue != 1.0 {
		return fmt.Errorf("invalid pin value for digital output")
	}

	// Check if value changed
	if normalizedValue == op.value {
		return nil // No change
	}

	op.value = normalizedValue

	// Send PWM command to MCU if output is configured
	if op.out != nil && op.cycleTicks > 0 {
		onTicks := uint32(normalizedValue*float64(op.cycleTicks) + 0.5)
		if err := op.out.setOnTicks(printTime, onTicks); err != nil {
			return err
		}
	}

	op.rt.tracef("SET_PIN %s: value=%.4f at time %.3f\n", op.name, normalizedValue, printTime)
	return nil
}

// getStatus returns the current status of the pin.
func (op *outputPin) getStatus() map[string]interface{} {
	return map[string]interface{}{
		"value": op.value * op.scale,
	}
}

// cmdSetPin handles the SET_PIN command.
// SET_PIN PIN=<name> VALUE=<value>
func (opm *outputPinManager) cmdSetPin(args map[string]string, printTime float64) error {
	pinName, ok := args["PIN"]
	if !ok {
		return fmt.Errorf("SET_PIN requires PIN parameter")
	}

	pin, ok := opm.pins[pinName]
	if !ok {
		return fmt.Errorf("unknown pin: %s", pinName)
	}

	// Parse value
	valueStr, ok := args["VALUE"]
	if !ok {
		return fmt.Errorf("SET_PIN requires VALUE parameter")
	}

	var value float64
	if _, err := fmt.Sscanf(valueStr, "%f", &value); err != nil {
		return fmt.Errorf("invalid VALUE parameter: %s", valueStr)
	}

	return pin.setValue(printTime, value)
}

