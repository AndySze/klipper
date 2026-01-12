package hosth4

import (
	"fmt"
	"strconv"
	"strings"

	"klipper-go-migration/pkg/config"
)

// pin represents a parsed pin specification (local type for hosth4).
type pin struct {
	chip   string // MCU chip name (default: "mcu")
	pin    string // Pin name on the MCU
	invert bool
	pullup int
}

// fullName returns the full pin name including chip prefix if not "mcu".
func (p pin) fullName() string {
	if p.chip != "" && p.chip != "mcu" {
		return p.chip + ":" + p.pin
	}
	return p.pin
}

// =============================================================================
// Backward Compatibility Layer
// =============================================================================
// These types and functions provide backward compatibility with the old config
// API until all code is migrated to use the new config package directly.

// configWrapper wraps config.Config to provide the old API.
// This allows gradual migration of code that uses cfg.section() pattern.
type configWrapper struct {
	*config.Config
	sections map[string]map[string]string // cached raw sections for old API
}

// loadConfig loads a configuration file and returns a configWrapper.
func loadConfig(path string) (*configWrapper, error) {
	cfg, err := config.Load(path)
	if err != nil {
		return nil, err
	}
	// Build sections map for backward compatibility
	sections := make(map[string]map[string]string)
	for _, sec := range cfg.GetSections() {
		sections[sec.GetName()] = sec.RawOptions()
	}
	return &configWrapper{Config: cfg, sections: sections}, nil
}

// section returns a section as a raw map (old API).
func (c *configWrapper) section(name string) (map[string]string, bool) {
	sec, ok := c.sections[name]
	return sec, ok
}

// sectionsByPrefix returns all section names that start with the given prefix.
func (c *configWrapper) sectionsByPrefix(prefix string) []string {
	return c.GetPrefixSectionNames(prefix)
}

// parseFloat parses a float from a raw section map (old API).
func parseFloat(sec map[string]string, key string, def *float64) (float64, error) {
	raw := strings.TrimSpace(sec[key])
	if raw == "" {
		if def == nil {
			return 0, fmt.Errorf("missing %s", key)
		}
		return *def, nil
	}
	v, err := strconv.ParseFloat(raw, 64)
	if err != nil {
		return 0, fmt.Errorf("bad float %s=%q", key, raw)
	}
	return v, nil
}

// parseInt parses an integer from a raw section map (old API).
func parseInt(sec map[string]string, key string, def *int) (int, error) {
	raw := strings.TrimSpace(sec[key])
	if raw == "" {
		if def == nil {
			return 0, fmt.Errorf("missing %s", key)
		}
		return *def, nil
	}
	v, err := strconv.Atoi(raw)
	if err != nil {
		return 0, fmt.Errorf("bad int %s=%q", key, raw)
	}
	return v, nil
}

// parseBool parses a boolean from a raw section map (old API).
func parseBool(sec map[string]string, key string, def *bool) (bool, error) {
	raw := strings.TrimSpace(sec[key])
	if raw == "" {
		if def == nil {
			return false, fmt.Errorf("missing %s", key)
		}
		return *def, nil
	}
	switch strings.ToLower(raw) {
	case "1", "true", "yes", "on":
		return true, nil
	case "0", "false", "no", "off":
		return false, nil
	default:
		return false, fmt.Errorf("bad bool %s=%q", key, raw)
	}
}

// parsePinDesc parses a pin description string (old API).
func parsePinDesc(desc string, canInvert bool, canPullup bool) (pin, error) {
	p, err := config.ParsePin(desc, config.PinOptions{CanInvert: canInvert, CanPullup: canPullup})
	if err != nil {
		return pin{}, err
	}
	return pinFromConfig(p), nil
}

// parsePin parses a pin from a raw section map (old API).
func parsePin(sec map[string]string, key string, canInvert bool, canPullup bool) (pin, error) {
	raw := strings.TrimSpace(sec[key])
	if raw == "" {
		return pin{}, fmt.Errorf("missing %s", key)
	}
	return parsePinDesc(raw, canInvert, canPullup)
}

// =============================================================================
// New API helpers
// =============================================================================

// pinFromConfig converts a config.Pin to the local pin type.
func pinFromConfig(p config.Pin) pin {
	chip := p.Chip
	if chip == "" {
		chip = "mcu"
	}
	return pin{
		chip:   chip,
		pin:    p.Name,
		invert: p.Invert,
		pullup: p.Pullup,
	}
}

// getPin reads a pin from a config section.
func getPin(sec *config.Section, key string, canInvert, canPullup bool) (pin, error) {
	p, err := sec.GetPin(key, config.PinOptions{CanInvert: canInvert, CanPullup: canPullup})
	if err != nil {
		return pin{}, err
	}
	return pinFromConfig(p), nil
}

// getPinOptional reads an optional pin from a config section.
func getPinOptional(sec *config.Section, key string, canInvert, canPullup bool) (*pin, error) {
	p, err := sec.GetPinOptional(key, config.PinOptions{CanInvert: canInvert, CanPullup: canPullup})
	if err != nil {
		return nil, err
	}
	if p == nil {
		return nil, nil
	}
	result := pinFromConfig(*p)
	return &result, nil
}

type stepperCfg struct {
	axis byte
	name string

	stepPin    pin
	dirPin     pin
	enablePin  pin
	endstopPin pin

	microsteps       int
	fullSteps        int
	rotationDistance float64

	positionMin     float64
	positionMax     float64
	positionEndstop float64

	homingSpeed         float64
	secondHomingSpeed   float64
	homingRetractSpeed  float64
	homingRetractDist   float64
	homingPositiveDir   bool
	homingPositiveKnown bool
}

type extruderStepperCfg struct {
	name string
	// extruderName is only meaningful for [extruder_stepper ...] sections.
	// It indicates which extruder motion queue this stepper syncs to by default.
	extruderName string

	stepPin   pin
	dirPin    pin
	enablePin pin

	microsteps       int
	fullSteps        int
	rotationDistance float64
}

func readStepper(cfg *configWrapper, axis byte) (stepperCfg, error) {
	secName := fmt.Sprintf("stepper_%c", axis)
	sec, err := cfg.GetSection(secName)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("missing [%s] section", secName)
	}

	stepPin, err := getPin(sec, "step_pin", true, false)
	if err != nil {
		return stepperCfg{}, err
	}
	dirPin, err := getPin(sec, "dir_pin", true, false)
	if err != nil {
		return stepperCfg{}, err
	}
	enablePin, err := getPin(sec, "enable_pin", true, false)
	if err != nil {
		return stepperCfg{}, err
	}
	endstopPin, err := getPin(sec, "endstop_pin", true, true)
	if err != nil {
		return stepperCfg{}, err
	}

	microsteps, err := sec.GetInt("microsteps")
	if err != nil {
		return stepperCfg{}, err
	}
	fullSteps, err := sec.GetInt("full_steps_per_rotation", 200)
	if err != nil {
		return stepperCfg{}, err
	}
	rotationDistance, err := sec.GetFloat("rotation_distance")
	if err != nil {
		return stepperCfg{}, err
	}

	positionMin, err := sec.GetFloat("position_min", 0.0)
	if err != nil {
		return stepperCfg{}, err
	}
	positionMax, err := sec.GetFloat("position_max")
	if err != nil {
		return stepperCfg{}, err
	}

	// position_endstop: optional for probe-based Z endstop
	var positionEndstop float64
	if sec.HasOption("position_endstop") {
		positionEndstop, err = sec.GetFloat("position_endstop")
		if err != nil {
			return stepperCfg{}, err
		}
	} else {
		// Some configs (e.g., probe-based Z endstop) do not specify position_endstop
		// on the stepper. Klippy derives it from the probe object (z_offset).
		if strings.HasPrefix(strings.ToLower(endstopPin.pin), "probe:") {
			positionEndstop = positionMin
		} else {
			return stepperCfg{}, config.ErrMissingOption(secName, "position_endstop")
		}
	}

	homingSpeed, err := sec.GetFloat("homing_speed", 5.0)
	if err != nil {
		return stepperCfg{}, err
	}
	secondHomingSpeed, err := sec.GetFloat("second_homing_speed", homingSpeed/2.0)
	if err != nil {
		return stepperCfg{}, err
	}
	homingRetractSpeed, err := sec.GetFloat("homing_retract_speed", homingSpeed)
	if err != nil {
		return stepperCfg{}, err
	}
	homingRetractDist, err := sec.GetFloat("homing_retract_dist", 5.0)
	if err != nil {
		return stepperCfg{}, err
	}

	// homing_positive_dir: optional, can be inferred from position_endstop
	homingPositiveKnown := false
	homingPositiveDir := false
	if sec.HasOption("homing_positive_dir") {
		v, err := sec.GetBool("homing_positive_dir")
		if err != nil {
			return stepperCfg{}, err
		}
		homingPositiveKnown = true
		homingPositiveDir = v
	} else {
		axisLen := positionMax - positionMin
		if positionEndstop <= positionMin+axisLen/4.0 {
			homingPositiveKnown = true
			homingPositiveDir = false
		} else if positionEndstop >= positionMax-axisLen/4.0 {
			homingPositiveKnown = true
			homingPositiveDir = true
		}
		if !homingPositiveKnown {
			return stepperCfg{}, config.NewConfigError(secName, "", "unable to infer homing_positive_dir")
		}
	}

	return stepperCfg{
		axis:                axis,
		name:                secName,
		stepPin:             stepPin,
		dirPin:              dirPin,
		enablePin:           enablePin,
		endstopPin:          endstopPin,
		microsteps:          microsteps,
		fullSteps:           fullSteps,
		rotationDistance:    rotationDistance,
		positionMin:         positionMin,
		positionMax:         positionMax,
		positionEndstop:     positionEndstop,
		homingSpeed:         homingSpeed,
		secondHomingSpeed:   secondHomingSpeed,
		homingRetractSpeed:  homingRetractSpeed,
		homingRetractDist:   homingRetractDist,
		homingPositiveDir:   homingPositiveDir,
		homingPositiveKnown: homingPositiveKnown,
	}, nil
}

// readStepperByName reads a stepper configuration from a specific section name.
// This is used for delta kinematics where steppers are named stepper_a, stepper_b, stepper_c.
func readStepperByName(cfg *configWrapper, secName string, axis byte) (stepperCfg, error) {
	sec, err := cfg.GetSection(secName)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("missing [%s] section", secName)
	}

	stepPin, err := getPin(sec, "step_pin", true, false)
	if err != nil {
		return stepperCfg{}, err
	}
	dirPin, err := getPin(sec, "dir_pin", true, false)
	if err != nil {
		return stepperCfg{}, err
	}
	enablePin, err := getPin(sec, "enable_pin", true, false)
	if err != nil {
		return stepperCfg{}, err
	}
	endstopPin, err := getPin(sec, "endstop_pin", true, true)
	if err != nil {
		return stepperCfg{}, err
	}

	microsteps, err := sec.GetInt("microsteps")
	if err != nil {
		return stepperCfg{}, err
	}
	fullSteps, err := sec.GetInt("full_steps_per_rotation", 200)
	if err != nil {
		return stepperCfg{}, err
	}
	rotationDistance, err := sec.GetFloat("rotation_distance")
	if err != nil {
		return stepperCfg{}, err
	}

	// For delta, position_min/max may not be specified on individual steppers
	// Use defaults appropriate for delta
	positionMin, err := sec.GetFloat("position_min", 0.0)
	if err != nil {
		return stepperCfg{}, err
	}

	// position_max is optional for delta steppers (derived from endstop positions)
	var positionMax float64
	if sec.HasOption("position_max") {
		positionMax, err = sec.GetFloat("position_max")
		if err != nil {
			return stepperCfg{}, err
		}
	}

	// position_endstop: required for delta towers
	positionEndstop, err := sec.GetFloat("position_endstop")
	if err != nil {
		return stepperCfg{}, err
	}

	// If position_max not set, use position_endstop as default
	if !sec.HasOption("position_max") {
		positionMax = positionEndstop
	}

	homingSpeed, err := sec.GetFloat("homing_speed", 5.0)
	if err != nil {
		return stepperCfg{}, err
	}
	secondHomingSpeed, err := sec.GetFloat("second_homing_speed", homingSpeed/2.0)
	if err != nil {
		return stepperCfg{}, err
	}
	homingRetractSpeed, err := sec.GetFloat("homing_retract_speed", homingSpeed)
	if err != nil {
		return stepperCfg{}, err
	}
	homingRetractDist, err := sec.GetFloat("homing_retract_dist", 5.0)
	if err != nil {
		return stepperCfg{}, err
	}

	// Delta towers always home positive (toward max endstop)
	homingPositiveDir := true
	if sec.HasOption("homing_positive_dir") {
		v, err := sec.GetBool("homing_positive_dir")
		if err != nil {
			return stepperCfg{}, err
		}
		homingPositiveDir = v
	}

	return stepperCfg{
		axis:                axis,
		name:                secName,
		stepPin:             stepPin,
		dirPin:              dirPin,
		enablePin:           enablePin,
		endstopPin:          endstopPin,
		microsteps:          microsteps,
		fullSteps:           fullSteps,
		rotationDistance:    rotationDistance,
		positionMin:         positionMin,
		positionMax:         positionMax,
		positionEndstop:     positionEndstop,
		homingSpeed:         homingSpeed,
		secondHomingSpeed:   secondHomingSpeed,
		homingRetractSpeed:  homingRetractSpeed,
		homingRetractDist:   homingRetractDist,
		homingPositiveDir:   homingPositiveDir,
		homingPositiveKnown: true,
	}, nil
}

func readExtruderStepper(cfg *configWrapper) (extruderStepperCfg, error) {
	secName := "extruder"
	sec, err := cfg.GetSection(secName)
	if err != nil {
		return extruderStepperCfg{}, fmt.Errorf("missing [%s] section", secName)
	}

	stepPin, err := getPin(sec, "step_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, err
	}
	dirPin, err := getPin(sec, "dir_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, err
	}
	enablePin, err := getPin(sec, "enable_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, err
	}

	microsteps, err := sec.GetInt("microsteps")
	if err != nil {
		return extruderStepperCfg{}, err
	}
	fullSteps, err := sec.GetInt("full_steps_per_rotation", 200)
	if err != nil {
		return extruderStepperCfg{}, err
	}
	rotationDistance, err := sec.GetFloat("rotation_distance")
	if err != nil {
		return extruderStepperCfg{}, err
	}

	return extruderStepperCfg{
		name:             secName,
		extruderName:     "",
		stepPin:          stepPin,
		dirPin:           dirPin,
		enablePin:        enablePin,
		microsteps:       microsteps,
		fullSteps:        fullSteps,
		rotationDistance: rotationDistance,
	}, nil
}

// readExtruderStepperOptional reads an extruder_stepper section if it exists
func readExtruderStepperOptional(cfg *configWrapper, secName string) (extruderStepperCfg, bool, error) {
	sec := cfg.GetSectionOptional(secName)
	if sec == nil {
		return extruderStepperCfg{}, false, nil
	}

	extruderName, _ := sec.Get("extruder", "extruder")

	stepPin, err := getPin(sec, "step_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, false, err
	}
	dirPin, err := getPin(sec, "dir_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, false, err
	}
	enablePin, err := getPin(sec, "enable_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, false, err
	}

	microsteps, err := sec.GetInt("microsteps")
	if err != nil {
		return extruderStepperCfg{}, false, err
	}
	fullSteps, err := sec.GetInt("full_steps_per_rotation", 200)
	if err != nil {
		return extruderStepperCfg{}, false, err
	}
	rotationDistance, err := sec.GetFloat("rotation_distance")
	if err != nil {
		return extruderStepperCfg{}, false, err
	}

	return extruderStepperCfg{
		name:             secName,
		extruderName:     extruderName,
		stepPin:          stepPin,
		dirPin:           dirPin,
		enablePin:        enablePin,
		microsteps:       microsteps,
		fullSteps:        fullSteps,
		rotationDistance: rotationDistance,
	}, true, nil
}

func readGcodeArcsResolution(cfg *configWrapper) (float64, error) {
	sec := cfg.GetSectionOptional("gcode_arcs")
	if sec == nil {
		return 1.0, nil
	}
	return sec.GetFloat("resolution", 1.0)
}

// readHeaterConfigs reads all heater configurations from the config file
func readHeaterConfigs(cfg *configWrapper) ([]*heaterConfig, error) {
	var heaters []*heaterConfig

	// Check for [extruder] section
	if sec := cfg.GetSectionOptional("extruder"); sec != nil {
		// Only create heater if heater_pin is present
		if sec.HasOption("heater_pin") {
			hc, err := parseHeaterConfigFromSection("extruder", sec)
			if err != nil {
				return nil, err
			}
			heaters = append(heaters, hc)
		}
	}

	// Check for [heater_bed] section
	if sec := cfg.GetSectionOptional("heater_bed"); sec != nil {
		hc, err := parseHeaterConfigFromSection("heater_bed", sec)
		if err != nil {
			return nil, err
		}
		heaters = append(heaters, hc)
	}

	// Check for [heater_generic] sections
	for _, sec := range cfg.GetPrefixSections("heater_generic ") {
		hc, err := parseHeaterConfigFromSection(sec.GetName(), sec)
		if err != nil {
			return nil, err
		}
		heaters = append(heaters, hc)
	}

	return heaters, nil
}

// parseHeaterConfigFromSection extracts heater configuration from a config.Section.
// This is the new version that uses the config package.
func parseHeaterConfigFromSection(name string, sec *config.Section) (*heaterConfig, error) {
	cfg := &heaterConfig{
		name:           name,
		control:        "pid",
		minTemp:        0.0,
		maxTemp:        250.0,
		maxPower:       1.0,
		smoothTime:     1.0,
		pwmCycleTime:   0.1,
		minExtrudeTemp: 170.0,
		maxDelta:       2.0,
	}

	// Parse required fields
	heaterPin, err := sec.Get("heater_pin")
	if err != nil {
		return nil, err
	}
	cfg.heaterPin = heaterPin

	sensorType, err := sec.Get("sensor_type")
	if err != nil {
		return nil, err
	}
	cfg.sensorType = sensorType

	sensorPin, err := sec.Get("sensor_pin")
	if err != nil {
		return nil, err
	}
	cfg.sensorPin = sensorPin

	// Parse optional fields
	cfg.control, _ = sec.Get("control", "pid")
	cfg.minTemp, _ = sec.GetFloat("min_temp", 0.0)
	cfg.maxTemp, _ = sec.GetFloat("max_temp", 250.0)

	// PID parameters
	if cfg.control == "pid" {
		cfg.pidKp, err = sec.GetFloat("pid_kp")
		if err != nil {
			return nil, config.NewConfigError(name, "pid_kp", "pid control requires pid_kp")
		}
		cfg.pidKi, err = sec.GetFloat("pid_ki")
		if err != nil {
			return nil, config.NewConfigError(name, "pid_ki", "pid control requires pid_ki")
		}
		cfg.pidKd, err = sec.GetFloat("pid_kd")
		if err != nil {
			return nil, config.NewConfigError(name, "pid_kd", "pid control requires pid_kd")
		}
	}

	// Additional optional parameters
	cfg.maxPower, _ = sec.GetFloat("max_power", 1.0)
	cfg.smoothTime, _ = sec.GetFloat("smooth_time", 1.0)
	cfg.pwmCycleTime, _ = sec.GetFloat("pwm_cycle_time", 0.1)
	cfg.minExtrudeTemp, _ = sec.GetFloat("min_extrude_temp", 170.0)
	cfg.maxDelta, _ = sec.GetFloat("max_delta", 2.0)

	return cfg, nil
}
