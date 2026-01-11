package hosth4

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

type config struct {
	sections map[string]map[string]string
}

func loadConfig(path string) (*config, error) {
	c := &config{sections: map[string]map[string]string{}}
	abs, err := filepath.Abs(path)
	if err != nil {
		return nil, err
	}
	f, err := os.Open(abs)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	var cur string
	s := bufio.NewScanner(f)
	for s.Scan() {
		line := strings.TrimSpace(s.Text())
		if line == "" {
			continue
		}
		if idx := strings.IndexByte(line, '#'); idx >= 0 {
			line = strings.TrimSpace(line[:idx])
			if line == "" {
				continue
			}
		}
		if strings.HasPrefix(line, "[") && strings.HasSuffix(line, "]") {
			cur = strings.TrimSpace(line[1 : len(line)-1])
			if cur == "" {
				return nil, fmt.Errorf("empty section header")
			}
			if _, ok := c.sections[cur]; !ok {
				c.sections[cur] = map[string]string{}
			}
			continue
		}
		if cur == "" {
			continue
		}
		kv := strings.SplitN(line, ":", 2)
		if len(kv) != 2 {
			kv = strings.SplitN(line, "=", 2)
		}
		if len(kv) != 2 {
			continue
		}
		k := strings.TrimSpace(kv[0])
		v := strings.TrimSpace(kv[1])
		if k == "" {
			continue
		}
		c.sections[cur][k] = v
	}
	if err := s.Err(); err != nil {
		return nil, err
	}
	return c, nil
}

func (c *config) section(name string) (map[string]string, bool) {
	sec, ok := c.sections[name]
	return sec, ok
}

// sectionsByPrefix returns all section names that start with the given prefix.
func (c *config) sectionsByPrefix(prefix string) []string {
	var result []string
	for name := range c.sections {
		if strings.HasPrefix(name, prefix) {
			result = append(result, name)
		}
	}
	return result
}

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

type pin struct {
	pin    string
	invert bool
	pullup int
}

func parsePinDesc(desc string, canInvert bool, canPullup bool) (pin, error) {
	d := strings.TrimSpace(desc)
	p := pin{}
	if canPullup && (strings.HasPrefix(d, "^") || strings.HasPrefix(d, "~")) {
		p.pullup = 1
		if strings.HasPrefix(d, "~") {
			p.pullup = -1
		}
		d = strings.TrimSpace(d[1:])
	}
	if canInvert && strings.HasPrefix(d, "!") {
		p.invert = true
		d = strings.TrimSpace(d[1:])
	}
	chip := "mcu"
	if strings.Contains(d, ":") {
		parts := strings.SplitN(d, ":", 2)
		chip = strings.TrimSpace(parts[0])
		if chip != "mcu" && chip != "probe" {
			return pin{}, fmt.Errorf("unsupported pin chip %q in %q", chip, desc)
		}
		d = strings.TrimSpace(parts[1])
	}
	if strings.Join(strings.Fields(d), "") != d {
		return pin{}, fmt.Errorf("invalid pin %q", desc)
	}
	if chip == "mcu" {
		if strings.ContainsAny(d, "^~!:") {
			return pin{}, fmt.Errorf("invalid pin %q", desc)
		}
		p.pin = d
	} else {
		// Virtual / non-MCU pin.
		if d == "" {
			return pin{}, fmt.Errorf("invalid pin %q", desc)
		}
		p.pin = chip + ":" + d
	}
	return p, nil
}

func parsePin(sec map[string]string, key string, canInvert bool, canPullup bool) (pin, error) {
	raw := strings.TrimSpace(sec[key])
	if raw == "" {
		return pin{}, fmt.Errorf("missing %s", key)
	}
	return parsePinDesc(raw, canInvert, canPullup)
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

func readStepper(cfg *config, axis byte) (stepperCfg, error) {
	secName := fmt.Sprintf("stepper_%c", axis)
	sec, ok := cfg.section(secName)
	if !ok {
		return stepperCfg{}, fmt.Errorf("missing [%s] section", secName)
	}
	stepPin, err := parsePin(sec, "step_pin", true, false)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	dirPin, err := parsePin(sec, "dir_pin", true, false)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	enablePin, err := parsePin(sec, "enable_pin", true, false)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	endstopPin, err := parsePin(sec, "endstop_pin", true, true)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	microsteps, err := parseInt(sec, "microsteps", nil)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	defFull := 200
	fullSteps, err := parseInt(sec, "full_steps_per_rotation", &defFull)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	rotationDistance, err := parseFloat(sec, "rotation_distance", nil)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}

	defPosMin := 0.0
	positionMin, err := parseFloat(sec, "position_min", &defPosMin)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	positionMax, err := parseFloat(sec, "position_max", nil)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	var positionEndstop float64
	rawEndstop := strings.TrimSpace(sec["position_endstop"])
	if rawEndstop == "" {
		// Some configs (e.g., probe-based Z endstop) do not specify position_endstop
		// on the stepper. Klippy derives it from the probe object (z_offset).
		if strings.HasPrefix(strings.ToLower(endstopPin.pin), "probe:") {
			positionEndstop = defPosMin
		} else {
			return stepperCfg{}, fmt.Errorf("[%s] missing position_endstop", secName)
		}
	} else {
		positionEndstop, err = parseFloat(sec, "position_endstop", nil)
		if err != nil {
			return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
		}
	}

	defHomingSpeed := 5.0
	homingSpeed, err := parseFloat(sec, "homing_speed", &defHomingSpeed)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	defSecond := homingSpeed / 2.0
	secondHomingSpeed, err := parseFloat(sec, "second_homing_speed", &defSecond)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	homingRetractSpeed, err := parseFloat(sec, "homing_retract_speed", &homingSpeed)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	defRetractDist := 5.0
	homingRetractDist, err := parseFloat(sec, "homing_retract_dist", &defRetractDist)
	if err != nil {
		return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	hpRaw := strings.TrimSpace(sec["homing_positive_dir"])
	homingPositiveKnown := false
	homingPositiveDir := false
	if hpRaw != "" {
		v, err := parseBool(sec, "homing_positive_dir", nil)
		if err != nil {
			return stepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
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
			return stepperCfg{}, fmt.Errorf("[%s] unable to infer homing_positive_dir", secName)
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

func readExtruderStepper(cfg *config) (extruderStepperCfg, error) {
	secName := "extruder"
	sec, ok := cfg.section(secName)
	if !ok {
		return extruderStepperCfg{}, fmt.Errorf("missing [%s] section", secName)
	}
	stepPin, err := parsePin(sec, "step_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	dirPin, err := parsePin(sec, "dir_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	enablePin, err := parsePin(sec, "enable_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	microsteps, err := parseInt(sec, "microsteps", nil)
	if err != nil {
		return extruderStepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	defFull := 200
	fullSteps, err := parseInt(sec, "full_steps_per_rotation", &defFull)
	if err != nil {
		return extruderStepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
	}
	rotationDistance, err := parseFloat(sec, "rotation_distance", nil)
	if err != nil {
		return extruderStepperCfg{}, fmt.Errorf("[%s] %v", secName, err)
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
func readExtruderStepperOptional(cfg *config, secName string) (extruderStepperCfg, bool, error) {
	sec, ok := cfg.section(secName)
	if !ok {
		return extruderStepperCfg{}, false, nil
	}
	extruderName := strings.TrimSpace(sec["extruder"])
	if extruderName == "" {
		extruderName = "extruder"
	}
	stepPin, err := parsePin(sec, "step_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, false, fmt.Errorf("[%s] %v", secName, err)
	}
	dirPin, err := parsePin(sec, "dir_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, false, fmt.Errorf("[%s] %v", secName, err)
	}
	enablePin, err := parsePin(sec, "enable_pin", true, false)
	if err != nil {
		return extruderStepperCfg{}, false, fmt.Errorf("[%s] %v", secName, err)
	}
	microsteps, err := parseInt(sec, "microsteps", nil)
	if err != nil {
		return extruderStepperCfg{}, false, fmt.Errorf("[%s] %v", secName, err)
	}
	defFull := 200
	fullSteps, err := parseInt(sec, "full_steps_per_rotation", &defFull)
	if err != nil {
		return extruderStepperCfg{}, false, fmt.Errorf("[%s] %v", secName, err)
	}
	rotationDistance, err := parseFloat(sec, "rotation_distance", nil)
	if err != nil {
		return extruderStepperCfg{}, false, fmt.Errorf("[%s] %v", secName, err)
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

func readGcodeArcsResolution(cfg *config) (float64, error) {
	sec, ok := cfg.section("gcode_arcs")
	if !ok {
		return 1.0, nil
	}
	def := 1.0
	return parseFloat(sec, "resolution", &def)
}

// readHeaterConfigs reads all heater configurations from the config file
func readHeaterConfigs(cfg *config) ([]*heaterConfig, error) {
	var heaters []*heaterConfig

	// Check for [extruder] section
	if sec, ok := cfg.section("extruder"); ok {
		// Only create heater if heater_pin is present
		if _, hasHeater := sec["heater_pin"]; hasHeater {
			hc, err := parseHeaterConfig("extruder", sec)
			if err != nil {
				return nil, fmt.Errorf("[extruder] %v", err)
			}
			heaters = append(heaters, hc)
		}
	}

	// Check for [heater_bed] section
	if sec, ok := cfg.section("heater_bed"); ok {
		hc, err := parseHeaterConfig("heater_bed", sec)
		if err != nil {
			return nil, fmt.Errorf("[heater_bed] %v", err)
		}
		heaters = append(heaters, hc)
	}

	// Check for [heater_generic] sections
	for secName := range cfg.sections {
		if strings.HasPrefix(secName, "heater_generic ") {
			sec, ok := cfg.section(secName)
			if !ok {
				continue
			}
			hc, err := parseHeaterConfig(secName, sec)
			if err != nil {
				return nil, fmt.Errorf("[%s] %v", secName, err)
			}
			heaters = append(heaters, hc)
		}
	}

	return heaters, nil
}
