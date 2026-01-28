// Package config provides Klipper printer configuration parsing.
// It extracts stepper, heater, and sensor configurations from printer.cfg files.
package config

import (
	"bufio"
	"fmt"
	"os"
	"strings"
)

// StepperConfig holds stepper motor configuration.
type StepperConfig struct {
	Name       string // e.g., "stepper_x", "stepper_y", "stepper_z"
	StepPin    string // e.g., "PK0"
	DirPin     string // e.g., "PK1"
	EnablePin  string // e.g., "PF7"
	EndstopPin string // e.g., "PL6"
	// Pin modifiers
	DirInvert     bool // ! prefix on dir_pin
	EnableInvert  bool // ! prefix on enable_pin
	EndstopPullup bool // ^ prefix on endstop_pin
	EndstopInvert bool // ! prefix on endstop_pin
	// Position parameters
	PositionEndstop float64 // position_endstop: 0 means at min, 300 means at max
	PositionMin     float64 // position_min
	PositionMax     float64 // position_max: maximum travel distance
	// Motion parameters
	Microsteps       int     // e.g., 16
	RotationDistance float64 // e.g., 40.0 mm per full rotation
	FullStepsPerRot  int     // e.g., 200 (standard stepper)
	// Homing parameters
	HomingSpeed       float64 // mm/s, default 5.0
	HomingRetractDist float64 // mm, default 5.0
	SecondHomingSpeed float64 // mm/s, default HomingSpeed/2
}

// ExtruderConfig holds extruder/heater configuration.
type ExtruderConfig struct {
	// Stepper
	StepPin          string
	DirPin           string
	EnablePin        string
	DirInvert        bool
	EnableInvert     bool
	Microsteps       int
	RotationDistance float64
	FullStepsPerRot  int
	NozzleDiameter   float64
	FilamentDiameter float64
	MaxExtrudeRatio  float64
	// Heater
	HeaterPin  string  // e.g., "PA7"
	SensorPin  string  // e.g., "PF0"
	SensorType string  // e.g., "ATC Semitec 104GT-2"
	PullupR    float64 // pullup resistor value, default 4700
	MinTemp    float64
	MaxTemp    float64
	ControlPID bool
	PID_Kp     float64
	PID_Ki     float64
	PID_Kd     float64
	// Extrusion
	MinExtrudeTemp  float64 // minimum temp for extrusion
	PressureAdvance float64
}

// HeaterBedConfig holds heated bed configuration.
type HeaterBedConfig struct {
	HeaterPin  string  // e.g., "PA6"
	SensorPin  string  // e.g., "PF2"
	SensorType string  // e.g., "EPCOS 100K B57560G104F"
	PullupR    float64 // pullup resistor value, default 4700
	MinTemp    float64
	MaxTemp    float64
	ControlPID bool
	PID_Kp     float64
	PID_Ki     float64
	PID_Kd     float64
}

// FanConfig holds fan configuration.
type FanConfig struct {
	Name           string
	Pin            string
	MaxPower       float64
	KickStartTime  float64
	OffBelow       float64
	CycleTime      float64
	HardwarePWM    bool
	ShutdownSpeed  float64
	// For heater_fan
	Heater         string
	HeaterTemp     float64
	FanSpeed       float64
}

// PrinterConfig holds the full printer configuration.
type PrinterConfig struct {
	Device       string
	Baud         int
	Kinematics   string // e.g., "cartesian", "corexy", "delta"
	MaxVelocity  float64
	MaxAccel     float64
	MaxZVelocity float64
	MaxZAccel    float64
	Steppers     map[string]*StepperConfig
	Extruder     *ExtruderConfig
	HeaterBed    *HeaterBedConfig
	Fans         map[string]*FanConfig
}

// parsePin extracts pin name and modifiers from a Klipper pin spec like "!PK1" or "^!PL6".
func parsePin(spec string) (pin string, invert bool, pullup bool) {
	spec = strings.TrimSpace(spec)
	for len(spec) > 0 {
		switch spec[0] {
		case '!':
			invert = true
			spec = spec[1:]
		case '^':
			pullup = true
			spec = spec[1:]
		case '~':
			// PWM modifier, ignore for now
			spec = spec[1:]
		default:
			pin = spec
			return
		}
	}
	return
}

// ParsePrinterConfig reads and parses a Klipper printer.cfg file.
func ParsePrinterConfig(path string) (*PrinterConfig, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	config := &PrinterConfig{
		Steppers: make(map[string]*StepperConfig),
		Fans:     make(map[string]*FanConfig),
		Baud:     250000, // Default baud rate
	}

	scanner := bufio.NewScanner(file)
	var currentSection string
	var currentStepper *StepperConfig
	var currentExtruder *ExtruderConfig
	var currentHeaterBed *HeaterBedConfig
	var currentFan *FanConfig

	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())

		// Skip comments and empty lines
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}

		// Check for section headers
		if strings.HasPrefix(line, "[") && strings.HasSuffix(line, "]") {
			section := strings.Trim(line, "[]")
			currentSection = strings.ToLower(section)

			// Reset current objects
			currentStepper = nil
			currentExtruder = nil
			currentHeaterBed = nil
			currentFan = nil

			// Create new section objects
			if strings.HasPrefix(currentSection, "stepper_") {
				currentStepper = &StepperConfig{
					Name:              currentSection,
					FullStepsPerRot:   200,
					Microsteps:        16,
					HomingSpeed:       5.0,
					HomingRetractDist: 5.0,
				}
				config.Steppers[currentSection] = currentStepper
			} else if currentSection == "extruder" {
				currentExtruder = &ExtruderConfig{
					PullupR:         4700,
					FullStepsPerRot: 200,
					Microsteps:      16,
					MinExtrudeTemp:  170,
				}
				config.Extruder = currentExtruder
			} else if currentSection == "heater_bed" {
				currentHeaterBed = &HeaterBedConfig{
					PullupR: 4700,
				}
				config.HeaterBed = currentHeaterBed
			} else if currentSection == "fan" {
				currentFan = &FanConfig{
					Name:     "fan",
					MaxPower: 1.0,
				}
				config.Fans["fan"] = currentFan
			} else if strings.HasPrefix(currentSection, "heater_fan ") {
				fanName := strings.TrimPrefix(currentSection, "heater_fan ")
				currentFan = &FanConfig{
					Name:       fanName,
					MaxPower:   1.0,
					HeaterTemp: 50.0,
					FanSpeed:   1.0,
				}
				config.Fans[fanName] = currentFan
			}
			continue
		}

		// Parse key: value or key = value
		var key, value string
		if idx := strings.Index(line, ":"); idx > 0 {
			key = strings.ToLower(strings.TrimSpace(line[:idx]))
			value = strings.TrimSpace(line[idx+1:])
		} else if idx := strings.Index(line, "="); idx > 0 {
			key = strings.ToLower(strings.TrimSpace(line[:idx]))
			value = strings.TrimSpace(line[idx+1:])
		} else {
			continue
		}

		// Parse settings based on current section
		switch currentSection {
		case "mcu":
			switch key {
			case "serial":
				config.Device = value
			case "baud":
				fmt.Sscanf(value, "%d", &config.Baud)
			}
		case "printer":
			switch key {
			case "kinematics":
				config.Kinematics = value
			case "max_velocity":
				fmt.Sscanf(value, "%f", &config.MaxVelocity)
			case "max_accel":
				fmt.Sscanf(value, "%f", &config.MaxAccel)
			case "max_z_velocity":
				fmt.Sscanf(value, "%f", &config.MaxZVelocity)
			case "max_z_accel":
				fmt.Sscanf(value, "%f", &config.MaxZAccel)
			}

		default:
			// Handle stepper sections
			if currentStepper != nil {
				switch key {
				case "step_pin":
					pin, _, _ := parsePin(value)
					currentStepper.StepPin = pin
				case "dir_pin":
					pin, invert, _ := parsePin(value)
					currentStepper.DirPin = pin
					currentStepper.DirInvert = invert
				case "enable_pin":
					pin, invert, _ := parsePin(value)
					currentStepper.EnablePin = pin
					currentStepper.EnableInvert = invert
				case "endstop_pin":
					pin, invert, pullup := parsePin(value)
					currentStepper.EndstopPin = pin
					currentStepper.EndstopInvert = invert
					currentStepper.EndstopPullup = pullup
				case "position_endstop":
					fmt.Sscanf(value, "%f", &currentStepper.PositionEndstop)
				case "position_min":
					fmt.Sscanf(value, "%f", &currentStepper.PositionMin)
				case "position_max":
					fmt.Sscanf(value, "%f", &currentStepper.PositionMax)
				case "microsteps":
					fmt.Sscanf(value, "%d", &currentStepper.Microsteps)
				case "rotation_distance":
					fmt.Sscanf(value, "%f", &currentStepper.RotationDistance)
				case "full_steps_per_rotation":
					fmt.Sscanf(value, "%d", &currentStepper.FullStepsPerRot)
				case "homing_speed":
					fmt.Sscanf(value, "%f", &currentStepper.HomingSpeed)
				case "homing_retract_dist":
					fmt.Sscanf(value, "%f", &currentStepper.HomingRetractDist)
				case "second_homing_speed":
					fmt.Sscanf(value, "%f", &currentStepper.SecondHomingSpeed)
				}
			}

			// Handle extruder section
			if currentExtruder != nil {
				switch key {
				case "step_pin":
					pin, _, _ := parsePin(value)
					currentExtruder.StepPin = pin
				case "dir_pin":
					pin, invert, _ := parsePin(value)
					currentExtruder.DirPin = pin
					currentExtruder.DirInvert = invert
				case "enable_pin":
					pin, invert, _ := parsePin(value)
					currentExtruder.EnablePin = pin
					currentExtruder.EnableInvert = invert
				case "heater_pin":
					pin, _, _ := parsePin(value)
					currentExtruder.HeaterPin = pin
				case "sensor_pin":
					pin, _, _ := parsePin(value)
					currentExtruder.SensorPin = pin
				case "sensor_type":
					currentExtruder.SensorType = value
				case "pullup_resistor":
					fmt.Sscanf(value, "%f", &currentExtruder.PullupR)
				case "min_temp":
					fmt.Sscanf(value, "%f", &currentExtruder.MinTemp)
				case "max_temp":
					fmt.Sscanf(value, "%f", &currentExtruder.MaxTemp)
				case "control":
					currentExtruder.ControlPID = value == "pid"
				case "pid_kp":
					fmt.Sscanf(value, "%f", &currentExtruder.PID_Kp)
				case "pid_ki":
					fmt.Sscanf(value, "%f", &currentExtruder.PID_Ki)
				case "pid_kd":
					fmt.Sscanf(value, "%f", &currentExtruder.PID_Kd)
				case "microsteps":
					fmt.Sscanf(value, "%d", &currentExtruder.Microsteps)
				case "rotation_distance":
					fmt.Sscanf(value, "%f", &currentExtruder.RotationDistance)
				case "nozzle_diameter":
					fmt.Sscanf(value, "%f", &currentExtruder.NozzleDiameter)
				case "filament_diameter":
					fmt.Sscanf(value, "%f", &currentExtruder.FilamentDiameter)
				case "min_extrude_temp":
					fmt.Sscanf(value, "%f", &currentExtruder.MinExtrudeTemp)
				case "pressure_advance":
					fmt.Sscanf(value, "%f", &currentExtruder.PressureAdvance)
				}
			}

			// Handle heater_bed section
			if currentHeaterBed != nil {
				switch key {
				case "heater_pin":
					pin, _, _ := parsePin(value)
					currentHeaterBed.HeaterPin = pin
				case "sensor_pin":
					pin, _, _ := parsePin(value)
					currentHeaterBed.SensorPin = pin
				case "sensor_type":
					currentHeaterBed.SensorType = value
				case "pullup_resistor":
					fmt.Sscanf(value, "%f", &currentHeaterBed.PullupR)
				case "min_temp":
					fmt.Sscanf(value, "%f", &currentHeaterBed.MinTemp)
				case "max_temp":
					fmt.Sscanf(value, "%f", &currentHeaterBed.MaxTemp)
				case "control":
					currentHeaterBed.ControlPID = value == "pid"
				case "pid_kp":
					fmt.Sscanf(value, "%f", &currentHeaterBed.PID_Kp)
				case "pid_ki":
					fmt.Sscanf(value, "%f", &currentHeaterBed.PID_Ki)
				case "pid_kd":
					fmt.Sscanf(value, "%f", &currentHeaterBed.PID_Kd)
				}
			}

			// Handle fan sections
			if currentFan != nil {
				switch key {
				case "pin":
					pin, _, _ := parsePin(value)
					currentFan.Pin = pin
				case "max_power":
					fmt.Sscanf(value, "%f", &currentFan.MaxPower)
				case "kick_start_time":
					fmt.Sscanf(value, "%f", &currentFan.KickStartTime)
				case "off_below":
					fmt.Sscanf(value, "%f", &currentFan.OffBelow)
				case "cycle_time":
					fmt.Sscanf(value, "%f", &currentFan.CycleTime)
				case "hardware_pwm":
					currentFan.HardwarePWM = strings.ToLower(value) == "true"
				case "heater":
					currentFan.Heater = value
				case "heater_temp":
					fmt.Sscanf(value, "%f", &currentFan.HeaterTemp)
				case "fan_speed":
					fmt.Sscanf(value, "%f", &currentFan.FanSpeed)
				}
			}
		}
	}

	// Set default second homing speed if not specified
	for _, stepper := range config.Steppers {
		if stepper.SecondHomingSpeed == 0 {
			stepper.SecondHomingSpeed = stepper.HomingSpeed / 2
		}
	}

	return config, scanner.Err()
}

// StepsPerMM calculates steps per mm for a stepper.
func (s *StepperConfig) StepsPerMM() float64 {
	if s.RotationDistance == 0 {
		return 0
	}
	return float64(s.FullStepsPerRot*s.Microsteps) / s.RotationDistance
}

// HomingDirection returns +1 for positive direction homing, -1 for negative.
func (s *StepperConfig) HomingDirection() int {
	if s.PositionEndstop > (s.PositionMin+s.PositionMax)/2 {
		return 1 // Home to max
	}
	return -1 // Home to min
}
