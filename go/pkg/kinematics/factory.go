// Factory functions for creating kinematics instances from configuration.
package kinematics

import (
	"fmt"
	"strings"
)

// Config represents the configuration needed to create a kinematics instance.
type Config struct {
	Type         string  // Kinematic type: "cartesian", "corexy", "corexz", etc.
	Rails        []Rail  // Rail configurations
	MaxZVelocity float64 // Maximum Z velocity
	MaxZAccel    float64 // Maximum Z acceleration
}

// NewFromConfig creates a new kinematics instance based on the configuration.
func NewFromConfig(cfg Config) (Kinematics, error) {
	// Normalize the kinematic type
	kinType := strings.ToLower(strings.TrimSpace(cfg.Type))

	// Validate rails configuration
	if len(cfg.Rails) < 3 {
		return nil, fmt.Errorf("kinematics requires at least 3 rails, got %d", len(cfg.Rails))
	}

	// Create the appropriate kinematics instance
	switch kinType {
	case "cartesian":
		return NewCartesianKinematics(cfg.Rails, cfg.MaxZVelocity, cfg.MaxZAccel), nil

	case "corexy":
		return NewCoreXYKinematics(cfg.Rails, cfg.MaxZVelocity, cfg.MaxZAccel), nil

	case "corexz":
		return NewCoreXZKinematics(cfg.Rails, cfg.MaxZVelocity, cfg.MaxZAccel), nil

	default:
		return nil, fmt.Errorf("unsupported kinematics type: %s", cfg.Type)
	}
}

// LoadFromPrinterConfig loads kinematics from a printer configuration map.
// This is designed to work with Klipper's configuration format.
func LoadFromPrinterConfig(printerCfg map[string]map[string]string) (Kinematics, error) {
	// Get the printer section for kinematic type
	printerSection, ok := printerCfg["printer"]
	if !ok {
		return nil, fmt.Errorf("missing [printer] section in configuration")
	}

	kinType, ok := printerSection["kinematics"]
	if !ok {
		return nil, fmt.Errorf("missing kinematics setting in [printer] section")
	}

	// Get max velocity and acceleration settings
	maxVelocity := 500.0  // Default
	maxAccel := 3000.0     // Default
	maxZVelocity := 25.0   // Default
	maxZAccel := 350.0     // Default

	if v, ok := printerSection["max_velocity"]; ok {
		if _, err := fmt.Sscanf(v, "%f", &maxVelocity); err != nil {
			return nil, fmt.Errorf("invalid max_velocity: %s", v)
		}
	}

	if v, ok := printerSection["max_accel"]; ok {
		if _, err := fmt.Sscanf(v, "%f", &maxAccel); err != nil {
			return nil, fmt.Errorf("invalid max_accel: %s", v)
		}
	}

	if v, ok := printerSection["max_z_velocity"]; ok {
		if _, err := fmt.Sscanf(v, "%f", &maxZVelocity); err != nil {
			return nil, fmt.Errorf("invalid max_z_velocity: %s", v)
		}
	} else {
		// If not specified, use max_velocity
		maxZVelocity = maxVelocity
	}

	if v, ok := printerSection["max_z_accel"]; ok {
		if _, err := fmt.Sscanf(v, "%f", &maxZAccel); err != nil {
			return nil, fmt.Errorf("invalid max_z_accel: %s", v)
		}
	} else {
		// If not specified, use max_accel
		maxZAccel = maxAccel
	}

	// Load rail configurations
	rails := make([]Rail, 0, 3)

	// Load stepper configurations
	for _, axis := range []string{"x", "y", "z"} {
		stepperSection, ok := printerCfg["stepper_"+axis]
		if !ok {
			continue
		}

		rail := Rail{
			Name: "stepper_" + axis,
		}

		// Parse step_distance
		if v, ok := stepperSection["step_distance"]; ok {
			if _, err := fmt.Sscanf(v, "%f", &rail.StepDist); err != nil {
				return nil, fmt.Errorf("invalid step_distance for stepper_%s: %s", axis, v)
			}
		}

		// Parse position_min
		if v, ok := stepperSection["position_min"]; ok {
			if _, err := fmt.Sscanf(v, "%f", &rail.PositionMin); err != nil {
				return nil, fmt.Errorf("invalid position_min for stepper_%s: %s", axis, v)
			}
		}

		// Parse position_max
		if v, ok := stepperSection["position_max"]; ok {
			if _, err := fmt.Sscanf(v, "%f", &rail.PositionMax); err != nil {
				return nil, fmt.Errorf("invalid position_max for stepper_%s: %s", axis, v)
			}
		}

		// Parse homing_speed
		if v, ok := stepperSection["homing_speed"]; ok {
			if _, err := fmt.Sscanf(v, "%f", &rail.HomingSpeed); err != nil {
				return nil, fmt.Errorf("invalid homing_speed for stepper_%s: %s", axis, v)
			}
		} else {
			rail.HomingSpeed = 5.0 // Default
		}

		// Parse position_endstop
		if v, ok := stepperSection["position_endstop"]; ok {
			if _, err := fmt.Sscanf(v, "%f", &rail.PositionEndstop); err != nil {
				return nil, fmt.Errorf("invalid position_endstop for stepper_%s: %s", axis, v)
			}
		}

		// Parse homing_positive_dir
		if v, ok := stepperSection["homing_positive_dir"]; ok {
			rail.HomingPositive = strings.ToLower(v) == "true"
		}

		// Parse homing_retract_dist
		if v, ok := stepperSection["homing_retract_dist"]; ok {
			if _, err := fmt.Sscanf(v, "%f", &rail.HomingRetract); err != nil {
				return nil, fmt.Errorf("invalid homing_retract_dist for stepper_%s: %s", axis, v)
			}
		} else {
			rail.HomingRetract = 5.0 // Default
		}

		// Parse second_homing_speed
		if v, ok := stepperSection["second_homing_speed"]; ok {
			if _, err := fmt.Sscanf(v, "%f", &rail.SecondHoming); err != nil {
				return nil, fmt.Errorf("invalid second_homing_speed for stepper_%s: %s", axis, v)
			}
		} else {
			rail.SecondHoming = rail.HomingSpeed / 2.0 // Default to half of homing_speed
		}

		rails = append(rails, rail)
	}

	// Ensure we have exactly 3 rails
	if len(rails) != 3 {
		return nil, fmt.Errorf("kinematics requires exactly 3 rails (x, y, z), got %d", len(rails))
	}

	// Create the configuration
	cfg := Config{
		Type:         kinType,
		Rails:        rails,
		MaxZVelocity: maxZVelocity,
		MaxZAccel:    maxZAccel,
	}

	return NewFromConfig(cfg)
}

// IsSupported returns true if the given kinematic type is supported.
func IsSupported(kinType string) bool {
	normalized := strings.ToLower(strings.TrimSpace(kinType))
	switch normalized {
	case "cartesian", "corexy", "corexz", "delta":
		return true
	default:
		return false
	}
}

// SupportedTypes returns a list of supported kinematic types.
func SupportedTypes() []string {
	return []string{"cartesian", "corexy", "corexz", "delta"}
}