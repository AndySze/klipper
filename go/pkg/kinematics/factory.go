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

	// Delta-specific configuration (only used when Type == "delta")
	DeltaRadius    float64   // Delta radius
	DeltaArmLength []float64 // Arm lengths [A, B, C]
	DeltaAngles    []float64 // Tower angles in degrees [A, B, C]
}

// NewFromConfig creates a new kinematics instance based on the configuration.
func NewFromConfig(cfg Config) (Kinematics, error) {
	// Normalize the kinematic type
	kinType := strings.ToLower(strings.TrimSpace(cfg.Type))

	// Polar requires at least 2 rails (arm, z), winch requires 3+, others require 3
	minRails := 3
	if kinType == "polar" {
		minRails = 2
	}
	if kinType == "winch" {
		minRails = 3 // Winch can have more, but minimum is 3
	}
	if len(cfg.Rails) < minRails {
		return nil, fmt.Errorf("kinematics %s requires at least %d rails, got %d", kinType, minRails, len(cfg.Rails))
	}

	// Create the appropriate kinematics instance
	switch kinType {
	case "cartesian":
		return NewCartesianKinematics(cfg.Rails, cfg.MaxZVelocity, cfg.MaxZAccel), nil

	case "corexy":
		return NewCoreXYKinematics(cfg.Rails, cfg.MaxZVelocity, cfg.MaxZAccel), nil

	case "corexz":
		return NewCoreXZKinematics(cfg.Rails, cfg.MaxZVelocity, cfg.MaxZAccel), nil

	case "polar":
		return NewPolarKinematics(cfg.Rails, cfg.MaxZVelocity, cfg.MaxZAccel), nil

	case "hybrid_corexy":
		return NewHybridCoreXYKinematics(cfg.Rails, cfg.MaxZVelocity, cfg.MaxZAccel), nil

	case "hybrid_corexz":
		return NewHybridCoreXZKinematics(cfg.Rails, cfg.MaxZVelocity, cfg.MaxZAccel), nil

	case "deltesian":
		// Deltesian uses similar bounds checking to cartesian
		// The actual stepper kinematics are handled separately via chelper
		return NewCartesianKinematics(cfg.Rails, cfg.MaxZVelocity, cfg.MaxZAccel), nil

	case "delta":
		// Delta uses a different configuration structure.
		// Create a DeltaConfig from the Rails.
		if len(cfg.Rails) < 3 {
			return nil, fmt.Errorf("delta kinematics requires 3 rails")
		}
		// Use provided delta configuration or defaults
		radius := cfg.DeltaRadius
		if radius <= 0 {
			radius = 140.0 // Default radius
		}
		armLengths := cfg.DeltaArmLength
		if len(armLengths) != 3 {
			armLengths = []float64{250.0, 250.0, 250.0} // Default arm lengths
		}
		angles := cfg.DeltaAngles
		if len(angles) != 3 {
			angles = []float64{210.0, 330.0, 90.0} // Default angles
		}
		deltaCfg := DeltaConfig{
			Radius:       radius,
			ArmLengths:   armLengths,
			Angles:       angles,
			Endstops:     []float64{cfg.Rails[0].PositionEndstop, cfg.Rails[1].PositionEndstop, cfg.Rails[2].PositionEndstop},
			MinZ:         cfg.Rails[0].PositionMin,
			MaxVelocity:  500.0,
			MaxAccel:     3000.0,
			MaxZVelocity: cfg.MaxZVelocity,
			MaxZAccel:    cfg.MaxZAccel,
		}
		return NewDeltaKinematics(deltaCfg)

	case "winch":
		// Winch kinematics uses cable lengths from anchor points.
		// Anchors should be configured in the stepper sections as anchor_x, anchor_y, anchor_z.
		// For now, use placeholder anchors based on rail bounds.
		anchors := make([][3]float64, len(cfg.Rails))
		for i, rail := range cfg.Rails {
			// Use position_max as a rough approximation of anchor position
			anchors[i] = [3]float64{
				rail.PositionMax,
				rail.PositionMax,
				rail.PositionMax,
			}
		}
		winchCfg := WinchConfig{
			Anchors:      anchors,
			MaxZVelocity: cfg.MaxZVelocity,
			MaxZAccel:    cfg.MaxZAccel,
		}
		return NewWinchKinematics(winchCfg)

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
	case "cartesian", "corexy", "corexz", "delta", "deltesian", "polar", "winch",
		"hybrid_corexy", "hybrid_corexz":
		return true
	default:
		return false
	}
}

// SupportedTypes returns a list of supported kinematic types.
func SupportedTypes() []string {
	return []string{"cartesian", "corexy", "corexz", "delta", "deltesian", "polar", "winch",
		"hybrid_corexy", "hybrid_corexz"}
}