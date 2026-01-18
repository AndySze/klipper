// Safe Z home - port of klippy/extras/safe_z_home.py
//
// Perform Z Homing at specific XY coordinates.
//
// Copyright (C) 2019 Florian Heilmann <Florian.Heilmann@gmx.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
)

// SafeZHome manages safe Z homing behavior.
type SafeZHome struct {
	rt *runtime

	homeXPos       float64 // X position for Z homing
	homeYPos       float64 // Y position for Z homing
	zHop           float64 // Z hop distance before homing
	zHopSpeed      float64 // Z hop movement speed
	maxZ           float64 // maximum Z position
	speed          float64 // XY movement speed
	moveToPrevious bool    // move back to previous XY after Z home

	// Original G28 handler (if any)
	originalG28 func(args map[string]string) error
}

// SafeZHomeConfig holds configuration for safe Z homing.
type SafeZHomeConfig struct {
	HomeXPos       float64
	HomeYPos       float64
	ZHop           float64
	ZHopSpeed      float64
	MaxZ           float64
	Speed          float64
	MoveToPrevious bool
}

// DefaultSafeZHomeConfig returns default safe Z home configuration.
func DefaultSafeZHomeConfig() SafeZHomeConfig {
	return SafeZHomeConfig{
		HomeXPos:       0,
		HomeYPos:       0,
		ZHop:           0,
		ZHopSpeed:      15.0,
		MaxZ:           200.0,
		Speed:          50.0,
		MoveToPrevious: false,
	}
}

// newSafeZHome creates a new safe Z home manager.
func newSafeZHome(rt *runtime, cfg SafeZHomeConfig) *SafeZHome {
	return &SafeZHome{
		rt:             rt,
		homeXPos:       cfg.HomeXPos,
		homeYPos:       cfg.HomeYPos,
		zHop:           cfg.ZHop,
		zHopSpeed:      cfg.ZHopSpeed,
		maxZ:           cfg.MaxZ,
		speed:          cfg.Speed,
		moveToPrevious: cfg.MoveToPrevious,
	}
}

// SetOriginalG28 sets the original G28 handler to call for X/Y homing.
func (szh *SafeZHome) SetOriginalG28(handler func(args map[string]string) error) {
	szh.originalG28 = handler
}

// cmdG28 handles the G28 homing command with safe Z behavior.
func (szh *SafeZHome) cmdG28(args map[string]string) error {
	if szh.rt == nil || szh.rt.toolhead == nil {
		return fmt.Errorf("toolhead not available")
	}

	th := szh.rt.toolhead

	// Determine which axes need homing
	_, needX := args["X"]
	_, needY := args["Y"]
	_, needZ := args["Z"]

	// If no axes specified, home all
	if !needX && !needY && !needZ {
		needX = true
		needY = true
		needZ = true
	}

	// Get current position
	pos := make([]float64, len(th.commandedPos))
	copy(pos, th.commandedPos)

	// Perform Z hop if configured
	if szh.zHop > 0 {
		homedAxes := szh.getHomedAxes()

		if !homedAxes["z"] {
			// Z not homed - set position and hop
			pos[2] = 0
			if err := th.setPosition(pos, "z"); err != nil {
				return fmt.Errorf("set position failed: %w", err)
			}
			if err := th.manualMove([]float64{pos[0], pos[1], szh.zHop}, szh.zHopSpeed); err != nil {
				return fmt.Errorf("z hop failed: %w", err)
			}
			szh.clearHomingState("z")
		} else if pos[2] < szh.zHop {
			// Z is homed but below z_hop - lift to z_hop
			if err := th.manualMove([]float64{pos[0], pos[1], szh.zHop}, szh.zHopSpeed); err != nil {
				return fmt.Errorf("z hop failed: %w", err)
			}
		}
	}

	// Home X and Y if needed
	if needX || needY {
		newArgs := make(map[string]string)
		if needX {
			newArgs["X"] = "0"
		}
		if needY {
			newArgs["Y"] = "0"
		}
		if szh.originalG28 != nil {
			if err := szh.originalG28(newArgs); err != nil {
				return err
			}
		}
	}

	// Home Z if needed
	if needZ {
		// Verify X and Y are homed first
		homedAxes := szh.getHomedAxes()
		if !homedAxes["x"] || !homedAxes["y"] {
			return fmt.Errorf("must home X and Y axes first")
		}

		// Save previous position
		prevPos := make([]float64, len(th.commandedPos))
		copy(prevPos, th.commandedPos)

		// Move to safe XY position
		if err := th.manualMove([]float64{szh.homeXPos, szh.homeYPos, prevPos[2]}, szh.speed); err != nil {
			return fmt.Errorf("move to safe position failed: %w", err)
		}

		// Home Z
		if szh.originalG28 != nil {
			zArgs := map[string]string{"Z": "0"}
			if err := szh.originalG28(zArgs); err != nil {
				return err
			}
		}

		// Perform Z hop again for pressure-based probes
		if szh.zHop > 0 {
			curPos := make([]float64, len(th.commandedPos))
			copy(curPos, th.commandedPos)
			if curPos[2] < szh.zHop {
				if err := th.manualMove([]float64{curPos[0], curPos[1], szh.zHop}, szh.zHopSpeed); err != nil {
					return fmt.Errorf("post-home z hop failed: %w", err)
				}
			}
		}

		// Move back to previous XY position if configured
		if szh.moveToPrevious {
			curPos := make([]float64, len(th.commandedPos))
			copy(curPos, th.commandedPos)
			if err := th.manualMove([]float64{prevPos[0], prevPos[1], curPos[2]}, szh.speed); err != nil {
				return fmt.Errorf("move to previous position failed: %w", err)
			}
		}
	}

	return nil
}

// getHomedAxes returns a map of which axes are currently homed.
func (szh *SafeZHome) getHomedAxes() map[string]bool {
	result := map[string]bool{"x": false, "y": false, "z": false}

	if szh.rt == nil || szh.rt.toolhead == nil {
		return result
	}

	// Get homed axes from kinematics if available
	if szh.rt.toolhead.kin != nil {
		status := szh.rt.toolhead.kin.GetStatus()
		if homedStr, ok := status["homed_axes"].(string); ok {
			for _, c := range homedStr {
				switch c {
				case 'x', 'X':
					result["x"] = true
				case 'y', 'Y':
					result["y"] = true
				case 'z', 'Z':
					result["z"] = true
				}
			}
		}
	}

	return result
}

// clearHomingState clears the homing state for an axis.
func (szh *SafeZHome) clearHomingState(axis string) {
	// In a full implementation, this would clear the homing state
	// through the kinematics system
	_ = axis
}

// GetStatus returns the safe Z home status.
func (szh *SafeZHome) GetStatus() map[string]any {
	return map[string]any{
		"home_x_pos":       szh.homeXPos,
		"home_y_pos":       szh.homeYPos,
		"z_hop":            szh.zHop,
		"z_hop_speed":      szh.zHopSpeed,
		"speed":            szh.speed,
		"move_to_previous": szh.moveToPrevious,
	}
}
