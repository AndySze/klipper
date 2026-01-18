// Firmware retraction - port of klippy/extras/firmware_retraction.py
//
// Support for Marlin/Smoothie/Reprap style firmware retraction via G10/G11
//
// Copyright (C) 2019 Len Trigg <lenbok@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"strconv"
)

// FirmwareRetraction manages firmware retraction settings.
type FirmwareRetraction struct {
	rt *runtime
	gm *gcodeMove

	retractLength        float64 // length to retract in mm
	retractSpeed         float64 // retract speed in mm/s
	unretractExtraLength float64 // extra length on unretract
	unretractSpeed       float64 // unretract speed in mm/s
	unretractLength      float64 // total unretract length (computed)
	isRetracted          bool    // whether currently retracted
}

// FirmwareRetractionConfig holds configuration for firmware retraction.
type FirmwareRetractionConfig struct {
	RetractLength        float64
	RetractSpeed         float64
	UnretractExtraLength float64
	UnretractSpeed       float64
}

// DefaultFirmwareRetractionConfig returns default firmware retraction settings.
func DefaultFirmwareRetractionConfig() FirmwareRetractionConfig {
	return FirmwareRetractionConfig{
		RetractLength:        0.0,
		RetractSpeed:         20.0,
		UnretractExtraLength: 0.0,
		UnretractSpeed:       10.0,
	}
}

// newFirmwareRetraction creates a new firmware retraction manager.
func newFirmwareRetraction(rt *runtime, gm *gcodeMove, cfg FirmwareRetractionConfig) *FirmwareRetraction {
	fr := &FirmwareRetraction{
		rt:                   rt,
		gm:                   gm,
		retractLength:        cfg.RetractLength,
		retractSpeed:         cfg.RetractSpeed,
		unretractExtraLength: cfg.UnretractExtraLength,
		unretractSpeed:       cfg.UnretractSpeed,
		isRetracted:          false,
	}
	fr.unretractLength = fr.retractLength + fr.unretractExtraLength
	return fr
}

// GetStatus returns the current firmware retraction status.
func (fr *FirmwareRetraction) GetStatus() map[string]any {
	return map[string]any{
		"retract_length":         fr.retractLength,
		"retract_speed":          fr.retractSpeed,
		"unretract_extra_length": fr.unretractExtraLength,
		"unretract_speed":        fr.unretractSpeed,
	}
}

// cmdSetRetraction handles the SET_RETRACTION command.
func (fr *FirmwareRetraction) cmdSetRetraction(args map[string]string) error {
	if v, ok := args["RETRACT_LENGTH"]; ok {
		val, err := strconv.ParseFloat(v, 64)
		if err != nil {
			return fmt.Errorf("invalid RETRACT_LENGTH: %w", err)
		}
		if val < 0 {
			return fmt.Errorf("RETRACT_LENGTH must be >= 0")
		}
		fr.retractLength = val
	}

	if v, ok := args["RETRACT_SPEED"]; ok {
		val, err := strconv.ParseFloat(v, 64)
		if err != nil {
			return fmt.Errorf("invalid RETRACT_SPEED: %w", err)
		}
		if val < 1 {
			return fmt.Errorf("RETRACT_SPEED must be >= 1")
		}
		fr.retractSpeed = val
	}

	if v, ok := args["UNRETRACT_EXTRA_LENGTH"]; ok {
		val, err := strconv.ParseFloat(v, 64)
		if err != nil {
			return fmt.Errorf("invalid UNRETRACT_EXTRA_LENGTH: %w", err)
		}
		if val < 0 {
			return fmt.Errorf("UNRETRACT_EXTRA_LENGTH must be >= 0")
		}
		fr.unretractExtraLength = val
	}

	if v, ok := args["UNRETRACT_SPEED"]; ok {
		val, err := strconv.ParseFloat(v, 64)
		if err != nil {
			return fmt.Errorf("invalid UNRETRACT_SPEED: %w", err)
		}
		if val < 1 {
			return fmt.Errorf("UNRETRACT_SPEED must be >= 1")
		}
		fr.unretractSpeed = val
	}

	// Update computed values
	fr.unretractLength = fr.retractLength + fr.unretractExtraLength
	fr.isRetracted = false

	return nil
}

// cmdGetRetraction handles the GET_RETRACTION command.
func (fr *FirmwareRetraction) cmdGetRetraction() string {
	return fmt.Sprintf("RETRACT_LENGTH=%.5f RETRACT_SPEED=%.5f UNRETRACT_EXTRA_LENGTH=%.5f UNRETRACT_SPEED=%.5f",
		fr.retractLength, fr.retractSpeed, fr.unretractExtraLength, fr.unretractSpeed)
}

// cmdG10 handles the G10 retract command.
func (fr *FirmwareRetraction) cmdG10() error {
	if fr.isRetracted {
		return nil
	}
	if fr.retractLength <= 0 {
		return nil
	}

	// Save state, move to relative, retract, restore state
	if fr.gm != nil {
		fr.gm.saveState("_retract_state")

		// Set relative extrusion
		wasAbsolute := fr.gm.absoluteExtrude
		fr.gm.absoluteExtrude = false

		// Perform retraction (negative E move)
		if err := fr.doExtruderMove(-fr.retractLength, fr.retractSpeed); err != nil {
			fr.gm.absoluteExtrude = wasAbsolute
			return err
		}

		// Restore absolute mode
		fr.gm.absoluteExtrude = wasAbsolute
		fr.gm.restoreState("_retract_state", false, 0)
	}

	fr.isRetracted = true
	return nil
}

// cmdG11 handles the G11 unretract command.
func (fr *FirmwareRetraction) cmdG11() error {
	if !fr.isRetracted {
		return nil
	}
	if fr.unretractLength <= 0 {
		return nil
	}

	// Save state, move to relative, unretract, restore state
	if fr.gm != nil {
		fr.gm.saveState("_retract_state")

		// Set relative extrusion
		wasAbsolute := fr.gm.absoluteExtrude
		fr.gm.absoluteExtrude = false

		// Perform unretraction (positive E move)
		if err := fr.doExtruderMove(fr.unretractLength, fr.unretractSpeed); err != nil {
			fr.gm.absoluteExtrude = wasAbsolute
			return err
		}

		// Restore absolute mode
		fr.gm.absoluteExtrude = wasAbsolute
		fr.gm.restoreState("_retract_state", false, 0)
	}

	fr.isRetracted = false
	return nil
}

// doExtruderMove performs an extruder-only move.
func (fr *FirmwareRetraction) doExtruderMove(distance, speed float64) error {
	if fr.gm == nil {
		return nil
	}

	// Get current position
	pos := fr.gm.getGcodePosition()
	if len(pos) < 4 {
		return fmt.Errorf("insufficient position data for extruder move")
	}

	// Create new position with E offset
	newPos := make([]float64, len(pos))
	copy(newPos, pos)
	newPos[3] += distance

	// Convert speed from mm/s to mm/min for gcode
	feedrate := speed * 60

	// Use the gcode move's move function
	if fr.gm.moveWithTransform != nil {
		return fr.gm.moveWithTransform(newPos, feedrate/60.0)
	}

	return nil
}
