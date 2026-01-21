// Homing Override - port of klippy/extras/homing_override.py
//
// Run user defined actions in place of a normal G28 homing command
//
// Copyright (C) 2018 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"strings"
	"sync"
)

// HomingOverrideHandler allows custom gcode to be executed instead of normal G28 homing.
// Note: HomingOverride struct is defined in macros.go for config parsing.
type HomingOverrideHandler struct {
	rt *runtime

	// Configuration
	startPos [3]*float64 // set_position_x/y/z (nil if not set)
	axes     string      // Axes that trigger override (e.g., "XYZ")
	gcode    []string    // Gcode lines to execute

	// State
	inScript bool
	mu       sync.Mutex
}

// newHomingOverrideHandler creates a new homing override handler from parsed config.
func newHomingOverrideHandler(rt *runtime, cfg *HomingOverride) (*HomingOverrideHandler, error) {
	if cfg == nil || len(cfg.Gcode) == 0 {
		return nil, fmt.Errorf("homing_override: gcode is required")
	}

	axes := strings.ToUpper(cfg.Axes)
	if axes == "" {
		axes = "XYZ"
	}

	ho := &HomingOverrideHandler{
		rt:    rt,
		axes:  axes,
		gcode: cfg.Gcode,
	}

	// Set start positions
	ho.startPos[0] = cfg.SetPositionX
	ho.startPos[1] = cfg.SetPositionY
	ho.startPos[2] = cfg.SetPositionZ

	log.Printf("homing_override: initialized with axes=%s, %d gcode lines", ho.axes, len(cfg.Gcode))
	return ho, nil
}

// HandleG28 handles G28 command, potentially using override.
// Returns true if override was used, false if normal homing should proceed.
func (ho *HomingOverrideHandler) HandleG28(cmd *gcodeCommand, normalHome func() error) error {
	ho.mu.Lock()
	defer ho.mu.Unlock()

	// If called recursively (from within override script), use normal homing
	if ho.inScript {
		return normalHome()
	}

	// Determine if we should override
	override := ho.shouldOverride(cmd)
	if !override {
		return normalHome()
	}

	// Calculate forced position (if configured)
	if ho.rt != nil && ho.rt.toolhead != nil {
		pos := make([]float64, len(ho.rt.toolhead.commandedPos))
		copy(pos, ho.rt.toolhead.commandedPos)

		homingAxes := ""
		for i, loc := range ho.startPos {
			if loc != nil {
				pos[i] = *loc
				homingAxes += string("xyz"[i])
			}
		}

		if homingAxes != "" {
			if err := ho.rt.toolhead.setPosition(pos, homingAxes); err != nil {
				return fmt.Errorf("homing_override: set position failed: %w", err)
			}
		}
	}

	// Execute override gcode
	ho.inScript = true
	defer func() { ho.inScript = false }()

	return ho.executeGcode()
}

// shouldOverride determines if the G28 command should trigger override.
func (ho *HomingOverrideHandler) shouldOverride(cmd *gcodeCommand) bool {
	// Check if any axis parameter is given
	hasAxisParam := false
	for _, axis := range "XYZ" {
		if _, ok := cmd.Args[string(axis)]; ok {
			hasAxisParam = true
			break
		}
	}

	// If no axis parameter, always override
	if !hasAxisParam {
		return true
	}

	// Check if any of the override axes are being homed
	for _, axis := range ho.axes {
		if _, ok := cmd.Args[string(axis)]; ok {
			return true
		}
	}

	return false
}

// executeGcode executes the stored gcode lines.
func (ho *HomingOverrideHandler) executeGcode() error {
	for _, line := range ho.gcode {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, "#") || strings.HasPrefix(line, ";") {
			continue
		}

		parsed, err := parseGCodeLine(line)
		if err != nil {
			return fmt.Errorf("homing_override: parse error: %w", err)
		}
		if parsed != nil {
			if err := ho.rt.exec(parsed); err != nil {
				return fmt.Errorf("homing_override: exec error: %w", err)
			}
		}
	}
	return nil
}

// GetAxes returns the axes that trigger override.
func (ho *HomingOverrideHandler) GetAxes() string {
	return ho.axes
}

// GetStartPos returns the start position for an axis (nil if not set).
func (ho *HomingOverrideHandler) GetStartPos(axis int) *float64 {
	if axis < 0 || axis >= 3 {
		return nil
	}
	return ho.startPos[axis]
}

// GetStatus returns the homing override status.
func (ho *HomingOverrideHandler) GetStatus() map[string]any {
	ho.mu.Lock()
	defer ho.mu.Unlock()

	return map[string]any{
		"axes":      ho.axes,
		"in_script": ho.inScript,
	}
}

// HomingOverrideManager provides management of homing override.
type HomingOverrideManager struct {
	rt       *runtime
	override *HomingOverrideHandler
	mu       sync.RWMutex
}

// newHomingOverrideManager creates a new homing override manager.
func newHomingOverrideManager(rt *runtime) *HomingOverrideManager {
	return &HomingOverrideManager{
		rt: rt,
	}
}

// Configure sets up the homing override from parsed config.
func (hom *HomingOverrideManager) Configure(cfg *HomingOverride) error {
	hom.mu.Lock()
	defer hom.mu.Unlock()

	if cfg == nil || len(cfg.Gcode) == 0 {
		// No override configured
		hom.override = nil
		return nil
	}

	ho, err := newHomingOverrideHandler(hom.rt, cfg)
	if err != nil {
		return err
	}

	hom.override = ho
	return nil
}

// HandleG28 handles G28 command with potential override.
func (hom *HomingOverrideManager) HandleG28(cmd *gcodeCommand, normalHome func() error) error {
	hom.mu.RLock()
	override := hom.override
	hom.mu.RUnlock()

	if override == nil {
		return normalHome()
	}

	return override.HandleG28(cmd, normalHome)
}

// IsConfigured returns true if homing override is configured.
func (hom *HomingOverrideManager) IsConfigured() bool {
	hom.mu.RLock()
	defer hom.mu.RUnlock()
	return hom.override != nil
}

// GetStatus returns the homing override status.
func (hom *HomingOverrideManager) GetStatus() map[string]any {
	hom.mu.RLock()
	defer hom.mu.RUnlock()

	if hom.override == nil {
		return map[string]any{
			"enabled": false,
		}
	}

	status := hom.override.GetStatus()
	status["enabled"] = true
	return status
}
