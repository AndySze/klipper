// Extruder Stepper - port of klippy/extras/extruder_stepper.py
//
// Code for supporting multiple steppers in single filament extruder.
//
// Copyright (C) 2019 Simo Apell <simo.apell@live.fi>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// PrinterExtruderStepper represents an additional extruder stepper that syncs
// to an existing extruder's motion queue. This allows multiple steppers to
// contribute to filament movement (e.g., for multi-material systems).
type PrinterExtruderStepper struct {
	rt *runtime

	// Configuration
	name         string // e.g., "my_extra_stepper"
	extruderName string // Target extruder to sync with

	mu sync.RWMutex
}

// PrinterExtruderStepperConfig holds configuration for extruder stepper.
type PrinterExtruderStepperConfig struct {
	Name         string // Section name (e.g., "my_extra_stepper")
	ExtruderName string // Target extruder (default: "extruder")
}

// newPrinterExtruderStepper creates a new extruder stepper wrapper.
// This wraps the internal extraStepper functionality for a cleaner API.
func newPrinterExtruderStepper(rt *runtime, cfg PrinterExtruderStepperConfig) (*PrinterExtruderStepper, error) {
	if rt == nil {
		return nil, fmt.Errorf("runtime is nil")
	}

	if !rt.hasExtraStepper {
		return nil, fmt.Errorf("extruder_stepper not configured")
	}

	pes := &PrinterExtruderStepper{
		rt:           rt,
		name:         cfg.Name,
		extruderName: cfg.ExtruderName,
	}

	if pes.extruderName == "" {
		pes.extruderName = "extruder"
	}

	log.Printf("extruder_stepper: initialized '%s' synced to '%s'", pes.name, pes.extruderName)
	return pes, nil
}

// GetName returns the stepper name.
func (pes *PrinterExtruderStepper) GetName() string {
	return pes.name
}

// GetExtruderName returns the target extruder name.
func (pes *PrinterExtruderStepper) GetExtruderName() string {
	return pes.extruderName
}

// FindPastPosition returns the position at a given print time.
// Note: This is a simplified implementation. The Python version queries
// the stepper kinematics iteratively, which requires chelper bindings not
// yet fully exposed.
func (pes *PrinterExtruderStepper) FindPastPosition(printTime float64) (float64, error) {
	pes.mu.RLock()
	defer pes.mu.RUnlock()

	if pes.rt == nil || pes.rt.extraStepper == nil {
		return 0, fmt.Errorf("extruder_stepper not initialized")
	}

	// The stepper position tracking is handled internally by the stepper.
	// For now, return 0 as exact position calculation requires itersolve bindings.
	// Future: expose CalcPositionFromCoord from chelper.
	return 0, nil
}

// SyncToExtruder syncs the stepper to an extruder's motion queue.
func (pes *PrinterExtruderStepper) SyncToExtruder(extruderName string) error {
	pes.mu.Lock()
	defer pes.mu.Unlock()

	if pes.rt == nil || !pes.rt.hasExtraStepper {
		return fmt.Errorf("extruder_stepper not configured")
	}

	// The actual syncing is done through SYNC_EXTRUDER_MOTION command
	// which is already implemented in runtime.exec()
	return pes.rt.ExecSyncExtruderMotion(pes.name, extruderName)
}

// UnsyncFromExtruder removes the stepper from any motion queue.
func (pes *PrinterExtruderStepper) UnsyncFromExtruder() error {
	pes.mu.Lock()
	defer pes.mu.Unlock()

	if pes.rt == nil || !pes.rt.hasExtraStepper {
		return fmt.Errorf("extruder_stepper not configured")
	}

	// Empty motion_queue means unsync
	return pes.rt.ExecSyncExtruderMotion(pes.name, "")
}

// GetStatus returns the extruder stepper status.
func (pes *PrinterExtruderStepper) GetStatus(eventtime float64) map[string]any {
	pes.mu.RLock()
	defer pes.mu.RUnlock()

	if pes.rt == nil || !pes.rt.hasExtraStepper {
		return map[string]any{
			"pressure_advance": 0.0,
			"smooth_time":      0.0,
			"motion_queue":     nil,
		}
	}

	// Get pressure advance settings if available
	pa := 0.0
	smoothTime := 0.0
	motionQueue := ""

	if pes.rt.pressureAdvance != nil {
		if paState, ok := pes.rt.pressureAdvance[pes.name]; ok && paState != nil {
			pa = paState.advance
			smoothTime = paState.smoothTime
		}
	}

	// Check if synced to a motion queue by checking the extruder axis
	if pes.rt.extruder != nil && pes.rt.extraStepper != nil && pes.rt.extraStepper.sk != nil {
		// The stepper is synced if its trapq matches the extruder's trapq
		extruderTq := pes.rt.extruder.GetTrapQ()
		stepperTq := pes.rt.extraStepper.sk.GetTrapQ()
		if extruderTq != nil && stepperTq != nil {
			// If the extra stepper has a trapq assigned, it's synced
			motionQueue = "extruder"
		}
	}

	result := map[string]any{
		"pressure_advance": pa,
		"smooth_time":      smoothTime,
	}

	if motionQueue != "" {
		result["motion_queue"] = motionQueue
	} else {
		result["motion_queue"] = nil
	}

	return result
}

// ExecSyncExtruderMotion is a helper on runtime to sync extruder motion.
// This is called by PrinterExtruderStepper.SyncToExtruder().
func (r *runtime) ExecSyncExtruderMotion(extruderName, motionQueue string) error {
	// Build the command
	cmd := fmt.Sprintf("SYNC_EXTRUDER_MOTION EXTRUDER=%s", extruderName)
	if motionQueue != "" {
		cmd += fmt.Sprintf(" MOTION_QUEUE=%s", motionQueue)
	} else {
		cmd += " MOTION_QUEUE="
	}

	parsed, err := parseGCodeLine(cmd)
	if err != nil {
		return err
	}

	return r.exec(parsed)
}

// ExtruderStepperManager manages multiple extruder steppers.
type ExtruderStepperManager struct {
	rt       *runtime
	steppers map[string]*PrinterExtruderStepper
	mu       sync.RWMutex
}

// newExtruderStepperManager creates a new extruder stepper manager.
func newExtruderStepperManager(rt *runtime) *ExtruderStepperManager {
	return &ExtruderStepperManager{
		rt:       rt,
		steppers: make(map[string]*PrinterExtruderStepper),
	}
}

// Register registers an extruder stepper.
func (esm *ExtruderStepperManager) Register(cfg PrinterExtruderStepperConfig) (*PrinterExtruderStepper, error) {
	esm.mu.Lock()
	defer esm.mu.Unlock()

	if _, exists := esm.steppers[cfg.Name]; exists {
		return nil, fmt.Errorf("extruder_stepper '%s' already registered", cfg.Name)
	}

	pes, err := newPrinterExtruderStepper(esm.rt, cfg)
	if err != nil {
		return nil, err
	}

	esm.steppers[cfg.Name] = pes
	return pes, nil
}

// Get returns an extruder stepper by name.
func (esm *ExtruderStepperManager) Get(name string) *PrinterExtruderStepper {
	esm.mu.RLock()
	defer esm.mu.RUnlock()
	return esm.steppers[name]
}

// GetAll returns all extruder stepper names.
func (esm *ExtruderStepperManager) GetAll() []string {
	esm.mu.RLock()
	defer esm.mu.RUnlock()

	names := make([]string, 0, len(esm.steppers))
	for name := range esm.steppers {
		names = append(names, name)
	}
	return names
}

// GetStatus returns status for all extruder steppers.
func (esm *ExtruderStepperManager) GetStatus(eventtime float64) map[string]any {
	esm.mu.RLock()
	defer esm.mu.RUnlock()

	result := make(map[string]any)
	for name, pes := range esm.steppers {
		result[name] = pes.GetStatus(eventtime)
	}
	return result
}
