// Homing - port of klippy/extras/homing.py
//
// Helper code for implementing homing operations
//
// Copyright (C) 2016-2024 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"log"
	"sync"
)

const (
	HomingStartDelay    = 0.001
	EndstopSampleTime   = 0.000015
	EndstopSampleCount  = 4
)

// HomingState tracks homing request state.
type HomingState struct {
	rt            *runtime
	changedAxes   []int
	triggerMCUPos map[string]int64
	adjustPos     map[string]float64
	mu            sync.Mutex
}

// newHomingState creates a new homing state tracker.
func newHomingState(rt *runtime) *HomingState {
	return &HomingState{
		rt:            rt,
		changedAxes:   []int{},
		triggerMCUPos: make(map[string]int64),
		adjustPos:     make(map[string]float64),
	}
}

// SetAxes sets the axes being homed.
func (hs *HomingState) SetAxes(axes []int) {
	hs.mu.Lock()
	defer hs.mu.Unlock()
	hs.changedAxes = axes
}

// GetAxes returns the axes being homed.
func (hs *HomingState) GetAxes() []int {
	hs.mu.Lock()
	defer hs.mu.Unlock()
	return hs.changedAxes
}

// GetTriggerPosition returns the trigger MCU position for a stepper.
func (hs *HomingState) GetTriggerPosition(stepperName string) int64 {
	hs.mu.Lock()
	defer hs.mu.Unlock()
	return hs.triggerMCUPos[stepperName]
}

// SetStepperAdjustment sets a homing adjustment for a stepper.
func (hs *HomingState) SetStepperAdjustment(stepperName string, adjustment float64) {
	hs.mu.Lock()
	defer hs.mu.Unlock()
	hs.adjustPos[stepperName] = adjustment
}

// SetHomedPosition sets the toolhead position after homing.
func (hs *HomingState) SetHomedPosition(pos []float64) error {
	if hs.rt == nil || hs.rt.toolhead == nil {
		return nil
	}
	return hs.rt.toolhead.setPosition(pos, "")
}

// PrinterHoming provides G28 homing command support.
type PrinterHoming struct {
	rt *runtime
	mu sync.Mutex
}

// newPrinterHoming creates a new printer homing handler.
func newPrinterHoming(rt *runtime) *PrinterHoming {
	return &PrinterHoming{rt: rt}
}

// HomingInfo holds homing configuration for a rail.
type HomingInfo struct {
	Speed             float64
	PositionEndstop   float64
	RetractDist       float64
	RetractSpeed      float64
	SecondHomingSpeed float64
	PositionMin       float64
	PositionMax       float64
	PositiveDir       bool
}

// DefaultHomingInfo returns default homing configuration.
func DefaultHomingInfo() HomingInfo {
	return HomingInfo{
		Speed:             5.0,
		RetractDist:       5.0,
		RetractSpeed:      25.0,
		SecondHomingSpeed: 2.5,
	}
}

// Home performs a homing sequence for the given axes.
// This is a simplified implementation - the full implementation requires
// more complex stepper position tracking.
func (ph *PrinterHoming) Home(axes []int, homingState *HomingState) error {
	ph.mu.Lock()
	defer ph.mu.Unlock()

	if ph.rt == nil || ph.rt.toolhead == nil {
		return nil
	}

	homingState.SetAxes(axes)
	log.Printf("homing: starting home for axes %v", axes)
	return nil
}

// GetStatus returns the homing status.
func (ph *PrinterHoming) GetStatus() map[string]any {
	return map[string]any{}
}
