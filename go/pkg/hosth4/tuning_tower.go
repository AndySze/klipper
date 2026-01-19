// Tuning Tower - port of klippy/extras/tuning_tower.py
//
// Helper script to adjust parameters based on Z level
//
// Copyright (C) 2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"math"
	"sync"
)

const (
	tuningCancelZDelta = 2.0
)

// TuningTower provides parameter adjustment based on Z height.
type TuningTower struct {
	rt               *runtime
	active           bool
	lastPosition     [4]float64
	lastZ            float64
	start            float64
	factor           float64
	band             float64
	stepDelta        float64
	stepHeight       float64
	skip             float64
	lastCommandValue *float64
	commandFmt       string
	command          string
	parameter        string
	mu               sync.Mutex
}

// TuningTowerConfig holds configuration for tuning tower.
type TuningTowerConfig struct {
	Command    string  // G-code command to adjust
	Parameter  string  // Parameter name
	Start      float64 // Starting value
	Factor     float64 // Value change per mm of Z
	Band       float64 // Z band for quantizing
	StepDelta  float64 // Value change per step
	StepHeight float64 // Z height per step
	Skip       float64 // Z height to skip before starting
}

// newTuningTower creates a new tuning tower.
func newTuningTower(rt *runtime) *TuningTower {
	return &TuningTower{
		rt:     rt,
		lastZ:  -99999999.9,
	}
}

// Start starts a tuning tower test.
func (tt *TuningTower) Start(cfg TuningTowerConfig) error {
	tt.mu.Lock()
	defer tt.mu.Unlock()

	if tt.active {
		tt.endTestLocked()
	}

	// Validate parameters
	if cfg.Factor != 0 && (cfg.StepHeight != 0 || cfg.StepDelta != 0) {
		return fmt.Errorf("cannot specify both FACTOR and STEP_DELTA/STEP_HEIGHT")
	}
	if (cfg.StepDelta != 0) != (cfg.StepHeight != 0) {
		return fmt.Errorf("must specify both STEP_DELTA and STEP_HEIGHT")
	}

	tt.start = cfg.Start
	tt.factor = cfg.Factor
	tt.band = cfg.Band
	tt.stepDelta = cfg.StepDelta
	tt.stepHeight = cfg.StepHeight
	tt.skip = cfg.Skip
	tt.command = cfg.Command
	tt.parameter = cfg.Parameter
	tt.lastZ = -99999999.9
	tt.lastCommandValue = nil
	tt.active = true

	// Build command format
	// Note: Would need to detect traditional vs extended G-code
	tt.commandFmt = fmt.Sprintf("%s %s=%%f", cfg.Command, cfg.Parameter)

	log.Printf("tuning_tower: started test command=%s parameter=%s start=%.3f",
		cfg.Command, cfg.Parameter, cfg.Start)

	return nil
}

// calcValue calculates the parameter value for a given Z height.
func (tt *TuningTower) calcValue(z float64) float64 {
	if tt.skip > 0 {
		z = max(0, z-tt.skip)
	}

	if tt.stepHeight > 0 {
		return tt.start + tt.stepDelta*math.Floor(z/tt.stepHeight)
	}

	if tt.band > 0 {
		z = (math.Floor(z/tt.band) + 0.5) * tt.band
	}

	return tt.start + z*tt.factor
}

// ProcessMove processes a move and updates parameter if needed.
func (tt *TuningTower) ProcessMove(newPos [4]float64, gcodeZ float64) (string, bool) {
	tt.mu.Lock()
	defer tt.mu.Unlock()

	if !tt.active {
		return "", false
	}

	// Check for extrusion move at new Z height
	if newPos[3] > tt.lastPosition[3] && newPos[2] != tt.lastZ {
		z := newPos[2]

		// Check for lower Z (probably new print)
		if z < tt.lastZ-tuningCancelZDelta {
			tt.endTestLocked()
			return "", false
		}

		// Calculate new value
		newVal := tt.calcValue(gcodeZ)
		tt.lastZ = z

		if tt.lastCommandValue == nil || *tt.lastCommandValue != newVal {
			tt.lastCommandValue = &newVal
			cmd := fmt.Sprintf(tt.commandFmt, newVal)
			tt.lastPosition = newPos
			return cmd, true
		}
	}

	tt.lastPosition = newPos
	return "", false
}

// EndTest ends the tuning tower test.
func (tt *TuningTower) EndTest() {
	tt.mu.Lock()
	defer tt.mu.Unlock()
	tt.endTestLocked()
}

// endTestLocked ends the test (must be called with lock held).
func (tt *TuningTower) endTestLocked() {
	if !tt.active {
		return
	}
	tt.active = false
	log.Printf("tuning_tower: ended test")
}

// IsActive returns true if a tuning tower test is active.
func (tt *TuningTower) IsActive() bool {
	tt.mu.Lock()
	defer tt.mu.Unlock()
	return tt.active
}

// GetStatus returns the tuning tower status.
func (tt *TuningTower) GetStatus() map[string]any {
	tt.mu.Lock()
	defer tt.mu.Unlock()

	status := map[string]any{
		"active": tt.active,
	}

	if tt.active {
		status["command"] = tt.command
		status["parameter"] = tt.parameter
		status["start"] = tt.start
		status["factor"] = tt.factor
		status["band"] = tt.band
		status["step_delta"] = tt.stepDelta
		status["step_height"] = tt.stepHeight
		status["skip"] = tt.skip
		if tt.lastCommandValue != nil {
			status["last_value"] = *tt.lastCommandValue
		}
	}

	return status
}
