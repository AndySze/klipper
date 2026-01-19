// Endstop Phase - port of klippy/extras/endstop_phase.py
//
// Endstop accuracy improvement via stepper phase tracking
//
// Copyright (C) 2016-2021 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"log"
	"math"
	"sort"
	"sync"
)

// TrinamicDrivers lists the supported Trinamic driver types.
var TrinamicDrivers = []string{"tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160"}

// PhaseCalc calculates the trigger phase of a stepper motor.
type PhaseCalc struct {
	name            string
	phases          int
	phaseHistory    []int
	lastPhase       int
	lastMCUPosition int64
	isPrimary       bool
	statsOnly       bool
	mu              sync.RWMutex
}

// newPhaseCalc creates a new phase calculator.
func newPhaseCalc(name string, phases int) *PhaseCalc {
	pc := &PhaseCalc{
		name:   name,
		phases: phases,
	}
	if phases > 0 {
		pc.phaseHistory = make([]int, phases)
	}
	return pc
}

// ConvertPhase converts a driver phase to internal phase.
func (pc *PhaseCalc) ConvertPhase(driverPhase, driverPhases int) int {
	if pc.phases == 0 || driverPhases == 0 {
		return 0
	}
	return int(float64(driverPhase)/float64(driverPhases)*float64(pc.phases)+0.5) % pc.phases
}

// CalcPhase calculates and records the phase from MCU position.
func (pc *PhaseCalc) CalcPhase(trigMCUPos int64, mcuPhaseOffset int) int {
	pc.mu.Lock()
	defer pc.mu.Unlock()

	if pc.phases == 0 {
		return 0
	}

	phase := int((trigMCUPos + int64(mcuPhaseOffset)) % int64(pc.phases))
	if phase < 0 {
		phase += pc.phases
	}

	if pc.phaseHistory != nil {
		pc.phaseHistory[phase]++
	}
	pc.lastPhase = phase
	pc.lastMCUPosition = trigMCUPos

	return phase
}

// GetPhaseHistory returns a copy of the phase history.
func (pc *PhaseCalc) GetPhaseHistory() []int {
	pc.mu.RLock()
	defer pc.mu.RUnlock()

	if pc.phaseHistory == nil {
		return nil
	}
	result := make([]int, len(pc.phaseHistory))
	copy(result, pc.phaseHistory)
	return result
}

// GetLastPhase returns the last calculated phase.
func (pc *PhaseCalc) GetLastPhase() int {
	pc.mu.RLock()
	defer pc.mu.RUnlock()
	return pc.lastPhase
}

// GetLastMCUPosition returns the last MCU position.
func (pc *PhaseCalc) GetLastMCUPosition() int64 {
	pc.mu.RLock()
	defer pc.mu.RUnlock()
	return pc.lastMCUPosition
}

// EndstopPhase provides adjusted endstop trigger positions.
type EndstopPhase struct {
	rt                    *runtime
	name                  string
	stepDist              float64
	phases                int
	endstopPhase          int
	endstopPhaseSet       bool
	endstopAlignZero      bool
	endstopPhaseAccuracy  int
	phaseCalc             *PhaseCalc
	mu                    sync.RWMutex
}

// EndstopPhaseConfig holds configuration for endstop phase.
type EndstopPhaseConfig struct {
	Name                 string
	StepDist             float64
	Microsteps           int
	TriggerPhase         *int    // Optional: known trigger phase
	TriggerPhases        int     // Total phases (e.g., 64 for 16 microsteps * 4)
	EndstopAlignZero     bool
	EndstopAccuracy      *float64 // Optional: accuracy in mm
}

// newEndstopPhase creates a new endstop phase tracker.
func newEndstopPhase(rt *runtime, cfg EndstopPhaseConfig) (*EndstopPhase, error) {
	phases := cfg.Microsteps * 4
	if phases == 0 {
		phases = 64 // Default for 16 microsteps
	}

	ep := &EndstopPhase{
		rt:               rt,
		name:             cfg.Name,
		stepDist:         cfg.StepDist,
		phases:           phases,
		endstopAlignZero: cfg.EndstopAlignZero,
		phaseCalc:        newPhaseCalc(cfg.Name, phases),
	}

	// Set trigger phase if provided
	if cfg.TriggerPhase != nil && cfg.TriggerPhases > 0 {
		ep.endstopPhase = ep.phaseCalc.ConvertPhase(*cfg.TriggerPhase, cfg.TriggerPhases)
		ep.endstopPhaseSet = true
	}

	// Calculate phase accuracy
	if cfg.EndstopAccuracy != nil {
		if ep.endstopPhaseSet {
			ep.endstopPhaseAccuracy = int(math.Ceil(*cfg.EndstopAccuracy * 0.5 / cfg.StepDist))
		} else {
			ep.endstopPhaseAccuracy = int(math.Ceil(*cfg.EndstopAccuracy / cfg.StepDist))
		}
	} else {
		ep.endstopPhaseAccuracy = phases/2 - 1
	}

	log.Printf("endstop_phase: initialized '%s' with %d phases, accuracy=%d",
		cfg.Name, phases, ep.endstopPhaseAccuracy)

	return ep, nil
}

// AlignEndstop calculates position adjustment to align 0.0 to a full step.
func (ep *EndstopPhase) AlignEndstop(positionEndstop float64) float64 {
	ep.mu.RLock()
	defer ep.mu.RUnlock()

	if !ep.endstopAlignZero || !ep.endstopPhaseSet {
		return 0
	}

	microsteps := ep.phases / 4
	halfMicrosteps := microsteps / 2
	phaseOffset := float64((((ep.endstopPhase+halfMicrosteps)%microsteps)-halfMicrosteps)) * ep.stepDist
	fullStep := float64(microsteps) * ep.stepDist

	return float64(int(positionEndstop/fullStep+0.5))*fullStep - positionEndstop + phaseOffset
}

// GetHomedOffset calculates the homing offset based on stepper phase.
func (ep *EndstopPhase) GetHomedOffset(trigMCUPos int64, mcuPhaseOffset int) (float64, error) {
	ep.mu.Lock()
	defer ep.mu.Unlock()

	phase := ep.phaseCalc.CalcPhase(trigMCUPos, mcuPhaseOffset)

	if !ep.endstopPhaseSet {
		log.Printf("endstop_phase: setting '%s' endstop phase to %d", ep.name, phase)
		ep.endstopPhase = phase
		ep.endstopPhaseSet = true
		return 0, nil
	}

	delta := (phase - ep.endstopPhase) % ep.phases
	if delta >= ep.phases-ep.endstopPhaseAccuracy {
		delta -= ep.phases
	} else if delta > ep.endstopPhaseAccuracy {
		return 0, &EndstopPhaseError{
			Name:     ep.name,
			Got:      phase,
			Expected: ep.endstopPhase,
		}
	}

	return float64(delta) * ep.stepDist, nil
}

// GetStatus returns the endstop phase status.
func (ep *EndstopPhase) GetStatus() map[string]any {
	ep.mu.RLock()
	defer ep.mu.RUnlock()

	return map[string]any{
		"name":          ep.name,
		"phases":        ep.phases,
		"endstop_phase": ep.endstopPhase,
		"phase_set":     ep.endstopPhaseSet,
		"last_phase":    ep.phaseCalc.GetLastPhase(),
	}
}

// EndstopPhaseError indicates an incorrect endstop phase.
type EndstopPhaseError struct {
	Name     string
	Got      int
	Expected int
}

func (e *EndstopPhaseError) Error() string {
	return "endstop " + e.Name + " incorrect phase"
}

// EndstopPhases manages endstop phase tracking for all steppers.
type EndstopPhases struct {
	rt       *runtime
	tracking map[string]*PhaseCalc
	mu       sync.RWMutex
}

// newEndstopPhases creates a new endstop phases manager.
func newEndstopPhases(rt *runtime) *EndstopPhases {
	return &EndstopPhases{
		rt:       rt,
		tracking: make(map[string]*PhaseCalc),
	}
}

// UpdateStepper updates phase tracking for a stepper.
func (eps *EndstopPhases) UpdateStepper(stepperName string, trigMCUPos int64, mcuPhaseOffset, phases int, isPrimary bool) {
	eps.mu.Lock()
	defer eps.mu.Unlock()

	pc := eps.tracking[stepperName]
	if pc == nil {
		pc = newPhaseCalc(stepperName, phases)
		pc.statsOnly = true
		eps.tracking[stepperName] = pc
	}

	if isPrimary {
		pc.isPrimary = true
	}

	if pc.phaseHistory != nil {
		pc.CalcPhase(trigMCUPos, mcuPhaseOffset)
	}
}

// GenerateStats generates phase statistics for a stepper.
func (eps *EndstopPhases) GenerateStats(stepperName string) (bestPhase, phases int) {
	eps.mu.RLock()
	pc := eps.tracking[stepperName]
	eps.mu.RUnlock()

	if pc == nil || pc.phaseHistory == nil {
		return 0, 0
	}

	phaseHistory := pc.GetPhaseHistory()
	phases = len(phaseHistory)
	if phases == 0 {
		return 0, 0
	}

	halfPhases := phases / 2

	// Wrap phase history
	wph := make([]int, phases*2)
	copy(wph, phaseHistory)
	copy(wph[phases:], phaseHistory)

	// Find best phase (minimum cost)
	type costEntry struct {
		cost  int
		phase int
	}

	costs := make([]costEntry, phases)
	for i := 0; i < phases; i++ {
		phase := i + halfPhases
		cost := 0
		for j := i; j < i+phases; j++ {
			cost += wph[j] * absInt(j-phase)
		}
		costs[i] = costEntry{cost: cost, phase: phase}
	}

	sort.Slice(costs, func(i, j int) bool {
		return costs[i].cost < costs[j].cost
	})

	bestPhase = costs[0].phase % phases

	// Find range
	best := costs[0].phase
	var lo, hi int
	for j := best - halfPhases; j < best+halfPhases; j++ {
		if wph[j] > 0 {
			if lo == 0 {
				lo = j
			}
			hi = j
		}
	}
	lo = lo % phases
	hi = hi % phases

	log.Printf("endstop_phase: %s trigger_phase=%d/%d (range %d to %d)",
		stepperName, bestPhase, phases, lo, hi)

	return bestPhase, phases
}

// GetStatus returns the endstop phases status.
func (eps *EndstopPhases) GetStatus() map[string]any {
	eps.mu.RLock()
	defer eps.mu.RUnlock()

	lastHome := make(map[string]map[string]any)
	for name, pc := range eps.tracking {
		if pc.phaseHistory != nil {
			lastHome[name] = map[string]any{
				"phase":        pc.lastPhase,
				"phases":       pc.phases,
				"mcu_position": pc.lastMCUPosition,
			}
		}
	}

	return map[string]any{
		"last_home": lastHome,
	}
}

func absInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}
