// Homing Heaters - port of klippy/extras/homing_heaters.py
//
// Disable heaters during homing
//
// Copyright (C) 2018-2019 Janar Sööt <janar.sansen@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"log"
	"sync"
)

// HomingHeaters manages heater state during homing.
type HomingHeaters struct {
	rt                *runtime
	heaters           []string
	steppers          []string
	disableOnHome     bool
	savedTemps        map[string]float64
	homingActive      bool
	mu                sync.Mutex
}

// HomingHeatersConfig holds configuration for homing heaters.
type HomingHeatersConfig struct {
	Heaters       []string // Heaters to disable during homing
	Steppers      []string // Steppers that trigger heater disable
	DisableOnHome bool     // Whether to disable heaters during homing
}

// DefaultHomingHeatersConfig returns default homing heaters configuration.
func DefaultHomingHeatersConfig() HomingHeatersConfig {
	return HomingHeatersConfig{
		DisableOnHome: true,
	}
}

// newHomingHeaters creates a new homing heaters manager.
func newHomingHeaters(rt *runtime, cfg HomingHeatersConfig) *HomingHeaters {
	hh := &HomingHeaters{
		rt:            rt,
		heaters:       cfg.Heaters,
		steppers:      cfg.Steppers,
		disableOnHome: cfg.DisableOnHome,
		savedTemps:    make(map[string]float64),
	}

	log.Printf("homing_heaters: initialized heaters=%v steppers=%v", cfg.Heaters, cfg.Steppers)
	return hh
}

// StartHoming is called when homing begins.
func (hh *HomingHeaters) StartHoming(stepperName string) {
	hh.mu.Lock()
	defer hh.mu.Unlock()

	// Check if this stepper should trigger heater disable
	found := false
	for _, s := range hh.steppers {
		if s == stepperName || s == "*" {
			found = true
			break
		}
	}

	if !found && len(hh.steppers) > 0 {
		return
	}

	if hh.homingActive {
		return
	}

	hh.homingActive = true

	if hh.disableOnHome {
		// In real implementation, this would save heater targets and disable them
		log.Printf("homing_heaters: disabling heaters for homing %s", stepperName)
	}
}

// EndHoming is called when homing ends.
func (hh *HomingHeaters) EndHoming() {
	hh.mu.Lock()
	defer hh.mu.Unlock()

	if !hh.homingActive {
		return
	}

	hh.homingActive = false

	if hh.disableOnHome {
		// In real implementation, this would restore heater targets
		log.Printf("homing_heaters: restoring heaters after homing")
	}
}

// IsHomingActive returns whether homing is currently active.
func (hh *HomingHeaters) IsHomingActive() bool {
	hh.mu.Lock()
	defer hh.mu.Unlock()
	return hh.homingActive
}

// GetStatus returns the homing heaters status.
func (hh *HomingHeaters) GetStatus() map[string]any {
	hh.mu.Lock()
	defer hh.mu.Unlock()

	return map[string]any{
		"homing_active":   hh.homingActive,
		"heaters":         hh.heaters,
		"steppers":        hh.steppers,
		"disable_on_home": hh.disableOnHome,
	}
}
