// Heater/sensor verification code - port of klippy/extras/verify_heater.py
//
// Copyright (C) 2018 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
	"time"
)

const hintThermal = `
See the 'verify_heater' section in docs/Config_Reference.md
for the parameters that control this check.
`

// HeaterCheck monitors a heater for proper operation.
type HeaterCheck struct {
	rt         *runtime
	heaterName string

	// Configuration
	hysteresis    float64 // Temperature tolerance around target
	maxError      float64 // Maximum accumulated error before fault
	heatingGain   float64 // Minimum temperature rise per check period
	checkGainTime float64 // Time to achieve heating gain

	// State
	approachingTarget bool
	startingApproach  bool
	lastTarget        float64
	goalTemp          float64
	error             float64 // Accumulated error
	goalTime          time.Time

	// Control
	running bool
	stopCh  chan struct{}
	mu      sync.Mutex
}

// HeaterCheckConfig holds configuration for a heater check.
type HeaterCheckConfig struct {
	Hysteresis    float64
	MaxError      float64
	HeatingGain   float64
	CheckGainTime float64
}

// DefaultHeaterCheckConfig returns default configuration for a heater.
func DefaultHeaterCheckConfig(heaterName string) HeaterCheckConfig {
	cfg := HeaterCheckConfig{
		Hysteresis:    5.0,
		MaxError:      120.0,
		HeatingGain:   2.0,
		CheckGainTime: 20.0,
	}
	// Heated beds are slower to heat
	if heaterName == "heater_bed" {
		cfg.CheckGainTime = 60.0
	}
	return cfg
}

// newHeaterCheck creates a new heater check monitor.
func newHeaterCheck(rt *runtime, heaterName string, cfg HeaterCheckConfig) *HeaterCheck {
	return &HeaterCheck{
		rt:            rt,
		heaterName:    heaterName,
		hysteresis:    cfg.Hysteresis,
		maxError:      cfg.MaxError,
		heatingGain:   cfg.HeatingGain,
		checkGainTime: cfg.CheckGainTime,
		stopCh:        make(chan struct{}),
	}
}

// Start begins monitoring the heater.
func (hc *HeaterCheck) Start() {
	hc.mu.Lock()
	if hc.running {
		hc.mu.Unlock()
		return
	}
	hc.running = true
	hc.mu.Unlock()

	log.Printf("Starting heater checks for %s", hc.heaterName)
	go hc.checkLoop()
}

// Stop stops monitoring the heater.
func (hc *HeaterCheck) Stop() {
	hc.mu.Lock()
	if !hc.running {
		hc.mu.Unlock()
		return
	}
	hc.running = false
	hc.mu.Unlock()
	close(hc.stopCh)
}

// checkLoop runs the periodic heater check.
func (hc *HeaterCheck) checkLoop() {
	ticker := time.NewTicker(time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-hc.stopCh:
			return
		case <-ticker.C:
			if err := hc.checkEvent(); err != nil {
				log.Printf("Heater check error: %v", err)
				return
			}
		}
	}
}

// getHeaterStatus gets the current temperature and target from the heater manager.
func (hc *HeaterCheck) getHeaterStatus() (temp, target float64, ok bool) {
	if hc.rt.heaterManager == nil {
		return 0, 0, false
	}
	heater, err := hc.rt.heaterManager.GetHeater(hc.heaterName)
	if err != nil || heater == nil {
		return 0, 0, false
	}
	// Use a simple eventtime (0.0 for non-realtime mode)
	temp, target = heater.GetTemp(0.0)
	return temp, target, true
}

// checkEvent performs a single check of the heater status.
func (hc *HeaterCheck) checkEvent() error {
	temp, target, ok := hc.getHeaterStatus()
	if !ok {
		return nil // Heater not available yet
	}

	now := time.Now()

	// Temperature near target - reset checks
	if temp >= target-hc.hysteresis || target <= 0 {
		if hc.approachingTarget && target > 0 {
			log.Printf("Heater %s within range of %.3f", hc.heaterName, target)
		}
		hc.approachingTarget = false
		hc.startingApproach = false
		if temp <= target+hc.hysteresis {
			hc.error = 0
		}
		hc.lastTarget = target
		return nil
	}

	// Accumulate error
	hc.error += (target - hc.hysteresis) - temp

	if !hc.approachingTarget {
		if target != hc.lastTarget {
			// Target changed - reset checks
			log.Printf("Heater %s approaching new target of %.3f", hc.heaterName, target)
			hc.approachingTarget = true
			hc.startingApproach = true
			hc.goalTemp = temp + hc.heatingGain
			hc.goalTime = now.Add(time.Duration(hc.checkGainTime * float64(time.Second)))
		} else if hc.error >= hc.maxError {
			// Failure due to inability to maintain target temperature
			return hc.heaterFault()
		}
	} else if temp >= hc.goalTemp {
		// Temperature approaching target - reset checks
		hc.startingApproach = false
		hc.error = 0
		hc.goalTemp = temp + hc.heatingGain
		hc.goalTime = now.Add(time.Duration(hc.checkGainTime * float64(time.Second)))
	} else if now.After(hc.goalTime) {
		// Temperature is no longer approaching target
		hc.approachingTarget = false
		log.Printf("Heater %s no longer approaching target %.3f", hc.heaterName, target)
	} else if hc.startingApproach {
		if temp+hc.heatingGain < hc.goalTemp {
			hc.goalTemp = temp + hc.heatingGain
		}
	}

	hc.lastTarget = target
	return nil
}

// heaterFault handles a heater fault condition.
func (hc *HeaterCheck) heaterFault() error {
	msg := fmt.Sprintf("Heater %s not heating at expected rate", hc.heaterName)
	log.Printf("ERROR: %s", msg)
	// In a full implementation, this would trigger a printer shutdown
	// For now, we just log and return an error
	return fmt.Errorf("%s%s", msg, hintThermal)
}

// VerifyHeaterManager manages all heater checks.
type VerifyHeaterManager struct {
	rt      *runtime
	checks  map[string]*HeaterCheck
	mu      sync.Mutex
}

// newVerifyHeaterManager creates a new verify heater manager.
func newVerifyHeaterManager(rt *runtime) *VerifyHeaterManager {
	return &VerifyHeaterManager{
		rt:     rt,
		checks: make(map[string]*HeaterCheck),
	}
}

// AddHeater adds a heater to be monitored.
func (vm *VerifyHeaterManager) AddHeater(heaterName string, cfg *HeaterCheckConfig) {
	vm.mu.Lock()
	defer vm.mu.Unlock()

	if _, exists := vm.checks[heaterName]; exists {
		return // Already being monitored
	}

	config := DefaultHeaterCheckConfig(heaterName)
	if cfg != nil {
		if cfg.Hysteresis > 0 {
			config.Hysteresis = cfg.Hysteresis
		}
		if cfg.MaxError > 0 {
			config.MaxError = cfg.MaxError
		}
		if cfg.HeatingGain > 0 {
			config.HeatingGain = cfg.HeatingGain
		}
		if cfg.CheckGainTime > 0 {
			config.CheckGainTime = cfg.CheckGainTime
		}
	}

	hc := newHeaterCheck(vm.rt, heaterName, config)
	vm.checks[heaterName] = hc
}

// Start begins monitoring all heaters.
func (vm *VerifyHeaterManager) Start() {
	vm.mu.Lock()
	defer vm.mu.Unlock()

	for _, hc := range vm.checks {
		hc.Start()
	}
}

// Stop stops monitoring all heaters.
func (vm *VerifyHeaterManager) Stop() {
	vm.mu.Lock()
	defer vm.mu.Unlock()

	for _, hc := range vm.checks {
		hc.Stop()
	}
}

// GetStatus returns the status of all heater checks.
func (vm *VerifyHeaterManager) GetStatus() map[string]any {
	vm.mu.Lock()
	defer vm.mu.Unlock()

	status := make(map[string]any)
	for name, hc := range vm.checks {
		status[name] = map[string]any{
			"approaching_target": hc.approachingTarget,
			"error":              hc.error,
		}
	}
	return status
}
