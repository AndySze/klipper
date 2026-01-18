// Delayed gcode - port of klippy/extras/delayed_gcode.py
//
// A simple timer for executing gcode templates
//
// Copyright (C) 2019 Eric Callahan <arksine.code@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"strconv"
	"sync"
	"time"
)

// DelayedGcode manages a timer for executing gcode after a delay.
type DelayedGcode struct {
	rt   *runtime
	name string

	gcode           []string      // gcode commands to execute
	duration        float64       // delay in seconds
	timer           *time.Timer   // current timer
	mu              sync.Mutex    // protects timer state
	insideTimer     bool          // true if currently executing timer gcode
	repeat          bool          // true if timer should repeat
	stopChan        chan struct{} // channel to signal stop
	gcodeRunner     func([]string) error // function to run gcode
}

// DelayedGcodeConfig holds configuration for a delayed gcode instance.
type DelayedGcodeConfig struct {
	Name            string   // unique name for this delayed_gcode
	Gcode           []string // gcode commands to execute
	InitialDuration float64  // initial delay (0 = disabled)
}

// newDelayedGcode creates a new delayed gcode manager.
func newDelayedGcode(rt *runtime, cfg DelayedGcodeConfig) *DelayedGcode {
	dg := &DelayedGcode{
		rt:       rt,
		name:     cfg.Name,
		gcode:    cfg.Gcode,
		duration: cfg.InitialDuration,
		stopChan: make(chan struct{}),
	}
	return dg
}

// SetGcodeRunner sets the function used to run gcode commands.
func (dg *DelayedGcode) SetGcodeRunner(f func([]string) error) {
	dg.gcodeRunner = f
}

// Start begins the delayed gcode timer if duration > 0.
func (dg *DelayedGcode) Start() {
	dg.mu.Lock()
	defer dg.mu.Unlock()

	if dg.duration <= 0 {
		return
	}

	dg.startTimer()
}

// Stop stops the delayed gcode timer.
func (dg *DelayedGcode) Stop() {
	dg.mu.Lock()
	defer dg.mu.Unlock()

	if dg.timer != nil {
		dg.timer.Stop()
		dg.timer = nil
	}
}

// startTimer starts or restarts the timer.
func (dg *DelayedGcode) startTimer() {
	if dg.timer != nil {
		dg.timer.Stop()
	}

	dg.timer = time.AfterFunc(time.Duration(dg.duration*float64(time.Second)), func() {
		dg.timerEvent()
	})
}

// timerEvent is called when the timer fires.
func (dg *DelayedGcode) timerEvent() {
	dg.mu.Lock()
	dg.insideTimer = true
	gcode := dg.gcode
	runner := dg.gcodeRunner
	dg.mu.Unlock()

	// Execute gcode
	if runner != nil && len(gcode) > 0 {
		if err := runner(gcode); err != nil {
			log.Printf("delayed_gcode %s: error running script: %v", dg.name, err)
		}
	}

	dg.mu.Lock()
	defer dg.mu.Unlock()

	// Check if we should repeat
	if dg.repeat && dg.duration > 0 {
		dg.startTimer()
	} else {
		dg.timer = nil
	}
	dg.insideTimer = false
	dg.repeat = false
}

// cmdUpdateDelayedGcode handles the UPDATE_DELAYED_GCODE command.
// UPDATE_DELAYED_GCODE ID=<name> DURATION=<seconds>
func (dg *DelayedGcode) cmdUpdateDelayedGcode(args map[string]string) error {
	durationStr, ok := args["DURATION"]
	if !ok {
		return fmt.Errorf("missing DURATION parameter")
	}

	duration, err := strconv.ParseFloat(durationStr, 64)
	if err != nil {
		return fmt.Errorf("invalid DURATION value: %w", err)
	}
	if duration < 0 {
		return fmt.Errorf("DURATION must be >= 0")
	}

	dg.mu.Lock()
	defer dg.mu.Unlock()

	dg.duration = duration

	if dg.insideTimer {
		// If called from within the timer callback, set repeat flag
		dg.repeat = (duration > 0)
	} else {
		// Otherwise, update the timer directly
		if dg.timer != nil {
			dg.timer.Stop()
			dg.timer = nil
		}
		if duration > 0 {
			dg.startTimer()
		}
	}

	return nil
}

// GetName returns the name of this delayed gcode.
func (dg *DelayedGcode) GetName() string {
	return dg.name
}

// GetDuration returns the current duration setting.
func (dg *DelayedGcode) GetDuration() float64 {
	dg.mu.Lock()
	defer dg.mu.Unlock()
	return dg.duration
}

// IsActive returns true if the timer is currently running.
func (dg *DelayedGcode) IsActive() bool {
	dg.mu.Lock()
	defer dg.mu.Unlock()
	return dg.timer != nil
}

// DelayedGcodeManager manages multiple delayed gcode instances.
type DelayedGcodeManager struct {
	rt      *runtime
	delayed map[string]*DelayedGcode
	mu      sync.RWMutex
}

// newDelayedGcodeManager creates a new delayed gcode manager.
func newDelayedGcodeManager(rt *runtime) *DelayedGcodeManager {
	return &DelayedGcodeManager{
		rt:      rt,
		delayed: make(map[string]*DelayedGcode),
	}
}

// Register registers a new delayed gcode instance.
func (dgm *DelayedGcodeManager) Register(cfg DelayedGcodeConfig) (*DelayedGcode, error) {
	dgm.mu.Lock()
	defer dgm.mu.Unlock()

	if _, exists := dgm.delayed[cfg.Name]; exists {
		return nil, fmt.Errorf("delayed_gcode %s already registered", cfg.Name)
	}

	dg := newDelayedGcode(dgm.rt, cfg)
	dgm.delayed[cfg.Name] = dg
	return dg, nil
}

// Get returns a delayed gcode instance by name.
func (dgm *DelayedGcodeManager) Get(name string) *DelayedGcode {
	dgm.mu.RLock()
	defer dgm.mu.RUnlock()
	return dgm.delayed[name]
}

// StartAll starts all delayed gcode instances that have initial_duration > 0.
func (dgm *DelayedGcodeManager) StartAll() {
	dgm.mu.RLock()
	defer dgm.mu.RUnlock()

	for _, dg := range dgm.delayed {
		dg.Start()
	}
}

// StopAll stops all delayed gcode instances.
func (dgm *DelayedGcodeManager) StopAll() {
	dgm.mu.RLock()
	defer dgm.mu.RUnlock()

	for _, dg := range dgm.delayed {
		dg.Stop()
	}
}
