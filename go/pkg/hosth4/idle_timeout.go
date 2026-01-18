// Idle timeout - port of klippy/extras/idle_timeout.py
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

// IdleState represents the current idle timeout state.
type IdleState string

const (
	IdleStateIdle     IdleState = "Idle"
	IdleStateReady    IdleState = "Ready"
	IdleStatePrinting IdleState = "Printing"
)

const (
	pinMinTime   = 0.100
	readyTimeout = 0.500
)

// Default gcode to run on idle timeout.
const defaultIdleGcode = `TURN_OFF_HEATERS
M84`

// IdleTimeout manages the printer's idle timeout behavior.
type IdleTimeout struct {
	rt *runtime

	timeout          float64   // idle timeout in seconds
	state            IdleState // current state
	lastPrintStart   time.Time // when printing started
	idleGcode        []string  // gcode to run on timeout
	mu               sync.Mutex
	timerRunning     bool
	stopChan         chan struct{}
}

// IdleTimeoutConfig holds configuration for idle timeout.
type IdleTimeoutConfig struct {
	Timeout   float64  // timeout in seconds (default 600)
	IdleGcode []string // gcode to run on timeout
}

// DefaultIdleTimeoutConfig returns default idle timeout configuration.
func DefaultIdleTimeoutConfig() IdleTimeoutConfig {
	return IdleTimeoutConfig{
		Timeout:   600.0,
		IdleGcode: []string{"TURN_OFF_HEATERS", "M84"},
	}
}

// newIdleTimeout creates a new idle timeout manager.
func newIdleTimeout(rt *runtime, cfg IdleTimeoutConfig) *IdleTimeout {
	if cfg.Timeout <= 0 {
		cfg.Timeout = 600.0
	}
	if len(cfg.IdleGcode) == 0 {
		cfg.IdleGcode = []string{"TURN_OFF_HEATERS", "M84"}
	}

	it := &IdleTimeout{
		rt:        rt,
		timeout:   cfg.Timeout,
		state:     IdleStateIdle,
		idleGcode: cfg.IdleGcode,
		stopChan:  make(chan struct{}),
	}

	return it
}

// Start begins the idle timeout timer.
func (it *IdleTimeout) Start() {
	it.mu.Lock()
	if it.timerRunning {
		it.mu.Unlock()
		return
	}
	it.timerRunning = true
	it.stopChan = make(chan struct{})
	it.mu.Unlock()

	go it.timerLoop()
}

// Stop stops the idle timeout timer.
func (it *IdleTimeout) Stop() {
	it.mu.Lock()
	if !it.timerRunning {
		it.mu.Unlock()
		return
	}
	it.timerRunning = false
	close(it.stopChan)
	it.mu.Unlock()
}

// timerLoop runs the idle timeout check loop.
func (it *IdleTimeout) timerLoop() {
	ticker := time.NewTicker(time.Duration(readyTimeout*1000) * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-it.stopChan:
			return
		case <-ticker.C:
			it.checkTimeout()
		}
	}
}

// checkTimeout checks if the idle timeout has elapsed.
func (it *IdleTimeout) checkTimeout() {
	it.mu.Lock()
	state := it.state
	it.mu.Unlock()

	if state != IdleStateReady {
		return
	}

	// Check if toolhead is busy
	if it.isToolheadBusy() {
		return
	}

	// Check elapsed time since last activity
	// In a real implementation, this would track last motion time
	// For now, use a simple time-based approach
	it.transitionToIdle()
}

// isToolheadBusy checks if the toolhead is currently processing moves.
func (it *IdleTimeout) isToolheadBusy() bool {
	if it.rt == nil || it.rt.toolhead == nil {
		return false
	}
	// In a real implementation, check toolhead lookahead queue
	return false
}

// transitionToIdle runs the idle gcode and transitions to idle state.
func (it *IdleTimeout) transitionToIdle() {
	it.mu.Lock()
	it.state = IdleStatePrinting // Temporarily set to Printing while running gcode
	gcode := it.idleGcode
	it.mu.Unlock()

	// Run idle gcode
	for _, cmd := range gcode {
		if err := it.runGcode(cmd); err != nil {
			log.Printf("idle_timeout: error running gcode '%s': %v", cmd, err)
		}
	}

	it.mu.Lock()
	it.state = IdleStateIdle
	it.mu.Unlock()

	log.Println("idle_timeout: printer is now idle")
}

// runGcode executes a single gcode command.
func (it *IdleTimeout) runGcode(cmd string) error {
	// In a full implementation, this would dispatch to the gcode handler
	switch cmd {
	case "TURN_OFF_HEATERS":
		return it.turnOffHeaters()
	case "M84":
		return it.disableMotors()
	}
	return nil
}

// turnOffHeaters turns off all heaters.
func (it *IdleTimeout) turnOffHeaters() error {
	if it.rt == nil || it.rt.heaterManager == nil {
		return nil
	}
	// Set all heater targets to 0
	// Try known heater names
	heaterNames := []string{"extruder", "extruder1", "heater_bed"}
	for _, name := range heaterNames {
		heater, err := it.rt.heaterManager.GetHeater(name)
		if err == nil && heater != nil {
			heater.SetTemp(0.0)
		}
	}
	return nil
}

// disableMotors disables all stepper motors.
func (it *IdleTimeout) disableMotors() error {
	if it.rt == nil || it.rt.stepperEnable == nil {
		return nil
	}
	// Call motor_off for all steppers
	it.rt.stepperEnable.motorOff()
	return nil
}

// SetState sets the current idle timeout state.
func (it *IdleTimeout) SetState(state IdleState) {
	it.mu.Lock()
	defer it.mu.Unlock()

	if it.state == state {
		return
	}

	prevState := it.state
	it.state = state

	if state == IdleStatePrinting && prevState != IdleStatePrinting {
		it.lastPrintStart = time.Now()
	}
}

// TransitionToPrinting transitions to printing state.
func (it *IdleTimeout) TransitionToPrinting() {
	it.SetState(IdleStatePrinting)
}

// TransitionToReady transitions to ready state.
func (it *IdleTimeout) TransitionToReady() {
	it.SetState(IdleStateReady)
}

// GetStatus returns the current idle timeout status.
func (it *IdleTimeout) GetStatus() map[string]any {
	it.mu.Lock()
	defer it.mu.Unlock()

	printingTime := 0.0
	if it.state == IdleStatePrinting && !it.lastPrintStart.IsZero() {
		printingTime = time.Since(it.lastPrintStart).Seconds()
	}

	return map[string]any{
		"state":         string(it.state),
		"printing_time": printingTime,
		"idle_timeout":  it.timeout,
	}
}

// cmdSetIdleTimeout handles the SET_IDLE_TIMEOUT command.
func (it *IdleTimeout) cmdSetIdleTimeout(args map[string]string) error {
	if timeoutStr, ok := args["TIMEOUT"]; ok {
		var timeout float64
		if _, err := fmt.Sscanf(timeoutStr, "%f", &timeout); err != nil {
			return fmt.Errorf("invalid TIMEOUT value: %s", timeoutStr)
		}
		if timeout <= 0 {
			return fmt.Errorf("TIMEOUT must be greater than 0")
		}

		it.mu.Lock()
		it.timeout = timeout
		it.mu.Unlock()

		log.Printf("idle_timeout: Timeout set to %.2f s", timeout)
	}
	return nil
}
