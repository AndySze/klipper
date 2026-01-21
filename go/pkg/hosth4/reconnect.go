// Reconnection manager - automatic MCU reconnection with exponential backoff
//
// This module handles automatic reconnection to MCUs when communication is lost.
// It implements exponential backoff to avoid overwhelming the system with
// reconnection attempts.
//
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"context"
	"fmt"
	"log"
	"sync"
	"time"
)

// ReconnectConfig configures the reconnection behavior.
type ReconnectConfig struct {
	// Enabled controls whether automatic reconnection is enabled.
	Enabled bool

	// InitialDelay is the delay before the first reconnection attempt.
	InitialDelay time.Duration

	// MaxDelay is the maximum delay between reconnection attempts.
	MaxDelay time.Duration

	// BackoffFactor is the multiplier applied to the delay after each failed attempt.
	BackoffFactor float64

	// MaxAttempts is the maximum number of reconnection attempts (0 = unlimited).
	MaxAttempts int

	// Callbacks
	OnReconnecting   func(attempt int, delay time.Duration) // Called before each attempt
	OnReconnected    func()                                 // Called after successful reconnection
	OnReconnectFailed func(err error)                       // Called when max attempts exceeded
}

// DefaultReconnectConfig returns the default reconnection configuration.
func DefaultReconnectConfig() ReconnectConfig {
	return ReconnectConfig{
		Enabled:       true,
		InitialDelay:  1 * time.Second,
		MaxDelay:      30 * time.Second,
		BackoffFactor: 2.0,
		MaxAttempts:   10,
	}
}

// ReconnectionState represents the current state of reconnection.
type ReconnectionState string

const (
	ReconnectStateIdle       ReconnectionState = "idle"
	ReconnectStateWaiting    ReconnectionState = "waiting"
	ReconnectStateConnecting ReconnectionState = "connecting"
	ReconnectStateFailed     ReconnectionState = "failed"
)

// ReconnectionManager handles automatic reconnection for an MCU.
type ReconnectionManager struct {
	mu sync.RWMutex

	// Configuration
	config  ReconnectConfig
	mcuName string

	// MCU connection to manage
	mcuConn *MCUConnection

	// State
	state        ReconnectionState
	attempts     int
	currentDelay time.Duration
	lastAttempt  time.Time
	lastError    error

	// Context for cancellation
	ctx    context.Context
	cancel context.CancelFunc

	// Control
	triggerCh chan string // Channel to trigger reconnection (carries reason)
	stopCh    chan struct{}
	wg        sync.WaitGroup
	running   bool

	// Tracing
	trace bool
}

// NewReconnectionManager creates a new reconnection manager.
func NewReconnectionManager(mcuName string, mcuConn *MCUConnection, config ReconnectConfig) *ReconnectionManager {
	ctx, cancel := context.WithCancel(context.Background())

	return &ReconnectionManager{
		config:       config,
		mcuName:      mcuName,
		mcuConn:      mcuConn,
		state:        ReconnectStateIdle,
		currentDelay: config.InitialDelay,
		ctx:          ctx,
		cancel:       cancel,
		triggerCh:    make(chan string, 1),
		stopCh:       make(chan struct{}),
	}
}

// Start begins the reconnection manager.
func (rm *ReconnectionManager) Start() {
	rm.mu.Lock()
	if rm.running {
		rm.mu.Unlock()
		return
	}
	rm.running = true
	rm.mu.Unlock()

	rm.wg.Add(1)
	go rm.reconnectLoop()

	rm.tracef("ReconnectionManager: Started for MCU %s\n", rm.mcuName)
}

// Stop halts the reconnection manager.
func (rm *ReconnectionManager) Stop() {
	rm.mu.Lock()
	if !rm.running {
		rm.mu.Unlock()
		return
	}
	rm.running = false
	rm.mu.Unlock()

	rm.cancel()
	close(rm.stopCh)
	rm.wg.Wait()

	rm.tracef("ReconnectionManager: Stopped for MCU %s\n", rm.mcuName)
}

// TriggerReconnect initiates a reconnection attempt.
// The reason is logged for diagnostic purposes.
func (rm *ReconnectionManager) TriggerReconnect(reason string) {
	rm.mu.Lock()
	if !rm.config.Enabled || !rm.running {
		rm.mu.Unlock()
		return
	}
	rm.mu.Unlock()

	select {
	case rm.triggerCh <- reason:
		rm.tracef("ReconnectionManager: Reconnection triggered for %s: %s\n", rm.mcuName, reason)
	default:
		// Already pending
	}
}

// reconnectLoop is the main reconnection goroutine.
func (rm *ReconnectionManager) reconnectLoop() {
	defer rm.wg.Done()

	for {
		select {
		case <-rm.stopCh:
			return
		case reason := <-rm.triggerCh:
			rm.handleReconnect(reason)
		}
	}
}

// handleReconnect performs the actual reconnection sequence.
func (rm *ReconnectionManager) handleReconnect(reason string) {
	rm.mu.Lock()
	rm.state = ReconnectStateWaiting
	rm.attempts = 0
	rm.currentDelay = rm.config.InitialDelay
	rm.mu.Unlock()

	log.Printf("ReconnectionManager: Starting reconnection for %s (reason: %s)", rm.mcuName, reason)

	for {
		rm.mu.Lock()
		if !rm.running {
			rm.mu.Unlock()
			return
		}

		rm.attempts++
		attempt := rm.attempts
		delay := rm.currentDelay
		maxAttempts := rm.config.MaxAttempts
		rm.mu.Unlock()

		// Check max attempts
		if maxAttempts > 0 && attempt > maxAttempts {
			rm.handleMaxAttemptsExceeded()
			return
		}

		// Call OnReconnecting callback
		rm.mu.RLock()
		callback := rm.config.OnReconnecting
		rm.mu.RUnlock()
		if callback != nil {
			callback(attempt, delay)
		}

		log.Printf("ReconnectionManager: Attempt %d for %s (delay: %v)", attempt, rm.mcuName, delay)

		// Wait before attempting
		rm.mu.Lock()
		rm.state = ReconnectStateWaiting
		rm.mu.Unlock()

		select {
		case <-rm.stopCh:
			return
		case <-time.After(delay):
		}

		// Attempt reconnection
		rm.mu.Lock()
		rm.state = ReconnectStateConnecting
		rm.lastAttempt = time.Now()
		rm.mu.Unlock()

		err := rm.attemptReconnect()
		if err == nil {
			rm.handleReconnectSuccess()
			return
		}

		rm.mu.Lock()
		rm.lastError = err
		rm.currentDelay = rm.calculateBackoff(rm.currentDelay)
		rm.mu.Unlock()

		log.Printf("ReconnectionManager: Attempt %d failed for %s: %v", attempt, rm.mcuName, err)
	}
}

// attemptReconnect performs a single reconnection attempt.
func (rm *ReconnectionManager) attemptReconnect() error {
	if rm.mcuConn == nil {
		return fmt.Errorf("no MCU connection configured")
	}

	// Step 1: Disconnect if still connected
	rm.tracef("ReconnectionManager: Disconnecting %s\n", rm.mcuName)
	if err := rm.mcuConn.Disconnect(); err != nil {
		rm.tracef("ReconnectionManager: Disconnect error (ignored): %v\n", err)
	}

	// Step 2: Reset OID counter for fresh configuration
	rm.mcuConn.ResetOIDCount()

	// Step 3: Attempt to reconnect
	rm.tracef("ReconnectionManager: Connecting to %s\n", rm.mcuName)
	if err := rm.mcuConn.Connect(); err != nil {
		return fmt.Errorf("connect failed: %w", err)
	}

	return nil
}

// handleReconnectSuccess is called after successful reconnection.
func (rm *ReconnectionManager) handleReconnectSuccess() {
	rm.mu.Lock()
	rm.state = ReconnectStateIdle
	rm.attempts = 0
	rm.currentDelay = rm.config.InitialDelay
	rm.lastError = nil
	callback := rm.config.OnReconnected
	rm.mu.Unlock()

	log.Printf("ReconnectionManager: Successfully reconnected to %s", rm.mcuName)

	if callback != nil {
		callback()
	}
}

// handleMaxAttemptsExceeded is called when max attempts have been exceeded.
func (rm *ReconnectionManager) handleMaxAttemptsExceeded() {
	rm.mu.Lock()
	rm.state = ReconnectStateFailed
	err := rm.lastError
	callback := rm.config.OnReconnectFailed
	rm.mu.Unlock()

	log.Printf("ReconnectionManager: Max attempts exceeded for %s", rm.mcuName)

	if callback != nil {
		if err == nil {
			err = fmt.Errorf("max reconnection attempts exceeded")
		}
		callback(err)
	}
}

// calculateBackoff calculates the next delay using exponential backoff.
func (rm *ReconnectionManager) calculateBackoff(currentDelay time.Duration) time.Duration {
	nextDelay := time.Duration(float64(currentDelay) * rm.config.BackoffFactor)
	if nextDelay > rm.config.MaxDelay {
		nextDelay = rm.config.MaxDelay
	}
	return nextDelay
}

// State returns the current reconnection state.
func (rm *ReconnectionManager) State() ReconnectionState {
	rm.mu.RLock()
	defer rm.mu.RUnlock()
	return rm.state
}

// Attempts returns the current attempt count.
func (rm *ReconnectionManager) Attempts() int {
	rm.mu.RLock()
	defer rm.mu.RUnlock()
	return rm.attempts
}

// IsReconnecting returns true if reconnection is in progress.
func (rm *ReconnectionManager) IsReconnecting() bool {
	rm.mu.RLock()
	defer rm.mu.RUnlock()
	return rm.state == ReconnectStateWaiting || rm.state == ReconnectStateConnecting
}

// LastError returns the last reconnection error.
func (rm *ReconnectionManager) LastError() error {
	rm.mu.RLock()
	defer rm.mu.RUnlock()
	return rm.lastError
}

// Reset resets the reconnection manager to initial state.
func (rm *ReconnectionManager) Reset() {
	rm.mu.Lock()
	defer rm.mu.Unlock()

	rm.state = ReconnectStateIdle
	rm.attempts = 0
	rm.currentDelay = rm.config.InitialDelay
	rm.lastError = nil
}

// SetTrace enables or disables trace logging.
func (rm *ReconnectionManager) SetTrace(enabled bool) {
	rm.mu.Lock()
	defer rm.mu.Unlock()
	rm.trace = enabled
}

// GetStatus returns the current reconnection status.
func (rm *ReconnectionManager) GetStatus() map[string]interface{} {
	rm.mu.RLock()
	defer rm.mu.RUnlock()

	status := map[string]interface{}{
		"mcu_name":      rm.mcuName,
		"state":         string(rm.state),
		"attempts":      rm.attempts,
		"current_delay": rm.currentDelay.Seconds(),
		"max_attempts":  rm.config.MaxAttempts,
		"enabled":       rm.config.Enabled,
	}

	if rm.lastError != nil {
		status["last_error"] = rm.lastError.Error()
	}

	if !rm.lastAttempt.IsZero() {
		status["last_attempt"] = rm.lastAttempt.Unix()
	}

	return status
}

// tracef writes trace output if tracing is enabled.
func (rm *ReconnectionManager) tracef(format string, args ...interface{}) {
	if rm.trace {
		log.Printf(format, args...)
	}
}
