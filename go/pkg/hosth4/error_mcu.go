// Error MCU - port of klippy/extras/error_mcu.py
//
// Handle MCU errors and shutdown recovery
//
// Copyright (C) 2019-2021 Kevin O'Connor <kevin@koconnor.net>
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

// MCU error types.
const (
	MCUErrorShutdown    = "shutdown"
	MCUErrorProtocol    = "protocol"
	MCUErrorTimeout     = "timeout"
	MCUErrorConnection  = "connection"
	MCUErrorConfig      = "config"
)

// ErrorMCU handles MCU errors and recovery.
type ErrorMCU struct {
	rt              *runtime
	errorState      bool
	errorMessage    string
	errorType       string
	errorTime       time.Time
	shutdownReason  string
	recoveryAttempts int
	maxRecoveryAttempts int
	errorCallback   func(errType, errMsg string)
	mu              sync.Mutex
}

// ErrorMCUConfig holds configuration for error MCU handler.
type ErrorMCUConfig struct {
	MaxRecoveryAttempts int
}

// DefaultErrorMCUConfig returns default configuration.
func DefaultErrorMCUConfig() ErrorMCUConfig {
	return ErrorMCUConfig{
		MaxRecoveryAttempts: 3,
	}
}

// newErrorMCU creates a new MCU error handler.
func newErrorMCU(rt *runtime, cfg ErrorMCUConfig) *ErrorMCU {
	em := &ErrorMCU{
		rt:                 rt,
		maxRecoveryAttempts: cfg.MaxRecoveryAttempts,
	}

	log.Printf("error_mcu: initialized max_recovery=%d", cfg.MaxRecoveryAttempts)
	return em
}

// SetErrorCallback sets the error callback function.
func (em *ErrorMCU) SetErrorCallback(cb func(errType, errMsg string)) {
	em.mu.Lock()
	defer em.mu.Unlock()
	em.errorCallback = cb
}

// SetError sets an error state.
func (em *ErrorMCU) SetError(errType, errMsg string) {
	em.mu.Lock()
	defer em.mu.Unlock()

	em.errorState = true
	em.errorType = errType
	em.errorMessage = errMsg
	em.errorTime = time.Now()

	log.Printf("error_mcu: ERROR %s: %s", errType, errMsg)

	if em.errorCallback != nil {
		em.errorCallback(errType, errMsg)
	}
}

// SetShutdown sets a shutdown state.
func (em *ErrorMCU) SetShutdown(reason string) {
	em.mu.Lock()
	defer em.mu.Unlock()

	em.errorState = true
	em.errorType = MCUErrorShutdown
	em.errorMessage = fmt.Sprintf("MCU shutdown: %s", reason)
	em.shutdownReason = reason
	em.errorTime = time.Now()

	log.Printf("error_mcu: SHUTDOWN: %s", reason)

	if em.errorCallback != nil {
		em.errorCallback(MCUErrorShutdown, reason)
	}
}

// ClearError clears the error state.
func (em *ErrorMCU) ClearError() {
	em.mu.Lock()
	defer em.mu.Unlock()

	em.errorState = false
	em.errorMessage = ""
	em.errorType = ""
	em.shutdownReason = ""

	log.Printf("error_mcu: error cleared")
}

// IsError returns whether there is an active error.
func (em *ErrorMCU) IsError() bool {
	em.mu.Lock()
	defer em.mu.Unlock()
	return em.errorState
}

// IsShutdown returns whether the MCU is in shutdown state.
func (em *ErrorMCU) IsShutdown() bool {
	em.mu.Lock()
	defer em.mu.Unlock()
	return em.errorType == MCUErrorShutdown
}

// GetError returns the current error.
func (em *ErrorMCU) GetError() (errType, errMsg string) {
	em.mu.Lock()
	defer em.mu.Unlock()
	return em.errorType, em.errorMessage
}

// GetShutdownReason returns the shutdown reason.
func (em *ErrorMCU) GetShutdownReason() string {
	em.mu.Lock()
	defer em.mu.Unlock()
	return em.shutdownReason
}

// AttemptRecovery attempts to recover from an error.
func (em *ErrorMCU) AttemptRecovery() (bool, error) {
	em.mu.Lock()
	defer em.mu.Unlock()

	if !em.errorState {
		return true, nil
	}

	em.recoveryAttempts++

	if em.recoveryAttempts > em.maxRecoveryAttempts {
		return false, fmt.Errorf("error_mcu: max recovery attempts (%d) exceeded",
			em.maxRecoveryAttempts)
	}

	log.Printf("error_mcu: recovery attempt %d/%d",
		em.recoveryAttempts, em.maxRecoveryAttempts)

	// In real implementation, this would attempt MCU reconnection
	return false, nil
}

// ResetRecoveryAttempts resets the recovery attempt counter.
func (em *ErrorMCU) ResetRecoveryAttempts() {
	em.mu.Lock()
	defer em.mu.Unlock()
	em.recoveryAttempts = 0
}

// GetStatus returns the error MCU status.
func (em *ErrorMCU) GetStatus() map[string]any {
	em.mu.Lock()
	defer em.mu.Unlock()

	return map[string]any{
		"error_state":       em.errorState,
		"error_type":        em.errorType,
		"error_message":     em.errorMessage,
		"shutdown_reason":   em.shutdownReason,
		"recovery_attempts": em.recoveryAttempts,
	}
}
