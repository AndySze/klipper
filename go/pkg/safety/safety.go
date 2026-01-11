// Package safety provides safety-critical features for 3D printer control.
// This includes emergency stop (M112), thermal runaway protection, watchdog,
// and shutdown state management.
package safety

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"
)

// ShutdownState represents the printer's shutdown state.
type ShutdownState int

const (
	// StateRunning indicates normal operation.
	StateRunning ShutdownState = iota

	// StateShuttingDown indicates shutdown is in progress.
	StateShuttingDown

	// StateShutdown indicates the printer is shut down.
	StateShutdown

	// StateError indicates an error-triggered shutdown.
	StateError
)

func (s ShutdownState) String() string {
	switch s {
	case StateRunning:
		return "running"
	case StateShuttingDown:
		return "shutting_down"
	case StateShutdown:
		return "shutdown"
	case StateError:
		return "error"
	default:
		return "unknown"
	}
}

// ShutdownReason describes why the printer was shut down.
type ShutdownReason string

const (
	ReasonNone            ShutdownReason = ""
	ReasonEmergencyStop   ShutdownReason = "emergency_stop"
	ReasonThermalRunaway  ShutdownReason = "thermal_runaway"
	ReasonHeatingFailed   ShutdownReason = "heating_failed"
	ReasonWatchdogTimeout ShutdownReason = "watchdog_timeout"
	ReasonFirmwareError   ShutdownReason = "firmware_error"
	ReasonUserRequest     ShutdownReason = "user_request"
	ReasonCommunication   ShutdownReason = "communication_error"
)

// Common errors
var (
	ErrShutdown       = errors.New("safety: printer is shut down")
	ErrEmergencyStop  = errors.New("safety: emergency stop triggered")
	ErrThermalRunaway = errors.New("safety: thermal runaway detected")
)

// HeaterDisabler can disable a heater.
type HeaterDisabler interface {
	DisableHeater() error
	GetHeaterID() string
}

// MotorDisabler can disable motors.
type MotorDisabler interface {
	DisableMotors() error
}

// PWMDisabler can disable PWM outputs.
type PWMDisabler interface {
	DisablePWM() error
}

// MCUCommander can send commands to an MCU.
type MCUCommander interface {
	SendEmergencyStop() error
	IsConnected() bool
}

// Manager manages safety features and shutdown state.
type Manager struct {
	mu sync.RWMutex

	// Current state
	state          ShutdownState
	shutdownReason ShutdownReason
	shutdownMsg    string
	shutdownTime   time.Time

	// Registered components
	heaters []HeaterDisabler
	motors  []MotorDisabler
	pwms    []PWMDisabler
	mcus    []MCUCommander

	// Watchdog
	watchdogCtx     context.Context
	watchdogCancel  context.CancelFunc
	watchdogTimeout time.Duration
	lastHeartbeat   time.Time
	watchdogMu      sync.Mutex

	// Callbacks
	onShutdown     []func(reason ShutdownReason, msg string)
	onStateChange  []func(oldState, newState ShutdownState)

	// Settings
	thermalRunawayHysteresis float64 // Temperature above target that triggers runaway
	heatingFailureTime       time.Duration // Time without temp increase that triggers failure
}

// New creates a new safety Manager.
func New() *Manager {
	return &Manager{
		state:                    StateRunning,
		watchdogTimeout:          5 * time.Second,
		thermalRunawayHysteresis: 10.0, // 10°C above target
		heatingFailureTime:       3 * time.Second,
	}
}

// Config holds configuration for the safety manager.
type Config struct {
	WatchdogTimeout          time.Duration
	ThermalRunawayHysteresis float64
	HeatingFailureTime       time.Duration
}

// Configure applies configuration to the manager.
func (m *Manager) Configure(cfg Config) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if cfg.WatchdogTimeout > 0 {
		m.watchdogTimeout = cfg.WatchdogTimeout
	}
	if cfg.ThermalRunawayHysteresis > 0 {
		m.thermalRunawayHysteresis = cfg.ThermalRunawayHysteresis
	}
	if cfg.HeatingFailureTime > 0 {
		m.heatingFailureTime = cfg.HeatingFailureTime
	}
}

// RegisterHeater registers a heater for emergency shutdown.
func (m *Manager) RegisterHeater(h HeaterDisabler) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.heaters = append(m.heaters, h)
}

// RegisterMotor registers a motor controller for emergency shutdown.
func (m *Manager) RegisterMotor(motor MotorDisabler) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.motors = append(m.motors, motor)
}

// RegisterPWM registers a PWM output for emergency shutdown.
func (m *Manager) RegisterPWM(pwm PWMDisabler) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.pwms = append(m.pwms, pwm)
}

// RegisterMCU registers an MCU for emergency shutdown.
func (m *Manager) RegisterMCU(mcu MCUCommander) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.mcus = append(m.mcus, mcu)
}

// OnShutdown registers a callback for when shutdown occurs.
func (m *Manager) OnShutdown(fn func(reason ShutdownReason, msg string)) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.onShutdown = append(m.onShutdown, fn)
}

// OnStateChange registers a callback for state changes.
func (m *Manager) OnStateChange(fn func(oldState, newState ShutdownState)) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.onStateChange = append(m.onStateChange, fn)
}

// GetState returns the current shutdown state.
func (m *Manager) GetState() ShutdownState {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.state
}

// GetShutdownInfo returns shutdown details.
func (m *Manager) GetShutdownInfo() (ShutdownReason, string, time.Time) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.shutdownReason, m.shutdownMsg, m.shutdownTime
}

// IsShutdown returns true if the printer is shut down.
func (m *Manager) IsShutdown() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.state == StateShutdown || m.state == StateError
}

// IsOperational returns true if the printer is running normally.
func (m *Manager) IsOperational() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.state == StateRunning
}

// CheckOperational returns an error if the printer is not operational.
func (m *Manager) CheckOperational() error {
	m.mu.RLock()
	defer m.mu.RUnlock()

	if m.state != StateRunning {
		return fmt.Errorf("%w: %s - %s", ErrShutdown, m.shutdownReason, m.shutdownMsg)
	}
	return nil
}

// EmergencyStop triggers an immediate emergency stop (M112).
// This disables all heaters and motors as quickly as possible.
func (m *Manager) EmergencyStop(msg string) error {
	return m.invokeShutdown(ReasonEmergencyStop, msg)
}

// ThermalRunaway triggers a shutdown due to thermal runaway.
func (m *Manager) ThermalRunaway(heaterID string, currentTemp, targetTemp float64) error {
	msg := fmt.Sprintf("heater %s: temperature %.1f exceeds target %.1f by more than %.1f",
		heaterID, currentTemp, targetTemp, m.thermalRunawayHysteresis)
	return m.invokeShutdown(ReasonThermalRunaway, msg)
}

// HeatingFailed triggers a shutdown due to heating failure.
func (m *Manager) HeatingFailed(heaterID string, currentTemp, targetTemp float64) error {
	msg := fmt.Sprintf("heater %s: temperature %.1f not approaching target %.1f",
		heaterID, currentTemp, targetTemp)
	return m.invokeShutdown(ReasonHeatingFailed, msg)
}

// WatchdogTimeout triggers a shutdown due to watchdog timeout.
func (m *Manager) WatchdogTimeout() error {
	return m.invokeShutdown(ReasonWatchdogTimeout, "main loop heartbeat timeout")
}

// FirmwareError triggers a shutdown due to MCU firmware error.
func (m *Manager) FirmwareError(mcuName, errMsg string) error {
	msg := fmt.Sprintf("MCU %s: %s", mcuName, errMsg)
	return m.invokeShutdown(ReasonFirmwareError, msg)
}

// CommunicationError triggers a shutdown due to communication failure.
func (m *Manager) CommunicationError(mcuName, errMsg string) error {
	msg := fmt.Sprintf("MCU %s: %s", mcuName, errMsg)
	return m.invokeShutdown(ReasonCommunication, msg)
}

// RequestShutdown triggers a graceful shutdown by user request.
func (m *Manager) RequestShutdown(msg string) error {
	return m.invokeShutdown(ReasonUserRequest, msg)
}

// invokeShutdown performs the shutdown sequence.
func (m *Manager) invokeShutdown(reason ShutdownReason, msg string) error {
	m.mu.Lock()

	// Don't shutdown if already shut down
	if m.state == StateShutdown || m.state == StateError {
		m.mu.Unlock()
		return nil
	}

	oldState := m.state
	m.state = StateShuttingDown
	m.shutdownReason = reason
	m.shutdownMsg = msg
	m.shutdownTime = time.Now()

	// Copy components to disable (to avoid holding lock during disable)
	heaters := make([]HeaterDisabler, len(m.heaters))
	copy(heaters, m.heaters)
	motors := make([]MotorDisabler, len(m.motors))
	copy(motors, m.motors)
	pwms := make([]PWMDisabler, len(m.pwms))
	copy(pwms, m.pwms)
	mcus := make([]MCUCommander, len(m.mcus))
	copy(mcus, m.mcus)

	m.mu.Unlock()

	// Stop watchdog
	m.StopWatchdog()

	// Disable heaters first (most critical for safety)
	for _, h := range heaters {
		_ = h.DisableHeater() // Best effort
	}

	// Send emergency stop to MCUs
	for _, mcu := range mcus {
		if mcu.IsConnected() {
			_ = mcu.SendEmergencyStop() // Best effort
		}
	}

	// Disable motors
	for _, motor := range motors {
		_ = motor.DisableMotors() // Best effort
	}

	// Disable PWM outputs
	for _, pwm := range pwms {
		_ = pwm.DisablePWM() // Best effort
	}

	// Update final state
	m.mu.Lock()
	finalState := StateShutdown
	if reason == ReasonEmergencyStop || reason == ReasonThermalRunaway ||
		reason == ReasonHeatingFailed || reason == ReasonFirmwareError {
		finalState = StateError
	}
	m.state = finalState

	// Copy callbacks
	onShutdown := make([]func(ShutdownReason, string), len(m.onShutdown))
	copy(onShutdown, m.onShutdown)
	onStateChange := make([]func(ShutdownState, ShutdownState), len(m.onStateChange))
	copy(onStateChange, m.onStateChange)
	m.mu.Unlock()

	// Call callbacks
	for _, fn := range onStateChange {
		fn(oldState, finalState)
	}
	for _, fn := range onShutdown {
		fn(reason, msg)
	}

	return nil
}

// StartWatchdog starts the watchdog timer.
func (m *Manager) StartWatchdog() {
	m.watchdogMu.Lock()
	defer m.watchdogMu.Unlock()

	if m.watchdogCancel != nil {
		return // Already running
	}

	m.watchdogCtx, m.watchdogCancel = context.WithCancel(context.Background())
	m.lastHeartbeat = time.Now()

	go m.watchdogLoop()
}

// StopWatchdog stops the watchdog timer.
func (m *Manager) StopWatchdog() {
	m.watchdogMu.Lock()
	defer m.watchdogMu.Unlock()

	if m.watchdogCancel != nil {
		m.watchdogCancel()
		m.watchdogCancel = nil
	}
}

// Heartbeat updates the watchdog timer.
// Call this regularly from the main loop.
func (m *Manager) Heartbeat() {
	m.watchdogMu.Lock()
	defer m.watchdogMu.Unlock()
	m.lastHeartbeat = time.Now()
}

// watchdogLoop runs the watchdog timer.
func (m *Manager) watchdogLoop() {
	ticker := time.NewTicker(500 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-m.watchdogCtx.Done():
			return
		case <-ticker.C:
			m.watchdogMu.Lock()
			elapsed := time.Since(m.lastHeartbeat)
			timeout := m.watchdogTimeout
			m.watchdogMu.Unlock()

			if elapsed > timeout {
				m.WatchdogTimeout()
				return
			}
		}
	}
}

// Reset attempts to reset the safety manager for a firmware restart.
// Only allowed from certain states.
func (m *Manager) Reset() error {
	m.mu.Lock()
	defer m.mu.Unlock()

	// Only allow reset from shutdown or error states
	if m.state == StateRunning || m.state == StateShuttingDown {
		return errors.New("safety: cannot reset while running or shutting down")
	}

	m.state = StateRunning
	m.shutdownReason = ReasonNone
	m.shutdownMsg = ""
	m.shutdownTime = time.Time{}

	return nil
}

// Status returns a status struct for reporting.
type Status struct {
	State          string
	ShutdownReason string
	ShutdownMsg    string
	ShutdownTime   time.Time
	IsOperational  bool
}

// GetStatus returns the current status.
func (m *Manager) GetStatus() Status {
	m.mu.RLock()
	defer m.mu.RUnlock()

	return Status{
		State:          m.state.String(),
		ShutdownReason: string(m.shutdownReason),
		ShutdownMsg:    m.shutdownMsg,
		ShutdownTime:   m.shutdownTime,
		IsOperational:  m.state == StateRunning,
	}
}
