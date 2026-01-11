package safety

import (
	"sync/atomic"
	"testing"
	"time"
)

// Mock implementations for testing

type mockHeater struct {
	id       string
	disabled atomic.Bool
}

func (h *mockHeater) DisableHeater() error {
	h.disabled.Store(true)
	return nil
}

func (h *mockHeater) GetHeaterID() string {
	return h.id
}

type mockMotor struct {
	disabled atomic.Bool
}

func (m *mockMotor) DisableMotors() error {
	m.disabled.Store(true)
	return nil
}

type mockPWM struct {
	disabled atomic.Bool
}

func (p *mockPWM) DisablePWM() error {
	p.disabled.Store(true)
	return nil
}

type mockMCU struct {
	connected     atomic.Bool
	emergencySent atomic.Bool
}

func (m *mockMCU) SendEmergencyStop() error {
	m.emergencySent.Store(true)
	return nil
}

func (m *mockMCU) IsConnected() bool {
	return m.connected.Load()
}

func TestNew(t *testing.T) {
	m := New()
	if m == nil {
		t.Fatal("New returned nil")
	}

	if m.GetState() != StateRunning {
		t.Errorf("Initial state should be Running, got %s", m.GetState())
	}

	if m.IsShutdown() {
		t.Error("Should not be shutdown initially")
	}

	if !m.IsOperational() {
		t.Error("Should be operational initially")
	}
}

func TestShutdownStateString(t *testing.T) {
	tests := []struct {
		state    ShutdownState
		expected string
	}{
		{StateRunning, "running"},
		{StateShuttingDown, "shutting_down"},
		{StateShutdown, "shutdown"},
		{StateError, "error"},
		{ShutdownState(99), "unknown"},
	}

	for _, tt := range tests {
		if tt.state.String() != tt.expected {
			t.Errorf("State %d String() = %s, want %s", tt.state, tt.state.String(), tt.expected)
		}
	}
}

func TestEmergencyStop(t *testing.T) {
	m := New()

	heater := &mockHeater{id: "extruder"}
	motor := &mockMotor{}
	pwm := &mockPWM{}
	mcu := &mockMCU{}
	mcu.connected.Store(true)

	m.RegisterHeater(heater)
	m.RegisterMotor(motor)
	m.RegisterPWM(pwm)
	m.RegisterMCU(mcu)

	err := m.EmergencyStop("test emergency stop")
	if err != nil {
		t.Errorf("EmergencyStop failed: %v", err)
	}

	// Check state
	if m.GetState() != StateError {
		t.Errorf("State should be Error, got %s", m.GetState())
	}

	// Check components were disabled
	if !heater.disabled.Load() {
		t.Error("Heater should be disabled")
	}

	if !motor.disabled.Load() {
		t.Error("Motor should be disabled")
	}

	if !pwm.disabled.Load() {
		t.Error("PWM should be disabled")
	}

	if !mcu.emergencySent.Load() {
		t.Error("MCU emergency stop should be sent")
	}

	// Check shutdown info
	reason, msg, shutdownTime := m.GetShutdownInfo()
	if reason != ReasonEmergencyStop {
		t.Errorf("Shutdown reason should be EmergencyStop, got %s", reason)
	}

	if msg != "test emergency stop" {
		t.Errorf("Shutdown message incorrect: %s", msg)
	}

	if shutdownTime.IsZero() {
		t.Error("Shutdown time should be set")
	}
}

func TestThermalRunaway(t *testing.T) {
	m := New()

	err := m.ThermalRunaway("extruder", 250.0, 200.0)
	if err != nil {
		t.Errorf("ThermalRunaway failed: %v", err)
	}

	reason, _, _ := m.GetShutdownInfo()
	if reason != ReasonThermalRunaway {
		t.Errorf("Shutdown reason should be ThermalRunaway, got %s", reason)
	}
}

func TestHeatingFailed(t *testing.T) {
	m := New()

	err := m.HeatingFailed("bed", 25.0, 60.0)
	if err != nil {
		t.Errorf("HeatingFailed failed: %v", err)
	}

	reason, _, _ := m.GetShutdownInfo()
	if reason != ReasonHeatingFailed {
		t.Errorf("Shutdown reason should be HeatingFailed, got %s", reason)
	}
}

func TestWatchdogTimeout(t *testing.T) {
	m := New()

	err := m.WatchdogTimeout()
	if err != nil {
		t.Errorf("WatchdogTimeout failed: %v", err)
	}

	reason, _, _ := m.GetShutdownInfo()
	if reason != ReasonWatchdogTimeout {
		t.Errorf("Shutdown reason should be WatchdogTimeout, got %s", reason)
	}
}

func TestUserShutdown(t *testing.T) {
	m := New()

	err := m.RequestShutdown("user requested shutdown")
	if err != nil {
		t.Errorf("RequestShutdown failed: %v", err)
	}

	// User shutdown should result in StateShutdown, not StateError
	if m.GetState() != StateShutdown {
		t.Errorf("State should be Shutdown for user request, got %s", m.GetState())
	}
}

func TestCheckOperational(t *testing.T) {
	m := New()

	// Initially operational
	if err := m.CheckOperational(); err != nil {
		t.Errorf("Should be operational initially: %v", err)
	}

	// After shutdown
	m.EmergencyStop("test")

	err := m.CheckOperational()
	if err == nil {
		t.Error("Should return error after shutdown")
	}
}

func TestOnShutdownCallback(t *testing.T) {
	m := New()

	var callbackReason ShutdownReason
	var callbackMsg string
	called := false

	m.OnShutdown(func(reason ShutdownReason, msg string) {
		called = true
		callbackReason = reason
		callbackMsg = msg
	})

	m.EmergencyStop("callback test")

	if !called {
		t.Error("Shutdown callback should have been called")
	}

	if callbackReason != ReasonEmergencyStop {
		t.Errorf("Callback reason should be EmergencyStop, got %s", callbackReason)
	}

	if callbackMsg != "callback test" {
		t.Errorf("Callback message incorrect: %s", callbackMsg)
	}
}

func TestOnStateChangeCallback(t *testing.T) {
	m := New()

	var oldState, newState ShutdownState
	called := false

	m.OnStateChange(func(old, new ShutdownState) {
		called = true
		oldState = old
		newState = new
	})

	m.EmergencyStop("state change test")

	if !called {
		t.Error("State change callback should have been called")
	}

	if oldState != StateRunning {
		t.Errorf("Old state should be Running, got %s", oldState)
	}

	if newState != StateError {
		t.Errorf("New state should be Error, got %s", newState)
	}
}

func TestDoubleShutdown(t *testing.T) {
	m := New()

	// First shutdown
	m.EmergencyStop("first")

	// Second shutdown should be no-op
	err := m.EmergencyStop("second")
	if err != nil {
		t.Errorf("Second shutdown should not error: %v", err)
	}

	// Should still have first shutdown reason
	reason, msg, _ := m.GetShutdownInfo()
	if reason != ReasonEmergencyStop {
		t.Errorf("Reason should still be EmergencyStop")
	}

	if msg != "first" {
		t.Errorf("Message should still be 'first', got %s", msg)
	}
}

func TestWatchdog(t *testing.T) {
	m := New()
	m.Configure(Config{
		WatchdogTimeout: 100 * time.Millisecond,
	})

	m.StartWatchdog()

	// Send heartbeats for a while
	for i := 0; i < 5; i++ {
		m.Heartbeat()
		time.Sleep(30 * time.Millisecond)
	}

	// Should still be running
	if !m.IsOperational() {
		t.Error("Should still be operational while sending heartbeats")
	}

	m.StopWatchdog()
}

func TestWatchdogTrigger(t *testing.T) {
	m := New()
	m.Configure(Config{
		WatchdogTimeout: 50 * time.Millisecond,
	})

	m.StartWatchdog()

	// Don't send heartbeats, wait for timeout with retries
	deadline := time.Now().Add(500 * time.Millisecond)
	for time.Now().Before(deadline) {
		if !m.IsOperational() {
			break
		}
		time.Sleep(20 * time.Millisecond)
	}

	if m.IsOperational() {
		t.Error("Should have triggered watchdog timeout")
	}

	reason, _, _ := m.GetShutdownInfo()
	if reason != ReasonWatchdogTimeout {
		t.Errorf("Reason should be WatchdogTimeout, got %s", reason)
	}
}

func TestReset(t *testing.T) {
	m := New()

	// Cannot reset while running
	err := m.Reset()
	if err == nil {
		t.Error("Should not be able to reset while running")
	}

	// Shutdown first
	m.RequestShutdown("test")

	// Now reset should work
	err = m.Reset()
	if err != nil {
		t.Errorf("Reset failed: %v", err)
	}

	if !m.IsOperational() {
		t.Error("Should be operational after reset")
	}

	reason, _, _ := m.GetShutdownInfo()
	if reason != ReasonNone {
		t.Errorf("Reason should be empty after reset, got %s", reason)
	}
}

func TestGetStatus(t *testing.T) {
	m := New()

	status := m.GetStatus()
	if status.State != "running" {
		t.Errorf("Status state should be 'running', got %s", status.State)
	}

	if !status.IsOperational {
		t.Error("Status should show operational")
	}

	m.EmergencyStop("status test")

	status = m.GetStatus()
	if status.State != "error" {
		t.Errorf("Status state should be 'error', got %s", status.State)
	}

	if status.IsOperational {
		t.Error("Status should not show operational after shutdown")
	}

	if status.ShutdownReason != string(ReasonEmergencyStop) {
		t.Errorf("Status reason incorrect: %s", status.ShutdownReason)
	}
}

func TestConfigure(t *testing.T) {
	m := New()

	m.Configure(Config{
		WatchdogTimeout:          10 * time.Second,
		ThermalRunawayHysteresis: 15.0,
		HeatingFailureTime:       5 * time.Second,
	})

	// The config values are internal, but we can verify by triggering behaviors
	// For now, just verify no panic
	if m.watchdogTimeout != 10*time.Second {
		t.Error("Watchdog timeout not configured correctly")
	}
}
