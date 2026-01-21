// Unit tests for shutdown coordinator, heartbeat monitor, reconnection manager,
// restart helper, and MCU diagnostics.
//
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"sync"
	"testing"
	"time"

	"klipper-go-migration/pkg/reactor"
)

// TestShutdownCoordinator tests the shutdown coordinator functionality.
func TestShutdownCoordinator(t *testing.T) {
	r := reactor.New()
	sc := NewShutdownCoordinator(r)

	// Test initial state
	if sc.State() != StateStartup {
		t.Errorf("Expected initial state to be StateStartup, got %v", sc.State())
	}

	// Test transition to ready
	sc.SetReady()
	if sc.State() != StateReady {
		t.Errorf("Expected state to be StateReady, got %v", sc.State())
	}

	// Test shutdown handler registration
	handlerCalled := false
	sc.RegisterShutdownHandler("test", func() {
		handlerCalled = true
	})

	// Test analyze handler registration
	analyzeCalled := false
	var analyzeMsg string
	sc.RegisterAnalyzeHandler("test", func(msg string, details *ShutdownDetails) {
		analyzeCalled = true
		analyzeMsg = msg
	})

	// Test invoke shutdown
	details := &ShutdownDetails{
		Reason:    "test reason",
		EventType: "test",
	}
	sc.InvokeShutdown("Test shutdown", details)

	// Verify handlers were called
	if !handlerCalled {
		t.Error("Shutdown handler was not called")
	}
	if !analyzeCalled {
		t.Error("Analyze handler was not called")
	}
	if analyzeMsg != "Test shutdown" {
		t.Errorf("Analyze handler received wrong message: %s", analyzeMsg)
	}

	// Verify state
	if !sc.IsShutdown() {
		t.Error("Expected IsShutdown() to be true")
	}
	if sc.ShutdownMessage() != "Test shutdown" {
		t.Errorf("Expected shutdown message 'Test shutdown', got '%s'", sc.ShutdownMessage())
	}
}

// TestShutdownIdempotent tests that multiple shutdown calls are ignored.
func TestShutdownIdempotent(t *testing.T) {
	r := reactor.New()
	sc := NewShutdownCoordinator(r)
	sc.SetReady()

	callCount := 0
	sc.RegisterShutdownHandler("counter", func() {
		callCount++
	})

	// Call shutdown multiple times
	sc.InvokeShutdown("First shutdown", nil)
	sc.InvokeShutdown("Second shutdown", nil)
	sc.InvokeShutdown("Third shutdown", nil)

	// Handler should only be called once
	if callCount != 1 {
		t.Errorf("Expected handler to be called once, got %d", callCount)
	}

	// Message should be from first shutdown
	if sc.ShutdownMessage() != "First shutdown" {
		t.Errorf("Expected 'First shutdown', got '%s'", sc.ShutdownMessage())
	}
}

// TestShutdownHandlerPanic tests that panics in handlers don't crash the coordinator.
func TestShutdownHandlerPanic(t *testing.T) {
	r := reactor.New()
	sc := NewShutdownCoordinator(r)
	sc.SetReady()

	// Register a handler that panics
	sc.RegisterShutdownHandler("panicking", func() {
		panic("test panic")
	})

	// Register another handler that should still be called
	secondCalled := false
	sc.RegisterShutdownHandler("second", func() {
		secondCalled = true
	})

	// This should not panic
	sc.InvokeShutdown("Test", nil)

	// Second handler should still be called
	if !secondCalled {
		t.Error("Second handler was not called after first handler panicked")
	}
}

// TestHeartbeatMonitor tests the heartbeat monitor functionality.
func TestHeartbeatMonitor(t *testing.T) {
	config := HeartbeatConfig{
		QueryInterval:     10 * time.Millisecond,
		MaxPendingQueries: 2,
	}

	hm := NewHeartbeatMonitor("test_mcu", nil, nil, config)

	// Test initial state
	if hm.IsActive() {
		t.Error("Expected monitor to not be active before start")
	}

	// Test queries pending tracking
	hm.Start()
	defer hm.Stop()

	// Initially should be active
	if !hm.IsActive() {
		t.Error("Expected monitor to be active after start")
	}

	// Simulate missed queries (increment pending count directly for testing)
	hm.mu.Lock()
	hm.queriesPending = 3 // Exceeds MaxPendingQueries
	hm.mu.Unlock()

	// Now should not be active
	if hm.IsActive() {
		t.Error("Expected monitor to not be active when queries pending exceeds max")
	}
}

// TestHeartbeatTimeout tests the timeout callback.
func TestHeartbeatTimeout(t *testing.T) {
	timeoutCalled := false
	var timeoutMCU string

	config := HeartbeatConfig{
		QueryInterval:     10 * time.Millisecond,
		MaxPendingQueries: 2,
		TimeoutCallback: func(mcuName string, eventtime float64) {
			timeoutCalled = true
			timeoutMCU = mcuName
		},
	}

	hm := NewHeartbeatMonitor("test_mcu", nil, nil, config)
	hm.Start()
	defer hm.Stop()

	// Simulate timeout condition
	hm.mu.Lock()
	hm.queriesPending = 5 // Well above MaxPendingQueries
	hm.mu.Unlock()

	// Trigger timeout check
	hm.CheckTimeout(1.0)

	// Verify callback was called
	if !timeoutCalled {
		t.Error("Timeout callback was not called")
	}
	if timeoutMCU != "test_mcu" {
		t.Errorf("Expected MCU name 'test_mcu', got '%s'", timeoutMCU)
	}

	// Verify timeout state
	if !hm.IsTimeout() {
		t.Error("Expected IsTimeout() to be true")
	}
}

// TestHeartbeatClockResponse tests clock response handling.
func TestHeartbeatClockResponse(t *testing.T) {
	config := DefaultHeartbeatConfig()
	hm := NewHeartbeatMonitor("test_mcu", nil, nil, config)

	hm.Start()
	defer hm.Stop()

	// Increment pending count
	hm.mu.Lock()
	hm.queriesPending = 3
	hm.mu.Unlock()

	// Handle clock response
	hm.HandleClockResponse()

	// Verify pending count decreased
	if hm.QueriesPending() != 2 {
		t.Errorf("Expected 2 pending queries, got %d", hm.QueriesPending())
	}
}

// TestReconnectionManager tests the reconnection manager.
func TestReconnectionManager(t *testing.T) {
	config := ReconnectConfig{
		Enabled:       true,
		InitialDelay:  10 * time.Millisecond,
		MaxDelay:      100 * time.Millisecond,
		BackoffFactor: 2.0,
		MaxAttempts:   3,
	}

	rm := NewReconnectionManager("test_mcu", nil, config)

	// Test initial state
	if rm.State() != ReconnectStateIdle {
		t.Errorf("Expected initial state to be idle, got %v", rm.State())
	}

	// Test backoff calculation
	delay1 := rm.calculateBackoff(10 * time.Millisecond)
	if delay1 != 20*time.Millisecond {
		t.Errorf("Expected 20ms backoff, got %v", delay1)
	}

	// Test max delay cap
	delay2 := rm.calculateBackoff(60 * time.Millisecond)
	if delay2 != 100*time.Millisecond {
		t.Errorf("Expected max delay 100ms, got %v", delay2)
	}
}

// TestReconnectionManagerCallbacks tests the reconnection callbacks.
func TestReconnectionManagerCallbacks(t *testing.T) {
	var mu sync.Mutex
	reconnectingCalls := 0
	failedCalled := false

	config := ReconnectConfig{
		Enabled:       true,
		InitialDelay:  1 * time.Millisecond,
		MaxDelay:      5 * time.Millisecond,
		BackoffFactor: 1.5,
		MaxAttempts:   2,
		OnReconnecting: func(attempt int, delay time.Duration) {
			mu.Lock()
			reconnectingCalls++
			mu.Unlock()
		},
		OnReconnectFailed: func(err error) {
			mu.Lock()
			failedCalled = true
			mu.Unlock()
		},
	}

	rm := NewReconnectionManager("test_mcu", nil, config)
	rm.Start()

	// Trigger reconnection (will fail because no MCU connection)
	rm.TriggerReconnect("test")

	// Wait for reconnection attempts to complete
	time.Sleep(50 * time.Millisecond)

	rm.Stop()

	mu.Lock()
	calls := reconnectingCalls
	failed := failedCalled
	mu.Unlock()

	// Should have had reconnecting calls
	if calls == 0 {
		t.Error("OnReconnecting callback was not called")
	}

	// Should have failed after max attempts
	if !failed {
		t.Error("OnReconnectFailed callback was not called")
	}
}

// TestRestartHelper tests the restart helper.
func TestRestartHelper(t *testing.T) {
	config := RestartHelperConfig{
		Method:      RestartArduino,
		IsMCUBridge: false,
	}

	rh := NewRestartHelper("test_mcu", nil, config)

	// Test initial state
	status := rh.GetStatus()
	if status["method"] != string(RestartArduino) {
		t.Errorf("Expected method 'arduino', got '%v'", status["method"])
	}
	if status["is_mcu_bridge"].(bool) {
		t.Error("Expected is_mcu_bridge to be false")
	}
}

// TestRestartHelperBridge tests that bridge MCUs skip restart.
func TestRestartHelperBridge(t *testing.T) {
	config := RestartHelperConfig{
		Method:      RestartCommand,
		IsMCUBridge: true,
	}

	rh := NewRestartHelper("bridge_mcu", nil, config)

	// Firmware restart should succeed (skip) for bridge MCU
	err := rh.FirmwareRestart()
	if err != nil {
		t.Errorf("Expected no error for bridge MCU restart, got %v", err)
	}
}

// TestMCUDiagnostics tests the MCU diagnostics.
func TestMCUDiagnostics(t *testing.T) {
	md := NewMCUDiagnostics("test_mcu", nil)
	md.SetHostVersion("v1.0.0")
	md.SetMCUVersion("v1.0.0")

	// Test analyze shutdown
	details := &ShutdownDetails{
		Reason:    "Timer too close",
		EventType: "shutdown",
	}
	report := md.AnalyzeShutdown("MCU 'test_mcu' shutdown: Timer too close", details)

	// Verify report
	if report.ErrorCategory != "Timer too close" {
		t.Errorf("Expected category 'Timer too close', got '%s'", report.ErrorCategory)
	}
	if report.Explanation == "" {
		t.Error("Expected non-empty explanation")
	}
	if len(report.Suggestions) == 0 {
		t.Error("Expected suggestions for Timer too close error")
	}

	// Verify version mismatch detection
	if report.VersionMismatch {
		t.Error("Expected no version mismatch")
	}
}

// TestMCUDiagnosticsVersionMismatch tests version mismatch detection.
func TestMCUDiagnosticsVersionMismatch(t *testing.T) {
	md := NewMCUDiagnostics("test_mcu", nil)
	md.SetHostVersion("v1.0.0")
	md.SetMCUVersion("v1.0.1") // Different version

	report := md.AnalyzeShutdown("Test shutdown", nil)

	if !report.VersionMismatch {
		t.Error("Expected version mismatch to be detected")
	}
}

// TestDiagnosticsManager tests the diagnostics manager.
func TestDiagnosticsManager(t *testing.T) {
	dm := NewDiagnosticsManager("v1.0.0")

	// Add MCUs
	dm.AddMCU("mcu", nil)
	dm.AddMCU("mcu2", nil)

	// Verify MCUs were added
	if dm.GetDiagnostics("mcu") == nil {
		t.Error("Expected diagnostics for 'mcu'")
	}
	if dm.GetDiagnostics("mcu2") == nil {
		t.Error("Expected diagnostics for 'mcu2'")
	}

	// Test analyze shutdown
	details := &ShutdownDetails{
		MCUName:   "mcu",
		EventType: "shutdown",
	}
	report := dm.AnalyzeShutdown("Test shutdown", details)

	if report.MCUName != "mcu" {
		t.Errorf("Expected MCU name 'mcu', got '%s'", report.MCUName)
	}
}

// TestShutdownStatus tests the status reporting.
func TestShutdownStatus(t *testing.T) {
	r := reactor.New()
	sc := NewShutdownCoordinator(r)

	// Test status before shutdown
	status := sc.GetStatus()
	if status["state"] != string(StateStartup) {
		t.Errorf("Expected state 'startup', got '%v'", status["state"])
	}

	// Set ready and shutdown
	sc.SetReady()
	sc.InvokeShutdown("Test", &ShutdownDetails{
		MCUName:   "mcu",
		EventType: "test",
	})

	// Test status after shutdown
	status = sc.GetStatus()
	if status["state"] != string(StateShutdown) {
		t.Errorf("Expected state 'shutdown', got '%v'", status["state"])
	}
	if status["message"] != "Test" {
		t.Errorf("Expected message 'Test', got '%v'", status["message"])
	}
	if status["mcu"] != "mcu" {
		t.Errorf("Expected mcu 'mcu', got '%v'", status["mcu"])
	}
}

// TestHeartbeatManager tests the heartbeat manager.
func TestHeartbeatManager(t *testing.T) {
	r := reactor.New()
	sc := NewShutdownCoordinator(r)

	hm := NewHeartbeatManager(r, sc)

	// Add MCUs
	hm.AddMCU("mcu", nil)
	hm.AddMCU("mcu2", nil)

	// Verify monitors were created
	if hm.GetMonitor("mcu") == nil {
		t.Error("Expected monitor for 'mcu'")
	}
	if hm.GetMonitor("mcu2") == nil {
		t.Error("Expected monitor for 'mcu2'")
	}

	// Test start/stop
	hm.StartAll()
	hm.StopAll()

	// Test clock response routing
	hm.HandleClockResponse("mcu")
}

// BenchmarkShutdownCoordinator benchmarks the shutdown coordinator.
func BenchmarkShutdownCoordinator(b *testing.B) {
	r := reactor.New()

	for i := 0; i < b.N; i++ {
		sc := NewShutdownCoordinator(r)
		sc.SetReady()

		// Register some handlers
		for j := 0; j < 10; j++ {
			sc.RegisterShutdownHandler("handler", func() {})
		}

		// Trigger shutdown
		sc.InvokeShutdown("Benchmark shutdown", nil)
	}
}

// BenchmarkHeartbeatClockResponse benchmarks clock response handling.
func BenchmarkHeartbeatClockResponse(b *testing.B) {
	config := DefaultHeartbeatConfig()
	hm := NewHeartbeatMonitor("test_mcu", nil, nil, config)
	hm.Start()
	defer hm.Stop()

	hm.mu.Lock()
	hm.queriesPending = 100
	hm.mu.Unlock()

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		hm.HandleClockResponse()
		hm.mu.Lock()
		hm.queriesPending++
		hm.mu.Unlock()
	}
}
