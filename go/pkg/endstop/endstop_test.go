package endstop

import (
	"testing"
	"time"
)

func TestDefaultEndstopConfig(t *testing.T) {
	cfg := DefaultEndstopConfig()

	if cfg.Name != "endstop" {
		t.Errorf("Name = %s, want endstop", cfg.Name)
	}
	if !cfg.PullUp {
		t.Error("PullUp should be true by default")
	}
	if cfg.Inverted {
		t.Error("Inverted should be false by default")
	}
	if cfg.DebounceTime != 1*time.Millisecond {
		t.Errorf("DebounceTime = %v, want 1ms", cfg.DebounceTime)
	}
}

func TestNew(t *testing.T) {
	cfg := DefaultEndstopConfig()
	cfg.Name = "x_endstop"
	cfg.Pin = "PA0"

	e := New(cfg)

	if e == nil {
		t.Fatal("New returned nil")
	}

	if e.GetName() != "x_endstop" {
		t.Errorf("Name = %s, want x_endstop", e.GetName())
	}

	if e.GetPin() != "PA0" {
		t.Errorf("Pin = %s, want PA0", e.GetPin())
	}

	if e.GetState() != StateUnknown {
		t.Errorf("Initial state = %s, want unknown", e.GetState())
	}
}

func TestEndstopStateString(t *testing.T) {
	tests := []struct {
		state    EndstopState
		expected string
	}{
		{StateOpen, "open"},
		{StateTriggered, "triggered"},
		{StateUnknown, "unknown"},
		{EndstopState(99), "unknown"},
	}

	for _, tt := range tests {
		if tt.state.String() != tt.expected {
			t.Errorf("State %d String() = %s, want %s", tt.state, tt.state.String(), tt.expected)
		}
	}
}

func TestHandleTrigger(t *testing.T) {
	cfg := DefaultEndstopConfig()
	cfg.DebounceTime = 0 // Disable debounce for testing
	e := New(cfg)

	// Track callback
	var callbackClock uint64
	callbackCalled := false
	e.SetTriggerCallback(func(clock uint64) {
		callbackCalled = true
		callbackClock = clock
	})

	// Trigger the endstop
	e.HandleTrigger(12345)

	if e.GetState() != StateTriggered {
		t.Errorf("State = %s, want triggered", e.GetState())
	}

	if !e.IsTriggered() {
		t.Error("IsTriggered should return true")
	}

	if !callbackCalled {
		t.Error("Trigger callback should have been called")
	}

	if callbackClock != 12345 {
		t.Errorf("Callback clock = %d, want 12345", callbackClock)
	}

	triggerTime, triggerClock := e.GetLastTrigger()
	if triggerClock != 12345 {
		t.Errorf("Last trigger clock = %d, want 12345", triggerClock)
	}
	if triggerTime.IsZero() {
		t.Error("Trigger time should be set")
	}
}

func TestDebounce(t *testing.T) {
	cfg := DefaultEndstopConfig()
	cfg.DebounceTime = 50 * time.Millisecond
	e := New(cfg)

	triggerCount := 0
	e.SetTriggerCallback(func(clock uint64) {
		triggerCount++
	})

	// First trigger should work
	e.HandleTrigger(100)
	if triggerCount != 1 {
		t.Errorf("First trigger: count = %d, want 1", triggerCount)
	}

	// Immediate second trigger should be debounced
	e.HandleTrigger(101)
	if triggerCount != 1 {
		t.Errorf("Second trigger (debounced): count = %d, want 1", triggerCount)
	}

	// Wait for debounce to expire
	time.Sleep(60 * time.Millisecond)

	// Third trigger should work
	e.HandleTrigger(102)
	if triggerCount != 2 {
		t.Errorf("Third trigger: count = %d, want 2", triggerCount)
	}
}

func TestQuery(t *testing.T) {
	cfg := DefaultEndstopConfig()
	e := New(cfg)

	// Without callback, should return error
	state, err := e.Query()
	if err == nil {
		t.Error("Query without callback should return error")
	}
	if state != StateUnknown {
		t.Errorf("State = %s, want unknown", state)
	}

	// Set query callback
	queryResult := false
	e.SetQueryCallback(func() (bool, error) {
		return queryResult, nil
	})

	// Query open state
	queryResult = false
	state, err = e.Query()
	if err != nil {
		t.Errorf("Query failed: %v", err)
	}
	if state != StateOpen {
		t.Errorf("State = %s, want open", state)
	}

	// Query triggered state
	queryResult = true
	state, err = e.Query()
	if err != nil {
		t.Errorf("Query failed: %v", err)
	}
	if state != StateTriggered {
		t.Errorf("State = %s, want triggered", state)
	}
}

func TestQueryInverted(t *testing.T) {
	cfg := DefaultEndstopConfig()
	cfg.Inverted = true
	e := New(cfg)

	queryResult := false
	e.SetQueryCallback(func() (bool, error) {
		return queryResult, nil
	})

	// With inverted=true, false from query means triggered
	queryResult = false
	state, err := e.Query()
	if err != nil {
		t.Errorf("Query failed: %v", err)
	}
	if state != StateTriggered {
		t.Errorf("Inverted state = %s, want triggered", state)
	}

	// true from query means open
	queryResult = true
	state, err = e.Query()
	if err != nil {
		t.Errorf("Query failed: %v", err)
	}
	if state != StateOpen {
		t.Errorf("Inverted state = %s, want open", state)
	}
}

func TestHoming(t *testing.T) {
	cfg := DefaultEndstopConfig()
	cfg.DebounceTime = 0
	e := New(cfg)

	if e.IsHoming() {
		t.Error("Should not be homing initially")
	}

	// Start homing
	err := e.StartHoming(1)
	if err != nil {
		t.Errorf("StartHoming failed: %v", err)
	}

	if !e.IsHoming() {
		t.Error("Should be homing after StartHoming")
	}

	// Simulate trigger in goroutine
	go func() {
		time.Sleep(10 * time.Millisecond)
		e.HandleTrigger(54321)
	}()

	// Wait for trigger
	clock, err := e.WaitForTrigger(100 * time.Millisecond)
	if err != nil {
		t.Errorf("WaitForTrigger failed: %v", err)
	}

	if clock != 54321 {
		t.Errorf("Trigger clock = %d, want 54321", clock)
	}

	// Stop homing
	e.StopHoming()

	if e.IsHoming() {
		t.Error("Should not be homing after StopHoming")
	}
}

func TestHomingTimeout(t *testing.T) {
	cfg := DefaultEndstopConfig()
	e := New(cfg)

	err := e.StartHoming(1)
	if err != nil {
		t.Errorf("StartHoming failed: %v", err)
	}

	// Wait for trigger with short timeout
	_, err = e.WaitForTrigger(50 * time.Millisecond)
	if err != ErrEndstopTimeout {
		t.Errorf("Expected ErrEndstopTimeout, got %v", err)
	}
}

func TestWaitForTriggerNotHoming(t *testing.T) {
	cfg := DefaultEndstopConfig()
	e := New(cfg)

	// Try to wait without homing
	_, err := e.WaitForTrigger(100 * time.Millisecond)
	if err != ErrNotHoming {
		t.Errorf("Expected ErrNotHoming, got %v", err)
	}
}

func TestGetStatus(t *testing.T) {
	cfg := DefaultEndstopConfig()
	cfg.Name = "test_endstop"
	cfg.Pin = "PB1"
	cfg.DebounceTime = 0
	e := New(cfg)

	// Initial status
	status := e.GetStatus()
	if status.Name != "test_endstop" {
		t.Errorf("Status.Name = %s, want test_endstop", status.Name)
	}
	if status.Pin != "PB1" {
		t.Errorf("Status.Pin = %s, want PB1", status.Pin)
	}
	if status.State != "unknown" {
		t.Errorf("Status.State = %s, want unknown", status.State)
	}
	if status.IsTriggered {
		t.Error("Status.IsTriggered should be false")
	}
	if status.IsHoming {
		t.Error("Status.IsHoming should be false")
	}

	// After trigger
	e.HandleTrigger(100)
	status = e.GetStatus()
	if status.State != "triggered" {
		t.Errorf("Status.State = %s, want triggered", status.State)
	}
	if !status.IsTriggered {
		t.Error("Status.IsTriggered should be true")
	}

	// After starting homing
	e.StartHoming(1)
	status = e.GetStatus()
	if !status.IsHoming {
		t.Error("Status.IsHoming should be true")
	}
}

// EndstopGroup tests

func TestEndstopGroup(t *testing.T) {
	g := NewEndstopGroup("x_axis")

	cfg := DefaultEndstopConfig()
	cfg.DebounceTime = 0

	e1 := New(cfg)
	e2 := New(cfg)

	g.Add(e1)
	g.Add(e2)

	// Initially none triggered
	if g.AnyTriggered() {
		t.Error("No endstops should be triggered initially")
	}

	// Trigger one
	e1.HandleTrigger(100)

	if !g.AnyTriggered() {
		t.Error("Should show triggered after one triggers")
	}
}

func TestEndstopGroupQueryAll(t *testing.T) {
	g := NewEndstopGroup("test")

	cfg := DefaultEndstopConfig()
	e1 := New(cfg)
	e2 := New(cfg)
	e3 := New(cfg)

	// Set up query callbacks
	e1.SetQueryCallback(func() (bool, error) { return false, nil })
	e2.SetQueryCallback(func() (bool, error) { return true, nil }) // triggered
	e3.SetQueryCallback(func() (bool, error) { return false, nil })

	g.Add(e1)
	g.Add(e2)
	g.Add(e3)

	triggered, err := g.QueryAll()
	if err != nil {
		t.Errorf("QueryAll failed: %v", err)
	}

	if len(triggered) != 1 {
		t.Errorf("QueryAll returned %d triggered, want 1", len(triggered))
	}

	if triggered[0] != e2 {
		t.Error("Wrong endstop returned as triggered")
	}
}

func TestEndstopGroupHoming(t *testing.T) {
	g := NewEndstopGroup("test")

	cfg := DefaultEndstopConfig()
	cfg.DebounceTime = 0
	e1 := New(cfg)
	e2 := New(cfg)

	g.Add(e1)
	g.Add(e2)

	// Start homing on all
	err := g.StartHomingAll(1)
	if err != nil {
		t.Errorf("StartHomingAll failed: %v", err)
	}

	if !e1.IsHoming() || !e2.IsHoming() {
		t.Error("All endstops should be homing")
	}

	// Trigger e1 in goroutine
	go func() {
		time.Sleep(10 * time.Millisecond)
		e1.HandleTrigger(999)
	}()

	// Wait for any trigger
	triggered, clock, err := g.WaitForAnyTrigger(100 * time.Millisecond)
	if err != nil {
		t.Errorf("WaitForAnyTrigger failed: %v", err)
	}

	if triggered != e1 {
		t.Error("Wrong endstop returned")
	}

	if clock != 999 {
		t.Errorf("Clock = %d, want 999", clock)
	}

	// Stop all
	g.StopHomingAll()

	if e1.IsHoming() || e2.IsHoming() {
		t.Error("No endstops should be homing after StopHomingAll")
	}
}

func TestEndstopGroupWaitTimeout(t *testing.T) {
	g := NewEndstopGroup("test")

	cfg := DefaultEndstopConfig()
	e1 := New(cfg)

	g.Add(e1)

	err := g.StartHomingAll(1)
	if err != nil {
		t.Errorf("StartHomingAll failed: %v", err)
	}

	// Wait with short timeout, no trigger
	_, _, err = g.WaitForAnyTrigger(50 * time.Millisecond)
	if err != ErrEndstopTimeout {
		t.Errorf("Expected ErrEndstopTimeout, got %v", err)
	}
}
