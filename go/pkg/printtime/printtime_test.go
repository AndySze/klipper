package printtime

import (
	"testing"

	"klipper-go-migration/pkg/clocksync"
)

func newTestManager() *Manager {
	cs := clocksync.New(72000000.0) // 72 MHz
	cs.Initialize(0, 0)
	return New(cs)
}

func TestNew(t *testing.T) {
	m := newTestManager()
	if m == nil {
		t.Fatal("New returned nil")
	}

	if m.GetPrintTime() != 0 {
		t.Errorf("Initial print time should be 0, got %f", m.GetPrintTime())
	}
}

func TestReset(t *testing.T) {
	m := newTestManager()

	// Advance print time
	m.AdvanceMoveTime(1.0)
	m.IncrementPrintStall()

	// Reset
	m.Reset()

	if m.GetPrintTime() != 0 {
		t.Errorf("Print time should be 0 after reset, got %f", m.GetPrintTime())
	}

	if m.GetPrintStall() != 0 {
		t.Errorf("Print stall should be 0 after reset, got %d", m.GetPrintStall())
	}
}

func TestAdvanceMoveTime(t *testing.T) {
	m := newTestManager()

	// Advance to 1.0
	m.AdvanceMoveTime(1.0)
	if m.GetPrintTime() != 1.0 {
		t.Errorf("Print time should be 1.0, got %f", m.GetPrintTime())
	}

	// Try to advance to earlier time (should not change)
	m.AdvanceMoveTime(0.5)
	if m.GetPrintTime() != 1.0 {
		t.Errorf("Print time should still be 1.0, got %f", m.GetPrintTime())
	}

	// Advance to 2.0
	m.AdvanceMoveTime(2.0)
	if m.GetPrintTime() != 2.0 {
		t.Errorf("Print time should be 2.0, got %f", m.GetPrintTime())
	}
}

func TestDwell(t *testing.T) {
	m := newTestManager()

	// Advance to 1.0
	m.AdvanceMoveTime(1.0)

	// Dwell for 0.5 seconds
	newTime := m.Dwell(0.5)
	if newTime != 1.5 {
		t.Errorf("Dwell should return 1.5, got %f", newTime)
	}

	if m.GetPrintTime() != 1.5 {
		t.Errorf("Print time should be 1.5 after dwell, got %f", m.GetPrintTime())
	}

	// Dwell with negative should be treated as 0
	m.Dwell(-0.5)
	if m.GetPrintTime() != 1.5 {
		t.Errorf("Print time should still be 1.5 after negative dwell, got %f", m.GetPrintTime())
	}
}

func TestGetBufferTime(t *testing.T) {
	m := newTestManager()

	// With print time at 0 and est_print_time at ~0, buffer should be ~0
	buffer := m.GetBufferTime(0)
	if buffer < 0 {
		t.Errorf("Buffer time should not be negative, got %f", buffer)
	}

	// Advance print time
	m.AdvanceMoveTime(1.0)

	// Buffer should be ~1.0 (since est_print_time is ~0)
	buffer = m.GetBufferTime(0)
	if buffer < 0.9 || buffer > 1.1 {
		t.Errorf("Buffer time should be ~1.0, got %f", buffer)
	}
}

func TestNeedPause(t *testing.T) {
	m := newTestManager()

	// With small buffer, no pause needed
	m.AdvanceMoveTime(0.5)
	if m.NeedPause(0) {
		t.Error("Should not need pause with small buffer")
	}

	// With large buffer, pause needed
	m.AdvanceMoveTime(5.0)
	if !m.NeedPause(0) {
		t.Error("Should need pause with large buffer")
	}
}

func TestGetPauseTime(t *testing.T) {
	m := newTestManager()

	// With small buffer, pause time should be 0
	m.AdvanceMoveTime(1.0)
	pauseTime := m.GetPauseTime(0)
	if pauseTime > 0 {
		t.Errorf("Pause time should be 0 with small buffer, got %f", pauseTime)
	}

	// With large buffer, pause time should be positive
	m.AdvanceMoveTime(10.0)
	pauseTime = m.GetPauseTime(0)
	if pauseTime <= 0 {
		t.Errorf("Pause time should be positive with large buffer, got %f", pauseTime)
	}
}

func TestCalcPrintTime(t *testing.T) {
	m := newTestManager()

	// Initially print time should be 0
	if m.GetPrintTime() != 0 {
		t.Errorf("Initial print time should be 0")
	}

	// Calc print time with kinTime = 0
	printTime := m.CalcPrintTime(0, 0)

	// Print time should be at least BUFFER_TIME_START
	if printTime < BUFFER_TIME_START {
		t.Errorf("Print time should be at least %f, got %f", BUFFER_TIME_START, printTime)
	}
}

func TestPrintStall(t *testing.T) {
	m := newTestManager()

	if m.GetPrintStall() != 0 {
		t.Error("Initial print stall should be 0")
	}

	m.IncrementPrintStall()
	if m.GetPrintStall() != 1 {
		t.Errorf("Print stall should be 1, got %d", m.GetPrintStall())
	}

	m.IncrementPrintStall()
	m.IncrementPrintStall()
	if m.GetPrintStall() != 3 {
		t.Errorf("Print stall should be 3, got %d", m.GetPrintStall())
	}
}

func TestNeedCheckPause(t *testing.T) {
	m := newTestManager()

	if m.GetNeedCheckPause() != 0 {
		t.Error("Initial need check pause should be 0")
	}

	m.SetNeedCheckPause(5.0)
	if m.GetNeedCheckPause() != 5.0 {
		t.Errorf("Need check pause should be 5.0, got %f", m.GetNeedCheckPause())
	}
}

func TestCheckStallTime(t *testing.T) {
	m := newTestManager()

	if m.GetCheckStallTime() != 0 {
		t.Error("Initial check stall time should be 0")
	}

	m.SetCheckStallTime(3.0)
	if m.GetCheckStallTime() != 3.0 {
		t.Errorf("Check stall time should be 3.0, got %f", m.GetCheckStallTime())
	}
}

func TestGetStatus(t *testing.T) {
	m := newTestManager()
	m.AdvanceMoveTime(1.0)
	m.IncrementPrintStall()

	status := m.GetStatus(0)

	if status.PrintTime != 1.0 {
		t.Errorf("Status.PrintTime should be 1.0, got %f", status.PrintTime)
	}

	if status.PrintStall != 1 {
		t.Errorf("Status.PrintStall should be 1, got %d", status.PrintStall)
	}

	if !status.IsActive {
		t.Error("Status.IsActive should be true")
	}

	if status.BufferTime < 0 {
		t.Error("Status.BufferTime should not be negative")
	}
}

func TestOnSyncPrintTime(t *testing.T) {
	m := newTestManager()

	var called bool
	var callCurtime, callEstPrintTime, callPrintTime float64

	m.OnSyncPrintTime(func(curtime, estPrintTime, printTime float64) {
		called = true
		callCurtime = curtime
		callEstPrintTime = estPrintTime
		callPrintTime = printTime
	})

	// Calc print time should trigger the callback when print time changes
	m.CalcPrintTime(0.1, 1.0)

	if !called {
		t.Error("OnSyncPrintTime callback should have been called")
	}

	if callCurtime != 0.1 {
		t.Errorf("curtime should be 0.1, got %f", callCurtime)
	}

	if callPrintTime <= 0 {
		t.Error("printTime should be positive")
	}

	_ = callEstPrintTime // unused but verified it was passed
}

func TestPrintTimeToMCUClock(t *testing.T) {
	m := newTestManager()

	// At 72 MHz, 1 second = 72000000 clocks
	clock := m.PrintTimeToMCUClock(1.0)

	expected := int64(72000000)
	if clock != expected {
		t.Errorf("PrintTimeToMCUClock(1.0) = %d, want %d", clock, expected)
	}
}

func TestMCUClockToPrintTime(t *testing.T) {
	m := newTestManager()

	// 72000000 clocks = 1 second at 72 MHz
	printTime := m.MCUClockToPrintTime(72000000)

	if printTime < 0.999 || printTime > 1.001 {
		t.Errorf("MCUClockToPrintTime(72M) = %f, want ~1.0", printTime)
	}
}
