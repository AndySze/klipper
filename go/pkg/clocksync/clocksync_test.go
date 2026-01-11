package clocksync

import (
	"math"
	"testing"
	"time"
)

func TestNew(t *testing.T) {
	mcuFreq := 72000000.0 // 72 MHz
	cs := New(mcuFreq)

	if cs == nil {
		t.Fatal("New returned nil")
	}

	if cs.MCUFreq != mcuFreq {
		t.Errorf("MCUFreq = %f, want %f", cs.MCUFreq, mcuFreq)
	}

	if cs.minHalfRTT < 999999999.0 {
		t.Error("minHalfRTT should be initialized to very large value")
	}
}

func TestInitialize(t *testing.T) {
	mcuFreq := 72000000.0
	cs := New(mcuFreq)

	clock := int64(1000000)
	sentTime := 0.5

	cs.Initialize(clock, sentTime)

	if cs.lastClock != clock {
		t.Errorf("lastClock = %d, want %d", cs.lastClock, clock)
	}

	est := cs.GetEstimate()
	if est.Clock != clock {
		t.Errorf("est.Clock = %d, want %d", est.Clock, clock)
	}

	if est.SampleTime != sentTime {
		t.Errorf("est.SampleTime = %f, want %f", est.SampleTime, sentTime)
	}

	if est.Freq != mcuFreq {
		t.Errorf("est.Freq = %f, want %f", est.Freq, mcuFreq)
	}
}

func TestPrintTimeToClock(t *testing.T) {
	mcuFreq := 72000000.0
	cs := New(mcuFreq)

	printTime := 1.0 // 1 second
	expectedClock := int64(printTime * mcuFreq)

	clock := cs.PrintTimeToClock(printTime)
	if clock != expectedClock {
		t.Errorf("PrintTimeToClock(%f) = %d, want %d", printTime, clock, expectedClock)
	}
}

func TestClockToPrintTime(t *testing.T) {
	mcuFreq := 72000000.0
	cs := New(mcuFreq)

	clock := int64(72000000) // 1 second worth of clocks
	expectedPrintTime := float64(clock) / mcuFreq

	printTime := cs.ClockToPrintTime(clock)
	if math.Abs(printTime-expectedPrintTime) > 1e-9 {
		t.Errorf("ClockToPrintTime(%d) = %f, want %f", clock, printTime, expectedPrintTime)
	}
}

func TestGetClock(t *testing.T) {
	mcuFreq := 72000000.0
	cs := New(mcuFreq)

	// Initialize at time 0 with clock 0
	cs.Initialize(0, 0)

	// At time 1.0, clock should be ~72M
	clock := cs.GetClock(1.0)
	expected := int64(mcuFreq)

	// Allow some tolerance for floating point
	if math.Abs(float64(clock-expected)) > 1 {
		t.Errorf("GetClock(1.0) = %d, want ~%d", clock, expected)
	}
}

func TestEstimateClockSystime(t *testing.T) {
	mcuFreq := 72000000.0
	cs := New(mcuFreq)

	// Initialize at time 0 with clock 0
	cs.Initialize(0, 0)

	// Clock 72M should be at time 1.0
	sysTime := cs.EstimateClockSystime(int64(mcuFreq))
	if math.Abs(sysTime-1.0) > 1e-9 {
		t.Errorf("EstimateClockSystime(72M) = %f, want ~1.0", sysTime)
	}
}

func TestClock32ToClock64(t *testing.T) {
	mcuFreq := 72000000.0
	cs := New(mcuFreq)

	// Set last clock to a value with high bits set
	cs.Initialize(0x100000000+1000, 0)

	// Extend a 32-bit clock that wraps around
	clock32 := uint32(2000)
	clock64 := cs.Clock32ToClock64(clock32)

	expected := int64(0x100000000 + 2000)
	if clock64 != expected {
		t.Errorf("Clock32ToClock64(%d) = %d, want %d", clock32, clock64, expected)
	}
}

func TestHandleClock(t *testing.T) {
	mcuFreq := 72000000.0
	cs := New(mcuFreq)

	// Initialize
	cs.Initialize(0, 0)

	// Simulate clock responses
	sentTime := 0.1
	receiveTime := 0.101 // 1ms RTT
	clock32 := uint32(mcuFreq / 10) // ~0.1 seconds worth of clocks

	est := cs.HandleClock(clock32, sentTime, receiveTime)

	if est.Freq < 0 {
		t.Error("Frequency should be positive")
	}

	if !cs.IsActive() {
		t.Error("ClockSync should be active after HandleClock")
	}
}

func TestHandleClockMultiple(t *testing.T) {
	mcuFreq := 72000000.0
	cs := New(mcuFreq)

	// Initialize
	cs.Initialize(0, 0)

	// Simulate multiple clock responses to converge frequency estimate
	for i := 1; i <= 10; i++ {
		sentTime := float64(i) * 0.1
		receiveTime := sentTime + 0.001 // 1ms RTT
		clock32 := uint32(float64(i) * mcuFreq / 10)

		cs.HandleClock(clock32, sentTime, receiveTime)
	}

	est := cs.GetEstimate()

	// Frequency should be close to mcuFreq
	freqError := math.Abs(est.Freq-mcuFreq) / mcuFreq
	if freqError > 0.01 { // Within 1%
		t.Errorf("Frequency estimate error too large: %f%%", freqError*100)
	}
}

func TestIsActive(t *testing.T) {
	mcuFreq := 72000000.0
	cs := New(mcuFreq)

	if !cs.IsActive() {
		t.Error("ClockSync should be active initially")
	}

	// Increment pending queries
	for i := 0; i < 5; i++ {
		cs.IncrementQueriesPending()
	}

	if cs.IsActive() {
		t.Error("ClockSync should not be active with 5 pending queries")
	}
}

func TestGetStats(t *testing.T) {
	mcuFreq := 72000000.0
	cs := New(mcuFreq)
	cs.Initialize(1000, 0.5)

	stats := cs.GetStats()

	if stats.MCUFreq != mcuFreq {
		t.Errorf("stats.MCUFreq = %f, want %f", stats.MCUFreq, mcuFreq)
	}

	if stats.LastClock != 1000 {
		t.Errorf("stats.LastClock = %d, want 1000", stats.LastClock)
	}
}

func TestMonotonicTime(t *testing.T) {
	mt := NewMonotonicTime()

	t1 := mt.Now()
	time.Sleep(10 * time.Millisecond)
	t2 := mt.Now()

	if t2 <= t1 {
		t.Errorf("Monotonic time not increasing: %f <= %f", t2, t1)
	}

	elapsed := t2 - t1
	if elapsed < 0.009 || elapsed > 0.050 {
		t.Errorf("Unexpected elapsed time: %f", elapsed)
	}
}

func TestSecondarySync(t *testing.T) {
	mainFreq := 72000000.0
	secondaryFreq := 48000000.0

	mainSync := New(mainFreq)
	mainSync.Initialize(0, 0)

	ss := NewSecondarySync(mainSync, secondaryFreq)
	ss.Initialize(0, 0)
	ss.InitializeSecondary(0.1)

	// Test print time to clock conversion
	printTime := 1.0
	clock := ss.PrintTimeToClock(printTime)

	if clock < 0 {
		t.Error("Secondary clock should be positive")
	}

	// Test clock to print time
	printTimeBack := ss.ClockToPrintTime(clock)
	if math.Abs(printTimeBack-printTime) > 0.001 {
		t.Errorf("ClockToPrintTime roundtrip error: got %f, want ~%f", printTimeBack, printTime)
	}
}

func TestCalibrateClock(t *testing.T) {
	mainFreq := 72000000.0
	secondaryFreq := 48000000.0

	mainSync := New(mainFreq)
	mainSync.Initialize(0, 0)

	ss := NewSecondarySync(mainSync, secondaryFreq)
	ss.Initialize(0, 0)
	ss.InitializeSecondary(0.1)

	// Calibrate clock
	result := ss.CalibrateClock(0.1, 0.2)

	if result.AdjustedFreq <= 0 {
		t.Error("Adjusted frequency should be positive")
	}
}
