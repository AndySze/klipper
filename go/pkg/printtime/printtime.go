// Package printtime provides print time management and synchronization.
// Print time is a virtual time that advances with the motion queue and
// is synchronized with MCU clocks.
package printtime

import (
	"sync"
	"time"

	"klipper-go-migration/pkg/clocksync"
)

// Constants
const (
	// BUFFER_TIME_LOW is the minimum buffer between print_time and est_print_time
	BUFFER_TIME_LOW = 1.0

	// BUFFER_TIME_HIGH is when we start pausing to let buffer drain
	BUFFER_TIME_HIGH = 2.0

	// BUFFER_TIME_START is the minimum buffer when resuming
	BUFFER_TIME_START = 0.250
)

// Manager manages print time and its synchronization with the MCU clock.
type Manager struct {
	mu sync.RWMutex

	// Print time state
	printTime       float64
	needCheckPause  float64
	checkStallTime  float64
	printStall      int

	// Clock synchronization
	clockSync *clocksync.ClockSync

	// Callbacks
	onSyncPrintTime func(curtime, estPrintTime, printTime float64)
}

// New creates a new print time Manager.
func New(clockSync *clocksync.ClockSync) *Manager {
	return &Manager{
		clockSync: clockSync,
	}
}

// OnSyncPrintTime sets a callback for when print time is synchronized.
func (m *Manager) OnSyncPrintTime(fn func(curtime, estPrintTime, printTime float64)) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.onSyncPrintTime = fn
}

// Reset resets the print time to 0.
func (m *Manager) Reset() {
	m.mu.Lock()
	defer m.mu.Unlock()

	m.printTime = 0
	m.needCheckPause = 0
	m.checkStallTime = 0
	m.printStall = 0
}

// GetPrintTime returns the current print time.
func (m *Manager) GetPrintTime() float64 {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.printTime
}

// AdvanceMoveTime advances the print time to at least the given time.
func (m *Manager) AdvanceMoveTime(nextPrintTime float64) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if nextPrintTime > m.printTime {
		m.printTime = nextPrintTime
	}
}

// CalcPrintTime calculates and updates the print time based on the current
// estimated print time and kinematic state.
func (m *Manager) CalcPrintTime(curtime float64, kinTime float64) float64 {
	m.mu.Lock()
	defer m.mu.Unlock()

	estPrintTime := m.clockSync.EstimatedPrintTime(curtime)
	minPrintTime := estPrintTime + BUFFER_TIME_START
	if kinTime > minPrintTime {
		minPrintTime = kinTime
	}

	if minPrintTime > m.printTime {
		oldPrintTime := m.printTime
		m.printTime = minPrintTime

		// Notify callback if registered
		if m.onSyncPrintTime != nil && oldPrintTime != m.printTime {
			m.onSyncPrintTime(curtime, estPrintTime, m.printTime)
		}
	}

	return m.printTime
}

// GetBufferTime returns the time buffered ahead of real time.
func (m *Manager) GetBufferTime(eventtime float64) float64 {
	m.mu.RLock()
	defer m.mu.RUnlock()

	estPrintTime := m.clockSync.EstimatedPrintTime(eventtime)
	bufferTime := m.printTime - estPrintTime
	if bufferTime < 0 {
		bufferTime = 0
	}
	return bufferTime
}

// NeedPause returns true if we need to pause to let the buffer drain.
func (m *Manager) NeedPause(eventtime float64) bool {
	m.mu.RLock()
	defer m.mu.RUnlock()

	estPrintTime := m.clockSync.EstimatedPrintTime(eventtime)
	willPauseTime := m.printTime - estPrintTime - BUFFER_TIME_HIGH
	return willPauseTime > 0
}

// GetPauseTime returns how long to pause, or 0 if no pause needed.
func (m *Manager) GetPauseTime(eventtime float64) float64 {
	m.mu.RLock()
	defer m.mu.RUnlock()

	estPrintTime := m.clockSync.EstimatedPrintTime(eventtime)
	pauseTime := m.printTime - estPrintTime - BUFFER_TIME_HIGH
	if pauseTime < 0 {
		pauseTime = 0
	}
	return pauseTime
}

// WaitForFlush waits until the print time buffer has drained below the threshold.
func (m *Manager) WaitForFlush(timeout time.Duration) bool {
	deadline := time.Now().Add(timeout)

	for time.Now().Before(deadline) {
		eventtime := float64(time.Now().UnixNano()) / 1e9

		m.mu.RLock()
		estPrintTime := m.clockSync.EstimatedPrintTime(eventtime)
		printTime := m.printTime
		m.mu.RUnlock()

		if estPrintTime >= printTime {
			return true
		}

		// Wait a bit before checking again
		sleepTime := (printTime - estPrintTime) * float64(time.Second)
		if sleepTime > float64(100*time.Millisecond) {
			sleepTime = float64(100 * time.Millisecond)
		}
		if sleepTime > 0 {
			time.Sleep(time.Duration(sleepTime))
		}
	}

	return false
}

// SetNeedCheckPause marks that pause checking is needed at the given print time.
func (m *Manager) SetNeedCheckPause(printTime float64) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.needCheckPause = printTime
}

// GetNeedCheckPause returns the print time when pause checking is needed.
func (m *Manager) GetNeedCheckPause() float64 {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.needCheckPause
}

// SetCheckStallTime sets the time to check for stalls.
func (m *Manager) SetCheckStallTime(t float64) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.checkStallTime = t
}

// GetCheckStallTime returns the stall check time.
func (m *Manager) GetCheckStallTime() float64 {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.checkStallTime
}

// IncrementPrintStall increments the print stall counter.
func (m *Manager) IncrementPrintStall() {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.printStall++
}

// GetPrintStall returns the print stall count.
func (m *Manager) GetPrintStall() int {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.printStall
}

// Status returns a status struct with current print time state.
type Status struct {
	PrintTime          float64
	EstimatedPrintTime float64
	BufferTime         float64
	PrintStall         int
	IsActive           bool
}

// GetStatus returns the current print time status.
func (m *Manager) GetStatus(eventtime float64) Status {
	m.mu.RLock()
	defer m.mu.RUnlock()

	estPrintTime := m.clockSync.EstimatedPrintTime(eventtime)
	bufferTime := m.printTime - estPrintTime
	if bufferTime < 0 {
		bufferTime = 0
	}

	return Status{
		PrintTime:          m.printTime,
		EstimatedPrintTime: estPrintTime,
		BufferTime:         bufferTime,
		PrintStall:         m.printStall,
		IsActive:           m.printTime > 0,
	}
}

// Dwell advances the print time by the given delay (for G4 commands).
func (m *Manager) Dwell(delay float64) float64 {
	m.mu.Lock()
	defer m.mu.Unlock()

	if delay < 0 {
		delay = 0
	}

	m.printTime += delay
	return m.printTime
}

// PrintTimeToMCUClock converts a print time to an MCU clock value.
func (m *Manager) PrintTimeToMCUClock(printTime float64) int64 {
	return m.clockSync.PrintTimeToClock(printTime)
}

// MCUClockToPrintTime converts an MCU clock value to a print time.
func (m *Manager) MCUClockToPrintTime(clock int64) float64 {
	return m.clockSync.ClockToPrintTime(clock)
}
