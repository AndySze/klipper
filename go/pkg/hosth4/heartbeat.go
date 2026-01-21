// Heartbeat monitor - MCU communication health monitoring
//
// This module monitors MCU communication health by tracking clock query responses.
// It detects communication timeouts and triggers shutdown when the MCU becomes unresponsive.
//
// The implementation mirrors klippy/clocksync.py timeout detection:
// - Periodic clock queries are sent to the MCU
// - If more than MAX_PENDING_QUERIES remain unanswered, timeout is triggered
//
// Copyright (C) 2019-2021 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"log"
	"sync"
	"time"

	"klipper-go-migration/pkg/reactor"
)

// HeartbeatConfig configures the heartbeat monitor.
type HeartbeatConfig struct {
	// QueryInterval is the time between clock queries.
	// Default: ~0.98s (Python uses 0.9839 to avoid resonance)
	QueryInterval time.Duration

	// MaxPendingQueries is the maximum number of unanswered queries before timeout.
	// Default: 4 (matching Python's clocksync.py)
	MaxPendingQueries int

	// TimeoutCallback is called when a timeout is detected.
	TimeoutCallback func(mcuName string, eventtime float64)
}

// DefaultHeartbeatConfig returns the default heartbeat configuration.
func DefaultHeartbeatConfig() HeartbeatConfig {
	return HeartbeatConfig{
		QueryInterval:     983900 * time.Microsecond, // ~0.9839s
		MaxPendingQueries: 4,
	}
}

// HeartbeatMonitor monitors MCU communication health.
type HeartbeatMonitor struct {
	mu sync.RWMutex

	// Configuration
	config  HeartbeatConfig
	mcuName string

	// MCU connection for sending queries
	mcuConn *MCUConnection

	// Reactor for timer scheduling
	reactor *reactor.Reactor

	// Query tracking
	queriesPending int  // Number of unanswered queries
	isTimeout      bool // True if timeout has been triggered
	isActive       bool // True if monitoring is active

	// Timer for periodic queries
	queryTimer *reactor.Timer
	stopCh     chan struct{}
	stopped    bool

	// Tracing
	trace bool
}

// NewHeartbeatMonitor creates a new heartbeat monitor.
func NewHeartbeatMonitor(mcuName string, mcuConn *MCUConnection, r *reactor.Reactor, config HeartbeatConfig) *HeartbeatMonitor {
	if config.QueryInterval == 0 {
		config.QueryInterval = DefaultHeartbeatConfig().QueryInterval
	}
	if config.MaxPendingQueries == 0 {
		config.MaxPendingQueries = DefaultHeartbeatConfig().MaxPendingQueries
	}

	return &HeartbeatMonitor{
		config:  config,
		mcuName: mcuName,
		mcuConn: mcuConn,
		reactor: r,
		stopCh:  make(chan struct{}),
	}
}

// Start begins heartbeat monitoring.
func (hm *HeartbeatMonitor) Start() {
	hm.mu.Lock()
	defer hm.mu.Unlock()

	if hm.isActive {
		return
	}

	hm.isActive = true
	hm.isTimeout = false
	hm.queriesPending = 0
	hm.stopped = false

	// Register timer for periodic queries
	if hm.reactor != nil {
		hm.queryTimer = hm.reactor.RegisterTimer(hm.queryTimerCallback, reactor.NOW)
	} else {
		// Fallback to goroutine-based timing
		go hm.queryLoop()
	}

	hm.tracef("HeartbeatMonitor: Started for MCU %s (interval=%.3fs, max_pending=%d)\n",
		hm.mcuName, hm.config.QueryInterval.Seconds(), hm.config.MaxPendingQueries)
}

// Stop halts heartbeat monitoring.
func (hm *HeartbeatMonitor) Stop() {
	hm.mu.Lock()
	defer hm.mu.Unlock()

	if !hm.isActive {
		return
	}

	hm.isActive = false
	hm.stopped = true

	// Unregister timer
	if hm.queryTimer != nil && hm.reactor != nil {
		hm.reactor.UnregisterTimer(hm.queryTimer)
		hm.queryTimer = nil
	}

	// Signal stop to goroutine
	select {
	case <-hm.stopCh:
		// Already closed
	default:
		close(hm.stopCh)
	}

	hm.tracef("HeartbeatMonitor: Stopped for MCU %s\n", hm.mcuName)
}

// HandleClockResponse should be called when a clock response is received.
// This resets the pending query counter.
func (hm *HeartbeatMonitor) HandleClockResponse() {
	hm.mu.Lock()
	defer hm.mu.Unlock()

	if hm.queriesPending > 0 {
		hm.queriesPending--
	}

	hm.tracef("HeartbeatMonitor: Clock response received, pending=%d\n", hm.queriesPending)
}

// CheckTimeout checks if a timeout condition exists and triggers callback if so.
// This is called periodically to evaluate the communication state.
func (hm *HeartbeatMonitor) CheckTimeout(eventtime float64) {
	hm.mu.Lock()

	// Already timed out or not active
	if hm.isTimeout || !hm.isActive {
		hm.mu.Unlock()
		return
	}

	// Check if too many queries are pending
	if hm.queriesPending > hm.config.MaxPendingQueries {
		hm.isTimeout = true
		callback := hm.config.TimeoutCallback
		mcuName := hm.mcuName
		hm.mu.Unlock()

		log.Printf("HeartbeatMonitor: Timeout detected for MCU %s (pending=%d > max=%d)",
			mcuName, hm.queriesPending, hm.config.MaxPendingQueries)

		if callback != nil {
			callback(mcuName, eventtime)
		}
		return
	}

	hm.mu.Unlock()
}

// IsActive returns true if the heartbeat monitor is active.
func (hm *HeartbeatMonitor) IsActive() bool {
	hm.mu.RLock()
	defer hm.mu.RUnlock()
	return hm.isActive && hm.queriesPending <= hm.config.MaxPendingQueries
}

// IsTimeout returns true if a timeout has been detected.
func (hm *HeartbeatMonitor) IsTimeout() bool {
	hm.mu.RLock()
	defer hm.mu.RUnlock()
	return hm.isTimeout
}

// QueriesPending returns the number of pending queries.
func (hm *HeartbeatMonitor) QueriesPending() int {
	hm.mu.RLock()
	defer hm.mu.RUnlock()
	return hm.queriesPending
}

// SetTrace enables or disables trace logging.
func (hm *HeartbeatMonitor) SetTrace(enabled bool) {
	hm.mu.Lock()
	defer hm.mu.Unlock()
	hm.trace = enabled
}

// queryTimerCallback is called by the reactor to send periodic queries.
func (hm *HeartbeatMonitor) queryTimerCallback(eventtime float64) float64 {
	hm.mu.Lock()
	if !hm.isActive || hm.stopped {
		hm.mu.Unlock()
		return reactor.NEVER
	}
	hm.mu.Unlock()

	// Send clock query
	hm.sendClockQuery()

	// Check for timeout
	hm.CheckTimeout(eventtime)

	// Schedule next query
	return eventtime + hm.config.QueryInterval.Seconds()
}

// queryLoop is the fallback goroutine-based query loop.
func (hm *HeartbeatMonitor) queryLoop() {
	ticker := time.NewTicker(hm.config.QueryInterval)
	defer ticker.Stop()

	for {
		select {
		case <-hm.stopCh:
			return
		case <-ticker.C:
			hm.mu.RLock()
			active := hm.isActive
			hm.mu.RUnlock()

			if !active {
				return
			}

			hm.sendClockQuery()

			// Check timeout
			var eventtime float64
			if hm.reactor != nil {
				eventtime = hm.reactor.Monotonic()
			} else {
				eventtime = float64(time.Now().UnixNano()) / 1e9
			}
			hm.CheckTimeout(eventtime)
		}
	}
}

// sendClockQuery sends a get_clock command to the MCU.
func (hm *HeartbeatMonitor) sendClockQuery() {
	hm.mu.Lock()
	hm.queriesPending++
	pending := hm.queriesPending
	hm.mu.Unlock()

	hm.tracef("HeartbeatMonitor: Sending clock query, pending=%d\n", pending)

	// Send get_clock command
	if hm.mcuConn != nil {
		if err := hm.mcuConn.SendCommand("get_clock", nil); err != nil {
			hm.tracef("HeartbeatMonitor: Failed to send clock query: %v\n", err)
		}
	}
}

// GetStatus returns the current heartbeat status.
func (hm *HeartbeatMonitor) GetStatus() map[string]interface{} {
	hm.mu.RLock()
	defer hm.mu.RUnlock()

	return map[string]interface{}{
		"mcu_name":        hm.mcuName,
		"is_active":       hm.isActive,
		"is_timeout":      hm.isTimeout,
		"queries_pending": hm.queriesPending,
		"max_pending":     hm.config.MaxPendingQueries,
		"query_interval":  hm.config.QueryInterval.Seconds(),
	}
}

// tracef writes trace output if tracing is enabled.
func (hm *HeartbeatMonitor) tracef(format string, args ...interface{}) {
	if hm.trace {
		log.Printf(format, args...)
	}
}

// HeartbeatManager manages heartbeat monitors for multiple MCUs.
type HeartbeatManager struct {
	mu sync.RWMutex

	monitors map[string]*HeartbeatMonitor
	reactor  *reactor.Reactor
	config   HeartbeatConfig

	// Shutdown coordinator for timeout callbacks
	shutdownCoord *ShutdownCoordinator
}

// NewHeartbeatManager creates a new heartbeat manager.
func NewHeartbeatManager(r *reactor.Reactor, shutdownCoord *ShutdownCoordinator) *HeartbeatManager {
	config := DefaultHeartbeatConfig()
	config.TimeoutCallback = nil // Set per-MCU below

	return &HeartbeatManager{
		monitors:      make(map[string]*HeartbeatMonitor),
		reactor:       r,
		config:        config,
		shutdownCoord: shutdownCoord,
	}
}

// AddMCU adds a heartbeat monitor for an MCU.
func (hm *HeartbeatManager) AddMCU(mcuName string, mcuConn *MCUConnection) {
	hm.mu.Lock()
	defer hm.mu.Unlock()

	if _, exists := hm.monitors[mcuName]; exists {
		return
	}

	config := hm.config
	config.TimeoutCallback = func(name string, eventtime float64) {
		if hm.shutdownCoord != nil {
			hm.shutdownCoord.HandleMCUTimeout(name, eventtime)
		}
	}

	monitor := NewHeartbeatMonitor(mcuName, mcuConn, hm.reactor, config)
	hm.monitors[mcuName] = monitor
}

// RemoveMCU removes a heartbeat monitor for an MCU.
func (hm *HeartbeatManager) RemoveMCU(mcuName string) {
	hm.mu.Lock()
	defer hm.mu.Unlock()

	if monitor, exists := hm.monitors[mcuName]; exists {
		monitor.Stop()
		delete(hm.monitors, mcuName)
	}
}

// StartAll starts heartbeat monitoring for all MCUs.
func (hm *HeartbeatManager) StartAll() {
	hm.mu.RLock()
	defer hm.mu.RUnlock()

	for _, monitor := range hm.monitors {
		monitor.Start()
	}
}

// StopAll stops heartbeat monitoring for all MCUs.
func (hm *HeartbeatManager) StopAll() {
	hm.mu.RLock()
	defer hm.mu.RUnlock()

	for _, monitor := range hm.monitors {
		monitor.Stop()
	}
}

// HandleClockResponse routes a clock response to the appropriate monitor.
func (hm *HeartbeatManager) HandleClockResponse(mcuName string) {
	hm.mu.RLock()
	monitor := hm.monitors[mcuName]
	hm.mu.RUnlock()

	if monitor != nil {
		monitor.HandleClockResponse()
	}
}

// GetMonitor returns the heartbeat monitor for an MCU.
func (hm *HeartbeatManager) GetMonitor(mcuName string) *HeartbeatMonitor {
	hm.mu.RLock()
	defer hm.mu.RUnlock()
	return hm.monitors[mcuName]
}

// GetStatus returns status for all monitors.
func (hm *HeartbeatManager) GetStatus() map[string]interface{} {
	hm.mu.RLock()
	defer hm.mu.RUnlock()

	status := make(map[string]interface{})
	for name, monitor := range hm.monitors {
		status[name] = monitor.GetStatus()
	}
	return status
}
