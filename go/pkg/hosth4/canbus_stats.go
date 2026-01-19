// CAN Bus Stats - port of klippy/extras/canbus_stats.py (partial from bus.py)
//
// Support for CAN bus statistics tracking
//
// Copyright (C) 2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"log"
	"sync"
	"time"
)

// CANBusStats tracks CAN bus statistics.
type CANBusStats struct {
	rt             *runtime
	interface_     string
	txCount        uint64
	rxCount        uint64
	txBytes        uint64
	rxBytes        uint64
	txErrors       uint64
	rxErrors       uint64
	busOffCount    uint64
	lastUpdate     time.Time
	statsStartTime time.Time
	mu             sync.Mutex
}

// CANBusStatsConfig holds configuration for CANBusStats.
type CANBusStatsConfig struct {
	Interface string
}

// newCANBusStats creates a new CAN bus statistics tracker.
func newCANBusStats(rt *runtime, cfg CANBusStatsConfig) (*CANBusStats, error) {
	if cfg.Interface == "" {
		cfg.Interface = "can0"
	}

	stats := &CANBusStats{
		rt:             rt,
		interface_:    cfg.Interface,
		statsStartTime: time.Now(),
		lastUpdate:     time.Now(),
	}

	log.Printf("canbus_stats: initialized for %s", cfg.Interface)
	return stats, nil
}

// RecordTx records a transmitted message.
func (cs *CANBusStats) RecordTx(bytes int) {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	cs.txCount++
	cs.txBytes += uint64(bytes)
	cs.lastUpdate = time.Now()
}

// RecordRx records a received message.
func (cs *CANBusStats) RecordRx(bytes int) {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	cs.rxCount++
	cs.rxBytes += uint64(bytes)
	cs.lastUpdate = time.Now()
}

// RecordTxError records a transmit error.
func (cs *CANBusStats) RecordTxError() {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	cs.txErrors++
	cs.lastUpdate = time.Now()
}

// RecordRxError records a receive error.
func (cs *CANBusStats) RecordRxError() {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	cs.rxErrors++
	cs.lastUpdate = time.Now()
}

// RecordBusOff records a bus-off event.
func (cs *CANBusStats) RecordBusOff() {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	cs.busOffCount++
	cs.lastUpdate = time.Now()
}

// Reset resets all statistics.
func (cs *CANBusStats) Reset() {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	cs.txCount = 0
	cs.rxCount = 0
	cs.txBytes = 0
	cs.rxBytes = 0
	cs.txErrors = 0
	cs.rxErrors = 0
	cs.busOffCount = 0
	cs.statsStartTime = time.Now()
	cs.lastUpdate = time.Now()

	log.Printf("canbus_stats: reset statistics for %s", cs.interface_)
}

// GetStatus returns the CAN bus statistics.
func (cs *CANBusStats) GetStatus() map[string]any {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	duration := time.Since(cs.statsStartTime).Seconds()
	var txRate, rxRate float64
	if duration > 0 {
		txRate = float64(cs.txCount) / duration
		rxRate = float64(cs.rxCount) / duration
	}

	return map[string]any{
		"interface":    cs.interface_,
		"tx_count":     cs.txCount,
		"rx_count":     cs.rxCount,
		"tx_bytes":     cs.txBytes,
		"rx_bytes":     cs.rxBytes,
		"tx_errors":    cs.txErrors,
		"rx_errors":    cs.rxErrors,
		"bus_off":      cs.busOffCount,
		"tx_rate":      txRate,
		"rx_rate":      rxRate,
		"uptime":       duration,
		"last_update":  cs.lastUpdate,
	}
}

// GetTxCount returns the transmit count.
func (cs *CANBusStats) GetTxCount() uint64 {
	cs.mu.Lock()
	defer cs.mu.Unlock()
	return cs.txCount
}

// GetRxCount returns the receive count.
func (cs *CANBusStats) GetRxCount() uint64 {
	cs.mu.Lock()
	defer cs.mu.Unlock()
	return cs.rxCount
}

// GetErrorCount returns total error count.
func (cs *CANBusStats) GetErrorCount() uint64 {
	cs.mu.Lock()
	defer cs.mu.Unlock()
	return cs.txErrors + cs.rxErrors
}
