// Statistics - port of klippy/extras/statistics.py
//
// Support for logging periodic statistics
//
// Copyright (C) 2018-2021 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"bufio"
	"fmt"
	"log"
	"os"
	goruntime "runtime"
	"strconv"
	"strings"
	"sync"
	"time"
)

// PrinterSysStats provides system statistics.
type PrinterSysStats struct {
	rt               *runtime
	lastProcessTime  float64
	totalProcessTime float64
	lastLoadAvg      float64
	lastMemAvail     int64
	memFile          *os.File
	startTime        time.Time
	mu               sync.RWMutex
}

// newPrinterSysStats creates a new system stats tracker.
func newPrinterSysStats(rt *runtime) *PrinterSysStats {
	pss := &PrinterSysStats{
		rt:        rt,
		startTime: time.Now(),
	}

	// Try to open meminfo file
	f, err := os.Open("/proc/meminfo")
	if err == nil {
		pss.memFile = f
	}

	return pss
}

// Close closes the meminfo file.
func (pss *PrinterSysStats) Close() {
	pss.mu.Lock()
	defer pss.mu.Unlock()

	if pss.memFile != nil {
		pss.memFile.Close()
		pss.memFile = nil
	}
}

// Stats collects and returns current system statistics.
func (pss *PrinterSysStats) Stats(eventtime float64) (bool, string) {
	pss.mu.Lock()
	defer pss.mu.Unlock()

	// Calculate process CPU time
	elapsed := time.Since(pss.startTime).Seconds()
	pss.totalProcessTime = elapsed

	// Get load average (Linux-specific)
	pss.lastLoadAvg = pss.getLoadAvg()

	msg := fmt.Sprintf("sysload=%.2f cputime=%.3f", pss.lastLoadAvg, pss.totalProcessTime)

	// Get available memory
	if pss.memFile != nil {
		memAvail := pss.getMemAvail()
		if memAvail > 0 {
			pss.lastMemAvail = memAvail
			msg = fmt.Sprintf("%s memavail=%d", msg, memAvail)
		}
	}

	return false, msg
}

// getLoadAvg reads the system load average.
func (pss *PrinterSysStats) getLoadAvg() float64 {
	data, err := os.ReadFile("/proc/loadavg")
	if err != nil {
		return 0
	}

	fields := strings.Fields(string(data))
	if len(fields) < 1 {
		return 0
	}

	loadAvg, err := strconv.ParseFloat(fields[0], 64)
	if err != nil {
		return 0
	}

	return loadAvg
}

// getMemAvail reads available memory from /proc/meminfo.
func (pss *PrinterSysStats) getMemAvail() int64 {
	if pss.memFile == nil {
		return 0
	}

	pss.memFile.Seek(0, 0)
	scanner := bufio.NewScanner(pss.memFile)

	for scanner.Scan() {
		line := scanner.Text()
		if strings.HasPrefix(line, "MemAvailable:") {
			fields := strings.Fields(line)
			if len(fields) >= 2 {
				val, err := strconv.ParseInt(fields[1], 10, 64)
				if err == nil {
					return val
				}
			}
		}
	}

	return 0
}

// GetStatus returns the system stats status.
func (pss *PrinterSysStats) GetStatus() map[string]any {
	pss.mu.RLock()
	defer pss.mu.RUnlock()

	return map[string]any{
		"sysload":  pss.lastLoadAvg,
		"cputime":  pss.totalProcessTime,
		"memavail": pss.lastMemAvail,
	}
}

// PrinterStats provides periodic statistics logging.
type PrinterStats struct {
	rt          *runtime
	sysStats    *PrinterSysStats
	statsTimer  *time.Ticker
	stopChan    chan struct{}
	running     bool
	statsCallbacks []func(float64) (bool, string)
	mu          sync.Mutex
}

// newPrinterStats creates a new statistics handler.
func newPrinterStats(rt *runtime) *PrinterStats {
	ps := &PrinterStats{
		rt:       rt,
		sysStats: newPrinterSysStats(rt),
		stopChan: make(chan struct{}),
	}

	return ps
}

// RegisterCallback registers a stats callback.
func (ps *PrinterStats) RegisterCallback(cb func(float64) (bool, string)) {
	ps.mu.Lock()
	defer ps.mu.Unlock()
	ps.statsCallbacks = append(ps.statsCallbacks, cb)
}

// Start starts periodic statistics collection.
func (ps *PrinterStats) Start() {
	ps.mu.Lock()
	if ps.running {
		ps.mu.Unlock()
		return
	}
	ps.running = true
	ps.statsTimer = time.NewTicker(1 * time.Second)
	ps.mu.Unlock()

	go ps.statsLoop()
}

// Stop stops periodic statistics collection.
func (ps *PrinterStats) Stop() {
	ps.mu.Lock()
	if !ps.running {
		ps.mu.Unlock()
		return
	}
	ps.running = false
	ps.mu.Unlock()

	close(ps.stopChan)
	ps.sysStats.Close()
}

// statsLoop runs the periodic stats collection.
func (ps *PrinterStats) statsLoop() {
	for {
		select {
		case <-ps.stopChan:
			ps.statsTimer.Stop()
			return
		case <-ps.statsTimer.C:
			ps.generateStats()
		}
	}
}

// generateStats generates and logs statistics.
func (ps *PrinterStats) generateStats() {
	ps.mu.Lock()
	callbacks := make([]func(float64) (bool, string), len(ps.statsCallbacks))
	copy(callbacks, ps.statsCallbacks)
	ps.mu.Unlock()

	eventtime := float64(time.Now().UnixNano()) / 1e9

	// Collect stats from system
	_, sysMsg := ps.sysStats.Stats(eventtime)

	// Collect stats from registered callbacks
	var statsStrings []string
	statsStrings = append(statsStrings, sysMsg)

	shouldLog := false
	for _, cb := range callbacks {
		log, msg := cb(eventtime)
		if log {
			shouldLog = true
		}
		if msg != "" {
			statsStrings = append(statsStrings, msg)
		}
	}

	if shouldLog {
		log.Printf("Stats %.1f: %s", eventtime, strings.Join(statsStrings, " "))
	}
}

// GetSysStats returns the system stats component.
func (ps *PrinterStats) GetSysStats() *PrinterSysStats {
	return ps.sysStats
}

// GetMemStats returns Go runtime memory statistics.
func (ps *PrinterStats) GetMemStats() map[string]any {
	var m goruntime.MemStats
	goruntime.ReadMemStats(&m)

	return map[string]any{
		"alloc":       m.Alloc,
		"total_alloc": m.TotalAlloc,
		"sys":         m.Sys,
		"num_gc":      m.NumGC,
		"goroutines":  goruntime.NumGoroutine(),
	}
}
