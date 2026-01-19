// Garbage Collection - port of klippy/extras/garbage_collection.py
//
// Manage garbage collection for timing-sensitive operations
//
// Copyright (C) 2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"log"
	goruntime "runtime"
	"runtime/debug"
	"sync"
	"time"
)

// GCMode represents garbage collection mode.
type GCMode int

const (
	GCModeAuto     GCMode = iota // Normal GC
	GCModeDisabled               // GC disabled during printing
	GCModeManual                 // Manual GC control
)

// GarbageCollection manages Go garbage collection.
type GarbageCollection struct {
	mode            GCMode
	enabled         bool
	lastGCTime      time.Time
	gcCount         uint64
	pauseNS         uint64 // Total GC pause time in nanoseconds
	forcedGCCount   uint64
	printingActive  bool
	gcPercent       int
	savedGCPercent  int
	mu              sync.Mutex
}

// GarbageCollectionConfig holds configuration.
type GarbageCollectionConfig struct {
	Mode       GCMode
	GCPercent  int // GOGC value (default 100)
}

// DefaultGarbageCollectionConfig returns default configuration.
func DefaultGarbageCollectionConfig() GarbageCollectionConfig {
	return GarbageCollectionConfig{
		Mode:      GCModeAuto,
		GCPercent: 100,
	}
}

// newGarbageCollection creates a new garbage collection manager.
func newGarbageCollection(cfg GarbageCollectionConfig) *GarbageCollection {
	gc := &GarbageCollection{
		mode:      cfg.Mode,
		enabled:   true,
		gcPercent: cfg.GCPercent,
	}

	log.Printf("garbage_collection: initialized mode=%d gcpercent=%d", cfg.Mode, cfg.GCPercent)
	return gc
}

// Enable enables garbage collection.
func (gc *GarbageCollection) Enable() {
	gc.mu.Lock()
	defer gc.mu.Unlock()

	gc.enabled = true
	if gc.savedGCPercent > 0 {
		debug.SetGCPercent(gc.savedGCPercent)
	} else {
		debug.SetGCPercent(gc.gcPercent)
	}

	log.Printf("garbage_collection: enabled")
}

// Disable disables garbage collection (for timing-sensitive operations).
func (gc *GarbageCollection) Disable() {
	gc.mu.Lock()
	defer gc.mu.Unlock()

	gc.enabled = false
	gc.savedGCPercent = debug.SetGCPercent(-1) // Disable GC

	log.Printf("garbage_collection: disabled")
}

// SetPrintingActive sets whether printing is active.
func (gc *GarbageCollection) SetPrintingActive(active bool) {
	gc.mu.Lock()
	defer gc.mu.Unlock()

	gc.printingActive = active

	if gc.mode == GCModeDisabled {
		if active {
			// Disable GC during printing
			gc.savedGCPercent = debug.SetGCPercent(-1)
			log.Printf("garbage_collection: disabled for printing")
		} else {
			// Re-enable GC after printing
			if gc.savedGCPercent > 0 {
				debug.SetGCPercent(gc.savedGCPercent)
			} else {
				debug.SetGCPercent(gc.gcPercent)
			}
			log.Printf("garbage_collection: re-enabled after printing")
		}
	}
}

// ForceGC forces a garbage collection cycle.
func (gc *GarbageCollection) ForceGC() {
	gc.mu.Lock()
	gc.forcedGCCount++
	gc.mu.Unlock()

	goruntime.GC()
	gc.mu.Lock()
	gc.lastGCTime = time.Now()
	gc.mu.Unlock()

	log.Printf("garbage_collection: forced GC complete")
}

// GetMemStats returns current memory statistics.
func (gc *GarbageCollection) GetMemStats() goruntime.MemStats {
	var m goruntime.MemStats
	goruntime.ReadMemStats(&m)
	return m
}

// UpdateStats updates GC statistics.
func (gc *GarbageCollection) UpdateStats() {
	var m goruntime.MemStats
	goruntime.ReadMemStats(&m)

	gc.mu.Lock()
	gc.gcCount = uint64(m.NumGC)
	gc.pauseNS = m.PauseTotalNs
	gc.mu.Unlock()
}

// IsEnabled returns whether GC is enabled.
func (gc *GarbageCollection) IsEnabled() bool {
	gc.mu.Lock()
	defer gc.mu.Unlock()
	return gc.enabled
}

// GetStatus returns garbage collection status.
func (gc *GarbageCollection) GetStatus() map[string]any {
	gc.mu.Lock()
	defer gc.mu.Unlock()

	var m goruntime.MemStats
	goruntime.ReadMemStats(&m)

	return map[string]any{
		"enabled":          gc.enabled,
		"mode":             gc.mode,
		"printing_active":  gc.printingActive,
		"gc_count":         m.NumGC,
		"forced_gc_count":  gc.forcedGCCount,
		"pause_total_ms":   float64(m.PauseTotalNs) / 1e6,
		"heap_alloc_mb":    float64(m.HeapAlloc) / 1024 / 1024,
		"heap_sys_mb":      float64(m.HeapSys) / 1024 / 1024,
		"gc_percent":       gc.gcPercent,
	}
}
