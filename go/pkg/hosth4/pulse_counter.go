// Pulse Counter - port of klippy/extras/pulse_counter.py
//
// Support for pulse counting (encoders, flow meters, etc.)
//
// Copyright (C) 2017-2021 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// PulseCounter counts pulses on a GPIO pin.
type PulseCounter struct {
	rt            *runtime
	name          string
	pin           string
	sampleTime    float64
	count         int64
	countPerRev   int     // Pulses per revolution (for encoders)
	lastCount     int64
	lastTime      float64
	frequency     float64 // Current frequency (pulses/sec)
	callback      func(count int64, frequency float64)
	mu            sync.Mutex
}

// PulseCounterConfig holds configuration for pulse counter.
type PulseCounterConfig struct {
	Name        string
	Pin         string
	SampleTime  float64
	CountPerRev int
}

// DefaultPulseCounterConfig returns default configuration.
func DefaultPulseCounterConfig() PulseCounterConfig {
	return PulseCounterConfig{
		SampleTime:  0.1,
		CountPerRev: 1,
	}
}

// newPulseCounter creates a new pulse counter.
func newPulseCounter(rt *runtime, cfg PulseCounterConfig) (*PulseCounter, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("pulse_counter: name is required")
	}
	if cfg.Pin == "" {
		return nil, fmt.Errorf("pulse_counter: pin is required")
	}
	if cfg.CountPerRev < 1 {
		cfg.CountPerRev = 1
	}

	pc := &PulseCounter{
		rt:          rt,
		name:        cfg.Name,
		pin:         cfg.Pin,
		sampleTime:  cfg.SampleTime,
		countPerRev: cfg.CountPerRev,
	}

	log.Printf("pulse_counter: initialized '%s' pin=%s", cfg.Name, cfg.Pin)
	return pc, nil
}

// GetName returns the counter name.
func (pc *PulseCounter) GetName() string {
	return pc.name
}

// SetCallback sets the measurement callback.
func (pc *PulseCounter) SetCallback(cb func(count int64, frequency float64)) {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pc.callback = cb
}

// UpdateCount updates the pulse count.
func (pc *PulseCounter) UpdateCount(eventTime float64, count int64) {
	pc.mu.Lock()
	defer pc.mu.Unlock()

	// Calculate frequency
	dt := eventTime - pc.lastTime
	if dt > 0 {
		dCount := count - pc.lastCount
		pc.frequency = float64(dCount) / dt
	}

	pc.count = count
	pc.lastCount = count
	pc.lastTime = eventTime

	if pc.callback != nil {
		pc.callback(count, pc.frequency)
	}
}

// GetCount returns the current pulse count.
func (pc *PulseCounter) GetCount() int64 {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	return pc.count
}

// GetFrequency returns the current frequency (pulses/sec).
func (pc *PulseCounter) GetFrequency() float64 {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	return pc.frequency
}

// GetRPM returns the current RPM (if countPerRev is set).
func (pc *PulseCounter) GetRPM() float64 {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	if pc.countPerRev == 0 {
		return 0
	}
	return pc.frequency / float64(pc.countPerRev) * 60.0
}

// Reset resets the counter.
func (pc *PulseCounter) Reset() {
	pc.mu.Lock()
	defer pc.mu.Unlock()
	pc.count = 0
	pc.lastCount = 0
	pc.frequency = 0
}

// GetStatus returns the counter status.
func (pc *PulseCounter) GetStatus() map[string]any {
	pc.mu.Lock()
	defer pc.mu.Unlock()

	rpm := 0.0
	if pc.countPerRev > 0 {
		rpm = pc.frequency / float64(pc.countPerRev) * 60.0
	}

	return map[string]any{
		"count":     pc.count,
		"frequency": pc.frequency,
		"rpm":       rpm,
	}
}
