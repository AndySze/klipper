// Heater Bed - port of klippy/extras/heater_bed.py
//
// Support for a heated bed
//
// Copyright (C) 2018-2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
	"time"

	"klipper-go-migration/pkg/temperature"
)

// HeaterBed represents a heated bed.
type HeaterBed struct {
	rt     *runtime
	heater *temperature.Heater

	mu sync.Mutex
}

// HeaterBedConfig holds configuration for the heated bed.
type HeaterBedConfig struct {
	HeaterPin    string
	SensorType   string
	SensorPin    string
	Control      string  // "pid" or "watermark"
	MinTemp      float64
	MaxTemp      float64
	PidKp        float64
	PidKi        float64
	PidKd        float64
	MaxDelta     float64 // For watermark control
	MaxPower     float64
	SmoothTime   float64
	PWMCycleTime float64
}

// DefaultHeaterBedConfig returns default heater bed configuration.
func DefaultHeaterBedConfig() HeaterBedConfig {
	return HeaterBedConfig{
		Control:      "pid",
		MinTemp:      0,
		MaxTemp:      130,
		MaxDelta:     2.0,
		MaxPower:     1.0,
		SmoothTime:   1.0,
		PWMCycleTime: 0.1,
	}
}

// newHeaterBed creates a new heated bed controller.
func newHeaterBed(rt *runtime, cfg HeaterBedConfig, heater *temperature.Heater) *HeaterBed {
	hb := &HeaterBed{
		rt:     rt,
		heater: heater,
	}

	log.Printf("heater_bed: initialized with control=%s, max_temp=%.1f", cfg.Control, cfg.MaxTemp)
	return hb
}

// GetHeater returns the underlying heater.
func (hb *HeaterBed) GetHeater() *temperature.Heater {
	return hb.heater
}

// cmdM140 handles M140 (set bed temperature, no wait).
func (hb *HeaterBed) cmdM140(temp float64) error {
	hb.mu.Lock()
	defer hb.mu.Unlock()

	if hb.heater == nil {
		return fmt.Errorf("heater_bed not initialized")
	}

	if err := hb.heater.SetTemp(temp); err != nil {
		return err
	}

	log.Printf("heater_bed: M140 set target temperature to %.1f", temp)
	return nil
}

// cmdM190 handles M190 (set bed temperature and wait).
func (hb *HeaterBed) cmdM190(temp float64) error {
	if err := hb.cmdM140(temp); err != nil {
		return err
	}

	// Wait for temperature if target > 0
	if temp > 0 {
		return hb.waitForTemperature()
	}
	return nil
}

// waitForTemperature waits for the bed to reach target temperature.
func (hb *HeaterBed) waitForTemperature() error {
	if hb.heater == nil {
		return fmt.Errorf("heater_bed not initialized")
	}

	// Wait for heater to reach target
	for {
		eventtime := float64(time.Now().UnixNano()) / 1e9
		if !hb.heater.CheckBusy(eventtime) {
			return nil
		}
		time.Sleep(1 * time.Second)
	}
}

// GetStatus returns the heater bed status.
func (hb *HeaterBed) GetStatus(eventtime float64) map[string]any {
	if hb.heater == nil {
		return map[string]any{
			"temperature": 0.0,
			"target":      0.0,
			"power":       0.0,
		}
	}
	return hb.heater.GetStatus(eventtime)
}

// Stats returns statistics for the heater bed.
func (hb *HeaterBed) Stats(eventtime float64) (bool, string) {
	if hb.heater == nil {
		return false, "heater_bed: not initialized"
	}
	return hb.heater.Stats(eventtime)
}

// HeaterGeneric represents a generic heater.
// This is a simple wrapper that just sets up a heater through the heaters manager.
type HeaterGeneric struct {
	rt     *runtime
	name   string
	heater *temperature.Heater

	mu sync.Mutex
}

// HeaterGenericConfig holds configuration for a generic heater.
type HeaterGenericConfig struct {
	Name         string
	HeaterPin    string
	SensorType   string
	SensorPin    string
	Control      string
	MinTemp      float64
	MaxTemp      float64
	PidKp        float64
	PidKi        float64
	PidKd        float64
	MaxDelta     float64
	MaxPower     float64
	SmoothTime   float64
	PWMCycleTime float64
	GcodeID      string // Optional G-code ID for M105 reporting
}

// DefaultHeaterGenericConfig returns default generic heater configuration.
func DefaultHeaterGenericConfig() HeaterGenericConfig {
	return HeaterGenericConfig{
		Control:      "pid",
		MinTemp:      0,
		MaxTemp:      250,
		MaxDelta:     2.0,
		MaxPower:     1.0,
		SmoothTime:   1.0,
		PWMCycleTime: 0.1,
	}
}

// newHeaterGeneric creates a new generic heater.
func newHeaterGeneric(rt *runtime, cfg HeaterGenericConfig, heater *temperature.Heater) *HeaterGeneric {
	hg := &HeaterGeneric{
		rt:     rt,
		name:   cfg.Name,
		heater: heater,
	}

	log.Printf("heater_generic: initialized '%s' with control=%s, max_temp=%.1f", cfg.Name, cfg.Control, cfg.MaxTemp)
	return hg
}

// GetName returns the heater name.
func (hg *HeaterGeneric) GetName() string {
	return hg.name
}

// GetHeater returns the underlying heater.
func (hg *HeaterGeneric) GetHeater() *temperature.Heater {
	return hg.heater
}

// SetTemperature sets the target temperature.
func (hg *HeaterGeneric) SetTemperature(temp float64) error {
	hg.mu.Lock()
	defer hg.mu.Unlock()

	if hg.heater == nil {
		return fmt.Errorf("heater_generic '%s' not initialized", hg.name)
	}
	return hg.heater.SetTemp(temp)
}

// GetStatus returns the heater status.
func (hg *HeaterGeneric) GetStatus(eventtime float64) map[string]any {
	if hg.heater == nil {
		return map[string]any{
			"temperature": 0.0,
			"target":      0.0,
			"power":       0.0,
		}
	}
	return hg.heater.GetStatus(eventtime)
}

// Stats returns statistics for the heater.
func (hg *HeaterGeneric) Stats(eventtime float64) (bool, string) {
	if hg.heater == nil {
		return false, fmt.Sprintf("heater_generic %s: not initialized", hg.name)
	}
	return hg.heater.Stats(eventtime)
}

// HeaterGenericManager manages multiple generic heaters.
type HeaterGenericManager struct {
	rt      *runtime
	heaters map[string]*HeaterGeneric
	mu      sync.RWMutex
}

// newHeaterGenericManager creates a new generic heater manager.
func newHeaterGenericManager(rt *runtime) *HeaterGenericManager {
	return &HeaterGenericManager{
		rt:      rt,
		heaters: make(map[string]*HeaterGeneric),
	}
}

// Register registers a new generic heater.
func (hgm *HeaterGenericManager) Register(cfg HeaterGenericConfig, heater *temperature.Heater) (*HeaterGeneric, error) {
	hgm.mu.Lock()
	defer hgm.mu.Unlock()

	if _, exists := hgm.heaters[cfg.Name]; exists {
		return nil, fmt.Errorf("heater_generic '%s' already registered", cfg.Name)
	}

	hg := newHeaterGeneric(hgm.rt, cfg, heater)
	hgm.heaters[cfg.Name] = hg
	return hg, nil
}

// Get returns a generic heater by name.
func (hgm *HeaterGenericManager) Get(name string) *HeaterGeneric {
	hgm.mu.RLock()
	defer hgm.mu.RUnlock()
	return hgm.heaters[name]
}

// GetAll returns all generic heater names.
func (hgm *HeaterGenericManager) GetAll() []string {
	hgm.mu.RLock()
	defer hgm.mu.RUnlock()

	names := make([]string, 0, len(hgm.heaters))
	for name := range hgm.heaters {
		names = append(names, name)
	}
	return names
}
