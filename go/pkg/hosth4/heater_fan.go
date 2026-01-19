// Heater fan - port of klippy/extras/heater_fan.py
//
// Support fans that are enabled when a heater is on
//
// Copyright (C) 2016-2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"sync"
	"time"
)

const (
	heaterFanPinMinTime   = 0.100 // 100ms minimum time between pin changes
	heaterFanCheckPeriod  = 1.0   // Check heater every 1 second
)

// HeaterFan represents a fan that turns on when heaters are active.
type HeaterFan struct {
	rt   *runtime
	name string

	heaterNames []string // names of heaters to monitor
	heaterTemp  float64  // temperature threshold to turn on fan
	fanSpeed    float64  // speed when heaters are active (0.0 to 1.0)

	lastSpeed float64
	mu        sync.Mutex

	stopChan chan struct{}
	running  bool

	// Fan control (in a full implementation, this would be a Fan object)
	fanPin string
}

// HeaterFanConfig holds configuration for a heater fan.
type HeaterFanConfig struct {
	Name        string
	Pin         string
	HeaterNames []string // default ["extruder"]
	HeaterTemp  float64  // default 50.0
	FanSpeed    float64  // default 1.0
}

// DefaultHeaterFanConfig returns default heater fan configuration.
func DefaultHeaterFanConfig() HeaterFanConfig {
	return HeaterFanConfig{
		HeaterNames: []string{"extruder"},
		HeaterTemp:  50.0,
		FanSpeed:    1.0,
	}
}

// newHeaterFan creates a new heater fan controller.
func newHeaterFan(rt *runtime, cfg HeaterFanConfig) *HeaterFan {
	if len(cfg.HeaterNames) == 0 {
		cfg.HeaterNames = []string{"extruder"}
	}
	if cfg.FanSpeed < 0 {
		cfg.FanSpeed = 0
	} else if cfg.FanSpeed > 1 {
		cfg.FanSpeed = 1
	}

	return &HeaterFan{
		rt:          rt,
		name:        cfg.Name,
		fanPin:      cfg.Pin,
		heaterNames: cfg.HeaterNames,
		heaterTemp:  cfg.HeaterTemp,
		fanSpeed:    cfg.FanSpeed,
		stopChan:    make(chan struct{}),
	}
}

// Start starts the heater fan monitoring loop.
func (hf *HeaterFan) Start() {
	hf.mu.Lock()
	if hf.running {
		hf.mu.Unlock()
		return
	}
	hf.running = true
	hf.mu.Unlock()

	go hf.monitorLoop()
}

// Stop stops the heater fan monitoring loop.
func (hf *HeaterFan) Stop() {
	hf.mu.Lock()
	if !hf.running {
		hf.mu.Unlock()
		return
	}
	hf.running = false
	hf.mu.Unlock()

	close(hf.stopChan)
}

// monitorLoop periodically checks heater temperatures and adjusts fan speed.
func (hf *HeaterFan) monitorLoop() {
	ticker := time.NewTicker(time.Duration(heaterFanCheckPeriod * float64(time.Second)))
	defer ticker.Stop()

	for {
		select {
		case <-hf.stopChan:
			return
		case <-ticker.C:
			hf.checkAndUpdate()
		}
	}
}

// checkAndUpdate checks heater temperatures and updates fan speed.
func (hf *HeaterFan) checkAndUpdate() {
	speed := 0.0

	// Check each monitored heater
	if hf.rt != nil && hf.rt.heaterManager != nil {
		for _, heaterName := range hf.heaterNames {
			heater, err := hf.rt.heaterManager.GetHeater(heaterName)
			if err != nil || heater == nil {
				continue
			}

			currentTemp, targetTemp := heater.GetTemp(0) // eventtime=0 for immediate reading
			// Turn on fan if heater has a target set OR current temp is above threshold
			if targetTemp > 0 || currentTemp > hf.heaterTemp {
				speed = hf.fanSpeed
				break
			}
		}
	}

	hf.mu.Lock()
	if speed != hf.lastSpeed {
		hf.lastSpeed = speed
		hf.mu.Unlock()
		hf.setFanSpeed(speed)
	} else {
		hf.mu.Unlock()
	}
}

// setFanSpeed sets the fan speed.
func (hf *HeaterFan) setFanSpeed(speed float64) {
	// In a full implementation, this would control the actual fan PWM
	_ = speed
}

// GetStatus returns the heater fan status.
func (hf *HeaterFan) GetStatus() map[string]any {
	hf.mu.Lock()
	defer hf.mu.Unlock()
	return map[string]any{
		"speed": hf.lastSpeed,
	}
}

// HeaterFanManager manages multiple heater fan instances.
type HeaterFanManager struct {
	rt   *runtime
	fans map[string]*HeaterFan
	mu   sync.RWMutex
}

// newHeaterFanManager creates a new heater fan manager.
func newHeaterFanManager(rt *runtime) *HeaterFanManager {
	return &HeaterFanManager{
		rt:   rt,
		fans: make(map[string]*HeaterFan),
	}
}

// Register registers a new heater fan.
func (hfm *HeaterFanManager) Register(cfg HeaterFanConfig) (*HeaterFan, error) {
	hfm.mu.Lock()
	defer hfm.mu.Unlock()

	hf := newHeaterFan(hfm.rt, cfg)
	hfm.fans[cfg.Name] = hf
	return hf, nil
}

// Get returns a heater fan by name.
func (hfm *HeaterFanManager) Get(name string) *HeaterFan {
	hfm.mu.RLock()
	defer hfm.mu.RUnlock()
	return hfm.fans[name]
}

// StartAll starts all heater fans.
func (hfm *HeaterFanManager) StartAll() {
	hfm.mu.RLock()
	defer hfm.mu.RUnlock()
	for _, hf := range hfm.fans {
		hf.Start()
	}
}

// StopAll stops all heater fans.
func (hfm *HeaterFanManager) StopAll() {
	hfm.mu.RLock()
	defer hfm.mu.RUnlock()
	for _, hf := range hfm.fans {
		hf.Stop()
	}
}
