// Controller fan - port of klippy/extras/controller_fan.py
//
// Support a fan for cooling the MCU whenever a stepper or heater is on
//
// Copyright (C) 2019 Nils Friedchen <nils.friedchen@googlemail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"sync"
	"time"
)

const (
	controllerFanPinMinTime  = 0.100 // 100ms minimum time between pin changes
	controllerFanCheckPeriod = 1.0   // Check every 1 second
)

// ControllerFan represents a fan that cools the MCU when steppers or heaters are active.
type ControllerFan struct {
	rt   *runtime
	name string

	stepperNames []string // names of steppers to monitor (nil = all)
	heaterNames  []string // names of heaters to monitor
	fanSpeed     float64  // speed when active (0.0 to 1.0)
	idleSpeed    float64  // speed when idle (0.0 to 1.0)
	idleTimeout  int      // seconds before switching to idle speed

	lastOn    int     // seconds since last activity
	lastSpeed float64
	mu        sync.Mutex

	stopChan chan struct{}
	running  bool

	// Fan control (in a full implementation, this would be a Fan object)
	fanPin string
}

// ControllerFanConfig holds configuration for a controller fan.
type ControllerFanConfig struct {
	Name         string
	Pin          string
	StepperNames []string // nil = all steppers
	HeaterNames  []string // default ["extruder"]
	FanSpeed     float64  // default 1.0
	IdleSpeed    float64  // default same as fan_speed
	IdleTimeout  int      // default 30 seconds
}

// DefaultControllerFanConfig returns default controller fan configuration.
func DefaultControllerFanConfig() ControllerFanConfig {
	return ControllerFanConfig{
		HeaterNames: []string{"extruder"},
		FanSpeed:    1.0,
		IdleSpeed:   1.0, // Will be set to FanSpeed if not specified
		IdleTimeout: 30,
	}
}

// newControllerFan creates a new controller fan.
func newControllerFan(rt *runtime, cfg ControllerFanConfig) *ControllerFan {
	if len(cfg.HeaterNames) == 0 {
		cfg.HeaterNames = []string{"extruder"}
	}
	if cfg.FanSpeed < 0 {
		cfg.FanSpeed = 0
	} else if cfg.FanSpeed > 1 {
		cfg.FanSpeed = 1
	}
	if cfg.IdleSpeed < 0 {
		cfg.IdleSpeed = 0
	} else if cfg.IdleSpeed > 1 {
		cfg.IdleSpeed = 1
	}
	if cfg.IdleTimeout < 0 {
		cfg.IdleTimeout = 0
	}

	return &ControllerFan{
		rt:           rt,
		name:         cfg.Name,
		fanPin:       cfg.Pin,
		stepperNames: cfg.StepperNames,
		heaterNames:  cfg.HeaterNames,
		fanSpeed:     cfg.FanSpeed,
		idleSpeed:    cfg.IdleSpeed,
		idleTimeout:  cfg.IdleTimeout,
		lastOn:       cfg.IdleTimeout, // Start as if just coming out of idle
		stopChan:     make(chan struct{}),
	}
}

// Start starts the controller fan monitoring loop.
func (cf *ControllerFan) Start() {
	cf.mu.Lock()
	if cf.running {
		cf.mu.Unlock()
		return
	}
	cf.running = true
	cf.mu.Unlock()

	go cf.monitorLoop()
}

// Stop stops the controller fan monitoring loop.
func (cf *ControllerFan) Stop() {
	cf.mu.Lock()
	if !cf.running {
		cf.mu.Unlock()
		return
	}
	cf.running = false
	cf.mu.Unlock()

	close(cf.stopChan)
}

// monitorLoop periodically checks activity and adjusts fan speed.
func (cf *ControllerFan) monitorLoop() {
	ticker := time.NewTicker(time.Duration(controllerFanCheckPeriod * float64(time.Second)))
	defer ticker.Stop()

	for {
		select {
		case <-cf.stopChan:
			return
		case <-ticker.C:
			cf.checkAndUpdate()
		}
	}
}

// checkAndUpdate checks stepper/heater activity and updates fan speed.
func (cf *ControllerFan) checkAndUpdate() {
	speed := 0.0
	active := false

	// Check steppers
	// Note: In a full implementation, this would check each stepper's enable state
	// via stepperEnable.lines[name].isEnabled() or similar.
	// For now, we check heaters only as a proxy for activity.
	if cf.rt != nil && cf.rt.stepperEnable != nil {
		// Check if any tracked steppers are enabled
		for name, et := range cf.rt.stepperEnable.lines {
			if cf.stepperNames != nil {
				// Only check specific steppers if configured
				found := false
				for _, sn := range cf.stepperNames {
					if sn == name {
						found = true
						break
					}
				}
				if !found {
					continue
				}
			}
			if et.isEnabled {
				active = true
				break
			}
		}
	}

	// Check heaters
	if !active && cf.rt != nil && cf.rt.heaterManager != nil {
		for _, heaterName := range cf.heaterNames {
			heater, err := cf.rt.heaterManager.GetHeater(heaterName)
			if err != nil || heater == nil {
				continue
			}
			_, targetTemp := heater.GetTemp(0)
			if targetTemp > 0 {
				active = true
				break
			}
		}
	}

	cf.mu.Lock()
	if active {
		cf.lastOn = 0
		speed = cf.fanSpeed
	} else if cf.lastOn < cf.idleTimeout {
		speed = cf.idleSpeed
		cf.lastOn++
	}

	if speed != cf.lastSpeed {
		cf.lastSpeed = speed
		cf.mu.Unlock()
		cf.setFanSpeed(speed)
	} else {
		cf.mu.Unlock()
	}
}

// setFanSpeed sets the fan speed.
func (cf *ControllerFan) setFanSpeed(speed float64) {
	// In a full implementation, this would control the actual fan PWM
	_ = speed
}

// GetStatus returns the controller fan status.
func (cf *ControllerFan) GetStatus() map[string]any {
	cf.mu.Lock()
	defer cf.mu.Unlock()
	return map[string]any{
		"speed": cf.lastSpeed,
	}
}

// ControllerFanManager manages multiple controller fan instances.
type ControllerFanManager struct {
	rt   *runtime
	fans map[string]*ControllerFan
	mu   sync.RWMutex
}

// newControllerFanManager creates a new controller fan manager.
func newControllerFanManager(rt *runtime) *ControllerFanManager {
	return &ControllerFanManager{
		rt:   rt,
		fans: make(map[string]*ControllerFan),
	}
}

// Register registers a new controller fan.
func (cfm *ControllerFanManager) Register(cfg ControllerFanConfig) (*ControllerFan, error) {
	cfm.mu.Lock()
	defer cfm.mu.Unlock()

	cf := newControllerFan(cfm.rt, cfg)
	cfm.fans[cfg.Name] = cf
	return cf, nil
}

// Get returns a controller fan by name.
func (cfm *ControllerFanManager) Get(name string) *ControllerFan {
	cfm.mu.RLock()
	defer cfm.mu.RUnlock()
	return cfm.fans[name]
}

// StartAll starts all controller fans.
func (cfm *ControllerFanManager) StartAll() {
	cfm.mu.RLock()
	defer cfm.mu.RUnlock()
	for _, cf := range cfm.fans {
		cf.Start()
	}
}

// StopAll stops all controller fans.
func (cfm *ControllerFanManager) StopAll() {
	cfm.mu.RLock()
	defer cfm.mu.RUnlock()
	for _, cf := range cfm.fans {
		cf.Stop()
	}
}
