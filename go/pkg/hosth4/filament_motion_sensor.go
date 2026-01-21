// Filament Motion Sensor - port of klippy/extras/filament_motion_sensor.py
//
// Support for filament motion detection using an encoder
//
// Copyright (C) 2021 Joshua Wherrett <thejoshw.code@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
	"time"
)

const (
	// checkRunoutTimeout is the interval between runout checks (250ms)
	checkRunoutTimeout = 0.250
)

// EncoderSensor implements a filament motion sensor using an encoder.
// It detects filament runout by monitoring whether the filament is moving
// while the extruder is commanded to move.
type EncoderSensor struct {
	rt *runtime

	name            string
	switchPin       string
	extruderName    string
	detectionLength float64 // Length of filament movement to detect

	runoutHelper *RunoutHelper

	// Extruder reference (set on ready)
	extruder Extruder

	// Internal state
	filamentRunoutPos float64 // Position where runout would be detected
	lastEncoderEvent  time.Time

	// Timer for periodic position checking
	checkTimer    *time.Timer
	checkInterval time.Duration
	checking      bool

	mu sync.Mutex
}

// Extruder interface for position queries
type Extruder interface {
	FindPastPosition(printTime float64) float64
	GetName() string
}

// EncoderSensorConfig holds configuration for the encoder sensor.
type EncoderSensorConfig struct {
	Name            string
	SwitchPin       string   // Pin connected to encoder
	ExtruderName    string   // Name of the extruder to monitor
	DetectionLength float64  // Detection length in mm (default 7.0)
	PauseOnRunout   bool
	RunoutGcode     []string
	InsertGcode     []string
	PauseDelay      float64
	EventDelay      float64
}

// DefaultEncoderSensorConfig returns the default encoder sensor configuration.
func DefaultEncoderSensorConfig() EncoderSensorConfig {
	return EncoderSensorConfig{
		ExtruderName:    "extruder",
		DetectionLength: 7.0,
		PauseOnRunout:   true,
		PauseDelay:      0.5,
		EventDelay:      3.0,
	}
}

// newEncoderSensor creates a new encoder-based filament motion sensor.
func newEncoderSensor(rt *runtime, cfg EncoderSensorConfig) (*EncoderSensor, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("filament_motion_sensor: name is required")
	}
	if cfg.SwitchPin == "" {
		return nil, fmt.Errorf("filament_motion_sensor %s: switch_pin is required", cfg.Name)
	}
	if cfg.DetectionLength <= 0 {
		cfg.DetectionLength = 7.0
	}

	// Create runout helper
	runoutCfg := RunoutHelperConfig{
		Name:          cfg.Name,
		PauseOnRunout: cfg.PauseOnRunout,
		RunoutGcode:   cfg.RunoutGcode,
		InsertGcode:   cfg.InsertGcode,
		PauseDelay:    cfg.PauseDelay,
		EventDelay:    cfg.EventDelay,
	}

	es := &EncoderSensor{
		rt:              rt,
		name:            cfg.Name,
		switchPin:       cfg.SwitchPin,
		extruderName:    cfg.ExtruderName,
		detectionLength: cfg.DetectionLength,
		runoutHelper:    newRunoutHelper(rt, runoutCfg),
		checkInterval:   time.Duration(checkRunoutTimeout * float64(time.Second)),
	}

	log.Printf("filament_motion_sensor: initialized '%s' pin=%s extruder=%s detection_length=%.1f",
		cfg.Name, cfg.SwitchPin, cfg.ExtruderName, cfg.DetectionLength)

	return es, nil
}

// GetName returns the sensor name.
func (es *EncoderSensor) GetName() string {
	return es.name
}

// HandleReady is called when the printer is ready.
func (es *EncoderSensor) HandleReady() {
	es.mu.Lock()
	defer es.mu.Unlock()

	// Look up extruder by name
	if es.rt != nil {
		switch es.extruderName {
		case "extruder":
			if es.rt.extruder != nil {
				es.extruder = es.rt.extruder
			}
		case "extruder1":
			if es.rt.extruder1 != nil {
				es.extruder = es.rt.extruder1
			}
		}
	}

	// Initialize runout position
	es.updateFilamentRunoutPos()

	// Initialize runout helper
	es.runoutHelper.HandleReady()

	log.Printf("filament_motion_sensor %s: ready", es.name)
}

// HandlePrinting is called when printing starts.
func (es *EncoderSensor) HandlePrinting(printTime float64) {
	es.mu.Lock()
	defer es.mu.Unlock()

	// Start periodic position checking
	es.startChecking()
}

// HandleNotPrinting is called when printing stops (idle or ready).
func (es *EncoderSensor) HandleNotPrinting(printTime float64) {
	es.mu.Lock()
	defer es.mu.Unlock()

	// Stop periodic position checking
	es.stopChecking()
}

// startChecking starts the periodic position checking timer.
func (es *EncoderSensor) startChecking() {
	if es.checking {
		return
	}
	es.checking = true
	es.checkTimer = time.AfterFunc(es.checkInterval, es.checkPosition)
}

// stopChecking stops the periodic position checking timer.
func (es *EncoderSensor) stopChecking() {
	es.checking = false
	if es.checkTimer != nil {
		es.checkTimer.Stop()
		es.checkTimer = nil
	}
}

// checkPosition periodically checks the extruder position for runout detection.
func (es *EncoderSensor) checkPosition() {
	es.mu.Lock()
	defer es.mu.Unlock()

	if !es.checking {
		return
	}

	// Get current extruder position
	extruderPos := es.getExtruderPos()

	// Check for filament runout
	// If extruder has moved beyond the runout position without encoder events,
	// filament is not present
	filamentPresent := extruderPos < es.filamentRunoutPos
	es.runoutHelper.NoteFilamentPresent(filamentPresent)

	// Schedule next check
	es.checkTimer = time.AfterFunc(es.checkInterval, es.checkPosition)
}

// updateFilamentRunoutPos updates the filament runout position based on current extruder position.
func (es *EncoderSensor) updateFilamentRunoutPos() {
	extruderPos := es.getExtruderPos()
	es.filamentRunoutPos = extruderPos + es.detectionLength
}

// getExtruderPos returns the current extruder position.
func (es *EncoderSensor) getExtruderPos() float64 {
	if es.extruder == nil {
		return 0
	}

	// Get print time from MCU
	printTime := 0.0
	if es.rt != nil && es.rt.mcu != nil {
		// Use estimated print time
		printTime = es.rt.mcu.EstimatedPrintTime(float64(time.Now().UnixNano()) / 1e9)
	}

	return es.extruder.FindPastPosition(printTime)
}

// EncoderEvent handles encoder events (button state changes).
// This is called when the encoder detects filament movement.
func (es *EncoderSensor) EncoderEvent(eventtime float64, state int) {
	es.mu.Lock()
	defer es.mu.Unlock()

	if es.extruder == nil {
		return
	}

	// Update the runout position on every encoder event
	es.updateFilamentRunoutPos()
	es.lastEncoderEvent = time.Now()

	// Filament is always assumed to be present on an encoder event
	es.runoutHelper.NoteFilamentPresent(true)
}

// GetStatus returns the sensor status.
func (es *EncoderSensor) GetStatus() map[string]any {
	es.mu.Lock()
	defer es.mu.Unlock()

	status := es.runoutHelper.GetStatus()
	status["detection_length"] = es.detectionLength
	status["filament_runout_pos"] = es.filamentRunoutPos
	return status
}

// SetGcodeRunner sets the function to run gcode commands.
func (es *EncoderSensor) SetGcodeRunner(runner func([]string) error) {
	es.runoutHelper.SetGcodeRunner(runner)
}

// SetEnabled enables or disables the sensor.
func (es *EncoderSensor) SetEnabled(enabled bool) {
	es.runoutHelper.SetEnabled(enabled)
}

// IsEnabled returns whether the sensor is enabled.
func (es *EncoderSensor) IsEnabled() bool {
	return es.runoutHelper.IsEnabled()
}

// cmdQueryFilamentSensor handles the QUERY_FILAMENT_SENSOR command.
func (es *EncoderSensor) cmdQueryFilamentSensor() string {
	status := es.GetStatus()
	detected := status["filament_detected"].(bool)
	if detected {
		return fmt.Sprintf("Filament Sensor %s: filament detected", es.name)
	}
	return fmt.Sprintf("Filament Sensor %s: filament not detected", es.name)
}

// FilamentMotionSensorManager manages multiple filament motion sensors.
type FilamentMotionSensorManager struct {
	rt      *runtime
	sensors map[string]*EncoderSensor
	mu      sync.RWMutex
}

// newFilamentMotionSensorManager creates a new filament motion sensor manager.
func newFilamentMotionSensorManager(rt *runtime) *FilamentMotionSensorManager {
	return &FilamentMotionSensorManager{
		rt:      rt,
		sensors: make(map[string]*EncoderSensor),
	}
}

// Register registers a new filament motion sensor.
func (fm *FilamentMotionSensorManager) Register(cfg EncoderSensorConfig) (*EncoderSensor, error) {
	fm.mu.Lock()
	defer fm.mu.Unlock()

	if _, exists := fm.sensors[cfg.Name]; exists {
		return nil, fmt.Errorf("filament_motion_sensor %s already registered", cfg.Name)
	}

	sensor, err := newEncoderSensor(fm.rt, cfg)
	if err != nil {
		return nil, err
	}

	fm.sensors[cfg.Name] = sensor
	return sensor, nil
}

// Get returns a filament motion sensor by name.
func (fm *FilamentMotionSensorManager) Get(name string) *EncoderSensor {
	fm.mu.RLock()
	defer fm.mu.RUnlock()
	return fm.sensors[name]
}

// List returns all registered filament motion sensors.
func (fm *FilamentMotionSensorManager) List() []*EncoderSensor {
	fm.mu.RLock()
	defer fm.mu.RUnlock()
	result := make([]*EncoderSensor, 0, len(fm.sensors))
	for _, sensor := range fm.sensors {
		result = append(result, sensor)
	}
	return result
}

// HandleReady is called when the printer is ready.
func (fm *FilamentMotionSensorManager) HandleReady() {
	fm.mu.RLock()
	defer fm.mu.RUnlock()
	for _, sensor := range fm.sensors {
		sensor.HandleReady()
	}
}

// HandlePrinting is called when printing starts.
func (fm *FilamentMotionSensorManager) HandlePrinting(printTime float64) {
	fm.mu.RLock()
	defer fm.mu.RUnlock()
	for _, sensor := range fm.sensors {
		sensor.HandlePrinting(printTime)
	}
}

// HandleNotPrinting is called when printing stops.
func (fm *FilamentMotionSensorManager) HandleNotPrinting(printTime float64) {
	fm.mu.RLock()
	defer fm.mu.RUnlock()
	for _, sensor := range fm.sensors {
		sensor.HandleNotPrinting(printTime)
	}
}

// GetStatus returns status of all filament motion sensors.
func (fm *FilamentMotionSensorManager) GetStatus() map[string]any {
	fm.mu.RLock()
	defer fm.mu.RUnlock()

	status := make(map[string]any)
	for name, sensor := range fm.sensors {
		status[name] = sensor.GetStatus()
	}
	return status
}
