// Filament switch sensor - port of klippy/extras/filament_switch_sensor.py
//
// Generic Filament Sensor Module
//
// Copyright (C) 2019 Eric Callahan <arksine.code@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"strconv"
	"sync"
	"time"
)

// RunoutHelper provides filament runout detection and handling.
type RunoutHelper struct {
	rt   *runtime
	name string

	runoutPause     bool     // pause on runout
	runoutGcode     []string // gcode to run on runout
	insertGcode     []string // gcode to run on insert
	pauseDelay      float64  // delay after pause before running gcode
	eventDelay      float64  // delay between events

	minEventTime    time.Time // minimum time before next event
	filamentPresent bool      // current filament state
	sensorEnabled   bool      // whether sensor is enabled

	mu sync.Mutex

	// GCode runner function
	gcodeRunner func([]string) error
}

// RunoutHelperConfig holds configuration for runout helper.
type RunoutHelperConfig struct {
	Name        string
	PauseOnRunout bool
	RunoutGcode []string
	InsertGcode []string
	PauseDelay  float64 // seconds
	EventDelay  float64 // seconds
}

// DefaultRunoutHelperConfig returns default runout helper configuration.
func DefaultRunoutHelperConfig() RunoutHelperConfig {
	return RunoutHelperConfig{
		PauseOnRunout: true,
		PauseDelay:    0.5,
		EventDelay:    3.0,
	}
}

// newRunoutHelper creates a new runout helper.
func newRunoutHelper(rt *runtime, cfg RunoutHelperConfig) *RunoutHelper {
	return &RunoutHelper{
		rt:            rt,
		name:          cfg.Name,
		runoutPause:   cfg.PauseOnRunout,
		runoutGcode:   cfg.RunoutGcode,
		insertGcode:   cfg.InsertGcode,
		pauseDelay:    cfg.PauseDelay,
		eventDelay:    cfg.EventDelay,
		minEventTime:  time.Now().Add(2 * time.Second), // Initial 2 second delay
		sensorEnabled: true,
	}
}

// SetGcodeRunner sets the function to run gcode commands.
func (rh *RunoutHelper) SetGcodeRunner(runner func([]string) error) {
	rh.gcodeRunner = runner
}

// HandleReady is called when the printer is ready.
func (rh *RunoutHelper) HandleReady() {
	rh.mu.Lock()
	defer rh.mu.Unlock()
	rh.minEventTime = time.Now().Add(2 * time.Second)
}

// runoutEventHandler handles a runout event.
func (rh *RunoutHelper) runoutEventHandler() {
	rh.mu.Lock()
	pauseDelay := rh.pauseDelay
	runoutPause := rh.runoutPause
	runoutGcode := rh.runoutGcode
	runner := rh.gcodeRunner
	rh.mu.Unlock()

	// Build gcode commands
	var commands []string

	if runoutPause {
		// Send pause command
		if rh.rt != nil && rh.rt.pauseResume != nil {
			rh.rt.pauseResume.sendPauseCommand()
		}
		commands = append(commands, "PAUSE")
		time.Sleep(time.Duration(pauseDelay * float64(time.Second)))
	}

	// Add runout gcode
	commands = append(commands, runoutGcode...)
	commands = append(commands, "M400")

	// Run gcode
	if runner != nil && len(commands) > 0 {
		if err := runner(commands); err != nil {
			log.Printf("filament_sensor %s: runout script error: %v", rh.name, err)
		}
	}

	rh.mu.Lock()
	rh.minEventTime = time.Now().Add(time.Duration(rh.eventDelay * float64(time.Second)))
	rh.mu.Unlock()
}

// insertEventHandler handles a filament insert event.
func (rh *RunoutHelper) insertEventHandler() {
	rh.mu.Lock()
	insertGcode := rh.insertGcode
	runner := rh.gcodeRunner
	rh.mu.Unlock()

	// Build gcode commands
	commands := append([]string{}, insertGcode...)
	commands = append(commands, "M400")

	// Run gcode
	if runner != nil && len(commands) > 0 {
		if err := runner(commands); err != nil {
			log.Printf("filament_sensor %s: insert script error: %v", rh.name, err)
		}
	}

	rh.mu.Lock()
	rh.minEventTime = time.Now().Add(time.Duration(rh.eventDelay * float64(time.Second)))
	rh.mu.Unlock()
}

// NoteFilamentPresent handles filament state changes.
func (rh *RunoutHelper) NoteFilamentPresent(isFilamentPresent bool) {
	rh.mu.Lock()
	if isFilamentPresent == rh.filamentPresent {
		rh.mu.Unlock()
		return
	}
	rh.filamentPresent = isFilamentPresent

	now := time.Now()
	if now.Before(rh.minEventTime) || !rh.sensorEnabled {
		// Don't process during initialization, event delay, or when disabled
		rh.mu.Unlock()
		return
	}

	// Determine if printing
	// Note: In a full implementation, this would check idleTimeout.GetStatus()
	// For now, we use the sdcard print state as a proxy
	isPrinting := false
	if rh.rt != nil && rh.rt.sdcard != nil {
		status := rh.rt.sdcard.GetStatus()
		if progress, ok := status["progress"].(float64); ok && progress > 0 {
			isPrinting = true
		}
	}

	// Handle state change
	if isFilamentPresent {
		// Insert detected
		if !isPrinting && len(rh.insertGcode) > 0 {
			rh.minEventTime = time.Unix(1<<62, 0) // Never (effectively)
			rh.mu.Unlock()
			log.Printf("Filament Sensor %s: insert event detected", rh.name)
			go rh.insertEventHandler()
			return
		}
	} else {
		// Runout detected
		if isPrinting && len(rh.runoutGcode) > 0 {
			rh.minEventTime = time.Unix(1<<62, 0) // Never
			rh.mu.Unlock()
			log.Printf("Filament Sensor %s: runout event detected", rh.name)
			go rh.runoutEventHandler()
			return
		}
	}
	rh.mu.Unlock()
}

// GetStatus returns the runout helper status.
func (rh *RunoutHelper) GetStatus() map[string]any {
	rh.mu.Lock()
	defer rh.mu.Unlock()
	return map[string]any{
		"filament_detected": rh.filamentPresent,
		"enabled":           rh.sensorEnabled,
	}
}

// SetEnabled enables or disables the sensor.
func (rh *RunoutHelper) SetEnabled(enabled bool) {
	rh.mu.Lock()
	defer rh.mu.Unlock()
	rh.sensorEnabled = enabled
}

// IsEnabled returns whether the sensor is enabled.
func (rh *RunoutHelper) IsEnabled() bool {
	rh.mu.Lock()
	defer rh.mu.Unlock()
	return rh.sensorEnabled
}

// FilamentSwitchSensor represents a filament switch sensor.
type FilamentSwitchSensor struct {
	rt           *runtime
	name         string
	runoutHelper *RunoutHelper
}

// FilamentSwitchSensorConfig holds configuration for filament switch sensor.
type FilamentSwitchSensorConfig struct {
	Name          string
	SwitchPin     string // pin specification
	PauseOnRunout bool
	RunoutGcode   []string
	InsertGcode   []string
	PauseDelay    float64
	EventDelay    float64
}

// DefaultFilamentSwitchSensorConfig returns default configuration.
func DefaultFilamentSwitchSensorConfig() FilamentSwitchSensorConfig {
	return FilamentSwitchSensorConfig{
		PauseOnRunout: true,
		PauseDelay:    0.5,
		EventDelay:    3.0,
	}
}

// newFilamentSwitchSensor creates a new filament switch sensor.
func newFilamentSwitchSensor(rt *runtime, cfg FilamentSwitchSensorConfig) *FilamentSwitchSensor {
	runoutCfg := RunoutHelperConfig{
		Name:          cfg.Name,
		PauseOnRunout: cfg.PauseOnRunout,
		RunoutGcode:   cfg.RunoutGcode,
		InsertGcode:   cfg.InsertGcode,
		PauseDelay:    cfg.PauseDelay,
		EventDelay:    cfg.EventDelay,
	}

	return &FilamentSwitchSensor{
		rt:           rt,
		name:         cfg.Name,
		runoutHelper: newRunoutHelper(rt, runoutCfg),
	}
}

// HandleButtonState handles button state changes from the switch pin.
func (fs *FilamentSwitchSensor) HandleButtonState(state bool) {
	fs.runoutHelper.NoteFilamentPresent(state)
}

// GetStatus returns the sensor status.
func (fs *FilamentSwitchSensor) GetStatus() map[string]any {
	return fs.runoutHelper.GetStatus()
}

// SetGcodeRunner sets the function to run gcode commands.
func (fs *FilamentSwitchSensor) SetGcodeRunner(runner func([]string) error) {
	fs.runoutHelper.SetGcodeRunner(runner)
}

// cmdQueryFilamentSensor handles the QUERY_FILAMENT_SENSOR command.
func (fs *FilamentSwitchSensor) cmdQueryFilamentSensor() string {
	status := fs.GetStatus()
	detected := status["filament_detected"].(bool)
	if detected {
		return fmt.Sprintf("Filament Sensor %s: filament detected", fs.name)
	}
	return fmt.Sprintf("Filament Sensor %s: filament not detected", fs.name)
}

// cmdSetFilamentSensor handles the SET_FILAMENT_SENSOR command.
func (fs *FilamentSwitchSensor) cmdSetFilamentSensor(args map[string]string) error {
	enableStr, ok := args["ENABLE"]
	if !ok {
		return fmt.Errorf("missing ENABLE parameter")
	}
	enable, err := strconv.Atoi(enableStr)
	if err != nil {
		return fmt.Errorf("invalid ENABLE value: %w", err)
	}
	fs.runoutHelper.SetEnabled(enable != 0)
	return nil
}

// FilamentSensorManager manages multiple filament sensors.
type FilamentSensorManager struct {
	rt      *runtime
	sensors map[string]*FilamentSwitchSensor
	mu      sync.RWMutex
}

// newFilamentSensorManager creates a new filament sensor manager.
func newFilamentSensorManager(rt *runtime) *FilamentSensorManager {
	return &FilamentSensorManager{
		rt:      rt,
		sensors: make(map[string]*FilamentSwitchSensor),
	}
}

// Register registers a new filament switch sensor.
func (fsm *FilamentSensorManager) Register(cfg FilamentSwitchSensorConfig) (*FilamentSwitchSensor, error) {
	fsm.mu.Lock()
	defer fsm.mu.Unlock()

	if _, exists := fsm.sensors[cfg.Name]; exists {
		return nil, fmt.Errorf("filament_switch_sensor %s already registered", cfg.Name)
	}

	fs := newFilamentSwitchSensor(fsm.rt, cfg)
	fsm.sensors[cfg.Name] = fs
	return fs, nil
}

// Get returns a filament sensor by name.
func (fsm *FilamentSensorManager) Get(name string) *FilamentSwitchSensor {
	fsm.mu.RLock()
	defer fsm.mu.RUnlock()
	return fsm.sensors[name]
}

// List returns all registered filament sensors.
func (fsm *FilamentSensorManager) List() []*FilamentSwitchSensor {
	fsm.mu.RLock()
	defer fsm.mu.RUnlock()
	result := make([]*FilamentSwitchSensor, 0, len(fsm.sensors))
	for _, fs := range fsm.sensors {
		result = append(result, fs)
	}
	return result
}

// HandleReady is called when the printer is ready.
func (fsm *FilamentSensorManager) HandleReady() {
	fsm.mu.RLock()
	defer fsm.mu.RUnlock()
	for _, fs := range fsm.sensors {
		fs.runoutHelper.HandleReady()
	}
}
