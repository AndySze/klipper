// Heaters - port of klippy/extras/heaters.py
//
// Tracking of PWM controlled heaters and their temperature control
//
// Copyright (C) 2016-2025 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sort"
	"sync"
	"time"

	"klipper-go-migration/pkg/temperature"
)

const (
	// Constants from Python heaters.py
	heatersMaxHeatTime      = 3.0
	heatersAmbientTemp      = 25.0
	heatersPIDParamBase     = 255.0
	heatersMaxMainthreadTime = 5.0
	heatersQuellStaleTime   = 7.0
)

// PrinterHeaters manages all heaters and temperature sensors.
// This wraps the temperature.HeaterManager to provide G-code command integration.
type PrinterHeaters struct {
	rt *runtime

	heaters          map[string]*temperature.Heater
	sensors          map[string]temperature.Sensor
	gcodeIDToSensor  map[string]temperature.Sensor
	availableHeaters []string
	availableSensors []string
	availableMonitors []string
	hasStarted       bool

	mu sync.RWMutex
}

// newPrinterHeaters creates a new heaters manager.
func newPrinterHeaters(rt *runtime) *PrinterHeaters {
	return &PrinterHeaters{
		rt:               rt,
		heaters:          make(map[string]*temperature.Heater),
		sensors:          make(map[string]temperature.Sensor),
		gcodeIDToSensor:  make(map[string]temperature.Sensor),
	}
}

// HandleReady marks the printer as ready.
func (ph *PrinterHeaters) HandleReady() {
	ph.mu.Lock()
	defer ph.mu.Unlock()
	ph.hasStarted = true
}

// SetupHeater creates and registers a heater.
func (ph *PrinterHeaters) SetupHeater(name string, config *temperature.HeaterConfig, sensor temperature.Sensor, pwm temperature.PWMInterface, gcodeID string) (*temperature.Heater, error) {
	ph.mu.Lock()
	defer ph.mu.Unlock()

	if _, exists := ph.heaters[name]; exists {
		return nil, fmt.Errorf("heater '%s' already registered", name)
	}

	heater, err := temperature.NewHeater(config, sensor, pwm, newPrinterAdapter(ph.rt))
	if err != nil {
		return nil, err
	}

	ph.heaters[name] = heater
	ph.availableHeaters = append(ph.availableHeaters, name)
	ph.registerSensor(name, sensor, gcodeID)

	log.Printf("heaters: registered heater '%s' with gcode_id '%s'", name, gcodeID)
	return heater, nil
}

// registerSensor registers a sensor with optional G-code ID.
func (ph *PrinterHeaters) registerSensor(name string, sensor temperature.Sensor, gcodeID string) {
	ph.sensors[name] = sensor
	ph.availableSensors = append(ph.availableSensors, name)

	if gcodeID != "" {
		if _, exists := ph.gcodeIDToSensor[gcodeID]; exists {
			log.Printf("heaters: warning - G-code sensor id '%s' already registered", gcodeID)
			return
		}
		ph.gcodeIDToSensor[gcodeID] = sensor
	}
}

// RegisterSensor registers a standalone sensor.
func (ph *PrinterHeaters) RegisterSensor(name string, sensor temperature.Sensor, gcodeID string) {
	ph.mu.Lock()
	defer ph.mu.Unlock()
	ph.registerSensor(name, sensor, gcodeID)
}

// RegisterMonitor registers a temperature monitor.
func (ph *PrinterHeaters) RegisterMonitor(name string) {
	ph.mu.Lock()
	defer ph.mu.Unlock()
	ph.availableMonitors = append(ph.availableMonitors, name)
}

// GetHeater returns a heater by name.
func (ph *PrinterHeaters) GetHeater(name string) (*temperature.Heater, error) {
	ph.mu.RLock()
	defer ph.mu.RUnlock()

	heater, exists := ph.heaters[name]
	if !exists {
		return nil, fmt.Errorf("unknown heater '%s'", name)
	}
	return heater, nil
}

// LookupHeater is an alias for GetHeater (Python compatibility).
func (ph *PrinterHeaters) LookupHeater(name string) (*temperature.Heater, error) {
	return ph.GetHeater(name)
}

// GetSensor returns a sensor by name.
func (ph *PrinterHeaters) GetSensor(name string) (temperature.Sensor, error) {
	ph.mu.RLock()
	defer ph.mu.RUnlock()

	sensor, exists := ph.sensors[name]
	if !exists {
		return nil, fmt.Errorf("unknown sensor '%s'", name)
	}
	return sensor, nil
}

// GetAllHeaters returns all heater names.
func (ph *PrinterHeaters) GetAllHeaters() []string {
	ph.mu.RLock()
	defer ph.mu.RUnlock()
	return append([]string{}, ph.availableHeaters...)
}

// GetAllSensors returns all sensor names.
func (ph *PrinterHeaters) GetAllSensors() []string {
	ph.mu.RLock()
	defer ph.mu.RUnlock()
	return append([]string{}, ph.availableSensors...)
}

// SetTemperature sets a heater's target temperature.
func (ph *PrinterHeaters) SetTemperature(heater *temperature.Heater, temp float64, wait bool) error {
	if err := heater.SetTemp(temp); err != nil {
		return err
	}

	if wait && temp > 0 {
		return ph.waitForTemperature(heater)
	}
	return nil
}

// SetTemperatureByName sets a heater's temperature by name.
func (ph *PrinterHeaters) SetTemperatureByName(heaterName string, temp float64, wait bool) error {
	heater, err := ph.GetHeater(heaterName)
	if err != nil {
		return err
	}
	return ph.SetTemperature(heater, temp, wait)
}

// waitForTemperature waits for a heater to reach its target temperature.
func (ph *PrinterHeaters) waitForTemperature(heater *temperature.Heater) error {
	for {
		eventtime := float64(time.Now().UnixNano()) / 1e9
		if !heater.CheckBusy(eventtime) {
			return nil
		}
		time.Sleep(1 * time.Second)
	}
}

// TurnOffAllHeaters turns off all heaters.
func (ph *PrinterHeaters) TurnOffAllHeaters() {
	ph.mu.RLock()
	defer ph.mu.RUnlock()

	for _, heater := range ph.heaters {
		_ = heater.SetTemp(0)
	}
}

// GetTemp returns temperature for a G-code ID sensor.
func (ph *PrinterHeaters) getTemp(eventtime float64) string {
	ph.mu.RLock()
	defer ph.mu.RUnlock()

	if !ph.hasStarted {
		return "T:0"
	}

	// Sort G-code IDs
	var gcodeIDs []string
	for id := range ph.gcodeIDToSensor {
		gcodeIDs = append(gcodeIDs, id)
	}
	sort.Strings(gcodeIDs)

	var out []string
	for _, id := range gcodeIDs {
		sensor := ph.gcodeIDToSensor[id]
		cur, target := sensor.GetTemp(eventtime)
		out = append(out, fmt.Sprintf("%s:%.1f /%.1f", id, cur, target))
	}

	if len(out) == 0 {
		return "T:0"
	}

	result := ""
	for i, s := range out {
		if i > 0 {
			result += " "
		}
		result += s
	}
	return result
}

// cmdM105 handles M105 (get temperature) command.
func (ph *PrinterHeaters) cmdM105(eventtime float64) string {
	return ph.getTemp(eventtime)
}

// cmdTurnOffHeaters handles TURN_OFF_HEATERS command.
func (ph *PrinterHeaters) cmdTurnOffHeaters() {
	ph.TurnOffAllHeaters()
}

// cmdTemperatureWait handles TEMPERATURE_WAIT command.
func (ph *PrinterHeaters) cmdTemperatureWait(sensorName string, minTemp, maxTemp float64) error {
	ph.mu.RLock()
	sensor, exists := ph.sensors[sensorName]
	ph.mu.RUnlock()

	if !exists {
		// Check if it's a heater
		heater, err := ph.GetHeater(sensorName)
		if err != nil {
			return fmt.Errorf("unknown sensor '%s'", sensorName)
		}
		// Use heater as sensor
		return ph.waitForTemperatureRange(heater, minTemp, maxTemp)
	}

	return ph.waitForSensorRange(sensor, minTemp, maxTemp)
}

// waitForTemperatureRange waits for a heater to reach a temperature range.
func (ph *PrinterHeaters) waitForTemperatureRange(heater *temperature.Heater, minTemp, maxTemp float64) error {
	for {
		eventtime := float64(time.Now().UnixNano()) / 1e9
		temp, _ := heater.GetTemp(eventtime)
		if temp >= minTemp && temp <= maxTemp {
			return nil
		}
		time.Sleep(1 * time.Second)
	}
}

// waitForSensorRange waits for a sensor to reach a temperature range.
func (ph *PrinterHeaters) waitForSensorRange(sensor temperature.Sensor, minTemp, maxTemp float64) error {
	for {
		eventtime := float64(time.Now().UnixNano()) / 1e9
		temp, _ := sensor.GetTemp(eventtime)
		if temp >= minTemp && temp <= maxTemp {
			return nil
		}
		time.Sleep(1 * time.Second)
	}
}

// GetStatus returns the heaters status.
func (ph *PrinterHeaters) GetStatus() map[string]any {
	ph.mu.RLock()
	defer ph.mu.RUnlock()

	return map[string]any{
		"available_heaters":  ph.availableHeaters,
		"available_sensors":  ph.availableSensors,
		"available_monitors": ph.availableMonitors,
	}
}

// VerifyMainThreadTime updates verification time for all heaters.
func (ph *PrinterHeaters) VerifyMainThreadTime(estPrintTime float64) {
	ph.mu.RLock()
	defer ph.mu.RUnlock()

	for _, heater := range ph.heaters {
		heater.VerifyMainThread(estPrintTime)
	}
}

// HandleShutdown handles shutdown event.
func (ph *PrinterHeaters) HandleShutdown() {
	ph.mu.RLock()
	defer ph.mu.RUnlock()

	for _, heater := range ph.heaters {
		heater.HandleShutdown()
	}
}
