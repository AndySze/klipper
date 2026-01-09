// Heater manager for Klipper Go migration
// Copyright (C) 2026  Klipper Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package temperature

import (
	"fmt"
	"sort"
)

// HeaterManager manages all heaters and sensors
type HeaterManager struct {
	sensorRegistry   *SensorRegistry
	heaters          map[string]*Heater
	sensors          map[string]Sensor
	gcodeIDToSensor  map[string]Sensor
	availableHeaters []string
	availableSensors []string
	hasStarted       bool
	printer          PrinterInterface
}

// NewHeaterManager creates a new heater manager
func NewHeaterManager(printer PrinterInterface) *HeaterManager {
	return &HeaterManager{
		sensorRegistry:  NewSensorRegistry(),
		heaters:         make(map[string]*Heater),
		sensors:         make(map[string]Sensor),
		gcodeIDToSensor: make(map[string]Sensor),
		printer:         printer,
	}
}

// RegisterSensorFactory registers a sensor type
func (hm *HeaterManager) RegisterSensorFactory(sensorType string, factory SensorFactory) {
	hm.sensorRegistry.RegisterSensorType(sensorType, factory)
}

// SetupHeater creates and registers a heater
func (hm *HeaterManager) SetupHeater(name string, config *HeaterConfig, sensor Sensor, pwm PWMInterface) (*Heater, error) {
	if _, exists := hm.heaters[name]; exists {
		return nil, fmt.Errorf("heater '%s' already registered", name)
	}

	heater, err := NewHeater(config, sensor, pwm, hm.printer)
	if err != nil {
		return nil, err
	}

	hm.heaters[name] = heater
	hm.availableHeaters = append(hm.availableHeaters, name)
	hm.registerSensor(name, sensor, "")

	return heater, nil
}

// SetupSensor creates and registers a sensor
func (hm *HeaterManager) SetupSensor(name string, sensorType string, config interface{}, gcodeID string) (Sensor, error) {
	sensor, err := hm.sensorRegistry.CreateSensor(sensorType, config)
	if err != nil {
		return nil, err
	}

	hm.sensors[name] = sensor
	hm.availableSensors = append(hm.availableSensors, name)

	return sensor, nil
}

// registerSensor registers a sensor with optional G-code ID
func (hm *HeaterManager) registerSensor(name string, sensor Sensor, gcodeID string) {
	hm.sensors[name] = sensor
	hm.availableSensors = append(hm.availableSensors, name)

	if gcodeID != "" {
		if _, exists := hm.gcodeIDToSensor[gcodeID]; exists {
			panic(fmt.Sprintf("G-Code sensor id '%s' already registered", gcodeID))
		}
		hm.gcodeIDToSensor[gcodeID] = sensor
	}
}

// GetHeater returns a heater by name
func (hm *HeaterManager) GetHeater(name string) (*Heater, error) {
	heater, exists := hm.heaters[name]
	if !exists {
		return nil, fmt.Errorf("unknown heater '%s'", name)
	}
	return heater, nil
}

// GetSensor returns a sensor by name
func (hm *HeaterManager) GetSensor(name string) (Sensor, error) {
	sensor, exists := hm.sensors[name]
	if !exists {
		return nil, fmt.Errorf("unknown sensor '%s'", name)
	}
	return sensor, nil
}

// GetAllHeaters returns all heater names
func (hm *HeaterManager) GetAllHeaters() []string {
	return hm.availableHeaters
}

// GetAllSensors returns all sensor names
func (hm *HeaterManager) GetAllSensors() []string {
	return hm.availableSensors
}

// SetTemperature sets a heater's temperature
func (hm *HeaterManager) SetTemperature(heaterName string, temp float64, wait bool) error {
	heater, err := hm.GetHeater(heaterName)
	if err != nil {
		return err
	}

	if err := heater.SetTemp(temp); err != nil {
		return err
	}

	// TODO: Implement wait functionality
	return nil
}

// TurnOffAllHeaters turns off all heaters
func (hm *HeaterManager) TurnOffAllHeaters() error {
	for _, heater := range hm.heaters {
		if err := heater.SetTemp(0); err != nil {
			return err
		}
	}
	return nil
}

// HandleReady marks that the printer has started
func (hm *HeaterManager) HandleReady() {
	hm.hasStarted = true
}

// GetM105Response returns the M105 temperature response
// Format: "T:XXX /YYY B:XXX /YYY" or "T0:XXX /YYY T1:XXX /YYY ..."
func (hm *HeaterManager) GetM105Response(eventtime float64) string {
	if !hm.hasStarted {
		return "T:0"
	}

	// Sort G-code IDs for consistent output
	var gcodeIDs []string
	for id := range hm.gcodeIDToSensor {
		gcodeIDs = append(gcodeIDs, id)
	}
	sort.Strings(gcodeIDs)

	var out []string
	for _, id := range gcodeIDs {
		sensor := hm.gcodeIDToSensor[id]
		cur, target := sensor.GetTemp(eventtime)
		out = append(out, fmt.Sprintf("%s:%.1f /%.1f", id, cur, target))
	}

	if len(out) == 0 {
		return "T:0"
	}

	// Join with spaces
	result := ""
	for i, s := range out {
		if i > 0 {
			result += " "
		}
		result += s
	}
	return result
}

// GetStatus returns the manager status
func (hm *HeaterManager) GetStatus(eventtime float64) map[string]interface{} {
	return map[string]interface{}{
		"available_heaters": hm.availableHeaters,
		"available_sensors": hm.availableSensors,
	}
}

// HeaterStats represents stats for a single heater
type HeaterStats struct {
	IsActive bool
	Stats    string
}

// GetHeaterStats returns stats for all heaters
func (hm *HeaterManager) GetHeaterStats(eventtime float64) map[string]HeaterStats {
	stats := make(map[string]HeaterStats)
	for name, heater := range hm.heaters {
		isActive, statStr := heater.Stats(eventtime)
		stats[name] = HeaterStats{
			IsActive: isActive,
			Stats:    statStr,
		}
	}
	return stats
}

// VerifyMainThreadTime updates main thread verification for all heaters
func (hm *HeaterManager) VerifyMainThreadTime(estPrintTime float64) {
	for _, heater := range hm.heaters {
		heater.VerifyMainThread(estPrintTime)
	}
}
