// G-code commands for temperature control
// Copyright (C) 2026  Klipper Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package temperature

import (
	"fmt"
)

// GCodeCommandHandler handles temperature-related G-code commands
type GCodeCommandHandler struct {
	heaterManager *HeaterManager
	gcode         GCodeInterface
}

// GCodeInterface represents the G-code handling interface
type GCodeInterface interface {
	Respond(msg string)
	RespondRaw(msg string)
	GetFloat(args map[string][]float64, param string, defaultValue float64) float64
}

// NewGCodeCommandHandler creates a new G-code command handler
func NewGCodeCommandHandler(heaterManager *HeaterManager, gcode GCodeInterface) *GCodeCommandHandler {
	return &GCodeCommandHandler{
		heaterManager: heaterManager,
		gcode:         gcode,
	}
}

// CmdM104 handles M104 - Set Extruder Temperature
// Usage: M104 [S<temp>] [T<index>]
func (gh *GCodeCommandHandler) CmdM104(args map[string][]float64) error {
	// Get temperature (S parameter is standard)
	temp := gh.gcode.GetFloat(args, "S", 0.0)

	// Get extruder index (T parameter, defaults to 0)
	index := int(gh.gcode.GetFloat(args, "T", 0.0))

	heaterName := gh.getExtruderHeaterName(index)
	if err := gh.heaterManager.SetTemperature(heaterName, temp, false); err != nil {
		return err
	}

	return nil
}

// CmdM140 handles M140 - Set Heated Bed Temperature
// Usage: M140 [S<temp>]
func (gh *GCodeCommandHandler) CmdM140(args map[string][]float64) error {
	// Get temperature (S parameter)
	temp := gh.gcode.GetFloat(args, "S", 0.0)

	if err := gh.heaterManager.SetTemperature("heater_bed", temp, false); err != nil {
		return err
	}

	return nil
}

// CmdM109 handles M109 - Set Extruder Temperature and Wait
// Usage: M109 [S<temp>] [T<index>]
func (gh *GCodeCommandHandler) CmdM109(args map[string][]float64) error {
	// Get temperature
	temp := gh.gcode.GetFloat(args, "S", 0.0)

	// Get extruder index
	index := int(gh.gcode.GetFloat(args, "T", 0.0))

	heaterName := gh.getExtruderHeaterName(index)

	// Set temperature and wait
	if err := gh.heaterManager.SetTemperature(heaterName, temp, true); err != nil {
		return err
	}

	return nil
}

// CmdM190 handles M190 - Set Heated Bed Temperature and Wait
// Usage: M190 [S<temp>]
func (gh *GCodeCommandHandler) CmdM190(args map[string][]float64) error {
	// Get temperature
	temp := gh.gcode.GetFloat(args, "S", 0.0)

	// Set temperature and wait
	if err := gh.heaterManager.SetTemperature("heater_bed", temp, true); err != nil {
		return err
	}

	return nil
}

// CmdM105 handles M105 - Get Temperature
// Usage: M105
func (gh *GCodeCommandHandler) CmdM105(args map[string][]float64) error {
	// Get current temperature report
	// This will be handled by the reactor/event system
	// For now, we return the response directly
	eventtime := 0.0 // TODO: Get actual eventtime
	response := gh.heaterManager.GetM105Response(eventtime)

	// Send response immediately
	gh.gcode.RespondRaw(response)

	return nil
}

// CmdSET_HEATER_TEMPERATURE handles SET_HEATER_TEMPERATURE
// Usage: SET_HEATER_TEMPERATURE HEATER=<name> TARGET=<temp>
func (gh *GCodeCommandHandler) CmdSET_HEATER_TEMPERATURE(args map[string]string) error {
	heaterName, ok := args["HEATER"]
	if !ok {
		return fmt.Errorf("missing HEATER parameter")
	}

	targetStr, ok := args["TARGET"]
	if !ok {
		return fmt.Errorf("missing TARGET parameter")
	}

	var target float64
	_, err := fmt.Sscanf(targetStr, "%f", &target)
	if err != nil {
		return fmt.Errorf("invalid TARGET value: %s", targetStr)
	}

	return gh.heaterManager.SetTemperature(heaterName, target, false)
}

// CmdTURN_OFF_HEATERS handles TURN_OFF_HEATERS
// Usage: TURN_OFF_HEATERS
func (gh *GCodeCommandHandler) CmdTURN_OFF_HEATERS() error {
	return gh.heaterManager.TurnOffAllHeaters()
}

// CmdTEMPERATURE_WAIT handles TEMPERATURE_WAIT
// Usage: TEMPERATURE_WAIT SENSOR=<name> [MINIMUM=<temp>] [MAXIMUM=<temp>]
func (gh *GCodeCommandHandler) CmdTEMPERATURE_WAIT(args map[string]string) error {
	sensorName, ok := args["SENSOR"]
	if !ok {
		return fmt.Errorf("missing SENSOR parameter")
	}

	minTemp := getFloatFromString(args, "MINIMUM", 0)
	maxTemp := getFloatFromString(args, "MAXIMUM", 0)

	if minTemp == 0 && maxTemp == 0 {
		return fmt.Errorf("missing MINIMUM or MAXIMUM parameter")
	}

	// TODO: Implement wait loop
	// For now, just verify the sensor exists
	_, err := gh.heaterManager.GetSensor(sensorName)
	return err
}

// getExtruderHeaterName returns the heater name for an extruder index
func (gh *GCodeCommandHandler) getExtruderHeaterName(index int) string {
	if index == 0 {
		return "extruder"
	}
	return fmt.Sprintf("extruder%d", index)
}

// getFloatFromString helper to parse float from string map
func getFloatFromString(args map[string]string, key string, defaultValue float64) float64 {
	valStr, ok := args[key]
	if !ok {
		return defaultValue
	}

	var val float64
	_, err := fmt.Sscanf(valStr, "%f", &val)
	if err != nil {
		return defaultValue
	}

	return val
}

// WaitHeater waits for a heater to reach its target temperature
func (gh *GCodeCommandHandler) WaitHeater(heater *Heater) error {
	// TODO: Implement wait loop with temperature reporting
	// This requires integration with the toolhead and reactor system
	return nil
}
