// Temperature control algorithms for Klipper Go migration
// Copyright (C) 2026  Klipper Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package temperature

import (
	"fmt"
	"math"
)

// ControlAlgorithm is the interface for temperature control algorithms
type ControlAlgorithm interface {
	// TemperatureUpdate is called when a new temperature reading is available
	TemperatureUpdate(readTime, temp, targetTemp float64)

	// CheckBusy returns true if the heater is still heating/cooling to target
	CheckBusy(eventtime, smoothedTemp, targetTemp float64) bool
}

// ControlBangBang implements simple bang-bang (on/off) control
type ControlBangBang struct {
	heater       *Heater
	maxPower     float64
	maxDelta     float64
	heating      bool
}

// NewControlBangBang creates a new bang-bang controller
func NewControlBangBang(config *HeaterConfig) (*ControlBangBang, error) {
	if config.MaxDelta <= 0 {
		return nil, fmt.Errorf("max_delta must be positive")
	}

	return &ControlBangBang{
		maxPower: config.MaxPower,
		maxDelta: config.MaxDelta,
		heating:  false,
	}, nil
}

// SetHeater sets the heater reference
func (c *ControlBangBang) SetHeater(heater *Heater) {
	c.heater = heater
}

// TemperatureUpdate implements bang-bang control
func (c *ControlBangBang) TemperatureUpdate(readTime, temp, targetTemp float64) {
	// Turn on if below target - max_delta
	// Turn off if above target + max_delta
	if c.heating && temp >= targetTemp+c.maxDelta {
		c.heating = false
	} else if !c.heating && temp <= targetTemp-c.maxDelta {
		c.heating = true
	}

	var pwmValue float64
	if c.heating {
		pwmValue = c.maxPower
	} else {
		pwmValue = 0.0
	}

	if c.heater != nil {
		c.heater.setPWM(readTime, pwmValue)
	}
}

// CheckBusy returns true if the heater is still heating to target
func (c *ControlBangBang) CheckBusy(eventtime, smoothedTemp, targetTemp float64) bool {
	return smoothedTemp < targetTemp-c.maxDelta
}

// ControlPID implements PID control
type ControlPID struct {
	heater         *Heater
	maxPower       float64
	Kp             float64
	Ki             float64
	Kd             float64
	minDerivTime   float64
	tempIntegMax   float64

	// State
	prevTemp       float64
	prevTempTime   float64
	prevTempDeriv  float64
	prevTempInteg  float64
}

// NewControlPID creates a new PID controller
func NewControlPID(config *HeaterConfig) (*ControlPID, error) {
	if config.PID_Kp == 0 || config.PID_Ki == 0 || config.PID_Kd == 0 {
		return nil, fmt.Errorf("PID parameters must be non-zero")
	}

	Kp := config.PID_Kp / PIDParamBase
	Ki := config.PID_Ki / PIDParamBase
	Kd := config.PID_Kd / PIDParamBase

	var tempIntegMax float64
	if Ki != 0 {
		tempIntegMax = config.MaxPower / Ki
	}

	return &ControlPID{
		maxPower:     config.MaxPower,
		Kp:           Kp,
		Ki:           Ki,
		Kd:           Kd,
		minDerivTime: config.SmoothTime,
		tempIntegMax: tempIntegMax,
		prevTemp:     AmbientTemp,
	}, nil
}

// SetHeater sets the heater reference
func (c *ControlPID) SetHeater(heater *Heater) {
	c.heater = heater
}

// TemperatureUpdate implements PID control
func (c *ControlPID) TemperatureUpdate(readTime, temp, targetTemp float64) {
	timeDiff := readTime - c.prevTempTime

	// Calculate temperature derivative
	var tempDeriv float64
	if timeDiff >= c.minDerivTime {
		tempDeriv = (temp - c.prevTemp) / timeDiff
	} else {
		// Smooth the derivative
		tempDiff := temp - c.prevTemp
		tempDeriv = (c.prevTempDeriv*(c.minDerivTime-timeDiff) + tempDiff) / c.minDerivTime
	}

	// Calculate temperature error
	tempErr := targetTemp - temp

	// Calculate integral
	tempInteg := c.prevTempInteg + tempErr*timeDiff
	if c.tempIntegMax > 0 {
		tempInteg = math.Max(0.0, math.Min(c.tempIntegMax, tempInteg))
	}

	// Calculate PID output
	co := c.Kp*tempErr + c.Ki*tempInteg - c.Kd*tempDeriv

	// Bound to valid range
	boundedCo := math.Max(0.0, math.Min(c.maxPower, co))

	// Apply PWM
	if c.heater != nil {
		c.heater.setPWM(readTime, boundedCo)
	}

	// Store state for next iteration
	c.prevTemp = temp
	c.prevTempTime = readTime
	c.prevTempDeriv = tempDeriv

	// Only update integral if output wasn't bounded
	if co == boundedCo {
		c.prevTempInteg = tempInteg
	}
}

// CheckBusy returns true if the heater is still settling to target
func (c *ControlPID) CheckBusy(eventtime, smoothedTemp, targetTemp float64) bool {
	tempDiff := targetTemp - smoothedTemp
	return math.Abs(tempDiff) > PIDSettleDelta || math.Abs(c.prevTempDeriv) > PIDSettleSlope
}
