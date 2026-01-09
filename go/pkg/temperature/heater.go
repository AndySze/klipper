// Heater control for Klipper Go migration
// Copyright (C) 2026  Klipper Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package temperature

import (
	"fmt"
	"math"
	"sync"
)

const (
	// MaxHeatTime is the maximum time the heater can be on
	MaxHeatTime = 3.0

	// AmbientTemp is the default ambient temperature
	AmbientTemp = 25.0

	// PIDParamBase is the base for PID parameter normalization
	PIDParamBase = 255.0

	// MaxMainthreadTime is the maximum time for main thread verification
	MaxMainthreadTime = 5.0

	// QuellStaleTime is the time before temperature readings are considered stale
	QuellStaleTime = 7.0

	// PIDSettleDelta is the temperature delta for PID settlement
	PIDSettleDelta = 1.0

	// PIDSettleSlope is the temperature slope for PID settlement
	PIDSettleSlope = 0.1
)

// HeaterConfig represents the heater configuration
type HeaterConfig struct {
	Name              string
	MinTemp           float64
	MaxTemp           float64
	MinExtrudeTemp    float64
	MaxPower          float64
	SmoothTime        float64
	PWMCycleTime      float64
	HeaterPin         string
	ControlType       string // "watermark" or "pid"
	PID_Kp            float64
	PID_Ki            float64
	PID_Kd            float64
	MaxDelta          float64 // For watermark control
}

// PWMInterface represents the PWM output to the heater
type PWMInterface interface {
	SetPWM(pwmTime, value float64) error
	SetupCycleTime(cycleTime float64) error
	SetupMaxDuration(maxDuration float64) error
	GetMCU() MCUInterface
}

// Heater controls a heating element with temperature feedback
type Heater struct {
	config    *HeaterConfig
	sensor    Sensor
	pwm       PWMInterface

	// Temperature state
	lastTemp        float64
	smoothedTemp    float64
	targetTemp      float64
	lastTempTime    float64

	// PWM state
	nextPWMTime     float64
	lastPWMValue    float64

	// Control
	control         ControlAlgorithm

	// Synchronization
	mu              sync.Mutex

	// Validation
	canExtrude      bool
	verifyMainThreadTime float64

	// Printer reference
	printer         PrinterInterface
}

// PrinterInterface represents the printer interface
type PrinterInterface interface {
	IsShutdown() bool
	CommandError(format string, args ...interface{}) error
}

// NewHeater creates a new heater
func NewHeater(config *HeaterConfig, sensor Sensor, pwm PWMInterface, printer PrinterInterface) (*Heater, error) {
	if config.MinTemp < KelvinToCelsius {
		return nil, fmt.Errorf("min_temp %.2f is below absolute zero", config.MinTemp)
	}
	if config.MaxTemp <= config.MinTemp {
		return nil, fmt.Errorf("max_temp %.2f must be greater than min_temp %.2f", config.MaxTemp, config.MinTemp)
	}

	// Setup sensor min/max
	if err := sensor.SetupMinMax(config.MinTemp, config.MaxTemp); err != nil {
		return nil, err
	}

	// Create control algorithm
	var control ControlAlgorithm
	var err error
	if config.ControlType == "pid" {
		control, err = NewControlPID(config)
	} else {
		control, err = NewControlBangBang(config)
	}
	if err != nil {
		return nil, err
	}

	h := &Heater{
		config:              config,
		sensor:              sensor,
		pwm:                 pwm,
		control:             control,
		canExtrude:          config.MinExtrudeTemp <= 0,
		lastTemp:            AmbientTemp,
		smoothedTemp:        AmbientTemp,
		verifyMainThreadTime: -999.0,
	}

	// Setup sensor callback
	sensor.SetupCallback(h.temperatureCallback)

	// Setup PWM
	if err := pwm.SetupCycleTime(config.PWMCycleTime); err != nil {
		return nil, err
	}
	if err := pwm.SetupMaxDuration(MaxHeatTime); err != nil {
		return nil, err
	}

	return h, nil
}

// temperatureCallback is called when the sensor reports a new temperature
func (h *Heater) temperatureCallback(readTime, temp float64) {
	h.mu.Lock()
	defer h.mu.Unlock()

	timeDiff := readTime - h.lastTempTime
	h.lastTemp = temp
	h.lastTempTime = readTime

	// Update control algorithm
	h.control.TemperatureUpdate(readTime, temp, h.targetTemp)

	// Update smoothed temperature
	tempDiff := temp - h.smoothedTemp
	invSmoothTime := 1.0 / h.config.SmoothTime
	adjTime := math.Min(timeDiff*invSmoothTime, 1.0)
	h.smoothedTemp += tempDiff * adjTime

	// Update extrude capability
	h.canExtrude = h.smoothedTemp >= h.config.MinExtrudeTemp
}

// SetPWM sets the PWM output for the heater
func (h *Heater) setPWM(readTime, value float64) error {
	// Turn off if target is 0 or main thread verification failed
	if h.targetTemp <= 0 || readTime > h.verifyMainThreadTime {
		value = 0
	}

	// Suppress insignificant changes
	if (readTime < h.nextPWMTime || h.lastPWMValue == 0) &&
		math.Abs(value-h.lastPWMValue) < 0.05 {
		return nil
	}

	pwmTime := readTime + h.sensor.GetReportTimeDelta()
	h.nextPWMTime = pwmTime + MaxHeatTime - (3.0*h.sensor.GetReportTimeDelta() + 0.001)
	h.lastPWMValue = value

	return h.pwm.SetPWM(pwmTime, value)
}

// SetTemp sets the target temperature
func (h *Heater) SetTemp(degrees float64) error {
	if degrees != 0 && (degrees < h.config.MinTemp || degrees > h.config.MaxTemp) {
		return h.printer.CommandError(
			"Requested temperature (%.1f) out of range (%.1f:%.1f)",
			degrees, h.config.MinTemp, h.config.MaxTemp)
	}

	h.mu.Lock()
	defer h.mu.Unlock()
	h.targetTemp = degrees
	return nil
}

// GetTemp returns the current temperature and target temperature
func (h *Heater) GetTemp(eventtime float64) (current, target float64) {
	h.mu.Lock()
	defer h.mu.Unlock()

	// Check if reading is stale
	if h.pwm != nil && h.pwm.GetMCU() != nil {
		estPrintTime := h.pwm.GetMCU().EstimatedPrintTime(eventtime)
		quellTime := estPrintTime - QuellStaleTime
		if h.lastTempTime < quellTime {
			return 0.0, h.targetTemp
		}
	}

	return h.smoothedTemp, h.targetTemp
}

// CheckBusy returns true if the heater is still heating/cooling to target
func (h *Heater) CheckBusy(eventtime float64) bool {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.control.CheckBusy(eventtime, h.smoothedTemp, h.targetTemp)
}

// GetStatus returns the heater status
func (h *Heater) GetStatus(eventtime float64) map[string]interface{} {
	h.mu.Lock()
	defer h.mu.Unlock()

	return map[string]interface{}{
		"temperature": roundToPlaces(h.smoothedTemp, 2),
		"target":      h.targetTemp,
		"power":       h.lastPWMValue,
	}
}

// SetControl changes the control algorithm
func (h *Heater) SetControl(control ControlAlgorithm) ControlAlgorithm {
	h.mu.Lock()
	defer h.mu.Unlock()

	oldControl := h.control
	h.control = control
	h.targetTemp = 0
	return oldControl
}

// GetMaxPower returns the maximum power (0-1)
func (h *Heater) GetMaxPower() float64 {
	return h.config.MaxPower
}

// GetSmoothTime returns the smoothing time constant
func (h *Heater) GetSmoothTime() float64 {
	return h.config.SmoothTime
}

// GetPWMDelay returns the PWM delay (sensor report time delta)
func (h *Heater) GetPWMDelay() float64 {
	return h.sensor.GetReportTimeDelta()
}

// AlterTarget alters the target temperature within safe limits
func (h *Heater) AlterTarget(targetTemp float64) {
	if targetTemp != 0 {
		targetTemp = math.Max(h.config.MinTemp, math.Min(h.config.MaxTemp, targetTemp))
	}
	h.targetTemp = targetTemp
}

// Stats returns stats information
func (h *Heater) Stats(eventtime float64) (isActive bool, stats string) {
	h.mu.Lock()
	defer h.mu.Unlock()

	targetTemp := h.targetTemp
	lastTemp := h.lastTemp
	lastPWMValue := h.lastPWMValue

	isActive = targetTemp != 0 || lastTemp > 50.0
	stats = fmt.Sprintf("%s: target=%.0f temp=%.1f pwm=%.3f",
		h.config.Name, targetTemp, lastTemp, lastPWMValue)

	return isActive, stats
}

// HandleShutdown handles shutdown event
func (h *Heater) HandleShutdown() {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.verifyMainThreadTime = -999.0
}

// VerifyMainThread updates the main thread verification time
func (h *Heater) VerifyMainThread(estPrintTime float64) {
	if !h.printer.IsShutdown() {
		h.verifyMainThreadTime = estPrintTime + MaxMainthreadTime
	}
}

// CanExtrude returns true if the heater is hot enough to extrude
func (h *Heater) CanExtrude() bool {
	return h.canExtrude
}

// GetLastTemp returns the last temperature reading
func (h *Heater) GetLastTemp() float64 {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.lastTemp
}

// GetLastPWMValue returns the last PWM value
func (h *Heater) GetLastPWMValue() float64 {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.lastPWMValue
}

// roundToPlaces rounds a float to a specific number of decimal places
func roundToPlaces(value float64, places int) float64 {
	multiplier := math.Pow(10, float64(places))
	return math.Round(value*multiplier) / multiplier
}
