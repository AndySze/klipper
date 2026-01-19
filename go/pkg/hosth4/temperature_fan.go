// Temperature fan - port of klippy/extras/temperature_fan.py
//
// Support fans that are enabled when temperature exceeds a set threshold
//
// Copyright (C) 2016-2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"math"
	"strconv"
	"sync"
)

const (
	maxFanTime       = 5.0
	ambientTemp      = 25.0
	pidParamBase     = 255.0
	pidSettleDelta   = 1.0
	pidSettleSlope   = 0.1
)

// TemperatureFanControl defines the interface for fan control algorithms.
type TemperatureFanControl interface {
	TemperatureCallback(readTime, temp float64)
}

// TemperatureFan represents a fan controlled by temperature.
type TemperatureFan struct {
	rt   *runtime
	name string

	minTemp   float64 // minimum temperature
	maxTemp   float64 // maximum temperature
	minSpeed  float64 // minimum fan speed
	maxSpeed  float64 // maximum fan speed
	targetTemp float64 // target temperature

	lastTemp     float64
	lastTempTime float64

	speedDelay     float64 // delay before applying speed change
	nextSpeedTime  float64
	lastSpeedValue float64

	control TemperatureFanControl
	mu      sync.Mutex

	// Fan control (in a full implementation, this would be a Fan object)
	fanPin string
}

// TemperatureFanConfig holds configuration for a temperature fan.
type TemperatureFanConfig struct {
	Name       string
	Pin        string
	Sensor     string // sensor name
	MinTemp    float64
	MaxTemp    float64
	MinSpeed   float64 // default 0.3
	MaxSpeed   float64 // default 1.0
	TargetTemp float64 // default 40.0
	Control    string  // "watermark" or "pid"

	// PID parameters
	PidKp         float64
	PidKi         float64
	PidKd         float64
	PidDerivTime  float64 // default 2.0

	// Watermark parameters
	MaxDelta float64 // default 2.0
}

// DefaultTemperatureFanConfig returns default temperature fan configuration.
func DefaultTemperatureFanConfig() TemperatureFanConfig {
	return TemperatureFanConfig{
		MinTemp:      0,
		MaxTemp:      100,
		MinSpeed:     0.3,
		MaxSpeed:     1.0,
		TargetTemp:   40.0,
		Control:      "watermark",
		MaxDelta:     2.0,
		PidDerivTime: 2.0,
	}
}

// newTemperatureFan creates a new temperature fan controller.
func newTemperatureFan(rt *runtime, cfg TemperatureFanConfig) (*TemperatureFan, error) {
	if cfg.MinTemp >= cfg.MaxTemp {
		return nil, fmt.Errorf("min_temp must be less than max_temp")
	}
	if cfg.MinSpeed < 0 || cfg.MinSpeed > 1 {
		return nil, fmt.Errorf("min_speed must be between 0 and 1")
	}
	if cfg.MaxSpeed < 0 || cfg.MaxSpeed > 1 {
		return nil, fmt.Errorf("max_speed must be between 0 and 1")
	}
	if cfg.MinSpeed > cfg.MaxSpeed {
		return nil, fmt.Errorf("min_speed must not exceed max_speed")
	}
	if cfg.TargetTemp < cfg.MinTemp || cfg.TargetTemp > cfg.MaxTemp {
		return nil, fmt.Errorf("target_temp must be between min_temp and max_temp")
	}

	tf := &TemperatureFan{
		rt:         rt,
		name:       cfg.Name,
		fanPin:     cfg.Pin,
		minTemp:    cfg.MinTemp,
		maxTemp:    cfg.MaxTemp,
		minSpeed:   cfg.MinSpeed,
		maxSpeed:   cfg.MaxSpeed,
		targetTemp: cfg.TargetTemp,
		lastTemp:   ambientTemp,
	}

	// Create control algorithm
	switch cfg.Control {
	case "watermark":
		tf.control = newControlBangBang(tf, cfg.MaxDelta)
	case "pid":
		tf.control = newControlPID(tf, cfg.PidKp, cfg.PidKi, cfg.PidKd, cfg.PidDerivTime)
	default:
		return nil, fmt.Errorf("unknown control type: %s", cfg.Control)
	}

	return tf, nil
}

// SetTFSpeed sets the fan speed.
func (tf *TemperatureFan) SetTFSpeed(readTime, value float64) {
	tf.mu.Lock()
	defer tf.mu.Unlock()

	if value <= 0 {
		value = 0
	} else if value < tf.minSpeed {
		value = tf.minSpeed
	}
	if tf.targetTemp <= 0 {
		value = 0
	}

	// Check if we should suppress the update
	if (readTime < tf.nextSpeedTime || tf.lastSpeedValue == 0) &&
		math.Abs(value-tf.lastSpeedValue) < 0.05 {
		return
	}

	speedTime := readTime + tf.speedDelay
	tf.nextSpeedTime = speedTime + 0.75*maxFanTime
	tf.lastSpeedValue = value

	// In a full implementation, this would call fan.set_speed(value, speedTime)
	tf.setFanSpeed(value)
}

// setFanSpeed sets the actual fan speed.
func (tf *TemperatureFan) setFanSpeed(speed float64) {
	// In a full implementation, this would control the actual fan PWM
	_ = speed
}

// TemperatureCallback is called when temperature is updated.
func (tf *TemperatureFan) TemperatureCallback(readTime, temp float64) {
	tf.mu.Lock()
	tf.lastTemp = temp
	tf.lastTempTime = readTime
	tf.mu.Unlock()

	tf.control.TemperatureCallback(readTime, temp)
}

// GetTemp returns the current and target temperature.
func (tf *TemperatureFan) GetTemp() (float64, float64) {
	tf.mu.Lock()
	defer tf.mu.Unlock()
	return tf.lastTemp, tf.targetTemp
}

// GetMinSpeed returns the minimum fan speed.
func (tf *TemperatureFan) GetMinSpeed() float64 {
	tf.mu.Lock()
	defer tf.mu.Unlock()
	return tf.minSpeed
}

// GetMaxSpeed returns the maximum fan speed.
func (tf *TemperatureFan) GetMaxSpeed() float64 {
	tf.mu.Lock()
	defer tf.mu.Unlock()
	return tf.maxSpeed
}

// SetTemp sets the target temperature.
func (tf *TemperatureFan) SetTemp(degrees float64) error {
	tf.mu.Lock()
	defer tf.mu.Unlock()

	if degrees != 0 && (degrees < tf.minTemp || degrees > tf.maxTemp) {
		return fmt.Errorf("temperature %.1f out of range (%.1f:%.1f)",
			degrees, tf.minTemp, tf.maxTemp)
	}
	tf.targetTemp = degrees
	return nil
}

// SetMinSpeed sets the minimum fan speed.
func (tf *TemperatureFan) SetMinSpeed(speed float64) error {
	tf.mu.Lock()
	defer tf.mu.Unlock()

	if speed != 0 && (speed < 0 || speed > 1) {
		return fmt.Errorf("speed %.1f out of range (0.0:1.0)", speed)
	}
	tf.minSpeed = speed
	return nil
}

// SetMaxSpeed sets the maximum fan speed.
func (tf *TemperatureFan) SetMaxSpeed(speed float64) error {
	tf.mu.Lock()
	defer tf.mu.Unlock()

	if speed != 0 && (speed < 0 || speed > 1) {
		return fmt.Errorf("speed %.1f out of range (0.0:1.0)", speed)
	}
	tf.maxSpeed = speed
	return nil
}

// GetStatus returns the temperature fan status.
func (tf *TemperatureFan) GetStatus() map[string]any {
	tf.mu.Lock()
	defer tf.mu.Unlock()
	return map[string]any{
		"speed":       tf.lastSpeedValue,
		"temperature": math.Round(tf.lastTemp*100) / 100,
		"target":      tf.targetTemp,
	}
}

// cmdSetTemperatureFanTarget handles the SET_TEMPERATURE_FAN_TARGET command.
// SET_TEMPERATURE_FAN_TARGET TEMPERATURE_FAN=<name> [TARGET=<temp>] [MIN_SPEED=<speed>] [MAX_SPEED=<speed>]
func (tf *TemperatureFan) cmdSetTemperatureFanTarget(args map[string]string) error {
	if targetStr, ok := args["TARGET"]; ok {
		target, err := strconv.ParseFloat(targetStr, 64)
		if err != nil {
			return fmt.Errorf("invalid TARGET value: %w", err)
		}
		if err := tf.SetTemp(target); err != nil {
			return err
		}
	}

	if minStr, ok := args["MIN_SPEED"]; ok {
		minSpeed, err := strconv.ParseFloat(minStr, 64)
		if err != nil {
			return fmt.Errorf("invalid MIN_SPEED value: %w", err)
		}
		if err := tf.SetMinSpeed(minSpeed); err != nil {
			return err
		}
	}

	if maxStr, ok := args["MAX_SPEED"]; ok {
		maxSpeed, err := strconv.ParseFloat(maxStr, 64)
		if err != nil {
			return fmt.Errorf("invalid MAX_SPEED value: %w", err)
		}
		if err := tf.SetMaxSpeed(maxSpeed); err != nil {
			return err
		}
	}

	// Validate min <= max
	tf.mu.Lock()
	if tf.minSpeed > tf.maxSpeed {
		tf.mu.Unlock()
		return fmt.Errorf("min_speed (%.1f) is greater than max_speed (%.1f)",
			tf.minSpeed, tf.maxSpeed)
	}
	tf.mu.Unlock()

	return nil
}

// ControlBangBang implements bang-bang (watermark) control.
type ControlBangBang struct {
	tf       *TemperatureFan
	maxDelta float64
	heating  bool
}

// newControlBangBang creates a new bang-bang controller.
func newControlBangBang(tf *TemperatureFan, maxDelta float64) *ControlBangBang {
	return &ControlBangBang{
		tf:       tf,
		maxDelta: maxDelta,
		heating:  false,
	}
}

// TemperatureCallback handles temperature updates for bang-bang control.
func (c *ControlBangBang) TemperatureCallback(readTime, temp float64) {
	_, targetTemp := c.tf.GetTemp()

	if c.heating && temp >= targetTemp+c.maxDelta {
		c.heating = false
	} else if !c.heating && temp <= targetTemp-c.maxDelta {
		c.heating = true
	}

	if c.heating {
		c.tf.SetTFSpeed(readTime, 0)
	} else {
		c.tf.SetTFSpeed(readTime, c.tf.GetMaxSpeed())
	}
}

// ControlPID implements PID control.
type ControlPID struct {
	tf           *TemperatureFan
	Kp           float64
	Ki           float64
	Kd           float64
	minDerivTime float64
	tempIntegMax float64

	prevTemp      float64
	prevTempTime  float64
	prevTempDeriv float64
	prevTempInteg float64
}

// newControlPID creates a new PID controller.
func newControlPID(tf *TemperatureFan, Kp, Ki, Kd, derivTime float64) *ControlPID {
	c := &ControlPID{
		tf:           tf,
		Kp:           Kp / pidParamBase,
		Ki:           Ki / pidParamBase,
		Kd:           Kd / pidParamBase,
		minDerivTime: derivTime,
		prevTemp:     ambientTemp,
	}
	if c.Ki > 0 {
		c.tempIntegMax = tf.GetMaxSpeed() / c.Ki
	}
	return c
}

// TemperatureCallback handles temperature updates for PID control.
func (c *ControlPID) TemperatureCallback(readTime, temp float64) {
	_, targetTemp := c.tf.GetTemp()
	timeDiff := readTime - c.prevTempTime

	// Calculate change of temperature
	tempDiff := temp - c.prevTemp
	var tempDeriv float64
	if timeDiff >= c.minDerivTime {
		tempDeriv = tempDiff / timeDiff
	} else {
		tempDeriv = (c.prevTempDeriv*(c.minDerivTime-timeDiff) + tempDiff) / c.minDerivTime
	}

	// Calculate accumulated temperature "error"
	tempErr := targetTemp - temp
	tempInteg := c.prevTempInteg + tempErr*timeDiff
	tempInteg = math.Max(0, math.Min(c.tempIntegMax, tempInteg))

	// Calculate output
	co := c.Kp*tempErr + c.Ki*tempInteg - c.Kd*tempDeriv
	boundedCo := math.Max(0, math.Min(c.tf.GetMaxSpeed(), co))

	// Set fan speed (inverted: higher temp = higher speed)
	speed := math.Max(c.tf.GetMinSpeed(), c.tf.GetMaxSpeed()-boundedCo)
	c.tf.SetTFSpeed(readTime, speed)

	// Store state for next measurement
	c.prevTemp = temp
	c.prevTempTime = readTime
	c.prevTempDeriv = tempDeriv
	if co == boundedCo {
		c.prevTempInteg = tempInteg
	}
}

// TemperatureFanManager manages multiple temperature fan instances.
type TemperatureFanManager struct {
	rt   *runtime
	fans map[string]*TemperatureFan
	mu   sync.RWMutex
}

// newTemperatureFanManager creates a new temperature fan manager.
func newTemperatureFanManager(rt *runtime) *TemperatureFanManager {
	return &TemperatureFanManager{
		rt:   rt,
		fans: make(map[string]*TemperatureFan),
	}
}

// Register registers a new temperature fan.
func (tfm *TemperatureFanManager) Register(cfg TemperatureFanConfig) (*TemperatureFan, error) {
	tfm.mu.Lock()
	defer tfm.mu.Unlock()

	if _, exists := tfm.fans[cfg.Name]; exists {
		return nil, fmt.Errorf("temperature_fan %s already registered", cfg.Name)
	}

	tf, err := newTemperatureFan(tfm.rt, cfg)
	if err != nil {
		return nil, err
	}
	tfm.fans[cfg.Name] = tf
	return tf, nil
}

// Get returns a temperature fan by name.
func (tfm *TemperatureFanManager) Get(name string) *TemperatureFan {
	tfm.mu.RLock()
	defer tfm.mu.RUnlock()
	return tfm.fans[name]
}

// List returns all registered temperature fans.
func (tfm *TemperatureFanManager) List() []*TemperatureFan {
	tfm.mu.RLock()
	defer tfm.mu.RUnlock()
	result := make([]*TemperatureFan, 0, len(tfm.fans))
	for _, tf := range tfm.fans {
		result = append(result, tf)
	}
	return result
}
