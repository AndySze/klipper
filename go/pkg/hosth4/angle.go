// Angle - port of klippy/extras/angle.py
//
// Support for reading rotary encoder position
//
// Copyright (C) 2021 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"math"
	"sync"
)

// Angle sensor constants.
const (
	angleReportTime = 0.5
	angleSampleTime = 0.001
)

// AngleSensor represents a rotary angle sensor.
type AngleSensor struct {
	rt           *runtime
	name         string
	samplePeriod float64
	angleOffset  float64
	lastAngle    float64
	lastTime     float64
	queryActive  bool
	samples      []AngleSample
	callback     func(time, angle float64)
	mu           sync.Mutex
}

// AngleSample holds a single angle measurement.
type AngleSample struct {
	Time  float64
	Angle float64
}

// AngleSensorConfig holds configuration for an angle sensor.
type AngleSensorConfig struct {
	Name         string
	SamplePeriod float64
	AngleOffset  float64
}

// DefaultAngleSensorConfig returns default angle sensor configuration.
func DefaultAngleSensorConfig() AngleSensorConfig {
	return AngleSensorConfig{
		SamplePeriod: angleSampleTime,
		AngleOffset:  0,
	}
}

// newAngleSensor creates a new angle sensor.
func newAngleSensor(rt *runtime, cfg AngleSensorConfig) (*AngleSensor, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("angle: name is required")
	}
	if cfg.SamplePeriod <= 0 {
		cfg.SamplePeriod = angleSampleTime
	}

	as := &AngleSensor{
		rt:           rt,
		name:         cfg.Name,
		samplePeriod: cfg.SamplePeriod,
		angleOffset:  cfg.AngleOffset,
		samples:      make([]AngleSample, 0),
	}

	log.Printf("angle: initialized '%s' sample_period=%.6f", cfg.Name, cfg.SamplePeriod)
	return as, nil
}

// GetName returns the sensor name.
func (as *AngleSensor) GetName() string {
	return as.name
}

// SetCallback sets the measurement callback.
func (as *AngleSensor) SetCallback(cb func(time, angle float64)) {
	as.mu.Lock()
	defer as.mu.Unlock()
	as.callback = cb
}

// StartMeasurements starts angle measurements.
func (as *AngleSensor) StartMeasurements() error {
	as.mu.Lock()
	defer as.mu.Unlock()

	if as.queryActive {
		return fmt.Errorf("angle: measurements already active")
	}

	as.samples = make([]AngleSample, 0)
	as.queryActive = true

	log.Printf("angle '%s': starting measurements", as.name)
	return nil
}

// StopMeasurements stops angle measurements.
func (as *AngleSensor) StopMeasurements() error {
	as.mu.Lock()
	defer as.mu.Unlock()

	if !as.queryActive {
		return nil
	}

	as.queryActive = false
	log.Printf("angle '%s': stopped measurements, %d samples collected",
		as.name, len(as.samples))
	return nil
}

// IsActive returns true if measurements are active.
func (as *AngleSensor) IsActive() bool {
	as.mu.Lock()
	defer as.mu.Unlock()
	return as.queryActive
}

// AddSample adds an angle measurement.
func (as *AngleSensor) AddSample(time float64, rawAngle int) {
	as.mu.Lock()
	defer as.mu.Unlock()

	if !as.queryActive {
		return
	}

	// Convert raw angle to degrees (assuming 16-bit resolution)
	angle := float64(rawAngle) * 360.0 / 65536.0
	angle += as.angleOffset

	// Normalize to 0-360
	angle = math.Mod(angle, 360.0)
	if angle < 0 {
		angle += 360.0
	}

	as.samples = append(as.samples, AngleSample{
		Time:  time,
		Angle: angle,
	})
	as.lastAngle = angle
	as.lastTime = time

	if as.callback != nil {
		as.callback(time, angle)
	}
}

// GetSamples returns all collected samples.
func (as *AngleSensor) GetSamples() []AngleSample {
	as.mu.Lock()
	defer as.mu.Unlock()

	result := make([]AngleSample, len(as.samples))
	copy(result, as.samples)
	return result
}

// ClearSamples clears all collected samples.
func (as *AngleSensor) ClearSamples() {
	as.mu.Lock()
	defer as.mu.Unlock()
	as.samples = make([]AngleSample, 0)
}

// GetLastAngle returns the last angle reading.
func (as *AngleSensor) GetLastAngle() float64 {
	as.mu.Lock()
	defer as.mu.Unlock()
	return as.lastAngle
}

// SetAngleOffset sets the angle offset.
func (as *AngleSensor) SetAngleOffset(offset float64) {
	as.mu.Lock()
	defer as.mu.Unlock()
	as.angleOffset = offset
}

// GetStatus returns the sensor status.
func (as *AngleSensor) GetStatus() map[string]any {
	as.mu.Lock()
	defer as.mu.Unlock()

	return map[string]any{
		"angle":        as.lastAngle,
		"sample_count": len(as.samples),
		"active":       as.queryActive,
	}
}

// CalculateVelocity calculates angular velocity from samples.
func (as *AngleSensor) CalculateVelocity() float64 {
	samples := as.GetSamples()
	if len(samples) < 2 {
		return 0
	}

	// Use last two samples for instantaneous velocity
	s1 := samples[len(samples)-2]
	s2 := samples[len(samples)-1]

	dt := s2.Time - s1.Time
	if dt <= 0 {
		return 0
	}

	// Calculate angle difference (handling wrap-around)
	dAngle := s2.Angle - s1.Angle
	if dAngle > 180 {
		dAngle -= 360
	} else if dAngle < -180 {
		dAngle += 360
	}

	return dAngle / dt // degrees per second
}
