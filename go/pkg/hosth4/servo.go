// Servo - port of klippy/extras/servo.py
//
// Support for servos
//
// Copyright (C) 2017-2024 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"strconv"
	"sync"
)

const (
	servoSignalPeriod = 0.020   // 20ms PWM period (50Hz)
	rescheduleSlack   = 0.0005  // 500us slack for rescheduling
)

// Servo represents a servo motor controlled via PWM.
type Servo struct {
	rt   *runtime
	name string

	minWidth     float64 // minimum pulse width in seconds
	maxWidth     float64 // maximum pulse width in seconds
	maxAngle     float64 // maximum servo angle in degrees
	angleToWidth float64 // conversion factor from angle to width
	widthToValue float64 // conversion factor from width to PWM value

	lastValue float64 // last PWM value (0.0 to 1.0)
	mu        sync.Mutex

	// PWM pin (in a full implementation, this would be an MCU PWM pin)
	pin string
}

// ServoConfig holds configuration for a servo.
type ServoConfig struct {
	Name              string
	Pin               string
	MinimumPulseWidth float64 // default 0.001 (1ms)
	MaximumPulseWidth float64 // default 0.002 (2ms)
	MaximumServoAngle float64 // default 180
	InitialAngle      *float64
	InitialPulseWidth float64 // default 0
}

// DefaultServoConfig returns default servo configuration.
func DefaultServoConfig() ServoConfig {
	return ServoConfig{
		MinimumPulseWidth: 0.001,
		MaximumPulseWidth: 0.002,
		MaximumServoAngle: 180.0,
		InitialPulseWidth: 0.0,
	}
}

// newServo creates a new servo controller.
func newServo(rt *runtime, cfg ServoConfig) (*Servo, error) {
	if cfg.MinimumPulseWidth <= 0 || cfg.MinimumPulseWidth >= servoSignalPeriod {
		return nil, fmt.Errorf("minimum_pulse_width must be between 0 and %.3f", servoSignalPeriod)
	}
	if cfg.MaximumPulseWidth <= cfg.MinimumPulseWidth || cfg.MaximumPulseWidth >= servoSignalPeriod {
		return nil, fmt.Errorf("maximum_pulse_width must be between minimum_pulse_width and %.3f", servoSignalPeriod)
	}

	s := &Servo{
		rt:           rt,
		name:         cfg.Name,
		pin:          cfg.Pin,
		minWidth:     cfg.MinimumPulseWidth,
		maxWidth:     cfg.MaximumPulseWidth,
		maxAngle:     cfg.MaximumServoAngle,
		angleToWidth: (cfg.MaximumPulseWidth - cfg.MinimumPulseWidth) / cfg.MaximumServoAngle,
		widthToValue: 1.0 / servoSignalPeriod,
	}

	// Set initial PWM value
	if cfg.InitialAngle != nil {
		s.lastValue = s.getPwmFromAngle(*cfg.InitialAngle)
	} else {
		s.lastValue = s.getPwmFromPulseWidth(cfg.InitialPulseWidth)
	}

	return s, nil
}

// getPwmFromAngle converts an angle to a PWM value.
func (s *Servo) getPwmFromAngle(angle float64) float64 {
	// Clamp angle to valid range
	if angle < 0 {
		angle = 0
	} else if angle > s.maxAngle {
		angle = s.maxAngle
	}
	width := s.minWidth + angle*s.angleToWidth
	return width * s.widthToValue
}

// getPwmFromPulseWidth converts a pulse width to a PWM value.
func (s *Servo) getPwmFromPulseWidth(width float64) float64 {
	if width > 0 {
		// Clamp width to valid range
		if width < s.minWidth {
			width = s.minWidth
		} else if width > s.maxWidth {
			width = s.maxWidth
		}
	}
	return width * s.widthToValue
}

// SetAngle sets the servo to a specific angle.
func (s *Servo) SetAngle(angle float64) error {
	value := s.getPwmFromAngle(angle)
	return s.setPwm(value)
}

// SetPulseWidth sets the servo to a specific pulse width.
func (s *Servo) SetPulseWidth(width float64) error {
	value := s.getPwmFromPulseWidth(width)
	return s.setPwm(value)
}

// setPwm sets the PWM value.
func (s *Servo) setPwm(value float64) error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if value == s.lastValue {
		return nil // No change needed
	}

	// In a full implementation, this would:
	// 1. Get aligned print time
	// 2. Queue the PWM change via the MCU
	s.lastValue = value

	return nil
}

// GetValue returns the current PWM value.
func (s *Servo) GetValue() float64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.lastValue
}

// GetStatus returns the servo status.
func (s *Servo) GetStatus() map[string]any {
	return map[string]any{
		"value": s.GetValue(),
	}
}

// cmdSetServo handles the SET_SERVO command.
// SET_SERVO SERVO=<name> [ANGLE=<degrees>] [WIDTH=<seconds>]
func (s *Servo) cmdSetServo(args map[string]string) error {
	// Check for WIDTH parameter first
	if widthStr, ok := args["WIDTH"]; ok {
		width, err := strconv.ParseFloat(widthStr, 64)
		if err != nil {
			return fmt.Errorf("invalid WIDTH value: %w", err)
		}
		return s.SetPulseWidth(width)
	}

	// Otherwise, require ANGLE parameter
	angleStr, ok := args["ANGLE"]
	if !ok {
		return fmt.Errorf("missing ANGLE or WIDTH parameter")
	}
	angle, err := strconv.ParseFloat(angleStr, 64)
	if err != nil {
		return fmt.Errorf("invalid ANGLE value: %w", err)
	}
	return s.SetAngle(angle)
}

// ServoManager manages multiple servo instances.
type ServoManager struct {
	rt     *runtime
	servos map[string]*Servo
	mu     sync.RWMutex
}

// newServoManager creates a new servo manager.
func newServoManager(rt *runtime) *ServoManager {
	return &ServoManager{
		rt:     rt,
		servos: make(map[string]*Servo),
	}
}

// Register registers a new servo.
func (sm *ServoManager) Register(cfg ServoConfig) (*Servo, error) {
	sm.mu.Lock()
	defer sm.mu.Unlock()

	if _, exists := sm.servos[cfg.Name]; exists {
		return nil, fmt.Errorf("servo %s already registered", cfg.Name)
	}

	servo, err := newServo(sm.rt, cfg)
	if err != nil {
		return nil, err
	}
	sm.servos[cfg.Name] = servo
	return servo, nil
}

// Get returns a servo by name.
func (sm *ServoManager) Get(name string) *Servo {
	sm.mu.RLock()
	defer sm.mu.RUnlock()
	return sm.servos[name]
}

// List returns all registered servos.
func (sm *ServoManager) List() []*Servo {
	sm.mu.RLock()
	defer sm.mu.RUnlock()
	result := make([]*Servo, 0, len(sm.servos))
	for _, s := range sm.servos {
		result = append(result, s)
	}
	return result
}
