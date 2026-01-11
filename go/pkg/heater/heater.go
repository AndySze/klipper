package heater

import (
	"errors"
	"math"
	"sync"
	"time"
)

// Heater control errors
var (
	ErrHeaterDisabled  = errors.New("heater: heater is disabled")
	ErrTargetTooHigh   = errors.New("heater: target temperature too high")
	ErrTargetTooLow    = errors.New("heater: target temperature too low")
	ErrThermalRunaway  = errors.New("heater: thermal runaway detected")
)

// PIDParams holds PID controller parameters.
type PIDParams struct {
	Kp float64 // Proportional gain
	Ki float64 // Integral gain
	Kd float64 // Derivative gain
}

// DefaultPIDParams returns sensible default PID parameters.
func DefaultPIDParams() PIDParams {
	return PIDParams{
		Kp: 0.05,
		Ki: 0.005,
		Kd: 0.25,
	}
}

// HeaterConfig holds configuration for a heater.
type HeaterConfig struct {
	Name        string
	Sensor      *TemperatureSensor
	PID         PIDParams
	MaxTemp     float64
	MinTemp     float64
	MaxPower    float64 // Maximum PWM duty cycle (0-1)
	PWMCycleTime time.Duration
	SoftPWM     bool // Use soft PWM (for slow-switching heaters)
}

// DefaultHeaterConfig returns a default heater configuration.
func DefaultHeaterConfig() HeaterConfig {
	return HeaterConfig{
		Name:         "heater",
		MaxTemp:      300,
		MinTemp:      0,
		MaxPower:     1.0,
		PWMCycleTime: 100 * time.Millisecond,
		SoftPWM:      false,
	}
}

// Heater represents a controlled heater with PID.
type Heater struct {
	mu sync.RWMutex

	// Configuration
	name        string
	sensor      *TemperatureSensor
	pid         PIDParams
	maxTemp     float64
	minTemp     float64
	maxPower    float64
	pwmCycle    time.Duration
	softPWM     bool

	// State
	target      float64
	enabled     bool
	pwmDuty     float64

	// PID state
	prevError   float64
	integral    float64
	lastTime    time.Time

	// Callbacks
	setPWM      func(duty float64) error
}

// NewHeater creates a new heater controller.
func NewHeater(cfg HeaterConfig) *Heater {
	return &Heater{
		name:     cfg.Name,
		sensor:   cfg.Sensor,
		pid:      cfg.PID,
		maxTemp:  cfg.MaxTemp,
		minTemp:  cfg.MinTemp,
		maxPower: cfg.MaxPower,
		pwmCycle: cfg.PWMCycleTime,
		softPWM:  cfg.SoftPWM,
	}
}

// SetPWMCallback sets the callback for PWM duty cycle changes.
func (h *Heater) SetPWMCallback(fn func(duty float64) error) {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.setPWM = fn
}

// SetTarget sets the target temperature.
func (h *Heater) SetTarget(target float64) error {
	h.mu.Lock()
	defer h.mu.Unlock()

	if target > h.maxTemp {
		return ErrTargetTooHigh
	}
	if target < h.minTemp && target != 0 {
		return ErrTargetTooLow
	}

	h.target = target
	if target == 0 {
		h.enabled = false
		h.pwmDuty = 0
		h.integral = 0
		if h.setPWM != nil {
			h.setPWM(0)
		}
	} else {
		h.enabled = true
		h.lastTime = time.Now()
	}

	return nil
}

// GetTarget returns the current target temperature.
func (h *Heater) GetTarget() float64 {
	h.mu.RLock()
	defer h.mu.RUnlock()
	return h.target
}

// IsEnabled returns true if the heater is enabled.
func (h *Heater) IsEnabled() bool {
	h.mu.RLock()
	defer h.mu.RUnlock()
	return h.enabled
}

// GetPWM returns the current PWM duty cycle.
func (h *Heater) GetPWM() float64 {
	h.mu.RLock()
	defer h.mu.RUnlock()
	return h.pwmDuty
}

// Update should be called periodically to update the PID controller.
func (h *Heater) Update() error {
	h.mu.Lock()
	defer h.mu.Unlock()

	if !h.enabled {
		return nil
	}

	if h.sensor == nil {
		return errors.New("heater: no sensor configured")
	}

	// Get current temperature
	currentTemp := h.sensor.GetTemperature()

	// Calculate time delta
	now := time.Now()
	dt := now.Sub(h.lastTime).Seconds()
	h.lastTime = now

	if dt <= 0 {
		return nil
	}

	// PID calculation
	error := h.target - currentTemp

	// Proportional term
	p := h.pid.Kp * error

	// Integral term with anti-windup
	h.integral += error * dt
	// Limit integral to prevent windup
	maxIntegral := h.maxPower / h.pid.Ki
	if h.integral > maxIntegral {
		h.integral = maxIntegral
	} else if h.integral < -maxIntegral {
		h.integral = -maxIntegral
	}
	i := h.pid.Ki * h.integral

	// Derivative term
	d := 0.0
	if dt > 0 {
		d = h.pid.Kd * (error - h.prevError) / dt
	}
	h.prevError = error

	// Calculate output
	output := p + i + d

	// Clamp to valid range
	if output > h.maxPower {
		output = h.maxPower
	} else if output < 0 {
		output = 0
	}

	h.pwmDuty = output

	// Apply PWM
	if h.setPWM != nil {
		return h.setPWM(output)
	}

	return nil
}

// Disable turns off the heater.
func (h *Heater) Disable() error {
	h.mu.Lock()
	defer h.mu.Unlock()

	h.enabled = false
	h.target = 0
	h.pwmDuty = 0
	h.integral = 0

	if h.setPWM != nil {
		return h.setPWM(0)
	}
	return nil
}

// DisableHeater implements the HeaterDisabler interface for safety.
func (h *Heater) DisableHeater() error {
	return h.Disable()
}

// GetHeaterID implements the HeaterDisabler interface.
func (h *Heater) GetHeaterID() string {
	return h.name
}

// SetPID updates the PID parameters.
func (h *Heater) SetPID(params PIDParams) {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.pid = params
	// Reset integral to prevent issues from parameter change
	h.integral = 0
}

// GetPID returns the current PID parameters.
func (h *Heater) GetPID() PIDParams {
	h.mu.RLock()
	defer h.mu.RUnlock()
	return h.pid
}

// HeaterStatus holds heater status information.
type HeaterStatus struct {
	Name        string
	Target      float64
	Temperature float64
	PWMDuty     float64
	IsEnabled   bool
}

// GetStatus returns the current heater status.
func (h *Heater) GetStatus() HeaterStatus {
	h.mu.RLock()
	defer h.mu.RUnlock()

	temp := 0.0
	if h.sensor != nil {
		temp = h.sensor.GetTemperature()
	}

	return HeaterStatus{
		Name:        h.name,
		Target:      h.target,
		Temperature: temp,
		PWMDuty:     h.pwmDuty,
		IsEnabled:   h.enabled,
	}
}

// CheckThermalRunaway checks for thermal runaway conditions.
// Returns an error if thermal runaway is detected.
func (h *Heater) CheckThermalRunaway(hysteresis float64) error {
	h.mu.RLock()
	defer h.mu.RUnlock()

	if !h.enabled || h.target == 0 {
		return nil
	}

	if h.sensor == nil {
		return nil
	}

	temp := h.sensor.GetTemperature()
	if temp > h.target+hysteresis {
		return ErrThermalRunaway
	}

	return nil
}

// CheckHeatingProgress checks if the heater is making progress.
// Call this periodically during heating to detect failures.
type HeatingCheck struct {
	mu           sync.Mutex
	lastTemp     float64
	lastTime     time.Time
	checkTimeout time.Duration
}

// NewHeatingCheck creates a new heating progress checker.
func NewHeatingCheck(timeout time.Duration) *HeatingCheck {
	return &HeatingCheck{
		checkTimeout: timeout,
	}
}

// Check checks if heating is progressing.
// Returns an error if no temperature increase in timeout period while heating.
func (hc *HeatingCheck) Check(heater *Heater) error {
	hc.mu.Lock()
	defer hc.mu.Unlock()

	status := heater.GetStatus()

	// Only check if we're heating (target > current + some margin)
	if status.Target <= status.Temperature+1.0 {
		// Not actively heating, reset
		hc.lastTemp = status.Temperature
		hc.lastTime = time.Now()
		return nil
	}

	// Check if temperature is increasing
	if status.Temperature > hc.lastTemp+0.1 {
		// Making progress
		hc.lastTemp = status.Temperature
		hc.lastTime = time.Now()
		return nil
	}

	// Check timeout
	if time.Since(hc.lastTime) > hc.checkTimeout {
		return errors.New("heating failure: no temperature increase")
	}

	return nil
}

// Reset resets the heating check state.
func (hc *HeatingCheck) Reset() {
	hc.mu.Lock()
	defer hc.mu.Unlock()
	hc.lastTemp = 0
	hc.lastTime = time.Time{}
}

// SoftPWM implements software PWM for slow-switching heaters.
type SoftPWM struct {
	mu        sync.Mutex
	cycleTime time.Duration
	onTime    time.Duration
	startTime time.Time
}

// NewSoftPWM creates a new software PWM controller.
func NewSoftPWM(cycleTime time.Duration) *SoftPWM {
	return &SoftPWM{
		cycleTime: cycleTime,
	}
}

// SetDuty sets the PWM duty cycle (0-1).
func (sp *SoftPWM) SetDuty(duty float64) {
	sp.mu.Lock()
	defer sp.mu.Unlock()

	duty = math.Max(0, math.Min(1, duty))
	sp.onTime = time.Duration(float64(sp.cycleTime) * duty)
	sp.startTime = time.Now()
}

// ShouldBeOn returns true if the output should be on at the current time.
func (sp *SoftPWM) ShouldBeOn() bool {
	sp.mu.Lock()
	defer sp.mu.Unlock()

	if sp.onTime == 0 {
		return false
	}
	if sp.onTime >= sp.cycleTime {
		return true
	}

	elapsed := time.Since(sp.startTime)
	cyclePos := elapsed % sp.cycleTime

	return cyclePos < sp.onTime
}
