// Hall Filament Width Sensor - port of klippy/extras/hall_filament_width_sensor.py
//
// Support for filament width sensor using Hall effect sensors
//
// Copyright (C) 2019 Mustafa YILDIZ <mydiz@hotmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
	"time"
)

// ADC timing constants
const (
	hallADCReportTime  = 0.500
	hallADCSampleTime  = 0.03
	hallADCSampleCount = 15
)

// HallFilamentWidthSensor measures filament diameter using Hall effect sensors.
type HallFilamentWidthSensor struct {
	rt      *runtime
	name    string
	enabled bool

	// ADC pins
	pin1 string
	pin2 string

	// Calibration values
	calDia1  float64 // Calibration diameter 1
	calDia2  float64 // Calibration diameter 2
	rawDia1  int     // Raw ADC value for diameter 1
	rawDia2  int     // Raw ADC value for diameter 2

	// Measurement settings
	measurementIntervalMM float64
	nominalFilamentDia    float64
	measurementDelay      float64
	measurementMaxDiff    float64
	maxDiameter           float64
	minDiameter           float64

	// Runout detection
	runoutDiaMin float64
	runoutDiaMax float64

	// Current state
	diameter                    float64
	filamentWidth               float64
	lastFilamentWidthReading    int
	lastFilamentWidthReading2   int
	firstExtruderUpdatePosition float64
	isLogging                   bool

	// Filament position array [position, width]
	filamentArray []filamentReading

	// Runout helper
	runoutHelper *RunoutHelper

	mu sync.Mutex
}

// filamentReading stores a position and measured width.
type filamentReading struct {
	position float64
	width    float64
}

// HallFilamentWidthSensorConfig holds configuration for the sensor.
type HallFilamentWidthSensorConfig struct {
	Name                       string
	ADC1Pin                    string
	ADC2Pin                    string
	CalDia1                    float64
	CalDia2                    float64
	RawDia1                    int
	RawDia2                    int
	MeasurementIntervalMM      float64
	DefaultNominalFilamentDia  float64
	MeasurementDelay           float64
	MaxDifference              float64
	MinDiameter                float64
	MaxDiameter                float64
	Enable                     bool
	Logging                    bool
	UseCurrentDiaWhileDelay    bool
}

// DefaultHallFilamentWidthSensorConfig returns default configuration.
func DefaultHallFilamentWidthSensorConfig() HallFilamentWidthSensorConfig {
	return HallFilamentWidthSensorConfig{
		CalDia1:                   1.5,
		CalDia2:                   2.0,
		RawDia1:                   9500,
		RawDia2:                   10500,
		MeasurementIntervalMM:     10.0,
		DefaultNominalFilamentDia: 1.75,
		MeasurementDelay:          0.0,
		MaxDifference:             0.2,
		MinDiameter:               1.0,
		Enable:                    false,
		Logging:                   false,
	}
}

// newHallFilamentWidthSensor creates a new Hall filament width sensor.
func newHallFilamentWidthSensor(rt *runtime, cfg HallFilamentWidthSensorConfig) (*HallFilamentWidthSensor, error) {
	if cfg.ADC1Pin == "" || cfg.ADC2Pin == "" {
		return nil, fmt.Errorf("hall_filament_width_sensor: adc1 and adc2 pins are required")
	}
	if cfg.DefaultNominalFilamentDia <= 1.0 {
		return nil, fmt.Errorf("hall_filament_width_sensor: default_nominal_filament_diameter must be > 1.0")
	}

	maxDia := cfg.MaxDiameter
	if maxDia == 0 {
		maxDia = cfg.DefaultNominalFilamentDia + cfg.MaxDifference
	}
	minDia := cfg.MinDiameter
	if minDia == 0 {
		minDia = cfg.DefaultNominalFilamentDia - cfg.MaxDifference
	}

	sensor := &HallFilamentWidthSensor{
		rt:                    rt,
		name:                  cfg.Name,
		enabled:               cfg.Enable,
		pin1:                  cfg.ADC1Pin,
		pin2:                  cfg.ADC2Pin,
		calDia1:               cfg.CalDia1,
		calDia2:               cfg.CalDia2,
		rawDia1:               cfg.RawDia1,
		rawDia2:               cfg.RawDia2,
		measurementIntervalMM: cfg.MeasurementIntervalMM,
		nominalFilamentDia:    cfg.DefaultNominalFilamentDia,
		measurementDelay:      cfg.MeasurementDelay,
		measurementMaxDiff:    cfg.MaxDifference,
		maxDiameter:           maxDia,
		minDiameter:           minDia,
		runoutDiaMin:          minDia,
		runoutDiaMax:          maxDia,
		diameter:              cfg.DefaultNominalFilamentDia,
		filamentWidth:         cfg.DefaultNominalFilamentDia,
		isLogging:             cfg.Logging,
		filamentArray:         make([]filamentReading, 0),
	}

	// Create runout helper
	runoutCfg := RunoutHelperConfig{
		Name:          cfg.Name,
		InsertGcode:   nil,
		RunoutGcode:   []string{"M600"},
		PauseOnRunout: true,
		EventDelay:    3.0,
		PauseDelay:    0.5,
	}
	sensor.runoutHelper = newRunoutHelper(rt, runoutCfg)

	log.Printf("hall_filament_width_sensor: initialized '%s' (pins: %s, %s)",
		cfg.Name, cfg.ADC1Pin, cfg.ADC2Pin)
	return sensor, nil
}

// GetName returns the sensor name.
func (s *HallFilamentWidthSensor) GetName() string {
	return s.name
}

// IsEnabled returns whether the sensor is active.
func (s *HallFilamentWidthSensor) IsEnabled() bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.enabled
}

// Enable turns on the filament width sensor.
func (s *HallFilamentWidthSensor) Enable() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.enabled = true
	log.Printf("hall_filament_width_sensor '%s': enabled", s.name)
}

// Disable turns off the filament width sensor.
func (s *HallFilamentWidthSensor) Disable() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.enabled = false
	s.filamentArray = make([]filamentReading, 0)
	log.Printf("hall_filament_width_sensor '%s': disabled", s.name)
}

// UpdateADC1 processes ADC reading from first sensor.
func (s *HallFilamentWidthSensor) UpdateADC1(readTime float64, readValue float64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.lastFilamentWidthReading = int(readValue * 10000)
}

// UpdateADC2 processes ADC reading from second sensor and calculates diameter.
func (s *HallFilamentWidthSensor) UpdateADC2(readTime float64, readValue float64) {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.lastFilamentWidthReading2 = int(readValue * 10000)

	// Calculate diameter using linear interpolation
	rawSum := s.lastFilamentWidthReading + s.lastFilamentWidthReading2
	diameterNew := (s.calDia2-s.calDia1)/float64(s.rawDia2-s.rawDia1)*
		float64(rawSum-s.rawDia1) + s.calDia1

	// Apply smoothing filter
	s.diameter = (5.0*s.diameter + diameterNew) / 6.0
}

// UpdateFilamentArray adds a new reading to the filament array.
func (s *HallFilamentWidthSensor) UpdateFilamentArray(lastEPos float64) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if len(s.filamentArray) > 0 {
		// Get last reading position and calculate next reading position
		nextReadingPos := s.filamentArray[len(s.filamentArray)-1].position + s.measurementIntervalMM
		if nextReadingPos <= lastEPos+s.measurementDelay {
			s.filamentArray = append(s.filamentArray, filamentReading{
				position: lastEPos + s.measurementDelay,
				width:    s.diameter,
			})
			if s.isLogging {
				log.Printf("hall_filament_width_sensor '%s': width=%.3f mm", s.name, s.diameter)
			}
		}
	} else {
		// Add first item to array
		s.filamentArray = append(s.filamentArray, filamentReading{
			position: s.measurementDelay + lastEPos,
			width:    s.diameter,
		})
		s.firstExtruderUpdatePosition = s.measurementDelay + lastEPos
	}
}

// UpdateExtrudeFactor checks filament width and updates extrude factor.
func (s *HallFilamentWidthSensor) UpdateExtrudeFactor(eventTime time.Time, lastEPos float64) (extrudeFactor float64, ok bool) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.enabled {
		return 1.0, false
	}

	// Update filament array
	s.updateFilamentArrayLocked(lastEPos)

	// Check runout based on diameter range
	isPresent := s.runoutDiaMin <= s.diameter && s.diameter <= s.runoutDiaMax
	s.runoutHelper.NoteFilamentPresent(isPresent)

	// Does filament exist?
	if s.diameter <= 0.5 {
		s.filamentArray = make([]filamentReading, 0)
		return 1.0, true
	}

	if len(s.filamentArray) > 0 {
		pendingPosition := s.filamentArray[0].position
		if pendingPosition <= lastEPos {
			// Get first item in queue
			item := s.filamentArray[0]
			s.filamentArray = s.filamentArray[1:]
			s.filamentWidth = item.width
		}
	}

	// Calculate extrude factor based on filament width
	if s.filamentWidth >= s.minDiameter && s.filamentWidth <= s.maxDiameter {
		// Area ratio: nominal^2 / measured^2
		factor := (s.nominalFilamentDia * s.nominalFilamentDia) /
			(s.filamentWidth * s.filamentWidth)
		return factor, true
	}

	return 1.0, true
}

func (s *HallFilamentWidthSensor) updateFilamentArrayLocked(lastEPos float64) {
	if len(s.filamentArray) > 0 {
		nextReadingPos := s.filamentArray[len(s.filamentArray)-1].position + s.measurementIntervalMM
		if nextReadingPos <= lastEPos+s.measurementDelay {
			s.filamentArray = append(s.filamentArray, filamentReading{
				position: lastEPos + s.measurementDelay,
				width:    s.diameter,
			})
		}
	} else {
		s.filamentArray = append(s.filamentArray, filamentReading{
			position: s.measurementDelay + lastEPos,
			width:    s.diameter,
		})
		s.firstExtruderUpdatePosition = s.measurementDelay + lastEPos
	}
}

// GetDiameter returns the current measured diameter.
func (s *HallFilamentWidthSensor) GetDiameter() float64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.diameter
}

// GetRawValues returns the raw ADC readings.
func (s *HallFilamentWidthSensor) GetRawValues() (adc1, adc2, sum int) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.lastFilamentWidthReading, s.lastFilamentWidthReading2,
		s.lastFilamentWidthReading + s.lastFilamentWidthReading2
}

// ClearFilamentArray clears all stored measurements.
func (s *HallFilamentWidthSensor) ClearFilamentArray() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.filamentArray = make([]filamentReading, 0)
	log.Printf("hall_filament_width_sensor '%s': measurements cleared", s.name)
}

// SetLogging enables or disables logging.
func (s *HallFilamentWidthSensor) SetLogging(enable bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.isLogging = enable
}

// GetStatus returns the sensor status.
func (s *HallFilamentWidthSensor) GetStatus() map[string]any {
	s.mu.Lock()
	defer s.mu.Unlock()

	status := map[string]any{
		"Diameter":  s.diameter,
		"Raw":       s.lastFilamentWidthReading + s.lastFilamentWidthReading2,
		"is_active": s.enabled,
	}

	// Merge runout helper status
	for k, v := range s.runoutHelper.GetStatus() {
		status[k] = v
	}

	return status
}

// HallFilamentWidthSensorManager manages multiple Hall filament width sensors.
type HallFilamentWidthSensorManager struct {
	rt      *runtime
	sensors map[string]*HallFilamentWidthSensor
	mu      sync.RWMutex
}

// newHallFilamentWidthSensorManager creates a new sensor manager.
func newHallFilamentWidthSensorManager(rt *runtime) *HallFilamentWidthSensorManager {
	return &HallFilamentWidthSensorManager{
		rt:      rt,
		sensors: make(map[string]*HallFilamentWidthSensor),
	}
}

// Register registers a new Hall filament width sensor.
func (m *HallFilamentWidthSensorManager) Register(cfg HallFilamentWidthSensorConfig) (*HallFilamentWidthSensor, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	sensor, err := newHallFilamentWidthSensor(m.rt, cfg)
	if err != nil {
		return nil, err
	}
	m.sensors[cfg.Name] = sensor
	return sensor, nil
}

// Get returns a sensor by name.
func (m *HallFilamentWidthSensorManager) Get(name string) *HallFilamentWidthSensor {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.sensors[name]
}

// GetAll returns all registered sensors.
func (m *HallFilamentWidthSensorManager) GetAll() []*HallFilamentWidthSensor {
	m.mu.RLock()
	defer m.mu.RUnlock()

	result := make([]*HallFilamentWidthSensor, 0, len(m.sensors))
	for _, s := range m.sensors {
		result = append(result, s)
	}
	return result
}
