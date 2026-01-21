// TSL1401CL Filament Width Sensor - port of klippy/extras/tsl1401cl_filament_width_sensor.py
//
// Support for optical filament width sensor using TSL1401CL line sensor
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

// TSL1401CL timing constants
const (
	tslADCReportTime         = 0.500
	tslADCSampleTime         = 0.001
	tslADCSampleCount        = 8
	tslMeasurementIntervalMM = 10.0
)

// TSL1401CLFilamentWidthSensor measures filament diameter using an optical line sensor.
type TSL1401CLFilamentWidthSensor struct {
	rt      *runtime
	name    string
	enabled bool

	// ADC pin
	pin string

	// Measurement settings
	nominalFilamentDia    float64
	measurementDelay      float64
	measurementMaxDiff    float64
	maxDiameter           float64
	minDiameter           float64

	// Current state
	lastFilamentWidthReading float64

	// Filament position array [position, width]
	filamentArray []filamentReading

	mu sync.Mutex
}

// TSL1401CLFilamentWidthSensorConfig holds configuration for the sensor.
type TSL1401CLFilamentWidthSensorConfig struct {
	Name                      string
	Pin                       string
	DefaultNominalFilamentDia float64
	MeasurementDelay          float64
	MaxDifference             float64
}

// DefaultTSL1401CLFilamentWidthSensorConfig returns default configuration.
func DefaultTSL1401CLFilamentWidthSensorConfig() TSL1401CLFilamentWidthSensorConfig {
	return TSL1401CLFilamentWidthSensorConfig{
		DefaultNominalFilamentDia: 1.75,
		MeasurementDelay:          0.0,
		MaxDifference:             0.2,
	}
}

// newTSL1401CLFilamentWidthSensor creates a new TSL1401CL filament width sensor.
func newTSL1401CLFilamentWidthSensor(rt *runtime, cfg TSL1401CLFilamentWidthSensorConfig) (*TSL1401CLFilamentWidthSensor, error) {
	if cfg.Pin == "" {
		return nil, fmt.Errorf("tsl1401cl_filament_width_sensor: pin is required")
	}
	if cfg.DefaultNominalFilamentDia <= 1.0 {
		return nil, fmt.Errorf("tsl1401cl_filament_width_sensor: default_nominal_filament_diameter must be > 1.0")
	}
	if cfg.MeasurementDelay <= 0 {
		return nil, fmt.Errorf("tsl1401cl_filament_width_sensor: measurement_delay must be > 0")
	}

	maxDia := cfg.DefaultNominalFilamentDia + cfg.MaxDifference
	minDia := cfg.DefaultNominalFilamentDia - cfg.MaxDifference

	sensor := &TSL1401CLFilamentWidthSensor{
		rt:                 rt,
		name:               cfg.Name,
		enabled:            true,
		pin:                cfg.Pin,
		nominalFilamentDia: cfg.DefaultNominalFilamentDia,
		measurementDelay:   cfg.MeasurementDelay,
		measurementMaxDiff: cfg.MaxDifference,
		maxDiameter:        maxDia,
		minDiameter:        minDia,
		filamentArray:      make([]filamentReading, 0),
	}

	log.Printf("tsl1401cl_filament_width_sensor: initialized '%s' (pin: %s)", cfg.Name, cfg.Pin)
	return sensor, nil
}

// GetName returns the sensor name.
func (s *TSL1401CLFilamentWidthSensor) GetName() string {
	return s.name
}

// IsEnabled returns whether the sensor is active.
func (s *TSL1401CLFilamentWidthSensor) IsEnabled() bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.enabled
}

// Enable turns on the filament width sensor.
func (s *TSL1401CLFilamentWidthSensor) Enable() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.enabled = true
	log.Printf("tsl1401cl_filament_width_sensor '%s': enabled", s.name)
}

// Disable turns off the filament width sensor.
func (s *TSL1401CLFilamentWidthSensor) Disable() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.enabled = false
	s.filamentArray = make([]filamentReading, 0)
	log.Printf("tsl1401cl_filament_width_sensor '%s': disabled", s.name)
}

// UpdateADC processes ADC reading and converts to diameter.
func (s *TSL1401CLFilamentWidthSensor) UpdateADC(readTime float64, readValue float64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	// TSL1401CL sensor outputs voltage proportional to filament diameter
	// Multiply by 5 to convert ADC ratio (0-1) to mm (0-5mm range)
	s.lastFilamentWidthReading = readValue * 5.0
}

// UpdateFilamentArray adds a new reading to the filament array.
func (s *TSL1401CLFilamentWidthSensor) UpdateFilamentArray(lastEPos float64) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if len(s.filamentArray) > 0 {
		// Get last reading position and calculate next reading position
		nextReadingPos := s.filamentArray[len(s.filamentArray)-1].position + tslMeasurementIntervalMM
		if nextReadingPos <= lastEPos+s.measurementDelay {
			s.filamentArray = append(s.filamentArray, filamentReading{
				position: lastEPos + s.measurementDelay,
				width:    s.lastFilamentWidthReading,
			})
		}
	} else {
		// Add first item to array
		s.filamentArray = append(s.filamentArray, filamentReading{
			position: s.measurementDelay + lastEPos,
			width:    s.lastFilamentWidthReading,
		})
	}
}

// UpdateExtrudeFactor checks filament width and calculates extrude factor.
func (s *TSL1401CLFilamentWidthSensor) UpdateExtrudeFactor(eventTime time.Time, lastEPos float64) (extrudeFactor float64, ok bool) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.enabled {
		return 1.0, false
	}

	// Update filament array
	s.updateFilamentArrayLocked(lastEPos)

	// Does filament exist?
	if s.lastFilamentWidthReading <= 0.5 {
		s.filamentArray = make([]filamentReading, 0)
		return 1.0, true
	}

	if len(s.filamentArray) > 0 {
		pendingPosition := s.filamentArray[0].position
		if pendingPosition <= lastEPos {
			// Get first item in queue
			item := s.filamentArray[0]
			s.filamentArray = s.filamentArray[1:]
			filamentWidth := item.width

			// Calculate extrude factor based on filament width
			if filamentWidth >= s.minDiameter && filamentWidth <= s.maxDiameter {
				// Area ratio: nominal^2 / measured^2
				factor := (s.nominalFilamentDia * s.nominalFilamentDia) /
					(filamentWidth * filamentWidth)
				return factor, true
			}
		}
	}

	return 1.0, true
}

func (s *TSL1401CLFilamentWidthSensor) updateFilamentArrayLocked(lastEPos float64) {
	if len(s.filamentArray) > 0 {
		nextReadingPos := s.filamentArray[len(s.filamentArray)-1].position + tslMeasurementIntervalMM
		if nextReadingPos <= lastEPos+s.measurementDelay {
			s.filamentArray = append(s.filamentArray, filamentReading{
				position: lastEPos + s.measurementDelay,
				width:    s.lastFilamentWidthReading,
			})
		}
	} else {
		s.filamentArray = append(s.filamentArray, filamentReading{
			position: s.measurementDelay + lastEPos,
			width:    s.lastFilamentWidthReading,
		})
	}
}

// GetDiameter returns the current measured diameter.
func (s *TSL1401CLFilamentWidthSensor) GetDiameter() float64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.lastFilamentWidthReading
}

// ClearFilamentArray clears all stored measurements.
func (s *TSL1401CLFilamentWidthSensor) ClearFilamentArray() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.filamentArray = make([]filamentReading, 0)
	log.Printf("tsl1401cl_filament_width_sensor '%s': measurements cleared", s.name)
}

// GetStatus returns the sensor status.
func (s *TSL1401CLFilamentWidthSensor) GetStatus() map[string]any {
	s.mu.Lock()
	defer s.mu.Unlock()

	return map[string]any{
		"Diameter":  s.lastFilamentWidthReading,
		"is_active": s.enabled,
	}
}

// TSL1401CLFilamentWidthSensorManager manages multiple TSL1401CL sensors.
type TSL1401CLFilamentWidthSensorManager struct {
	rt      *runtime
	sensors map[string]*TSL1401CLFilamentWidthSensor
	mu      sync.RWMutex
}

// newTSL1401CLFilamentWidthSensorManager creates a new sensor manager.
func newTSL1401CLFilamentWidthSensorManager(rt *runtime) *TSL1401CLFilamentWidthSensorManager {
	return &TSL1401CLFilamentWidthSensorManager{
		rt:      rt,
		sensors: make(map[string]*TSL1401CLFilamentWidthSensor),
	}
}

// Register registers a new TSL1401CL filament width sensor.
func (m *TSL1401CLFilamentWidthSensorManager) Register(cfg TSL1401CLFilamentWidthSensorConfig) (*TSL1401CLFilamentWidthSensor, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	sensor, err := newTSL1401CLFilamentWidthSensor(m.rt, cfg)
	if err != nil {
		return nil, err
	}
	m.sensors[cfg.Name] = sensor
	return sensor, nil
}

// Get returns a sensor by name.
func (m *TSL1401CLFilamentWidthSensorManager) Get(name string) *TSL1401CLFilamentWidthSensor {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.sensors[name]
}

// GetAll returns all registered sensors.
func (m *TSL1401CLFilamentWidthSensorManager) GetAll() []*TSL1401CLFilamentWidthSensor {
	m.mu.RLock()
	defer m.mu.RUnlock()

	result := make([]*TSL1401CLFilamentWidthSensor, 0, len(m.sensors))
	for _, s := range m.sensors {
		result = append(result, s)
	}
	return result
}
