// Temperature sensor support for Klipper Go migration
// Copyright (C) 2026  Klipper Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package temperature

import (
	"fmt"
	"math"
)

const (
	// KelvinToCelsius is the conversion from Kelvin to Celsius
	KelvinToCelsius = -273.15

	// SampleTime is the time between ADC samples
	SampleTime = 0.001

	// SampleCount is the number of samples to average
	SampleCount = 8

	// ReportTime is the time between temperature reports
	ReportTime = 0.300
)

// Sensor is the interface for temperature sensors
type Sensor interface {
	// SetupCallback registers a callback for temperature updates
	SetupCallback(callback TemperatureCallback)

	// GetReportTimeDelta returns the time between temperature reports
	GetReportTimeDelta() float64

	// SetupMinMax sets the minimum and maximum valid temperatures
	SetupMinMax(minTemp, maxTemp float64) error

	// GetTemp returns the current temperature and target temperature
	GetTemp(eventtime float64) (current, target float64)

	// GetName returns the sensor name
	GetName() string
}

// TemperatureCallback is called when a new temperature reading is available
type TemperatureCallback func(readTime, temp float64)

// SensorFactory creates a sensor from configuration
type SensorFactory func(config interface{}) (Sensor, error)

// SensorRegistry manages available sensor types
type SensorRegistry struct {
	sensorTypes map[string]SensorFactory
}

// NewSensorRegistry creates a new sensor registry
func NewSensorRegistry() *SensorRegistry {
	return &SensorRegistry{
		sensorTypes: make(map[string]SensorFactory),
	}
}

// RegisterSensorType registers a sensor factory
func (sr *SensorRegistry) RegisterSensorType(sensorType string, factory SensorFactory) {
	sr.sensorTypes[sensorType] = factory
}

// CreateSensor creates a sensor instance
func (sr *SensorRegistry) CreateSensor(sensorType string, config interface{}) (Sensor, error) {
	factory, ok := sr.sensorTypes[sensorType]
	if !ok {
		return nil, fmt.Errorf("unknown temperature sensor '%s'", sensorType)
	}
	return factory(config)
}

// MCUConfig represents the configuration for MCU-based sensors
type MCUConfig struct {
	SensorMCU          string
	SensorTemperature1 *float64
	SensorADC1         *float64
	SensorTemperature2 *float64
	SensorADC2         *float64
}

// MCUSensor implements temperature sensor reading from MCU ADC
type MCUSensor struct {
	name               string
	baseTemperature    float64
	slope              float64
	minTemp            float64
	maxTemp            float64
	callback           TemperatureCallback
	lastTemp           float64
	lastTempTime       float64
	measuredMin        float64
	measuredMax        float64

	// Configuration
	config             *MCUConfig

	// ADC interface (to be connected)
	adc                ADCInterface
}

// ADCInterface represents the ADC connection to the MCU
type ADCInterface interface {
	SetupADCCallback(reportTime float64, callback func(readTime, readValue float64))
	SetupADCSample(sampleTime float64, sampleCount int, options ...ADCSampleOption) error
	GetMCU() MCUInterface
}

// MCUInterface represents the MCU connection
type MCUInterface interface {
	EstimatedPrintTime(eventtime float64) float64
}

// ADCSampleOption configures ADC sampling
type ADCSampleOption func(*ADCSampleConfig)

type ADCSampleConfig struct {
	MinVal           *float64
	MaxVal           *float64
	RangeCheckCount  *int
}

// NewMCUSensor creates a new MCU temperature sensor
func NewMCUSensor(name string, config *MCUConfig, adc ADCInterface) *MCUSensor {
	return &MCUSensor{
		name:         name,
		config:       config,
		adc:          adc,
		measuredMin:  math.MaxFloat64,
		measuredMax:  0,
	}
}

// GetName returns the sensor name
func (s *MCUSensor) GetName() string {
	return s.name
}

// SetupCallback registers a callback for temperature updates
func (s *MCUSensor) SetupCallback(callback TemperatureCallback) {
	s.callback = callback
	s.adc.SetupADCCallback(ReportTime, s.adcCallback)
}

// adcCallback processes ADC readings and converts to temperature
func (s *MCUSensor) adcCallback(readTime, readValue float64) {
	temp := s.baseTemperature + readValue*s.slope
	s.lastTemp = temp
	s.lastTempTime = readTime + SampleTime*float64(SampleCount)

	if temp < s.measuredMin {
		s.measuredMin = temp
	}
	if temp > s.measuredMax {
		s.measuredMax = temp
	}

	if s.callback != nil {
		s.callback(s.lastTempTime, temp)
	}
}

// GetReportTimeDelta returns the time between temperature reports
func (s *MCUSensor) GetReportTimeDelta() float64 {
	return ReportTime
}

// SetupMinMax sets the minimum and maximum valid temperatures
func (s *MCUSensor) SetupMinMax(minTemp, maxTemp float64) error {
	if minTemp < KelvinToCelsius {
		return fmt.Errorf("min_temp %.2f is below absolute zero", minTemp)
	}
	if maxTemp <= minTemp {
		return fmt.Errorf("max_temp %.2f must be greater than min_temp %.2f", maxTemp, minTemp)
	}

	s.minTemp = minTemp
	s.maxTemp = maxTemp
	return nil
}

// GetTemp returns the current temperature and target temperature
func (s *MCUSensor) GetTemp(eventtime float64) (current, target float64) {
	// Check if reading is stale
	if s.adc != nil && s.adc.GetMCU() != nil {
		estPrintTime := s.adc.GetMCU().EstimatedPrintTime(eventtime)
		quellTime := estPrintTime - 7.0 // QUELL_STALE_TIME
		if s.lastTempTime < quellTime {
			return 0.0, 0.0
		}
	}
	return s.lastTemp, 0.0
}

// SetCalibration sets the base temperature and slope for calibration
func (s *MCUSensor) SetCalibration(baseTemperature, slope float64) {
	s.baseTemperature = baseTemperature
	s.slope = slope
}

// CalcTemp calculates temperature from ADC value
func (s *MCUSensor) CalcTemp(adc float64) float64 {
	return s.baseTemperature + adc*s.slope
}

// CalcADC calculates ADC value from temperature
func (s *MCUSensor) CalcADC(temp float64) float64 {
	if s.slope == 0 {
		return 0
	}
	return (temp - s.baseTemperature) / s.slope
}

// CalcBase calculates base temperature from temperature and ADC
func (s *MCUSensor) CalcBase(temp, adc float64) float64 {
	return temp - adc*s.slope
}
