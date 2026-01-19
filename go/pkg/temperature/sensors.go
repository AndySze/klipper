// Temperature sensor implementations for Klipper Go migration
// Copyright (C) 2026  Klipper Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package temperature

import (
	"fmt"
	"math"
)

// SensorTypes defines available sensor types
const (
	SensorTypeAD595     = "AD595"
	SensorectorPT100    = "PT100 INA826"
	SensorTypeThermistor = "Thermistor"
	SensorTypeMCU       = "temperature_mcu"
)

// AD595Sensor implements AD595 thermocouple amplifier
type AD595Sensor struct {
	sensor        *MCUSensor
	adc           ADCInterface
	config        *AD595Config
}

type AD595Config struct {
	VoltageSupply  float64  // Supply voltage (usually 5.0V)
	VoltageOffset  float64  // Offset at 0°C (usually 0.0V)
}

// NewAD595Sensor creates a new AD595 sensor
func NewAD595Sensor(name string, pin string, adc ADCInterface, config *AD595Config) (*AD595Sensor, error) {
	if config == nil {
		config = &AD595Config{
			VoltageSupply: 5.0,
			VoltageOffset: 0.0,
		}
	}

	// Create base MCU sensor
	mcuConfig := &MCUConfig{
		SensorMCU: "mcu",
	}
	sensor := NewMCUSensor(name, mcuConfig, adc)

	ad595 := &AD595Sensor{
		sensor: sensor,
		adc:    adc,
		config: config,
	}

	// Setup ADC callback - this processes raw ADC values
	adc.SetupADCCallback(ReportTime, ad595.adcCallback)

	return ad595, nil
}

// Forward Sensor interface methods to embedded MCUSensor
func (s *AD595Sensor) GetName() string {
	return s.sensor.GetName()
}

func (s *AD595Sensor) SetupCallback(callback TemperatureCallback) {
	s.sensor.SetupCallback(callback)
}

func (s *AD595Sensor) GetReportTimeDelta() float64 {
	return s.sensor.GetReportTimeDelta()
}

func (s *AD595Sensor) SetupMinMax(minTemp, maxTemp float64) error {
	return s.sensor.SetupMinMax(minTemp, maxTemp)
}

func (s *AD595Sensor) GetTemp(eventtime float64) (current, target float64) {
	return s.sensor.GetTemp(eventtime)
}

// adcCallback processes ADC readings and converts to temperature
func (s *AD595Sensor) adcCallback(readTime, adcValue float64) {
	// Convert ADC value to voltage (0-5V range)
	// ADC value is normalized 0.0-1.0
	voltage := adcValue * s.config.VoltageSupply

	// Convert to temperature
	// AD595: 10mV/°C linear output
	temp := (voltage - s.config.VoltageOffset) / 0.010

	// Store last temperature
	s.sensor.lastTemp = temp
	s.sensor.lastTempTime = readTime
}

// PT100Sensor implements PT100 RTD sensor with INA826 amplifier
type PT100Sensor struct {
	sensor        *MCUSensor
	adc           ADCInterface
	config        *PT100Config
}

type PT100Config struct {
	ReferenceResistor float64 // Reference resistor (usually 4300.0 ohms)
	R0                float64 // PT100 resistance at 0°C (100.0 ohms)
}

// NewPT100Sensor creates a new PT100 sensor
func NewPT100Sensor(name string, pin string, adc ADCInterface, config *PT100Config) (*PT100Sensor, error) {
	if config == nil {
		config = &PT100Config{
			ReferenceResistor: 4300.0,
			R0:                100.0,
		}
	}

	// Create base MCU sensor
	mcuConfig := &MCUConfig{
		SensorMCU: "mcu",
	}
	sensor := NewMCUSensor(name, mcuConfig, adc)

	pt100 := &PT100Sensor{
		sensor: sensor,
		adc:    adc,
		config: config,
	}

	// Setup ADC callback - this processes raw ADC values
	adc.SetupADCCallback(ReportTime, pt100.adcCallback)

	return pt100, nil
}

// Forward Sensor interface methods to embedded MCUSensor
func (s *PT100Sensor) GetName() string {
	return s.sensor.GetName()
}

func (s *PT100Sensor) SetupCallback(callback TemperatureCallback) {
	s.sensor.SetupCallback(callback)
}

func (s *PT100Sensor) GetReportTimeDelta() float64 {
	return s.sensor.GetReportTimeDelta()
}

func (s *PT100Sensor) SetupMinMax(minTemp, maxTemp float64) error {
	return s.sensor.SetupMinMax(minTemp, maxTemp)
}

func (s *PT100Sensor) GetTemp(eventtime float64) (current, target float64) {
	return s.sensor.GetTemp(eventtime)
}

// adcCallback processes ADC readings and converts to temperature
func (s *PT100Sensor) adcCallback(readTime, adcValue float64) {
	// PT100 calculation using standard formula
	// adcValue is normalized 0.0-1.0, representing resistance ratio
	// R = adcValue * ReferenceResistor
	resistance := adcValue * s.config.ReferenceResistor

	// PT100 formula: R = R0 * (1 + alpha * T)
	// Solving for T: T = (R / R0 - 1) / alpha
	// where alpha = 0.00385 (European standard)
	alpha := 0.00385
	temp := (resistance/s.config.R0 - 1.0) / alpha

	// Store last temperature
	s.sensor.lastTemp = temp
	s.sensor.lastTempTime = readTime
}

// ThermistorSensor implements thermistor sensor using Beta parameter equation
type ThermistorSensor struct {
	sensor        *MCUSensor
	adc           ADCInterface
	config        *ThermistorConfig
}

type ThermistorConfig struct {
	Beta           float64  // Beta parameter
	R0             float64  // Resistance at T0 (25°C)
	T0             float64  // Reference temperature (usually 25°C)
	PullupResistor float64  // Pullup resistor (usually 4700 ohms)
}

// NewThermistorSensor creates a new thermistor sensor
func NewThermistorSensor(name string, pin string, adc ADCInterface, config *ThermistorConfig) (*ThermistorSensor, error) {
	if config == nil {
		config = &ThermistorConfig{
			Beta:           3950.0,
			R0:             100000.0,
			T0:             25.0,
			PullupResistor: 4700.0,
		}
	}

	// Create base MCU sensor
	mcuConfig := &MCUConfig{
		SensorMCU: "mcu",
	}
	sensor := NewMCUSensor(name, mcuConfig, adc)

	// Thermistor using Beta parameter equation:
	// 1/T = 1/T0 + (1/Beta) * ln(R/R0)
	// Where R = Pullup * (1 - adc_ratio) / adc_ratio
	// This is complex, so we use a simplified linear approximation for the range 0-250°C

	// For EPCOS 100K Beta 3950 thermistor:
	// At 25°C: 100K ohms
	// At 200°C: ~140 ohms
	// We'll use a simplified table-based interpolation

	thermistor := &ThermistorSensor{
		sensor: sensor,
		adc:    adc,
		config: config,
	}

	// Setup ADC callback - this processes raw ADC values
	adc.SetupADCCallback(ReportTime, thermistor.adcCallback)

	return thermistor, nil
}

// Forward Sensor interface methods to embedded MCUSensor
func (s *ThermistorSensor) GetName() string {
	return s.sensor.GetName()
}

func (s *ThermistorSensor) SetupCallback(callback TemperatureCallback) {
	s.sensor.SetupCallback(callback)
}

func (s *ThermistorSensor) GetReportTimeDelta() float64 {
	return s.sensor.GetReportTimeDelta()
}

func (s *ThermistorSensor) SetupMinMax(minTemp, maxTemp float64) error {
	return s.sensor.SetupMinMax(minTemp, maxTemp)
}

func (s *ThermistorSensor) GetTemp(eventtime float64) (current, target float64) {
	return s.sensor.GetTemp(eventtime)
}

// adcCallback processes ADC readings and converts to temperature using Beta equation
func (s *ThermistorSensor) adcCallback(readTime, adcValue float64) {
	// adcValue is normalized 0.0-1.0
	// For thermistor with pullup: R = Pullup * adcValue / (1 - adcValue)
	// Or with voltage divider: R = Pullup * (1 - adcValue) / adcValue (if thermistor is on bottom)

	// Avoid division by zero
	if adcValue <= 0.0 || adcValue >= 1.0 {
		if adcValue <= 0.0 {
			s.sensor.lastTemp = 300.0 // Very high temperature (open circuit)
		} else {
			s.sensor.lastTemp = 0.0 // Very low temperature (short circuit)
		}
		s.sensor.lastTempTime = readTime
		return
	}

	// Calculate resistance (assuming thermistor on bottom of voltage divider)
	resistance := s.config.PullupResistor * (1.0 - adcValue) / adcValue

	// Beta equation: 1/T = 1/T0 + (1/Beta) * ln(R/R0)
	// T0 is in Kelvin
	t0Kelvin := s.config.T0 + 273.15
	lnR := math.Log(resistance / s.config.R0)
	invT := 1.0/t0Kelvin + lnR/s.config.Beta

	// Temperature in Kelvin
	tempKelvin := 1.0 / invT

	// Convert to Celsius
	temp := tempKelvin - 273.15

	// Store last temperature
	s.sensor.lastTemp = temp
	s.sensor.lastTempTime = readTime
}

// ThermistorTableSensor implements thermistor sensor using lookup table
type ThermistorTableSensor struct {
	sensor        *MCUSensor
	adc           ADCInterface
	config        *ThermistorTableConfig
	table          []ThermistorEntry
}

type ThermistorTableConfig struct {
	PullupResistor float64
	Table          []ThermistorEntry
}

type ThermistorEntry struct {
	Temp       float64
	Resistance float64
}

// NewThermistorTableSensor creates a new table-based thermistor sensor
func NewThermistorTableSensor(name string, pin string, adc ADCInterface, config *ThermistorTableConfig) (*ThermistorTableSensor, error) {
	if config == nil || len(config.Table) == 0 {
		return nil, fmt.Errorf("thermistor table is required")
	}

	// Create base MCU sensor
	mcuConfig := &MCUConfig{
		SensorMCU: "mcu",
	}
	sensor := NewMCUSensor(name, mcuConfig, adc)

	thermistor := &ThermistorTableSensor{
		sensor: sensor,
		adc:    adc,
		config: config,
		table:  config.Table,
	}

	// Setup ADC callback - this processes raw ADC values
	adc.SetupADCCallback(ReportTime, thermistor.adcCallback)

	return thermistor, nil
}

// Forward Sensor interface methods to embedded MCUSensor
func (s *ThermistorTableSensor) GetName() string {
	return s.sensor.GetName()
}

func (s *ThermistorTableSensor) SetupCallback(callback TemperatureCallback) {
	s.sensor.SetupCallback(callback)
}

func (s *ThermistorTableSensor) GetReportTimeDelta() float64 {
	return s.sensor.GetReportTimeDelta()
}

func (s *ThermistorTableSensor) SetupMinMax(minTemp, maxTemp float64) error {
	return s.sensor.SetupMinMax(minTemp, maxTemp)
}

func (s *ThermistorTableSensor) GetTemp(eventtime float64) (current, target float64) {
	return s.sensor.GetTemp(eventtime)
}

// adcCallback processes ADC readings and converts to temperature using table lookup
func (s *ThermistorTableSensor) adcCallback(readTime, adcValue float64) {
	// adcValue is normalized 0.0-1.0
	// Calculate resistance: R = Pullup * (1 - adcValue) / adcValue

	if adcValue <= 0.0 {
		s.sensor.lastTemp = 300.0 // Max temperature for open circuit
		s.sensor.lastTempTime = readTime
		return
	}

	if adcValue >= 1.0 {
		s.sensor.lastTemp = 0.0 // Min temperature for short circuit
		s.sensor.lastTempTime = readTime
		return
	}

	resistance := s.config.PullupResistor * ((1.0 - adcValue) / adcValue)

	// Interpolate from table
	temp := s.interpolate(resistance)

	// Store last temperature
	s.sensor.lastTemp = temp
	s.sensor.lastTempTime = readTime
}

func (s *ThermistorTableSensor) interpolate(resistance float64) float64 {
	// Find surrounding table entries and interpolate
	for i := 0; i < len(s.table)-1; i++ {
		entry1 := s.table[i]
		entry2 := s.table[i+1]

		if resistance >= entry1.Resistance && resistance <= entry2.Resistance {
			// Linear interpolation
			r1 := entry1.Resistance
			r2 := entry2.Resistance
			t1 := entry1.Temp
			t2 := entry2.Temp

			ratio := (resistance - r1) / (r2 - r1)
			return t1 + ratio*(t2-t1)
		}
	}

	// Extrapolate if outside table
	if len(s.table) == 0 {
		return 25.0
	}

	if resistance < s.table[0].Resistance {
		// Below minimum - extrapolate
		entry := s.table[0]
		return entry.Temp - 10.0
	}

	// Above maximum - extrapolate
	entry := s.table[len(s.table)-1]
	return entry.Temp + 10.0
}
