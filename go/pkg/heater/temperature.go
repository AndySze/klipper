// Package heater provides temperature reading and heater control functionality.
package heater

import (
	"errors"
	"math"
	"sync"
)

// Common errors
var (
	ErrSensorFault   = errors.New("heater: sensor fault detected")
	ErrSensorOpen    = errors.New("heater: sensor open circuit")
	ErrSensorShort   = errors.New("heater: sensor short circuit")
	ErrOutOfRange    = errors.New("heater: temperature out of range")
	ErrNoReadings    = errors.New("heater: no temperature readings")
)

// ThermistorParams holds parameters for NTC thermistor calculation.
// Uses the Steinhart-Hart equation or simplified B-parameter model.
type ThermistorParams struct {
	// Beta coefficient (for simplified B-parameter model)
	Beta float64

	// Resistance at reference temperature (T0)
	R0 float64

	// Reference temperature in Kelvin
	T0 float64

	// Pullup resistor value
	Pullup float64

	// Inline resistor value (optional)
	InlineR float64

	// Steinhart-Hart coefficients (for advanced model)
	SteinhartA float64
	SteinhartB float64
	SteinhartC float64
}

// DefaultThermistorParams returns common NTC 100K thermistor parameters.
func DefaultThermistorParams() ThermistorParams {
	return ThermistorParams{
		Beta:   3950,
		R0:     100000, // 100K ohms at 25°C
		T0:     298.15, // 25°C in Kelvin
		Pullup: 4700,   // 4.7K pullup resistor
	}
}

// Temperature sensor types
type SensorType int

const (
	SensorNTC SensorType = iota
	SensorPT100
	SensorPT1000
	SensorThermocouple
	SensorMAX31855
	SensorMAX31856
	SensorMAX6675
)

// TemperatureSensor represents a temperature sensor.
type TemperatureSensor struct {
	mu sync.RWMutex

	// Sensor configuration
	name       string
	params     ThermistorParams
	adcMax     float64 // Maximum ADC value (e.g., 4095 for 12-bit)

	// Current state
	lastTemp   float64
	lastADC    float64
	minTemp    float64
	maxTemp    float64
	sampleTime float64

	// Averaging
	samples    []float64
	sampleIdx  int
	avgCount   int

	// Fault detection
	faultCount int
	maxFaults  int
}

// SensorConfig holds configuration for a temperature sensor.
type SensorConfig struct {
	Name       string
	Params     ThermistorParams
	ADCMax     float64
	MinTemp    float64
	MaxTemp    float64
	AvgCount   int
	MaxFaults  int
}

// DefaultSensorConfig returns a default sensor configuration.
func DefaultSensorConfig() SensorConfig {
	return SensorConfig{
		Name:      "sensor",
		Params:    DefaultThermistorParams(),
		ADCMax:    4095, // 12-bit ADC
		MinTemp:   -10,
		MaxTemp:   400,
		AvgCount:  8,
		MaxFaults: 3,
	}
}

// NewTemperatureSensor creates a new temperature sensor.
func NewTemperatureSensor(cfg SensorConfig) *TemperatureSensor {
	if cfg.AvgCount <= 0 {
		cfg.AvgCount = 1
	}
	if cfg.MaxFaults <= 0 {
		cfg.MaxFaults = 3
	}

	return &TemperatureSensor{
		name:      cfg.Name,
		params:    cfg.Params,
		adcMax:    cfg.ADCMax,
		minTemp:   cfg.MinTemp,
		maxTemp:   cfg.MaxTemp,
		samples:   make([]float64, cfg.AvgCount),
		avgCount:  cfg.AvgCount,
		maxFaults: cfg.MaxFaults,
	}
}

// UpdateADC processes a new ADC reading and updates the temperature.
func (ts *TemperatureSensor) UpdateADC(adcValue float64, sampleTime float64) error {
	ts.mu.Lock()
	defer ts.mu.Unlock()

	ts.lastADC = adcValue
	ts.sampleTime = sampleTime

	// Check for open/short circuit
	if adcValue <= 0 {
		ts.faultCount++
		if ts.faultCount >= ts.maxFaults {
			return ErrSensorShort
		}
		return nil
	}
	if adcValue >= ts.adcMax {
		ts.faultCount++
		if ts.faultCount >= ts.maxFaults {
			return ErrSensorOpen
		}
		return nil
	}

	// Convert ADC to temperature
	temp, err := ts.adcToTemp(adcValue)
	if err != nil {
		ts.faultCount++
		if ts.faultCount >= ts.maxFaults {
			return err
		}
		return nil
	}

	// Check temperature range
	if temp < ts.minTemp || temp > ts.maxTemp {
		ts.faultCount++
		if ts.faultCount >= ts.maxFaults {
			return ErrOutOfRange
		}
		return nil
	}

	// Reset fault count on good reading
	ts.faultCount = 0

	// Add to averaging buffer
	ts.samples[ts.sampleIdx] = temp
	ts.sampleIdx = (ts.sampleIdx + 1) % ts.avgCount

	// Calculate average
	var sum float64
	var count int
	for _, s := range ts.samples {
		if s != 0 { // Skip uninitialized samples
			sum += s
			count++
		}
	}
	if count > 0 {
		ts.lastTemp = sum / float64(count)
	} else {
		ts.lastTemp = temp
	}

	return nil
}

// adcToTemp converts an ADC value to temperature using the thermistor equation.
func (ts *TemperatureSensor) adcToTemp(adcValue float64) (float64, error) {
	params := ts.params

	// Calculate resistance from ADC value
	// Voltage divider: ADC = Vref * R / (R + Pullup)
	// Solving for R: R = Pullup * ADC / (ADCmax - ADC)
	if adcValue >= ts.adcMax {
		return 0, ErrSensorOpen
	}

	r := params.Pullup * adcValue / (ts.adcMax - adcValue)
	if params.InlineR > 0 {
		r -= params.InlineR
	}

	if r <= 0 {
		return 0, ErrSensorShort
	}

	// Use Steinhart-Hart if coefficients are set, otherwise use B-parameter
	var tempK float64
	if params.SteinhartA != 0 && params.SteinhartB != 0 && params.SteinhartC != 0 {
		// Steinhart-Hart equation: 1/T = A + B*ln(R) + C*ln(R)^3
		lnR := math.Log(r)
		lnR3 := lnR * lnR * lnR
		tempK = 1.0 / (params.SteinhartA + params.SteinhartB*lnR + params.SteinhartC*lnR3)
	} else {
		// Simplified B-parameter equation: 1/T = 1/T0 + (1/B)*ln(R/R0)
		tempK = 1.0 / (1.0/params.T0 + math.Log(r/params.R0)/params.Beta)
	}

	// Convert Kelvin to Celsius
	tempC := tempK - 273.15

	return tempC, nil
}

// GetTemperature returns the current temperature in Celsius.
func (ts *TemperatureSensor) GetTemperature() float64 {
	ts.mu.RLock()
	defer ts.mu.RUnlock()
	return ts.lastTemp
}

// GetLastADC returns the last raw ADC value.
func (ts *TemperatureSensor) GetLastADC() float64 {
	ts.mu.RLock()
	defer ts.mu.RUnlock()
	return ts.lastADC
}

// GetSampleTime returns the time of the last sample.
func (ts *TemperatureSensor) GetSampleTime() float64 {
	ts.mu.RLock()
	defer ts.mu.RUnlock()
	return ts.sampleTime
}

// GetName returns the sensor name.
func (ts *TemperatureSensor) GetName() string {
	return ts.name
}

// IsFaulted returns true if the sensor has a fault.
func (ts *TemperatureSensor) IsFaulted() bool {
	ts.mu.RLock()
	defer ts.mu.RUnlock()
	return ts.faultCount >= ts.maxFaults
}

// Reset resets the sensor state.
func (ts *TemperatureSensor) Reset() {
	ts.mu.Lock()
	defer ts.mu.Unlock()

	ts.lastTemp = 0
	ts.lastADC = 0
	ts.faultCount = 0
	ts.sampleIdx = 0
	for i := range ts.samples {
		ts.samples[i] = 0
	}
}

// Status holds sensor status information.
type SensorStatus struct {
	Name        string
	Temperature float64
	RawADC      float64
	SampleTime  float64
	IsFaulted   bool
	FaultCount  int
}

// GetStatus returns the current sensor status.
func (ts *TemperatureSensor) GetStatus() SensorStatus {
	ts.mu.RLock()
	defer ts.mu.RUnlock()

	return SensorStatus{
		Name:        ts.name,
		Temperature: ts.lastTemp,
		RawADC:      ts.lastADC,
		SampleTime:  ts.sampleTime,
		IsFaulted:   ts.faultCount >= ts.maxFaults,
		FaultCount:  ts.faultCount,
	}
}

// TempToADC converts a temperature to an expected ADC value.
// Useful for calibration and testing.
func (ts *TemperatureSensor) TempToADC(tempC float64) float64 {
	params := ts.params
	tempK := tempC + 273.15

	// Calculate resistance from temperature using B-parameter equation
	// 1/T = 1/T0 + (1/B)*ln(R/R0)
	// ln(R/R0) = B * (1/T - 1/T0)
	// R = R0 * exp(B * (1/T - 1/T0))
	r := params.R0 * math.Exp(params.Beta*(1.0/tempK-1.0/params.T0))

	if params.InlineR > 0 {
		r += params.InlineR
	}

	// Calculate ADC from resistance
	// R = Pullup * ADC / (ADCmax - ADC)
	// ADC * (R + Pullup) = R * ADCmax + ADC * Pullup
	// ADC = R * ADCmax / (R + Pullup)
	adc := r * ts.adcMax / (r + params.Pullup)

	return adc
}
