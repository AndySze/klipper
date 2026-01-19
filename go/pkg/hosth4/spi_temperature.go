// SPI Temperature - port of klippy/extras/spi_temperature.py
//
// Support for SPI-based temperature sensors (MAX31855, MAX31856, MAX31865, MAX6675)
//
// Copyright (C) 2018 Kevin O'Connor <kevin@koconnor.net>
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

// SPI temperature sensor types.
const (
	SPITempMAX31855 = "MAX31855"
	SPITempMAX31856 = "MAX31856"
	SPITempMAX31865 = "MAX31865"
	SPITempMAX6675  = "MAX6675"
)

// MAX31865 constants.
const (
	max31865ConfigReg   = 0x00
	max31865RTDMsbReg   = 0x01
	max31865FaultReg    = 0x07
	max31865RefResistor = 430.0 // Reference resistor in Ohms
	max31865RTDNominal  = 100.0 // PT100 nominal resistance at 0°C
)

// MAX31856 constants.
const (
	max31856CR0Reg        = 0x00
	max31856CR1Reg        = 0x01
	max31856MaskReg       = 0x02
	max31856LinearizedReg = 0x0C
)

// SPITemperatureSensor represents an SPI temperature sensor.
type SPITemperatureSensor struct {
	rt           *runtime
	name         string
	sensorType   string
	spiPin       string
	reportTime   float64
	temperature  float64
	minTemp      float64
	maxTemp      float64
	fault        bool
	faultMessage string
	callback     func(readTime, temp float64)
	mu           sync.Mutex
}

// SPITemperatureSensorConfig holds configuration for SPI temperature sensor.
type SPITemperatureSensorConfig struct {
	Name       string
	SensorType string
	SPIPin     string
	ReportTime float64
	MinTemp    float64
	MaxTemp    float64
}

// DefaultSPITemperatureSensorConfig returns default SPI temperature sensor configuration.
func DefaultSPITemperatureSensorConfig() SPITemperatureSensorConfig {
	return SPITemperatureSensorConfig{
		SensorType: SPITempMAX31855,
		ReportTime: 0.3,
		MinTemp:    0,
		MaxTemp:    1800, // Thermocouple range
	}
}

// newSPITemperatureSensor creates a new SPI temperature sensor.
func newSPITemperatureSensor(rt *runtime, cfg SPITemperatureSensorConfig) (*SPITemperatureSensor, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("spi_temperature: name is required")
	}
	if cfg.SPIPin == "" {
		return nil, fmt.Errorf("spi_temperature: spi_pin is required")
	}

	sensor := &SPITemperatureSensor{
		rt:         rt,
		name:       cfg.Name,
		sensorType: cfg.SensorType,
		spiPin:     cfg.SPIPin,
		reportTime: cfg.ReportTime,
		minTemp:    cfg.MinTemp,
		maxTemp:    cfg.MaxTemp,
	}

	log.Printf("spi_temperature: initialized '%s' type=%s", cfg.Name, cfg.SensorType)
	return sensor, nil
}

// GetName returns the sensor name.
func (s *SPITemperatureSensor) GetName() string {
	return s.name
}

// GetSensorType returns the sensor type.
func (s *SPITemperatureSensor) GetSensorType() string {
	return s.sensorType
}

// SetCallback sets the measurement callback.
func (s *SPITemperatureSensor) SetCallback(cb func(readTime, temp float64)) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.callback = cb
}

// UpdateMeasurement updates the temperature value.
func (s *SPITemperatureSensor) UpdateMeasurement(readTime, temp float64) {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.temperature = temp
	s.fault = false
	s.faultMessage = ""

	if s.callback != nil {
		s.callback(readTime, temp)
	}
}

// SetFault sets a fault condition.
func (s *SPITemperatureSensor) SetFault(message string) {
	s.mu.Lock()
	defer s.mu.Unlock()

	s.fault = true
	s.faultMessage = message
}

// ProcessMAX31855 processes MAX31855 data (32-bit).
func (s *SPITemperatureSensor) ProcessMAX31855(data []byte) (float64, error) {
	if len(data) < 4 {
		return 0, fmt.Errorf("spi_temperature: insufficient MAX31855 data")
	}

	val := (uint32(data[0]) << 24) | (uint32(data[1]) << 16) | (uint32(data[2]) << 8) | uint32(data[3])

	// Check for faults
	if val&0x00010000 != 0 {
		if val&0x01 != 0 {
			return 0, fmt.Errorf("spi_temperature: MAX31855 open circuit")
		}
		if val&0x02 != 0 {
			return 0, fmt.Errorf("spi_temperature: MAX31855 short to ground")
		}
		if val&0x04 != 0 {
			return 0, fmt.Errorf("spi_temperature: MAX31855 short to VCC")
		}
		return 0, fmt.Errorf("spi_temperature: MAX31855 fault")
	}

	// Temperature is in upper 14 bits, signed, 0.25°C resolution
	tempVal := int32(val >> 18)
	if tempVal&0x2000 != 0 {
		tempVal |= -0x4000 // Sign extend
	}
	temp := float64(tempVal) * 0.25

	return temp, nil
}

// ProcessMAX6675 processes MAX6675 data (16-bit).
func (s *SPITemperatureSensor) ProcessMAX6675(data []byte) (float64, error) {
	if len(data) < 2 {
		return 0, fmt.Errorf("spi_temperature: insufficient MAX6675 data")
	}

	val := (uint16(data[0]) << 8) | uint16(data[1])

	// Check for open thermocouple
	if val&0x04 != 0 {
		return 0, fmt.Errorf("spi_temperature: MAX6675 open circuit")
	}

	// Temperature is in bits 3-14, 0.25°C resolution
	temp := float64((val>>3)&0x0FFF) * 0.25

	return temp, nil
}

// ProcessMAX31856 processes MAX31856 data.
func (s *SPITemperatureSensor) ProcessMAX31856(data []byte) (float64, error) {
	if len(data) < 3 {
		return 0, fmt.Errorf("spi_temperature: insufficient MAX31856 data")
	}

	// 19-bit signed value with 0.0078125°C resolution
	tempVal := int32(data[0])<<16 | int32(data[1])<<8 | int32(data[2])
	tempVal >>= 5 // Right-align 19-bit value
	if tempVal&0x20000 != 0 {
		tempVal |= -0x40000 // Sign extend
	}
	temp := float64(tempVal) * 0.0078125

	return temp, nil
}

// ProcessMAX31865 processes MAX31865 RTD data.
func (s *SPITemperatureSensor) ProcessMAX31865(data []byte, refResistor, rtdNominal float64) (float64, error) {
	if len(data) < 2 {
		return 0, fmt.Errorf("spi_temperature: insufficient MAX31865 data")
	}

	// Check for fault
	if data[1]&0x01 != 0 {
		return 0, fmt.Errorf("spi_temperature: MAX31865 fault detected")
	}

	// RTD resistance calculation
	rtd := float64((uint16(data[0])<<8 | uint16(data[1])) >> 1)
	resistance := rtd * refResistor / 32768.0

	// Callendar-Van Dusen equation for PT100
	// T = (R/R0 - 1) / (A + B*T) approximately
	// Using simplified formula for positive temperatures
	A := 3.9083e-3
	B := -5.775e-7
	R0 := rtdNominal

	// Solve quadratic: R = R0 * (1 + A*T + B*T^2)
	// Rearranging: B*T^2 + A*T + (1 - R/R0) = 0
	ratio := resistance / R0
	discriminant := A*A - 4*B*(1-ratio)
	if discriminant < 0 {
		return 0, fmt.Errorf("spi_temperature: MAX31865 invalid resistance")
	}

	temp := (-A + math.Sqrt(discriminant)) / (2 * B)

	return temp, nil
}

// GetTemperature returns the last temperature reading.
func (s *SPITemperatureSensor) GetTemperature() float64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.temperature
}

// GetStatus returns the sensor status.
func (s *SPITemperatureSensor) GetStatus() map[string]any {
	s.mu.Lock()
	defer s.mu.Unlock()

	return map[string]any{
		"temperature": s.temperature,
		"type":        s.sensorType,
		"fault":       s.fault,
		"fault_msg":   s.faultMessage,
	}
}

// GetMinReportTime returns the minimum report time.
func (s *SPITemperatureSensor) GetMinReportTime() float64 {
	return s.reportTime
}
