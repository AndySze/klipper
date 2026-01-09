// Example usage of the temperature package
// Copyright (C) 2026  Klipper Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package temperature_test

import (
	"fmt"

	"klipper-go-migration/pkg/temperature"
)

// MockPrinter implements PrinterInterface for testing
type MockPrinter struct {
	shutdown bool
}

func (m *MockPrinter) IsShutdown() bool {
	return m.shutdown
}

func (m *MockPrinter) CommandError(format string, args ...interface{}) error {
	return fmt.Errorf(format, args...)
}

// MockMCU implements MCUInterface for testing
type MockMCU struct{}

func (m *MockMCU) EstimatedPrintTime(eventtime float64) float64 {
	return eventtime + 0.1
}

// MockPWM implements PWMInterface for testing
type MockPWM struct {
	mcu     *MockMCU
	lastPWM float64
	lastTime float64
}

func (m *MockPWM) SetPWM(pwmTime, value float64) error {
	m.lastPWM = value
	m.lastTime = pwmTime
	fmt.Printf("  PWM: %.3f @ %.3f\n", value, pwmTime)
	return nil
}

func (m *MockPWM) SetupCycleTime(cycleTime float64) error {
	fmt.Printf("Setup PWM cycle time: %.3f\n", cycleTime)
	return nil
}

func (m *MockPWM) SetupMaxDuration(maxDuration float64) error {
	fmt.Printf("Setup PWM max duration: %.3f\n", maxDuration)
	return nil
}

func (m *MockPWM) GetMCU() temperature.MCUInterface {
	return m.mcu
}

// MockADC implements ADCInterface for testing
type MockADC struct {
	mcu        *MockMCU
	baseTemp   float64
	slope      float64
	callback   func(float64, float64)
}

func (m *MockADC) SetupADCCallback(reportTime float64, callback func(float64, float64)) {
	m.callback = callback
	// Simulate temperature updates
	go func() {
		for i := 0; i < 10; i++ {
			adcValue := 0.5 // Mock ADC value
			callback(float64(i)*reportTime, adcValue)
		}
	}()
}

func (m *MockADC) SetupADCSample(sampleTime float64, sampleCount int, options ...temperature.ADCSampleOption) error {
	fmt.Printf("Setup ADC sample: time=%.3f count=%d\n", sampleTime, sampleCount)
	return nil
}

func (m *MockADC) GetMCU() temperature.MCUInterface {
	return m.mcu
}

// ExampleHeaterCreation demonstrates creating a heater with PID control
func ExampleHeaterCreation() {
	printer := &MockPrinter{}
	mcu := &MockMCU{}
	pwm := &MockPWM{mcu: mcu}
	adc := &MockADC{mcu: mcu}

	// Create MCU sensor
	sensorConfig := &temperature.MCUConfig{
		SensorMCU: "mcu",
	}
	sensor := temperature.NewMCUSensor("temperature_sensor", sensorConfig, adc)
	sensor.SetCalibration(25.0, 100.0) // 25°C at 0.5V, 125°C at 1.5V

	// Create heater config with PID control
	heaterConfig := &temperature.HeaterConfig{
		Name:           "extruder",
		MinTemp:        0.0,
		MaxTemp:        250.0,
		MinExtrudeTemp: 170.0,
		MaxPower:       1.0,
		SmoothTime:     1.0,
		PWMCycleTime:   0.1,
		HeaterPin:      "PA0",
		ControlType:    "pid",
		PID_Kp:         22.2,
		PID_Ki:         1.08,
		PID_Kd:         114.0,
	}

	heater, err := temperature.NewHeater(heaterConfig, sensor, pwm, printer)
	if err != nil {
		fmt.Printf("Error creating heater: %v\n", err)
		return
	}

	// Set target temperature
	err = heater.SetTemp(210.0)
	if err != nil {
		fmt.Printf("Error setting temperature: %v\n", err)
		return
	}

	// Get status
	status := heater.GetStatus(0.0)
	fmt.Printf("Heater status: temp=%.1f target=%.0f power=%.3f\n",
		status["temperature"], status["target"], status["power"])
}

// ExampleBangBangControl demonstrates bang-bang control
func ExampleBangBangControl() {
	printer := &MockPrinter{}
	mcu := &MockMCU{}
	pwm := &MockPWM{mcu: mcu}
	adc := &MockADC{mcu: mcu}

	sensorConfig := &temperature.MCUConfig{
		SensorMCU: "mcu",
	}
	sensor := temperature.NewMCUSensor("heater_bed", sensorConfig, adc)
	sensor.SetCalibration(25.0, 100.0)

	// Create heater config with bang-bang control
	heaterConfig := &temperature.HeaterConfig{
		Name:           "heater_bed",
		MinTemp:        0.0,
		MaxTemp:        120.0,
		MinExtrudeTemp: 0.0,
		MaxPower:       1.0,
		SmoothTime:     1.0,
		PWMCycleTime:   0.1,
		HeaterPin:      "PA1",
		ControlType:    "watermark",
		MaxDelta:       2.0, // Turn on at target-2, off at target+2
	}

	heater, err := temperature.NewHeater(heaterConfig, sensor, pwm, printer)
	if err != nil {
		fmt.Printf("Error creating heater: %v\n", err)
		return
	}

	// Set target temperature to 60°C
	heater.SetTemp(60.0)

	// Check if heater is still busy
	busy := heater.CheckBusy(0.0)
	fmt.Printf("Heater busy: %v\n", busy)
}

// ExampleHeaterManager demonstrates managing multiple heaters
func ExampleHeaterManager() {
	printer := &MockPrinter{}
	mcu := &MockMCU{}

	manager := temperature.NewHeaterManager(printer)

	// Create extruder heater
	extruderSensor := temperature.NewMCUSensor("extruder_sensor",
		&temperature.MCUConfig{SensorMCU: "mcu"},
		&MockADC{mcu: mcu})
	extruderSensor.SetCalibration(25.0, 100.0)

	extruderConfig := &temperature.HeaterConfig{
		Name:        "extruder",
		MinTemp:     0.0,
		MaxTemp:     250.0,
		MaxPower:    1.0,
		SmoothTime:  1.0,
		PWMCycleTime: 0.1,
		ControlType: "pid",
		PID_Kp:      22.2,
		PID_Ki:      1.08,
		PID_Kd:      114.0,
	}

	_, err := manager.SetupHeater("extruder", extruderConfig, extruderSensor, &MockPWM{mcu: mcu})
	if err != nil {
		fmt.Printf("Error: %v\n", err)
		return
	}

	// Create heated bed
	bedSensor := temperature.NewMCUSensor("bed_sensor",
		&temperature.MCUConfig{SensorMCU: "mcu"},
		&MockADC{mcu: mcu})
	bedSensor.SetCalibration(25.0, 100.0)

	bedConfig := &temperature.HeaterConfig{
		Name:        "heater_bed",
		MinTemp:     0.0,
		MaxTemp:     120.0,
		MaxPower:    1.0,
		SmoothTime:  1.0,
		PWMCycleTime: 0.1,
		ControlType: "watermark",
		MaxDelta:    2.0,
	}

	_, err = manager.SetupHeater("heater_bed", bedConfig, bedSensor, &MockPWM{mcu: mcu})
	if err != nil {
		fmt.Printf("Error: %v\n", err)
		return
	}

	// Set temperatures
	manager.SetTemperature("extruder", 210.0, false)
	manager.SetTemperature("heater_bed", 60.0, false)

	// Get M105 response
	manager.HandleReady()
	response := manager.GetM105Response(0.0)
	fmt.Printf("M105: %s\n", response)

	// Get all heaters
	heaters := manager.GetAllHeaters()
	fmt.Printf("Available heaters: %v\n", heaters)
}
