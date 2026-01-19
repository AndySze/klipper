// Temperature MCU - port of klippy/extras/temperature_mcu.py
//
// Support for micro-controller chip based temperature sensors
//
// Copyright (C) 2020-2024 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"strings"
	"sync"
)

const (
	mcuSampleTime       = 0.001
	mcuSampleCount      = 8
	mcuReportTime       = 0.300
	mcuRangeCheckCount  = 4
)

// PrinterTemperatureMCU reads temperature from MCU's internal temperature sensor.
type PrinterTemperatureMCU struct {
	rt              *runtime
	mcuName         string
	mcuType         string
	baseTemperature float64
	slope           float64
	minTemp         float64
	maxTemp         float64
	lastTemp        float64
	callback        func(readTime, temp float64)
	mu              sync.RWMutex

	// Optional calibration points
	temp1 *float64
	adc1  *float64
	temp2 *float64
	adc2  *float64
}

// TemperatureMCUConfig holds configuration for MCU temperature sensor.
type TemperatureMCUConfig struct {
	MCUName          string   // MCU name (default: "mcu")
	SensorTemp1      *float64 // Optional calibration temperature 1
	SensorADC1       *float64 // Optional calibration ADC value 1
	SensorTemp2      *float64 // Optional calibration temperature 2
	SensorADC2       *float64 // Optional calibration ADC value 2
}

// DefaultTemperatureMCUConfig returns default MCU temperature configuration.
func DefaultTemperatureMCUConfig() TemperatureMCUConfig {
	return TemperatureMCUConfig{
		MCUName: "mcu",
	}
}

// newPrinterTemperatureMCU creates a new MCU temperature sensor.
func newPrinterTemperatureMCU(rt *runtime, cfg TemperatureMCUConfig) (*PrinterTemperatureMCU, error) {
	ptm := &PrinterTemperatureMCU{
		rt:      rt,
		mcuName: cfg.MCUName,
		temp1:   cfg.SensorTemp1,
		adc1:    cfg.SensorADC1,
		temp2:   cfg.SensorTemp2,
		adc2:    cfg.SensorADC2,
	}

	log.Printf("temperature_mcu: initialized for MCU '%s'", cfg.MCUName)
	return ptm, nil
}

// Setup configures the MCU temperature sensor based on MCU type.
func (ptm *PrinterTemperatureMCU) Setup(mcuType string) error {
	ptm.mu.Lock()
	defer ptm.mu.Unlock()

	ptm.mcuType = mcuType

	// Configure based on MCU type
	switch {
	case strings.HasPrefix(mcuType, "rp2"):
		ptm.configRP2040()
	case strings.HasPrefix(mcuType, "sam3"):
		ptm.configSAM3()
	case strings.HasPrefix(mcuType, "sam4"):
		ptm.configSAM4()
	case strings.HasPrefix(mcuType, "same70"):
		ptm.configSAME70()
	case strings.HasPrefix(mcuType, "samd21"):
		ptm.configSAMD21()
	case strings.HasPrefix(mcuType, "samd51"), strings.HasPrefix(mcuType, "same5"):
		ptm.configSAMD51()
	case strings.HasPrefix(mcuType, "stm32f1"):
		ptm.configSTM32F1()
	case strings.HasPrefix(mcuType, "stm32f2"):
		ptm.configSTM32F2()
	case strings.HasPrefix(mcuType, "stm32f4"):
		ptm.configSTM32F4()
	case strings.HasPrefix(mcuType, "stm32f042"):
		ptm.configSTM32F0x2()
	case strings.HasPrefix(mcuType, "stm32f070"):
		ptm.configSTM32F070()
	case strings.HasPrefix(mcuType, "stm32f072"):
		ptm.configSTM32F0x2()
	case strings.HasPrefix(mcuType, "stm32g0"), strings.HasPrefix(mcuType, "stm32g4"),
		strings.HasPrefix(mcuType, "stm32l4"):
		ptm.configSTM32G0()
	case strings.HasPrefix(mcuType, "stm32h723"):
		ptm.configSTM32H723()
	case strings.HasPrefix(mcuType, "stm32h7"):
		ptm.configSTM32H7()
	default:
		return fmt.Errorf("MCU temperature not supported on %s", mcuType)
	}

	// Apply manual calibration override if provided
	if ptm.temp1 != nil && ptm.adc1 != nil {
		if ptm.temp2 != nil && ptm.adc2 != nil {
			ptm.slope = (*ptm.temp2 - *ptm.temp1) / (*ptm.adc2 - *ptm.adc1)
		}
		ptm.baseTemperature = ptm.calcBase(*ptm.temp1, *ptm.adc1)
	}

	log.Printf("mcu_temperature '%s' nominal base=%.6f slope=%.6f",
		ptm.mcuName, ptm.baseTemperature, ptm.slope)
	return nil
}

// MCU-specific configuration functions

func (ptm *PrinterTemperatureMCU) configRP2040() {
	ptm.slope = 3.3 / -0.001721
	ptm.baseTemperature = ptm.calcBase(27.0, 0.706/3.3)
}

func (ptm *PrinterTemperatureMCU) configSAM3() {
	ptm.slope = 3.3 / 0.002650
	ptm.baseTemperature = ptm.calcBase(27.0, 0.8/3.3)
}

func (ptm *PrinterTemperatureMCU) configSAM4() {
	ptm.slope = 3.3 / 0.004700
	ptm.baseTemperature = ptm.calcBase(27.0, 1.44/3.3)
}

func (ptm *PrinterTemperatureMCU) configSAME70() {
	ptm.slope = 3.3 / 0.002330
	ptm.baseTemperature = ptm.calcBase(25.0, 0.72/3.3)
}

func (ptm *PrinterTemperatureMCU) configSAMD21() {
	// SAMD21 uses factory calibration data
	// Simplified - use typical values
	ptm.slope = 3.3 / 0.002330
	ptm.baseTemperature = ptm.calcBase(25.0, 0.72/3.3)
}

func (ptm *PrinterTemperatureMCU) configSAMD51() {
	// SAMD51 uses factory calibration data
	// Simplified - use typical values
	ptm.slope = 3.3 / 0.002330
	ptm.baseTemperature = ptm.calcBase(25.0, 0.72/3.3)
}

func (ptm *PrinterTemperatureMCU) configSTM32F1() {
	ptm.slope = 3.3 / -0.004300
	ptm.baseTemperature = ptm.calcBase(25.0, 1.43/3.3)
}

func (ptm *PrinterTemperatureMCU) configSTM32F2() {
	ptm.slope = 3.3 / 0.002500
	ptm.baseTemperature = ptm.calcBase(25.0, 0.76/3.3)
}

func (ptm *PrinterTemperatureMCU) configSTM32F4() {
	// STM32F4 uses factory calibration data at 0x1FFF7A2C/0x1FFF7A2E
	// Simplified - use typical values
	ptm.slope = (110.0 - 30.0) / (0.85 - 0.35)
	ptm.baseTemperature = ptm.calcBase(30.0, 0.35)
}

func (ptm *PrinterTemperatureMCU) configSTM32F0x2() {
	// Similar to STM32F4 but different calibration addresses
	ptm.slope = (110.0 - 30.0) / (0.85 - 0.35)
	ptm.baseTemperature = ptm.calcBase(30.0, 0.35)
}

func (ptm *PrinterTemperatureMCU) configSTM32F070() {
	ptm.slope = 3.3 / -0.004300
	ptm.baseTemperature = ptm.calcBase(30.0, 0.35)
}

func (ptm *PrinterTemperatureMCU) configSTM32G0() {
	// STM32G0 uses factory calibration data
	// Simplified - use typical values
	ptm.slope = (130.0 - 30.0) / (0.85 - 0.35)
	ptm.baseTemperature = ptm.calcBase(30.0, 0.35)
}

func (ptm *PrinterTemperatureMCU) configSTM32H723() {
	// STM32H723 calibration
	ptm.slope = (130.0 - 30.0) / (0.85 - 0.35)
	ptm.baseTemperature = ptm.calcBase(30.0, 0.35)
}

func (ptm *PrinterTemperatureMCU) configSTM32H7() {
	// STM32H7 calibration
	ptm.slope = (110.0 - 30.0) / (0.85 - 0.35)
	ptm.baseTemperature = ptm.calcBase(30.0, 0.35)
}

// calcBase calculates base temperature from a calibration point.
func (ptm *PrinterTemperatureMCU) calcBase(temp, adc float64) float64 {
	return temp - adc*ptm.slope
}

// CalcTemp calculates temperature from ADC value.
func (ptm *PrinterTemperatureMCU) CalcTemp(adc float64) float64 {
	ptm.mu.RLock()
	defer ptm.mu.RUnlock()
	return ptm.baseTemperature + adc*ptm.slope
}

// CalcADC calculates ADC value from temperature.
func (ptm *PrinterTemperatureMCU) CalcADC(temp float64) float64 {
	ptm.mu.RLock()
	defer ptm.mu.RUnlock()
	return (temp - ptm.baseTemperature) / ptm.slope
}

// ADCCallback is called when a new ADC reading is available.
func (ptm *PrinterTemperatureMCU) ADCCallback(readTime, readValue float64) {
	temp := ptm.CalcTemp(readValue)
	ptm.mu.Lock()
	ptm.lastTemp = temp
	cb := ptm.callback
	ptm.mu.Unlock()

	if cb != nil {
		cb(readTime+float64(mcuSampleCount)*mcuSampleTime, temp)
	}
}

// SetupCallback registers a temperature callback.
func (ptm *PrinterTemperatureMCU) SetupCallback(cb func(readTime, temp float64)) {
	ptm.mu.Lock()
	defer ptm.mu.Unlock()
	ptm.callback = cb
}

// GetReportTimeDelta returns the report time interval.
func (ptm *PrinterTemperatureMCU) GetReportTimeDelta() float64 {
	return mcuReportTime
}

// SetupMinMax sets the temperature limits.
func (ptm *PrinterTemperatureMCU) SetupMinMax(minTemp, maxTemp float64) {
	ptm.mu.Lock()
	defer ptm.mu.Unlock()
	ptm.minTemp = minTemp
	ptm.maxTemp = maxTemp
}

// GetTemp returns the last temperature reading.
func (ptm *PrinterTemperatureMCU) GetTemp(eventtime float64) (float64, float64) {
	ptm.mu.RLock()
	defer ptm.mu.RUnlock()
	return ptm.lastTemp, 0
}

// GetStatus returns the sensor status.
func (ptm *PrinterTemperatureMCU) GetStatus() map[string]any {
	ptm.mu.RLock()
	defer ptm.mu.RUnlock()

	return map[string]any{
		"mcu_name":    ptm.mcuName,
		"mcu_type":    ptm.mcuType,
		"temperature": round2(ptm.lastTemp),
	}
}
