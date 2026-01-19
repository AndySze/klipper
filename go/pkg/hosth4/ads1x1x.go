// ADS1x1x - port of klippy/extras/ads1x1x.py
//
// Support for ADS1013/ADS1014/ADS1015/ADS1113/ADS1114/ADS1115 I2C ADCs
//
// Copyright (C) 2024 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// ADS1x1x chip types.
const (
	ADS1013 = "ADS1013"
	ADS1014 = "ADS1014"
	ADS1015 = "ADS1015"
	ADS1113 = "ADS1113"
	ADS1114 = "ADS1114"
	ADS1115 = "ADS1115"
)

// ADS1x1x register addresses.
const (
	ads1x1xRegConversion = 0x00
	ads1x1xRegConfig     = 0x01
	ads1x1xRegLoThresh   = 0x02
	ads1x1xRegHiThresh   = 0x03
)

// ADS1x1x I2C addresses.
const (
	ads1x1xI2CAddrGnd   = 0x48
	ads1x1xI2CAddrVdd   = 0x49
	ads1x1xI2CAddrSda   = 0x4A
	ads1x1xI2CAddrScl   = 0x4B
)

// ADS1x1x gain (PGA) settings.
var ads1x1xGainVoltages = map[int]float64{
	0: 6.144,
	1: 4.096,
	2: 2.048,
	3: 1.024,
	4: 0.512,
	5: 0.256,
}

// ADS1x1x data rates for ADS101x (12-bit).
var ads101xDataRates = []int{128, 250, 490, 920, 1600, 2400, 3300}

// ADS1x1x data rates for ADS111x (16-bit).
var ads111xDataRates = []int{8, 16, 32, 64, 128, 250, 475, 860}

// ADS1x1x represents an ADS1x1x ADC.
type ADS1x1x struct {
	rt         *runtime
	name       string
	chipType   string
	i2cAddr    int
	gain       int
	dataRate   int
	mux        int // Input multiplexer configuration
	rawValue   int16
	voltage    float64
	is16Bit    bool
	callback   func(readTime float64, voltage float64)
	mu         sync.Mutex
}

// ADS1x1xConfig holds configuration for ADS1x1x.
type ADS1x1xConfig struct {
	Name     string
	ChipType string
	I2CAddr  int
	Gain     int // 0-5, see ads1x1xGainVoltages
	DataRate int // Index into data rate table
	Mux      int // 0-7 for input configuration
}

// DefaultADS1x1xConfig returns default ADS1x1x configuration.
func DefaultADS1x1xConfig() ADS1x1xConfig {
	return ADS1x1xConfig{
		ChipType: ADS1115,
		I2CAddr:  ads1x1xI2CAddrGnd,
		Gain:     2, // +/- 2.048V
		DataRate: 4, // 128 SPS for ADS111x
		Mux:      0, // AIN0 vs GND
	}
}

// newADS1x1x creates a new ADS1x1x ADC.
func newADS1x1x(rt *runtime, cfg ADS1x1xConfig) (*ADS1x1x, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("ads1x1x: name is required")
	}

	is16Bit := cfg.ChipType == ADS1113 || cfg.ChipType == ADS1114 || cfg.ChipType == ADS1115

	adc := &ADS1x1x{
		rt:       rt,
		name:     cfg.Name,
		chipType: cfg.ChipType,
		i2cAddr:  cfg.I2CAddr,
		gain:     cfg.Gain,
		dataRate: cfg.DataRate,
		mux:      cfg.Mux,
		is16Bit:  is16Bit,
	}

	log.Printf("ads1x1x: initialized '%s' type=%s addr=0x%02x", cfg.Name, cfg.ChipType, cfg.I2CAddr)
	return adc, nil
}

// GetName returns the ADC name.
func (adc *ADS1x1x) GetName() string {
	return adc.name
}

// SetCallback sets the measurement callback.
func (adc *ADS1x1x) SetCallback(cb func(readTime float64, voltage float64)) {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	adc.callback = cb
}

// GetFullScaleVoltage returns the full scale voltage for current gain.
func (adc *ADS1x1x) GetFullScaleVoltage() float64 {
	if v, ok := ads1x1xGainVoltages[adc.gain]; ok {
		return v
	}
	return 2.048 // Default
}

// ProcessData processes raw ADS1x1x data.
func (adc *ADS1x1x) ProcessData(data []byte) (int16, error) {
	if len(data) < 2 {
		return 0, fmt.Errorf("ads1x1x: insufficient data")
	}

	// 16-bit signed, MSB first
	raw := int16(data[0])<<8 | int16(data[1])

	// For 12-bit devices, data is left-justified
	if !adc.is16Bit {
		raw >>= 4
	}

	return raw, nil
}

// RawToVoltage converts raw ADC value to voltage.
func (adc *ADS1x1x) RawToVoltage(raw int16) float64 {
	fsVoltage := adc.GetFullScaleVoltage()
	if adc.is16Bit {
		return float64(raw) * fsVoltage / 32767.0
	}
	return float64(raw) * fsVoltage / 2047.0
}

// UpdateMeasurement updates the ADC value.
func (adc *ADS1x1x) UpdateMeasurement(readTime float64, rawValue int16) {
	adc.mu.Lock()
	defer adc.mu.Unlock()

	adc.rawValue = rawValue
	adc.voltage = adc.RawToVoltage(rawValue)

	if adc.callback != nil {
		adc.callback(readTime, adc.voltage)
	}
}

// GetRawValue returns the current raw ADC value.
func (adc *ADS1x1x) GetRawValue() int16 {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	return adc.rawValue
}

// GetVoltage returns the current voltage.
func (adc *ADS1x1x) GetVoltage() float64 {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	return adc.voltage
}

// SetMux sets the input multiplexer configuration.
func (adc *ADS1x1x) SetMux(mux int) error {
	if mux < 0 || mux > 7 {
		return fmt.Errorf("ads1x1x: mux must be 0-7")
	}
	adc.mu.Lock()
	defer adc.mu.Unlock()
	adc.mux = mux
	return nil
}

// SetGain sets the PGA gain.
func (adc *ADS1x1x) SetGain(gain int) error {
	if _, ok := ads1x1xGainVoltages[gain]; !ok {
		return fmt.Errorf("ads1x1x: invalid gain %d", gain)
	}
	adc.mu.Lock()
	defer adc.mu.Unlock()
	adc.gain = gain
	return nil
}

// GetStatus returns the ADC status.
func (adc *ADS1x1x) GetStatus() map[string]any {
	adc.mu.Lock()
	defer adc.mu.Unlock()

	return map[string]any{
		"raw_value":     adc.rawValue,
		"voltage":       adc.voltage,
		"chip_type":     adc.chipType,
		"gain":          adc.gain,
		"fs_voltage":    adc.GetFullScaleVoltage(),
		"mux":           adc.mux,
	}
}
