// ADS1220 - port of klippy/extras/ads1220.py
//
// Support for ADS1220 24-bit ADC
//
// Copyright (C) 2022 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// ADS1220 commands.
const (
	ads1220CmdReset     = 0x06
	ads1220CmdStartSync = 0x08
	ads1220CmdPowerDown = 0x02
	ads1220CmdRData     = 0x10
	ads1220CmdRReg      = 0x20
	ads1220CmdWReg      = 0x40
)

// ADS1220 register addresses.
const (
	ads1220Reg0 = 0x00
	ads1220Reg1 = 0x01
	ads1220Reg2 = 0x02
	ads1220Reg3 = 0x03
)

// ADS1220 data rates.
const (
	ads1220Rate20   = 0
	ads1220Rate45   = 1
	ads1220Rate90   = 2
	ads1220Rate175  = 3
	ads1220Rate330  = 4
	ads1220Rate600  = 5
	ads1220Rate1000 = 6
)

// ADS1220 gain settings.
const (
	ads1220Gain1   = 0
	ads1220Gain2   = 1
	ads1220Gain4   = 2
	ads1220Gain8   = 3
	ads1220Gain16  = 4
	ads1220Gain32  = 5
	ads1220Gain64  = 6
	ads1220Gain128 = 7
)

// ADS1220 represents an ADS1220 ADC.
type ADS1220 struct {
	rt         *runtime
	name       string
	spiPin     string
	dataRate   int
	gain       int
	pga        bool // Programmable gain amplifier bypass
	rawValue   int32
	voltage    float64
	vref       float64
	callback   func(readTime float64, value int32)
	mu         sync.Mutex
}

// ADS1220Config holds configuration for ADS1220.
type ADS1220Config struct {
	Name     string
	SPIPin   string
	DataRate int
	Gain     int
	PGA      bool
	Vref     float64 // Reference voltage
}

// DefaultADS1220Config returns default ADS1220 configuration.
func DefaultADS1220Config() ADS1220Config {
	return ADS1220Config{
		DataRate: ads1220Rate20,
		Gain:     ads1220Gain1,
		PGA:      true,
		Vref:     2.048, // Internal reference
	}
}

// newADS1220 creates a new ADS1220 ADC.
func newADS1220(rt *runtime, cfg ADS1220Config) (*ADS1220, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("ads1220: name is required")
	}
	if cfg.SPIPin == "" {
		return nil, fmt.Errorf("ads1220: spi_pin is required")
	}

	adc := &ADS1220{
		rt:       rt,
		name:     cfg.Name,
		spiPin:   cfg.SPIPin,
		dataRate: cfg.DataRate,
		gain:     cfg.Gain,
		pga:      cfg.PGA,
		vref:     cfg.Vref,
	}

	log.Printf("ads1220: initialized '%s' rate=%d gain=%d", cfg.Name, cfg.DataRate, cfg.Gain)
	return adc, nil
}

// GetName returns the ADC name.
func (adc *ADS1220) GetName() string {
	return adc.name
}

// SetCallback sets the measurement callback.
func (adc *ADS1220) SetCallback(cb func(readTime float64, value int32)) {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	adc.callback = cb
}

// GetGainMultiplier returns the gain multiplier.
func (adc *ADS1220) GetGainMultiplier() float64 {
	return float64(int(1) << adc.gain)
}

// ProcessData processes raw ADS1220 data (24-bit two's complement).
func (adc *ADS1220) ProcessData(data []byte) (int32, error) {
	if len(data) < 3 {
		return 0, fmt.Errorf("ads1220: insufficient data")
	}

	// 24-bit two's complement, MSB first
	raw := int32(data[0])<<16 | int32(data[1])<<8 | int32(data[2])

	// Sign extend from 24 bits
	if raw&0x800000 != 0 {
		raw |= -0x1000000
	}

	return raw, nil
}

// RawToVoltage converts raw ADC value to voltage.
func (adc *ADS1220) RawToVoltage(raw int32) float64 {
	// Full scale is 2^23 - 1 for positive values
	gain := adc.GetGainMultiplier()
	return float64(raw) * adc.vref / (8388607.0 * gain)
}

// UpdateMeasurement updates the ADC value.
func (adc *ADS1220) UpdateMeasurement(readTime float64, rawValue int32) {
	adc.mu.Lock()
	defer adc.mu.Unlock()

	adc.rawValue = rawValue
	adc.voltage = adc.RawToVoltage(rawValue)

	if adc.callback != nil {
		adc.callback(readTime, rawValue)
	}
}

// GetRawValue returns the current raw ADC value.
func (adc *ADS1220) GetRawValue() int32 {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	return adc.rawValue
}

// GetVoltage returns the current voltage.
func (adc *ADS1220) GetVoltage() float64 {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	return adc.voltage
}

// GetStatus returns the ADC status.
func (adc *ADS1220) GetStatus() map[string]any {
	adc.mu.Lock()
	defer adc.mu.Unlock()

	return map[string]any{
		"raw_value": adc.rawValue,
		"voltage":   adc.voltage,
		"gain":      adc.GetGainMultiplier(),
		"data_rate": adc.dataRate,
	}
}
