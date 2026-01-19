// HX71x - port of klippy/extras/hx71x.py
//
// Support for HX711/HX717 ADC chips (load cell amplifiers)
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

// HX71x chip types.
const (
	HX711Chip = "HX711"
	HX717Chip = "HX717"
)

// HX71x gain/channel configurations.
const (
	HX71xGain128ChanA = 1 // 25 pulses: Channel A, gain 128
	HX71xGain32ChanB  = 2 // 26 pulses: Channel B, gain 32
	HX71xGain64ChanA  = 3 // 27 pulses: Channel A, gain 64
)

// HX71x sample rates.
const (
	hx711SampleRate10  = 10  // 10 Hz
	hx711SampleRate80  = 80  // 80 Hz
	hx717SampleRate10  = 10  // 10 Hz
	hx717SampleRate20  = 20  // 20 Hz
	hx717SampleRate80  = 80  // 80 Hz
	hx717SampleRate320 = 320 // 320 Hz
)

// HX71x represents an HX711/HX717 ADC.
type HX71x struct {
	rt         *runtime
	name       string
	chipType   string
	sclkPin    string
	doutPin    string
	gainConfig int
	sampleRate int
	rawValue   int32
	samples    []int32
	callback   func(readTime float64, value int32)
	mu         sync.Mutex
}

// HX71xConfig holds configuration for HX71x.
type HX71xConfig struct {
	Name       string
	ChipType   string
	SclkPin    string
	DoutPin    string
	GainConfig int
	SampleRate int
}

// DefaultHX71xConfig returns default HX71x configuration.
func DefaultHX71xConfig() HX71xConfig {
	return HX71xConfig{
		ChipType:   HX711Chip,
		GainConfig: HX71xGain128ChanA,
		SampleRate: hx711SampleRate80,
	}
}

// newHX71x creates a new HX71x ADC.
func newHX71x(rt *runtime, cfg HX71xConfig) (*HX71x, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("hx71x: name is required")
	}
	if cfg.SclkPin == "" || cfg.DoutPin == "" {
		return nil, fmt.Errorf("hx71x: sclk_pin and dout_pin are required")
	}

	hx := &HX71x{
		rt:         rt,
		name:       cfg.Name,
		chipType:   cfg.ChipType,
		sclkPin:    cfg.SclkPin,
		doutPin:    cfg.DoutPin,
		gainConfig: cfg.GainConfig,
		sampleRate: cfg.SampleRate,
		samples:    make([]int32, 0),
	}

	log.Printf("hx71x: initialized '%s' type=%s gain=%d rate=%d",
		cfg.Name, cfg.ChipType, cfg.GainConfig, cfg.SampleRate)
	return hx, nil
}

// GetName returns the sensor name.
func (hx *HX71x) GetName() string {
	return hx.name
}

// SetCallback sets the measurement callback.
func (hx *HX71x) SetCallback(cb func(readTime float64, value int32)) {
	hx.mu.Lock()
	defer hx.mu.Unlock()
	hx.callback = cb
}

// GetPulseCount returns the number of SCLK pulses for current gain configuration.
func (hx *HX71x) GetPulseCount() int {
	// 24 data bits + gain configuration pulses
	return 24 + hx.gainConfig
}

// ProcessData processes raw ADC data (24-bit two's complement).
func (hx *HX71x) ProcessData(data []byte) (int32, error) {
	if len(data) < 3 {
		return 0, fmt.Errorf("hx71x: insufficient data")
	}

	// HX711/HX717 output is 24-bit two's complement, MSB first
	raw := int32(data[0])<<16 | int32(data[1])<<8 | int32(data[2])

	// Sign extend from 24 bits to 32 bits
	if raw&0x800000 != 0 {
		raw |= -0x1000000
	}

	return raw, nil
}

// UpdateMeasurement updates the ADC value.
func (hx *HX71x) UpdateMeasurement(readTime float64, rawValue int32) {
	hx.mu.Lock()
	defer hx.mu.Unlock()

	hx.rawValue = rawValue
	hx.samples = append(hx.samples, rawValue)

	// Keep last 100 samples for averaging
	if len(hx.samples) > 100 {
		hx.samples = hx.samples[1:]
	}

	if hx.callback != nil {
		hx.callback(readTime, rawValue)
	}
}

// GetRawValue returns the current raw ADC value.
func (hx *HX71x) GetRawValue() int32 {
	hx.mu.Lock()
	defer hx.mu.Unlock()
	return hx.rawValue
}

// GetAverageValue returns the average of recent samples.
func (hx *HX71x) GetAverageValue(count int) float64 {
	hx.mu.Lock()
	defer hx.mu.Unlock()

	if len(hx.samples) == 0 {
		return 0
	}

	if count <= 0 || count > len(hx.samples) {
		count = len(hx.samples)
	}

	var sum int64
	start := len(hx.samples) - count
	for i := start; i < len(hx.samples); i++ {
		sum += int64(hx.samples[i])
	}

	return float64(sum) / float64(count)
}

// GetStatus returns the sensor status.
func (hx *HX71x) GetStatus() map[string]any {
	hx.mu.Lock()
	defer hx.mu.Unlock()

	return map[string]any{
		"raw_value":   hx.rawValue,
		"chip_type":   hx.chipType,
		"gain_config": hx.gainConfig,
		"sample_rate": hx.sampleRate,
		"sample_count": len(hx.samples),
	}
}

// ClearSamples clears the sample buffer.
func (hx *HX71x) ClearSamples() {
	hx.mu.Lock()
	defer hx.mu.Unlock()
	hx.samples = make([]int32, 0)
}
