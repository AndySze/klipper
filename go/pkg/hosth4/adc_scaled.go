// ADC Scaled - port of klippy/extras/adc_scaled.py
//
// Support for scaled ADC inputs
//
// Copyright (C) 2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// ADCScaled represents a scaled ADC input.
type ADCScaled struct {
	rt           *runtime
	name         string
	vrefPin      string
	vssPin       string
	adcPin       string
	smoothTime   float64
	vref         float64
	vss          float64
	lastVref     float64
	lastVss      float64
	lastAdc      float64
	scaledValue  float64
	callback     func(readTime, scaledValue float64)
	mu           sync.Mutex
}

// ADCScaledConfig holds configuration for ADC scaled.
type ADCScaledConfig struct {
	Name       string
	VrefPin    string  // ADC pin for reference voltage
	VssPin     string  // ADC pin for ground reference (optional)
	AdcPin     string  // ADC pin to scale
	SmoothTime float64
	Vref       float64 // Expected reference voltage
	Vss        float64 // Expected ground voltage (usually 0)
}

// DefaultADCScaledConfig returns default ADC scaled configuration.
func DefaultADCScaledConfig() ADCScaledConfig {
	return ADCScaledConfig{
		SmoothTime: 2.0,
		Vref:       3.3,
		Vss:        0.0,
	}
}

// newADCScaled creates a new scaled ADC input.
func newADCScaled(rt *runtime, cfg ADCScaledConfig) (*ADCScaled, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("adc_scaled: name is required")
	}
	if cfg.VrefPin == "" {
		return nil, fmt.Errorf("adc_scaled: vref_pin is required")
	}
	if cfg.AdcPin == "" {
		return nil, fmt.Errorf("adc_scaled: adc_pin is required")
	}

	adc := &ADCScaled{
		rt:         rt,
		name:       cfg.Name,
		vrefPin:    cfg.VrefPin,
		vssPin:     cfg.VssPin,
		adcPin:     cfg.AdcPin,
		smoothTime: cfg.SmoothTime,
		vref:       cfg.Vref,
		vss:        cfg.Vss,
		lastVref:   cfg.Vref,
		lastVss:    cfg.Vss,
	}

	log.Printf("adc_scaled: initialized '%s' vref_pin=%s adc_pin=%s", cfg.Name, cfg.VrefPin, cfg.AdcPin)
	return adc, nil
}

// GetName returns the ADC name.
func (adc *ADCScaled) GetName() string {
	return adc.name
}

// SetCallback sets the measurement callback.
func (adc *ADCScaled) SetCallback(cb func(readTime, scaledValue float64)) {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	adc.callback = cb
}

// UpdateVref updates the reference voltage measurement.
func (adc *ADCScaled) UpdateVref(readTime, value float64) {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	adc.lastVref = value
	adc.recalculate(readTime)
}

// UpdateVss updates the ground reference measurement.
func (adc *ADCScaled) UpdateVss(readTime, value float64) {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	adc.lastVss = value
	adc.recalculate(readTime)
}

// UpdateAdc updates the ADC measurement.
func (adc *ADCScaled) UpdateAdc(readTime, value float64) {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	adc.lastAdc = value
	adc.recalculate(readTime)
}

// recalculate recalculates the scaled value (must be called with lock held).
func (adc *ADCScaled) recalculate(readTime float64) {
	// Scale the ADC value based on reference measurements
	// scaled = (adc - vss_measured) / (vref_measured - vss_measured) * (vref - vss) + vss
	vrefRange := adc.lastVref - adc.lastVss
	if vrefRange <= 0.001 {
		// Avoid division by zero
		adc.scaledValue = adc.lastAdc
	} else {
		normalized := (adc.lastAdc - adc.lastVss) / vrefRange
		adc.scaledValue = normalized*(adc.vref-adc.vss) + adc.vss
	}

	if adc.callback != nil {
		adc.callback(readTime, adc.scaledValue)
	}
}

// GetScaledValue returns the current scaled value.
func (adc *ADCScaled) GetScaledValue() float64 {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	return adc.scaledValue
}

// GetRawValues returns the raw ADC values.
func (adc *ADCScaled) GetRawValues() (vref, vss, adcVal float64) {
	adc.mu.Lock()
	defer adc.mu.Unlock()
	return adc.lastVref, adc.lastVss, adc.lastAdc
}

// GetStatus returns the ADC status.
func (adc *ADCScaled) GetStatus() map[string]any {
	adc.mu.Lock()
	defer adc.mu.Unlock()

	return map[string]any{
		"scaled_value": adc.scaledValue,
		"raw_vref":     adc.lastVref,
		"raw_vss":      adc.lastVss,
		"raw_adc":      adc.lastAdc,
		"expected_vref": adc.vref,
		"expected_vss":  adc.vss,
	}
}
