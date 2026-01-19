// Query ADC - port of klippy/extras/query_adc.py
//
// Utility for querying the current state of adc pins
//
// Copyright (C) 2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"sort"
	"strconv"
	"strings"
	"sync"
)

// AdcValue holds the last ADC reading.
type AdcValue struct {
	Value     float64 // normalized value (0.0 to 1.0)
	Timestamp float64 // time of reading
}

// McuAdc represents an ADC channel that can be queried.
type McuAdc interface {
	GetLastValue() (float64, float64) // returns (value, timestamp)
}

// adcEntry holds a registered ADC and its name.
type adcEntry struct {
	adc  McuAdc
	name string
}

// QueryAdc provides ADC query functionality.
type QueryAdc struct {
	rt *runtime

	adcs map[string]McuAdc
	mu   sync.RWMutex
}

// newQueryAdc creates a new query ADC instance.
func newQueryAdc(rt *runtime) *QueryAdc {
	return &QueryAdc{
		rt:   rt,
		adcs: make(map[string]McuAdc),
	}
}

// RegisterAdc registers an ADC for querying.
func (qa *QueryAdc) RegisterAdc(name string, adc McuAdc) {
	qa.mu.Lock()
	defer qa.mu.Unlock()
	qa.adcs[name] = adc
}

// GetAdcNames returns all registered ADC names.
func (qa *QueryAdc) GetAdcNames() []string {
	qa.mu.RLock()
	defer qa.mu.RUnlock()
	names := make([]string, 0, len(qa.adcs))
	for name := range qa.adcs {
		names = append(names, name)
	}
	sort.Strings(names)
	return names
}

// GetAdc returns an ADC by name.
func (qa *QueryAdc) GetAdc(name string) (McuAdc, bool) {
	qa.mu.RLock()
	defer qa.mu.RUnlock()
	adc, ok := qa.adcs[name]
	return adc, ok
}

// cmdQueryAdc handles the QUERY_ADC command.
// QUERY_ADC [NAME=<adc_name>] [PULLUP=<resistance>]
func (qa *QueryAdc) cmdQueryAdc(args map[string]string) (string, error) {
	name, hasName := args["NAME"]

	if !hasName {
		// Return list of available ADC objects
		names := qa.GetAdcNames()
		quotedNames := make([]string, len(names))
		for i, n := range names {
			quotedNames[i] = fmt.Sprintf(`"%s"`, n)
		}
		return fmt.Sprintf("Available ADC objects: %s", strings.Join(quotedNames, ", ")), nil
	}

	qa.mu.RLock()
	adc, ok := qa.adcs[name]
	qa.mu.RUnlock()

	if !ok {
		names := qa.GetAdcNames()
		quotedNames := make([]string, len(names))
		for i, n := range names {
			quotedNames[i] = fmt.Sprintf(`"%s"`, n)
		}
		return fmt.Sprintf("Available ADC objects: %s", strings.Join(quotedNames, ", ")), nil
	}

	// Get the last value
	value, timestamp := adc.GetLastValue()
	msg := fmt.Sprintf("ADC object \"%s\" has value %.6f (timestamp %.3f)", name, value, timestamp)

	// Handle optional PULLUP parameter for resistance calculation
	if pullupStr, ok := args["PULLUP"]; ok {
		pullup, err := strconv.ParseFloat(pullupStr, 64)
		if err != nil {
			return "", fmt.Errorf("invalid PULLUP value: %w", err)
		}
		if pullup <= 0 {
			return "", fmt.Errorf("PULLUP must be > 0")
		}

		// Calculate resistance
		// Clamp value to avoid division by zero or negative log
		v := value
		if v < 0.00001 {
			v = 0.00001
		} else if v > 0.99999 {
			v = 0.99999
		}

		// Calculate resistance using voltage divider formula
		// V_out = V_in * R / (R + R_pullup)
		// value = R / (R + pullup)
		// value * (R + pullup) = R
		// value * R + value * pullup = R
		// value * pullup = R - value * R
		// value * pullup = R * (1 - value)
		// R = value * pullup / (1 - value)
		r := pullup * v / (1.0 - v)
		msg += fmt.Sprintf("\n resistance %.3f (with %.0f pullup)", r, pullup)
	}

	return msg, nil
}

// SimpleAdc is a simple implementation of McuAdc for testing.
type SimpleAdc struct {
	value     float64
	timestamp float64
	mu        sync.RWMutex
}

// NewSimpleAdc creates a new simple ADC.
func NewSimpleAdc() *SimpleAdc {
	return &SimpleAdc{}
}

// SetValue sets the ADC value.
func (sa *SimpleAdc) SetValue(value, timestamp float64) {
	sa.mu.Lock()
	defer sa.mu.Unlock()
	sa.value = value
	sa.timestamp = timestamp
}

// GetLastValue returns the last ADC value.
func (sa *SimpleAdc) GetLastValue() (float64, float64) {
	sa.mu.RLock()
	defer sa.mu.RUnlock()
	return sa.value, sa.timestamp
}
