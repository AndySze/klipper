// Temperature Combined - port of klippy/extras/temperature_combined.py
//
// Support for combining multiple temperature sensors
//
// Copyright (C) 2020 Kevin O'Connor <kevin@koconnor.net>
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

// Combination methods.
const (
	CombineMax     = "max"
	CombineMin     = "min"
	CombineAverage = "average"
)

// TemperatureCombined represents a combined temperature sensor.
type TemperatureCombined struct {
	rt            *runtime
	name          string
	sensors       []string
	combineMethod string
	maxDiff       float64
	temperature   float64
	callback      func(readTime, temp float64)
	mu            sync.Mutex
}

// TemperatureCombinedConfig holds configuration for temperature combined.
type TemperatureCombinedConfig struct {
	Name          string
	Sensors       []string
	CombineMethod string
	MaxDiff       float64 // Maximum allowed difference between sensors
}

// DefaultTemperatureCombinedConfig returns default configuration.
func DefaultTemperatureCombinedConfig() TemperatureCombinedConfig {
	return TemperatureCombinedConfig{
		CombineMethod: CombineMax,
		MaxDiff:       0, // 0 = no limit
	}
}

// newTemperatureCombined creates a new combined temperature sensor.
func newTemperatureCombined(rt *runtime, cfg TemperatureCombinedConfig) (*TemperatureCombined, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("temperature_combined: name is required")
	}
	if len(cfg.Sensors) == 0 {
		return nil, fmt.Errorf("temperature_combined: at least one sensor is required")
	}

	tc := &TemperatureCombined{
		rt:            rt,
		name:          cfg.Name,
		sensors:       cfg.Sensors,
		combineMethod: cfg.CombineMethod,
		maxDiff:       cfg.MaxDiff,
	}

	log.Printf("temperature_combined: initialized '%s' method=%s sensors=%v",
		cfg.Name, cfg.CombineMethod, cfg.Sensors)
	return tc, nil
}

// GetName returns the sensor name.
func (tc *TemperatureCombined) GetName() string {
	return tc.name
}

// SetCallback sets the measurement callback.
func (tc *TemperatureCombined) SetCallback(cb func(readTime, temp float64)) {
	tc.mu.Lock()
	defer tc.mu.Unlock()
	tc.callback = cb
}

// CombineTemperatures combines temperatures from multiple sensors.
func (tc *TemperatureCombined) CombineTemperatures(temps []float64) (float64, error) {
	if len(temps) == 0 {
		return 0, fmt.Errorf("temperature_combined: no temperatures provided")
	}

	// Check max difference if configured
	if tc.maxDiff > 0 && len(temps) > 1 {
		minTemp := temps[0]
		maxTemp := temps[0]
		for _, t := range temps[1:] {
			if t < minTemp {
				minTemp = t
			}
			if t > maxTemp {
				maxTemp = t
			}
		}
		if maxTemp-minTemp > tc.maxDiff {
			return 0, fmt.Errorf("temperature_combined: sensor difference %.1f exceeds max_diff %.1f",
				maxTemp-minTemp, tc.maxDiff)
		}
	}

	var result float64
	switch tc.combineMethod {
	case CombineMax:
		result = temps[0]
		for _, t := range temps[1:] {
			if t > result {
				result = t
			}
		}
	case CombineMin:
		result = temps[0]
		for _, t := range temps[1:] {
			if t < result {
				result = t
			}
		}
	case CombineAverage:
		sum := 0.0
		for _, t := range temps {
			sum += t
		}
		result = sum / float64(len(temps))
	default:
		return 0, fmt.Errorf("temperature_combined: unknown method %s", tc.combineMethod)
	}

	return result, nil
}

// UpdateMeasurement updates the combined temperature.
func (tc *TemperatureCombined) UpdateMeasurement(readTime float64, temps []float64) error {
	combined, err := tc.CombineTemperatures(temps)
	if err != nil {
		return err
	}

	tc.mu.Lock()
	defer tc.mu.Unlock()

	tc.temperature = combined

	if tc.callback != nil {
		tc.callback(readTime, combined)
	}

	return nil
}

// GetTemperature returns the combined temperature.
func (tc *TemperatureCombined) GetTemperature() float64 {
	tc.mu.Lock()
	defer tc.mu.Unlock()
	return tc.temperature
}

// GetStatus returns the sensor status.
func (tc *TemperatureCombined) GetStatus() map[string]any {
	tc.mu.Lock()
	defer tc.mu.Unlock()

	return map[string]any{
		"temperature": tc.temperature,
		"sensors":     tc.sensors,
		"method":      tc.combineMethod,
	}
}

// TemperatureWeight represents a weighted temperature input.
type TemperatureWeight struct {
	SensorName string
	Weight     float64
}

// TemperatureWeightedAverage calculates weighted average of temperatures.
func TemperatureWeightedAverage(temps map[string]float64, weights []TemperatureWeight) (float64, error) {
	if len(weights) == 0 {
		return 0, fmt.Errorf("temperature_combined: no weights provided")
	}

	var sum, weightSum float64
	for _, w := range weights {
		temp, exists := temps[w.SensorName]
		if !exists {
			return 0, fmt.Errorf("temperature_combined: sensor %s not found", w.SensorName)
		}
		if math.IsNaN(temp) {
			return 0, fmt.Errorf("temperature_combined: sensor %s has NaN temperature", w.SensorName)
		}
		sum += temp * w.Weight
		weightSum += w.Weight
	}

	if weightSum == 0 {
		return 0, fmt.Errorf("temperature_combined: total weight is zero")
	}

	return sum / weightSum, nil
}
