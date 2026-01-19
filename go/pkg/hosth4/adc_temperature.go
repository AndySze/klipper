// ADC Temperature - port of klippy/extras/adc_temperature.py
//
// Obtain temperature using linear interpolation of ADC values
//
// Copyright (C) 2016-2024 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sort"
	"sync"
)

const (
	adcTempSampleTime       = 0.001
	adcTempSampleCount      = 8
	adcTempReportTime       = 0.300
	adcTempRangeCheckCount  = 4
)

// LinearInterpolate performs linear interpolation between sample points.
type LinearInterpolate struct {
	keys   []float64
	slopes []struct {
		gain   float64
		offset float64
	}
}

// newLinearInterpolate creates a new linear interpolator from samples.
func newLinearInterpolate(samples []struct{ key, value float64 }) (*LinearInterpolate, error) {
	if len(samples) < 2 {
		return nil, fmt.Errorf("need at least two samples")
	}

	// Sort samples by key
	sort.Slice(samples, func(i, j int) bool {
		return samples[i].key < samples[j].key
	})

	li := &LinearInterpolate{}
	var lastKey, lastValue float64
	first := true

	for _, s := range samples {
		if first {
			lastKey = s.key
			lastValue = s.value
			first = false
			continue
		}

		if s.key <= lastKey {
			return nil, fmt.Errorf("duplicate value at key %f", s.key)
		}

		gain := (s.value - lastValue) / (s.key - lastKey)
		offset := lastValue - lastKey*gain

		// Skip if same slope as previous
		if len(li.slopes) > 0 && li.slopes[len(li.slopes)-1].gain == gain &&
			li.slopes[len(li.slopes)-1].offset == offset {
			continue
		}

		lastKey = s.key
		lastValue = s.value
		li.keys = append(li.keys, s.key)
		li.slopes = append(li.slopes, struct{ gain, offset float64 }{gain, offset})
	}

	if len(li.keys) == 0 {
		return nil, fmt.Errorf("need at least two different samples")
	}

	// Add sentinel
	li.keys = append(li.keys, 9999999999999.0)
	li.slopes = append(li.slopes, li.slopes[len(li.slopes)-1])

	return li, nil
}

// Interpolate returns the interpolated value for a key.
func (li *LinearInterpolate) Interpolate(key float64) float64 {
	pos := sort.SearchFloat64s(li.keys, key)
	if pos >= len(li.slopes) {
		pos = len(li.slopes) - 1
	}
	slope := li.slopes[pos]
	return key*slope.gain + slope.offset
}

// ReverseInterpolate returns the key for a given value.
func (li *LinearInterpolate) ReverseInterpolate(value float64) float64 {
	// Calculate values at each key
	values := make([]float64, len(li.keys))
	for i, key := range li.keys {
		values[i] = key*li.slopes[i].gain + li.slopes[i].offset
	}

	// Find which segment contains the value
	var validIdx int
	if values[0] < values[len(values)-2] {
		// Ascending
		for i := range values {
			if values[i] >= value {
				validIdx = i
				break
			}
		}
	} else {
		// Descending
		for i := range values {
			if values[i] <= value {
				validIdx = i
				break
			}
		}
	}

	if validIdx >= len(li.slopes) {
		validIdx = len(li.slopes) - 1
	}

	slope := li.slopes[validIdx]
	if slope.gain == 0 {
		return li.keys[validIdx]
	}
	return (value - slope.offset) / slope.gain
}

// LinearVoltage converts ADC voltage readings to temperature.
type LinearVoltage struct {
	li *LinearInterpolate
}

// newLinearVoltage creates a new voltage-based temperature converter.
func newLinearVoltage(adcVoltage, voltageOffset float64, params []struct{ temp, volt float64 }) (*LinearVoltage, error) {
	samples := make([]struct{ key, value float64 }, 0, len(params))
	for _, p := range params {
		adc := (p.volt - voltageOffset) / adcVoltage
		if adc < 0 || adc > 1 {
			log.Printf("adc_temperature: ignoring sample %.3f/%.3f (out of ADC range)", p.temp, p.volt)
			continue
		}
		samples = append(samples, struct{ key, value float64 }{adc, p.temp})
	}

	li, err := newLinearInterpolate(samples)
	if err != nil {
		return nil, err
	}

	return &LinearVoltage{li: li}, nil
}

// CalcTemp calculates temperature from ADC value.
func (lv *LinearVoltage) CalcTemp(adc float64) float64 {
	return lv.li.Interpolate(adc)
}

// CalcADC calculates ADC value from temperature.
func (lv *LinearVoltage) CalcADC(temp float64) float64 {
	return lv.li.ReverseInterpolate(temp)
}

// LinearResistance converts ADC resistance readings to temperature.
type LinearResistance struct {
	pullup float64
	li     *LinearInterpolate
}

// newLinearResistance creates a new resistance-based temperature converter.
func newLinearResistance(pullupResistor float64, params []struct{ temp, resistance float64 }) (*LinearResistance, error) {
	samples := make([]struct{ key, value float64 }, 0, len(params))
	for _, p := range params {
		samples = append(samples, struct{ key, value float64 }{p.resistance, p.temp})
	}

	li, err := newLinearInterpolate(samples)
	if err != nil {
		return nil, err
	}

	return &LinearResistance{pullup: pullupResistor, li: li}, nil
}

// CalcTemp calculates temperature from ADC value.
func (lr *LinearResistance) CalcTemp(adc float64) float64 {
	// Clamp ADC value
	if adc < 0.00001 {
		adc = 0.00001
	}
	if adc > 0.99999 {
		adc = 0.99999
	}
	r := lr.pullup * adc / (1.0 - adc)
	return lr.li.Interpolate(r)
}

// CalcADC calculates ADC value from temperature.
func (lr *LinearResistance) CalcADC(temp float64) float64 {
	r := lr.li.ReverseInterpolate(temp)
	return r / (lr.pullup + r)
}

// ADCToTemperature provides the interface between MCU ADC and temperature callbacks.
type ADCToTemperature struct {
	rt          *runtime
	name        string
	converter   ADCConverter
	minTemp     float64
	maxTemp     float64
	lastTemp    float64
	callback    func(readTime, temp float64)
	mu          sync.RWMutex
}

// ADCConverter is the interface for ADC to temperature conversion.
type ADCConverter interface {
	CalcTemp(adc float64) float64
	CalcADC(temp float64) float64
}

// ADCToTemperatureConfig holds configuration for ADC temperature sensor.
type ADCToTemperatureConfig struct {
	Name      string
	Converter ADCConverter
}

// newADCToTemperature creates a new ADC temperature sensor.
func newADCToTemperature(rt *runtime, cfg ADCToTemperatureConfig) (*ADCToTemperature, error) {
	if cfg.Converter == nil {
		return nil, fmt.Errorf("ADC converter is required")
	}

	at := &ADCToTemperature{
		rt:        rt,
		name:      cfg.Name,
		converter: cfg.Converter,
	}

	log.Printf("adc_temperature: initialized '%s'", cfg.Name)
	return at, nil
}

// SetupCallback registers a temperature callback.
func (at *ADCToTemperature) SetupCallback(cb func(readTime, temp float64)) {
	at.mu.Lock()
	defer at.mu.Unlock()
	at.callback = cb
}

// GetReportTimeDelta returns the report time interval.
func (at *ADCToTemperature) GetReportTimeDelta() float64 {
	return adcTempReportTime
}

// ADCCallback is called when a new ADC reading is available.
func (at *ADCToTemperature) ADCCallback(readTime, readValue float64) {
	temp := at.converter.CalcTemp(readValue)

	at.mu.Lock()
	at.lastTemp = temp
	cb := at.callback
	at.mu.Unlock()

	if cb != nil {
		cb(readTime+float64(adcTempSampleCount)*adcTempSampleTime, temp)
	}
}

// SetupMinMax sets the temperature limits.
func (at *ADCToTemperature) SetupMinMax(minTemp, maxTemp float64) {
	at.mu.Lock()
	defer at.mu.Unlock()
	at.minTemp = minTemp
	at.maxTemp = maxTemp
}

// GetTemp returns the last temperature reading.
func (at *ADCToTemperature) GetTemp(eventtime float64) (float64, float64) {
	at.mu.RLock()
	defer at.mu.RUnlock()
	return at.lastTemp, 0
}

// GetStatus returns the sensor status.
func (at *ADCToTemperature) GetStatus() map[string]any {
	at.mu.RLock()
	defer at.mu.RUnlock()

	return map[string]any{
		"temperature": round2(at.lastTemp),
	}
}

// Pre-defined sensor voltage tables

// AD595 thermocouple amplifier voltage table.
var AD595Params = []struct{ temp, volt float64 }{
	{0, 0.0027}, {10, 0.101}, {20, 0.200}, {25, 0.250}, {30, 0.300},
	{40, 0.401}, {50, 0.503}, {60, 0.605}, {80, 0.810}, {100, 1.015},
	{120, 1.219}, {140, 1.420}, {160, 1.620}, {180, 1.817}, {200, 2.015},
	{220, 2.213}, {240, 2.413}, {260, 2.614}, {280, 2.817}, {300, 3.022},
	{320, 3.227}, {340, 3.434}, {360, 3.641}, {380, 3.849}, {400, 4.057},
	{420, 4.266}, {440, 4.476}, {460, 4.686}, {480, 4.896},
}

// AD597 thermocouple amplifier voltage table.
var AD597Params = []struct{ temp, volt float64 }{
	{0, 0}, {10, 0.097}, {20, 0.196}, {25, 0.245}, {30, 0.295},
	{40, 0.395}, {50, 0.496}, {60, 0.598}, {80, 0.802}, {100, 1.005},
	{120, 1.207}, {140, 1.407}, {160, 1.605}, {180, 1.801}, {200, 1.997},
	{220, 2.194}, {240, 2.392}, {260, 2.592}, {280, 2.794}, {300, 2.996},
	{320, 3.201}, {340, 3.406}, {360, 3.611}, {380, 3.817}, {400, 4.024},
	{420, 4.232}, {440, 4.440}, {460, 4.649}, {480, 4.857}, {500, 5.066},
}

// CalcPT100Params generates PT100/PT1000 resistance values using Callendar-Van Dusen formula.
func CalcPT100Params(baseResistance float64) []struct{ temp, resistance float64 } {
	const A = 3.9083e-3
	const B = -5.775e-7

	params := make([]struct{ temp, resistance float64 }, 0)
	for t := 0; t < 500; t += 10 {
		temp := float64(t)
		resistance := baseResistance * (1.0 + A*temp + B*temp*temp)
		params = append(params, struct{ temp, resistance float64 }{temp, resistance})
	}
	return params
}

// CalcINA826PT100Params generates PT100 params for INA826 circuit (4400ohm pullup, 10x gain to 5V).
func CalcINA826PT100Params() []struct{ temp, volt float64 } {
	pt100 := CalcPT100Params(100.0)
	params := make([]struct{ temp, volt float64 }, len(pt100))
	for i, p := range pt100 {
		params[i] = struct{ temp, volt float64 }{
			temp: p.temp,
			volt: 10.0 * 5.0 * p.resistance / (4400.0 + p.resistance),
		}
	}
	return params
}
