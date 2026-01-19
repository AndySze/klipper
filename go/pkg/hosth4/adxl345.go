// ADXL345 - port of klippy/extras/adxl345.py
//
// Support for reading acceleration data from an adxl345 chip
//
// Copyright (C) 2020-2023 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
	"time"
)

// ADXL345 register addresses
const (
	adxl345RegDEVID      = 0x00
	adxl345RegBWRate     = 0x2C
	adxl345RegPowerCTL   = 0x2D
	adxl345RegDataFormat = 0x31
	adxl345RegFIFOCTL    = 0x38
	adxl345RegModRead    = 0x80
	adxl345RegModMulti   = 0x40
)

// ADXL345 device ID
const adxl345DevID = 0xe5

// ADXL345 query rates
var adxl345QueryRates = map[int]uint8{
	25:   0x8,
	50:   0x9,
	100:  0xa,
	200:  0xb,
	400:  0xc,
	800:  0xd,
	1600: 0xe,
	3200: 0xf,
}

// Scaling factors (mg/LSB to mm/s^2)
const (
	adxl345FreefallAccel = 9.80665 * 1000.0
	adxl345ScaleXY       = 0.003774 * adxl345FreefallAccel // 1/265 mg/LSB at 3.3V
	adxl345ScaleZ        = 0.003906 * adxl345FreefallAccel // 1/256 mg/LSB at 3.3V
)

// AccelMeasurement represents a single acceleration measurement.
type AccelMeasurement struct {
	Time   float64
	AccelX float64
	AccelY float64
	AccelZ float64
}

// AccelQueryHelper helps collect acceleration measurements.
type AccelQueryHelper struct {
	printer          *runtime
	isFinished       bool
	requestStartTime float64
	requestEndTime   float64
	samples          []AccelMeasurement
	mu               sync.Mutex
}

// newAccelQueryHelper creates a new acceleration query helper.
func newAccelQueryHelper(rt *runtime) *AccelQueryHelper {
	now := float64(time.Now().UnixNano()) / 1e9
	return &AccelQueryHelper{
		printer:          rt,
		requestStartTime: now,
		requestEndTime:   now,
		samples:          make([]AccelMeasurement, 0),
	}
}

// FinishMeasurements marks measurements as complete.
func (aqh *AccelQueryHelper) FinishMeasurements() {
	aqh.mu.Lock()
	defer aqh.mu.Unlock()
	aqh.requestEndTime = float64(time.Now().UnixNano()) / 1e9
	aqh.isFinished = true
}

// AddSample adds a sample to the collection.
func (aqh *AccelQueryHelper) AddSample(t, x, y, z float64) bool {
	aqh.mu.Lock()
	defer aqh.mu.Unlock()

	if aqh.isFinished {
		return false
	}
	if len(aqh.samples) >= 10000 {
		return false
	}

	aqh.samples = append(aqh.samples, AccelMeasurement{
		Time:   t,
		AccelX: x,
		AccelY: y,
		AccelZ: z,
	})
	return true
}

// GetSamples returns all collected samples.
func (aqh *AccelQueryHelper) GetSamples() []AccelMeasurement {
	aqh.mu.Lock()
	defer aqh.mu.Unlock()

	// Filter to request time window
	filtered := make([]AccelMeasurement, 0, len(aqh.samples))
	for _, s := range aqh.samples {
		if s.Time >= aqh.requestStartTime && s.Time <= aqh.requestEndTime {
			filtered = append(filtered, s)
		}
	}
	return filtered
}

// HasValidSamples checks if there are valid samples in the time window.
func (aqh *AccelQueryHelper) HasValidSamples() bool {
	aqh.mu.Lock()
	defer aqh.mu.Unlock()

	for _, s := range aqh.samples {
		if s.Time >= aqh.requestStartTime && s.Time <= aqh.requestEndTime {
			return true
		}
	}
	return false
}

// ADXL345 represents an ADXL345 accelerometer chip.
type ADXL345 struct {
	rt       *runtime
	name     string
	dataRate int
	axesMap  [3]struct {
		pos   int
		scale float64
	}
	lastErrorCount int
	measuring      bool
	samples        []AccelMeasurement
	mu             sync.RWMutex
}

// ADXL345Config holds configuration for an ADXL345 chip.
type ADXL345Config struct {
	Name     string
	Rate     int      // Sample rate (25, 50, 100, 200, 400, 800, 1600, 3200)
	AxesMap  []string // Axes mapping (e.g., ["x", "y", "z"] or ["-x", "z", "y"])
}

// DefaultADXL345Config returns default ADXL345 configuration.
func DefaultADXL345Config() ADXL345Config {
	return ADXL345Config{
		Rate:    3200,
		AxesMap: []string{"x", "y", "z"},
	}
}

// newADXL345 creates a new ADXL345 accelerometer.
func newADXL345(rt *runtime, cfg ADXL345Config) (*ADXL345, error) {
	// Validate rate
	if _, ok := adxl345QueryRates[cfg.Rate]; !ok {
		return nil, fmt.Errorf("adxl345: invalid rate %d", cfg.Rate)
	}

	// Parse axes map
	axesMap, err := parseAxesMap(cfg.AxesMap)
	if err != nil {
		return nil, err
	}

	adxl := &ADXL345{
		rt:       rt,
		name:     cfg.Name,
		dataRate: cfg.Rate,
		axesMap:  axesMap,
		samples:  make([]AccelMeasurement, 0),
	}

	log.Printf("adxl345: initialized '%s' at %d Hz", cfg.Name, cfg.Rate)
	return adxl, nil
}

// parseAxesMap parses the axes mapping configuration.
func parseAxesMap(axes []string) ([3]struct{ pos int; scale float64 }, error) {
	am := map[string]struct{ pos int; scale float64 }{
		"x":  {0, adxl345ScaleXY},
		"y":  {1, adxl345ScaleXY},
		"z":  {2, adxl345ScaleZ},
		"-x": {0, -adxl345ScaleXY},
		"-y": {1, -adxl345ScaleXY},
		"-z": {2, -adxl345ScaleZ},
	}

	if len(axes) != 3 {
		axes = []string{"x", "y", "z"}
	}

	var result [3]struct{ pos int; scale float64 }
	for i, axis := range axes {
		mapping, ok := am[axis]
		if !ok {
			return result, fmt.Errorf("adxl345: invalid axis '%s'", axis)
		}
		result[i] = mapping
	}
	return result, nil
}

// StartMeasurements starts collecting acceleration data.
func (a *ADXL345) StartMeasurements() error {
	a.mu.Lock()
	defer a.mu.Unlock()

	if a.measuring {
		return fmt.Errorf("adxl345: already measuring")
	}

	a.measuring = true
	a.samples = make([]AccelMeasurement, 0)
	a.lastErrorCount = 0

	log.Printf("adxl345: starting '%s' measurements", a.name)
	return nil
}

// StopMeasurements stops collecting acceleration data.
func (a *ADXL345) StopMeasurements() {
	a.mu.Lock()
	defer a.mu.Unlock()

	a.measuring = false
	log.Printf("adxl345: stopped '%s' measurements", a.name)
}

// AddRawSample adds a raw sample and converts it using axes mapping.
func (a *ADXL345) AddRawSample(t float64, rx, ry, rz int) {
	a.mu.Lock()
	defer a.mu.Unlock()

	if !a.measuring {
		return
	}

	raw := [3]int{rx, ry, rz}

	x := float64(raw[a.axesMap[0].pos]) * a.axesMap[0].scale
	y := float64(raw[a.axesMap[1].pos]) * a.axesMap[1].scale
	z := float64(raw[a.axesMap[2].pos]) * a.axesMap[2].scale

	a.samples = append(a.samples, AccelMeasurement{
		Time:   t,
		AccelX: x,
		AccelY: y,
		AccelZ: z,
	})
}

// GetSamples returns all collected samples.
func (a *ADXL345) GetSamples() []AccelMeasurement {
	a.mu.RLock()
	defer a.mu.RUnlock()

	result := make([]AccelMeasurement, len(a.samples))
	copy(result, a.samples)
	return result
}

// GetLastSample returns the most recent sample.
func (a *ADXL345) GetLastSample() (AccelMeasurement, bool) {
	a.mu.RLock()
	defer a.mu.RUnlock()

	if len(a.samples) == 0 {
		return AccelMeasurement{}, false
	}
	return a.samples[len(a.samples)-1], true
}

// StartInternalClient creates a new query helper for collecting samples.
func (a *ADXL345) StartInternalClient() *AccelQueryHelper {
	return newAccelQueryHelper(a.rt)
}

// GetStatus returns the accelerometer status.
func (a *ADXL345) GetStatus() map[string]any {
	a.mu.RLock()
	defer a.mu.RUnlock()

	status := map[string]any{
		"name":       a.name,
		"data_rate":  a.dataRate,
		"measuring":  a.measuring,
		"samples":    len(a.samples),
		"errors":     a.lastErrorCount,
	}

	if len(a.samples) > 0 {
		last := a.samples[len(a.samples)-1]
		status["last_accel_x"] = last.AccelX
		status["last_accel_y"] = last.AccelY
		status["last_accel_z"] = last.AccelZ
	}

	return status
}

// GetName returns the accelerometer name.
func (a *ADXL345) GetName() string {
	return a.name
}
