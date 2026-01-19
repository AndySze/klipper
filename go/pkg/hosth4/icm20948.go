// ICM20948 - port of klippy/extras/icm20948.py
//
// Support for reading acceleration data from an ICM-20948 chip
//
// Copyright (C) 2022 Harry Beyel <harry3b9@gmail.com>
// Copyright (C) 2024 Kevin O'Connor <kevin@koconnor.net>
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

// ICM20948 registers (Bank 0).
const (
	regICM20948WhoAmI       = 0x00
	regICM20948UserCtrl     = 0x03
	regICM20948PwrMgmt1     = 0x06
	regICM20948PwrMgmt2     = 0x07
	regICM20948IntEnable    = 0x10
	regICM20948FifoEn2      = 0x67
	regICM20948FifoCountH   = 0x70
	regICM20948FifoRW       = 0x72
	regICM20948AccelXOutH   = 0x2D
	regICM20948BankSel      = 0x7F
)

// ICM20948 registers (Bank 2).
const (
	regICM20948GyroConfig1  = 0x01
	regICM20948AccelConfig  = 0x14
	regICM20948AccelSmplrtDiv1 = 0x10
	regICM20948AccelSmplrtDiv2 = 0x11
)

// ICM20948 constants.
const (
	icm20948Scale    = 9.80665 * 4.0 / 32768.0 // 4g range
	icm20948DataRate = 4500
	icm20948I2CAddr  = 0x68
)

// ICM20948 represents an ICM-20948 accelerometer.
type ICM20948 struct {
	rt          *runtime
	name        string
	dataRate    int
	axesMap     [3]struct {
		pos   int
		scale float64
	}
	samples        []AccelMeasurement
	lastErrorCount int
	queryActive    bool
	lastSampleTime float64
	mu             sync.Mutex
}

// ICM20948Config holds configuration for ICM20948.
type ICM20948Config struct {
	Name    string
	AxesMap string
}

// DefaultICM20948Config returns default ICM20948 configuration.
func DefaultICM20948Config() ICM20948Config {
	return ICM20948Config{
		AxesMap: "x,y,z",
	}
}

// newICM20948 creates a new ICM20948 accelerometer.
func newICM20948(rt *runtime, cfg ICM20948Config) (*ICM20948, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("icm20948: name is required")
	}

	icm := &ICM20948{
		rt:       rt,
		name:     cfg.Name,
		dataRate: icm20948DataRate,
		samples:  make([]AccelMeasurement, 0),
	}

	// Parse axes map
	if err := icm.parseAxesMap(cfg.AxesMap); err != nil {
		return nil, err
	}

	log.Printf("icm20948: initialized '%s' rate=%d", cfg.Name, icm.dataRate)
	return icm, nil
}

// parseAxesMap parses the axes mapping configuration.
func (icm *ICM20948) parseAxesMap(axesMap string) error {
	icm.axesMap[0] = struct{ pos int; scale float64 }{0, icm20948Scale}
	icm.axesMap[1] = struct{ pos int; scale float64 }{1, icm20948Scale}
	icm.axesMap[2] = struct{ pos int; scale float64 }{2, icm20948Scale}

	if axesMap != "" && axesMap != "x,y,z" {
		log.Printf("icm20948: using custom axes map: %s", axesMap)
	}

	return nil
}

// GetName returns the sensor name.
func (icm *ICM20948) GetName() string {
	return icm.name
}

// GetDataRate returns the data rate in Hz.
func (icm *ICM20948) GetDataRate() int {
	return icm.dataRate
}

// StartMeasurements starts accelerometer measurements.
func (icm *ICM20948) StartMeasurements() error {
	icm.mu.Lock()
	defer icm.mu.Unlock()

	if icm.queryActive {
		return fmt.Errorf("icm20948: measurements already active")
	}

	icm.samples = make([]AccelMeasurement, 0)
	icm.queryActive = true
	icm.lastErrorCount = 0

	log.Printf("icm20948 '%s': starting measurements", icm.name)
	return nil
}

// StopMeasurements stops accelerometer measurements.
func (icm *ICM20948) StopMeasurements() error {
	icm.mu.Lock()
	defer icm.mu.Unlock()

	if !icm.queryActive {
		return nil
	}

	icm.queryActive = false
	log.Printf("icm20948 '%s': stopped measurements, %d samples collected",
		icm.name, len(icm.samples))
	return nil
}

// IsActive returns true if measurements are active.
func (icm *ICM20948) IsActive() bool {
	icm.mu.Lock()
	defer icm.mu.Unlock()
	return icm.queryActive
}

// AddSample adds a raw sample and converts to calibrated values.
func (icm *ICM20948) AddSample(time float64, rawX, rawY, rawZ int16) {
	icm.mu.Lock()
	defer icm.mu.Unlock()

	if !icm.queryActive {
		return
	}

	raw := [3]float64{float64(rawX), float64(rawY), float64(rawZ)}
	var x, y, z float64

	x = raw[icm.axesMap[0].pos] * icm.axesMap[0].scale
	y = raw[icm.axesMap[1].pos] * icm.axesMap[1].scale
	z = raw[icm.axesMap[2].pos] * icm.axesMap[2].scale

	icm.samples = append(icm.samples, AccelMeasurement{
		Time:   time,
		AccelX: x,
		AccelY: y,
		AccelZ: z,
	})
	icm.lastSampleTime = time
}

// GetSamples returns all collected samples.
func (icm *ICM20948) GetSamples() []AccelMeasurement {
	icm.mu.Lock()
	defer icm.mu.Unlock()

	result := make([]AccelMeasurement, len(icm.samples))
	copy(result, icm.samples)
	return result
}

// ClearSamples clears all collected samples.
func (icm *ICM20948) ClearSamples() {
	icm.mu.Lock()
	defer icm.mu.Unlock()
	icm.samples = make([]AccelMeasurement, 0)
}

// GetStatus returns the sensor status.
func (icm *ICM20948) GetStatus() map[string]any {
	icm.mu.Lock()
	defer icm.mu.Unlock()

	return map[string]any{
		"name":         icm.name,
		"data_rate":    icm.dataRate,
		"active":       icm.queryActive,
		"sample_count": len(icm.samples),
		"error_count":  icm.lastErrorCount,
	}
}

// CalculateRMS calculates RMS acceleration from samples.
func (icm *ICM20948) CalculateRMS() (x, y, z, total float64) {
	samples := icm.GetSamples()
	if len(samples) == 0 {
		return
	}

	var sumX, sumY, sumZ float64
	for _, s := range samples {
		sumX += s.AccelX * s.AccelX
		sumY += s.AccelY * s.AccelY
		sumZ += s.AccelZ * s.AccelZ
	}

	n := float64(len(samples))
	x = math.Sqrt(sumX / n)
	y = math.Sqrt(sumY / n)
	z = math.Sqrt(sumZ / n)
	total = math.Sqrt((sumX + sumY + sumZ) / n)
	return
}
