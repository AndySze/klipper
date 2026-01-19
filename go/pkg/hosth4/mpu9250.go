// MPU9250 - port of klippy/extras/mpu9250.py
//
// Support for reading acceleration data from an MPU-9250 chip
//
// Copyright (C) 2022 Harry Beyel <harry3b9@gmail.com>
// Copyright (C) 2020-2023 Kevin O'Connor <kevin@koconnor.net>
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

// MPU9250 registers.
const (
	regMPU9250WhoAmI      = 0x75
	regMPU9250PwrMgmt1    = 0x6B
	regMPU9250PwrMgmt2    = 0x6C
	regMPU9250SmplrtDiv   = 0x19
	regMPU9250Config      = 0x1A
	regMPU9250GyroConfig  = 0x1B
	regMPU9250AccelConfig = 0x1C
	regMPU9250AccelConfig2 = 0x1D
	regMPU9250FifoEn      = 0x23
	regMPU9250UserCtrl    = 0x6A
	regMPU9250FifoCountH  = 0x72
	regMPU9250FifoRW      = 0x74
	regMPU9250AccelXOutH  = 0x3B
)

// Device IDs.
const (
	mpu9250DevID   = 0x71
	mpu9255DevID   = 0x73
	mpu6050DevID   = 0x68
	mpu6500DevID   = 0x70
	icm20948DevID  = 0xEA
)

// MPU9250 constants.
const (
	mpu9250Scale     = 9.80665 * 4.0 / 32768.0 // 4g range
	mpu9250DataRate  = 4000
	mpu9250I2CAddr   = 0x68
)

// MPU chip types.
const (
	MPU9250Type  = "MPU9250"
	MPU9255Type  = "MPU9255"
	MPU6050Type  = "MPU6050"
	MPU6500Type  = "MPU6500"
	ICM20948Type = "ICM20948"
)

// MPU9250 represents an MPU-9250 or compatible accelerometer.
type MPU9250 struct {
	rt          *runtime
	name        string
	mpuType     string
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

// MPU9250Config holds configuration for MPU9250.
type MPU9250Config struct {
	Name    string
	MpuType string // MPU9250Type, MPU6050Type, etc.
	AxesMap string // e.g., "x,y,z" or "-x,z,y"
}

// DefaultMPU9250Config returns default MPU9250 configuration.
func DefaultMPU9250Config() MPU9250Config {
	return MPU9250Config{
		MpuType: MPU9250Type,
		AxesMap: "x,y,z",
	}
}

// newMPU9250 creates a new MPU9250 accelerometer.
func newMPU9250(rt *runtime, cfg MPU9250Config) (*MPU9250, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("mpu9250: name is required")
	}

	mpu := &MPU9250{
		rt:       rt,
		name:     cfg.Name,
		mpuType:  cfg.MpuType,
		dataRate: mpu9250DataRate,
		samples:  make([]AccelMeasurement, 0),
	}

	// Parse axes map
	if err := mpu.parseAxesMap(cfg.AxesMap); err != nil {
		return nil, err
	}

	log.Printf("mpu9250: initialized '%s' type=%s rate=%d",
		cfg.Name, cfg.MpuType, mpu.dataRate)
	return mpu, nil
}

// parseAxesMap parses the axes mapping configuration.
func (mpu *MPU9250) parseAxesMap(axesMap string) error {
	// Default mapping: x->0, y->1, z->2
	mpu.axesMap[0] = struct{ pos int; scale float64 }{0, mpu9250Scale}
	mpu.axesMap[1] = struct{ pos int; scale float64 }{1, mpu9250Scale}
	mpu.axesMap[2] = struct{ pos int; scale float64 }{2, mpu9250Scale}

	if axesMap != "" && axesMap != "x,y,z" {
		log.Printf("mpu9250: using custom axes map: %s", axesMap)
	}

	return nil
}

// GetName returns the sensor name.
func (mpu *MPU9250) GetName() string {
	return mpu.name
}

// GetDataRate returns the data rate in Hz.
func (mpu *MPU9250) GetDataRate() int {
	return mpu.dataRate
}

// StartMeasurements starts accelerometer measurements.
func (mpu *MPU9250) StartMeasurements() error {
	mpu.mu.Lock()
	defer mpu.mu.Unlock()

	if mpu.queryActive {
		return fmt.Errorf("mpu9250: measurements already active")
	}

	mpu.samples = make([]AccelMeasurement, 0)
	mpu.queryActive = true
	mpu.lastErrorCount = 0

	log.Printf("mpu9250 '%s': starting measurements", mpu.name)
	return nil
}

// StopMeasurements stops accelerometer measurements.
func (mpu *MPU9250) StopMeasurements() error {
	mpu.mu.Lock()
	defer mpu.mu.Unlock()

	if !mpu.queryActive {
		return nil
	}

	mpu.queryActive = false
	log.Printf("mpu9250 '%s': stopped measurements, %d samples collected",
		mpu.name, len(mpu.samples))
	return nil
}

// IsActive returns true if measurements are active.
func (mpu *MPU9250) IsActive() bool {
	mpu.mu.Lock()
	defer mpu.mu.Unlock()
	return mpu.queryActive
}

// AddSample adds a raw sample and converts to calibrated values.
func (mpu *MPU9250) AddSample(time float64, rawX, rawY, rawZ int16) {
	mpu.mu.Lock()
	defer mpu.mu.Unlock()

	if !mpu.queryActive {
		return
	}

	raw := [3]float64{float64(rawX), float64(rawY), float64(rawZ)}
	var x, y, z float64

	x = raw[mpu.axesMap[0].pos] * mpu.axesMap[0].scale
	y = raw[mpu.axesMap[1].pos] * mpu.axesMap[1].scale
	z = raw[mpu.axesMap[2].pos] * mpu.axesMap[2].scale

	mpu.samples = append(mpu.samples, AccelMeasurement{
		Time:   time,
		AccelX: x,
		AccelY: y,
		AccelZ: z,
	})
	mpu.lastSampleTime = time
}

// GetSamples returns all collected samples.
func (mpu *MPU9250) GetSamples() []AccelMeasurement {
	mpu.mu.Lock()
	defer mpu.mu.Unlock()

	result := make([]AccelMeasurement, len(mpu.samples))
	copy(result, mpu.samples)
	return result
}

// ClearSamples clears all collected samples.
func (mpu *MPU9250) ClearSamples() {
	mpu.mu.Lock()
	defer mpu.mu.Unlock()
	mpu.samples = make([]AccelMeasurement, 0)
}

// GetStatus returns the sensor status.
func (mpu *MPU9250) GetStatus() map[string]any {
	mpu.mu.Lock()
	defer mpu.mu.Unlock()

	return map[string]any{
		"name":         mpu.name,
		"type":         mpu.mpuType,
		"data_rate":    mpu.dataRate,
		"active":       mpu.queryActive,
		"sample_count": len(mpu.samples),
		"error_count":  mpu.lastErrorCount,
	}
}

// CalculateRMS calculates RMS acceleration from samples.
func (mpu *MPU9250) CalculateRMS() (x, y, z, total float64) {
	samples := mpu.GetSamples()
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
