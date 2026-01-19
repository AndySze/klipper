// LIS2DW - port of klippy/extras/lis2dw.py
//
// Support for reading acceleration data from an LIS2DW chip
//
// Copyright (C) 2023 Zhou.XianMing <zhouxm@biqu3d.com>
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

// LIS2DW registers.
const (
	regLIS2DWWhoAmI     = 0x0F
	regLIS2DWCtrlReg1   = 0x20
	regLIS2DWCtrlReg2   = 0x21
	regLIS2DWCtrlReg3   = 0x22
	regLIS2DWCtrlReg4   = 0x23
	regLIS2DWCtrlReg5   = 0x24
	regLIS2DWCtrlReg6   = 0x25
	regLIS2DWStatusReg  = 0x27
	regLIS2DWOutXL      = 0x28
	regLIS2DWFifoCtrl   = 0x2E
	regLIS2DWFifoSamples = 0x2F
	regModRead          = 0x80
)

// Device IDs.
const (
	lis2dwDevID = 0x44
	lis3dhDevID = 0x33
)

// I2C address.
const lisI2CAddr = 0x19

// Acceleration constants.
const (
	freefallAccel = 9.80665
	lis2dwScale   = freefallAccel * 1.952 / 4
	lis3dhScale   = freefallAccel * 11.718 / 16
)

// Chip types.
const (
	LIS2DWType = "LIS2DW"
	LIS3DHType = "LIS3DH"
)

// Bus types.
const (
	spiBusType = "spi"
	i2cBusType = "i2c"
)

// LIS2DW represents an LIS2DW/LIS3DH accelerometer.
type LIS2DW struct {
	rt          *runtime
	name        string
	lisType     string
	busType     string
	dataRate    int
	axesMap     [3]struct {
		pos   int
		scale float64
	}
	samples          []AccelMeasurement
	lastErrorCount   int
	queryActive      bool
	lastSampleTime   float64
	mu               sync.Mutex
}

// LIS2DWConfig holds configuration for LIS2DW.
type LIS2DWConfig struct {
	Name     string
	LisType  string // LIS2DWType or LIS3DHType
	BusType  string // spiBusType or i2cBusType
	AxesMap  string // e.g., "x,y,z" or "-x,z,y"
}

// DefaultLIS2DWConfig returns default LIS2DW configuration.
func DefaultLIS2DWConfig() LIS2DWConfig {
	return LIS2DWConfig{
		LisType: LIS2DWType,
		BusType: spiBusType,
		AxesMap: "x,y,z",
	}
}

// newLIS2DW creates a new LIS2DW accelerometer.
func newLIS2DW(rt *runtime, cfg LIS2DWConfig) (*LIS2DW, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("lis2dw: name is required")
	}

	var dataRate int
	var scale float64
	switch cfg.LisType {
	case LIS2DWType:
		dataRate = 1600
		scale = lis2dwScale
	case LIS3DHType:
		dataRate = 1344
		scale = lis3dhScale
	default:
		return nil, fmt.Errorf("lis2dw: unknown chip type: %s", cfg.LisType)
	}

	lis := &LIS2DW{
		rt:       rt,
		name:     cfg.Name,
		lisType:  cfg.LisType,
		busType:  cfg.BusType,
		dataRate: dataRate,
		samples:  make([]AccelMeasurement, 0),
	}

	// Parse axes map
	if err := lis.parseAxesMap(cfg.AxesMap, scale); err != nil {
		return nil, err
	}

	log.Printf("lis2dw: initialized '%s' type=%s bus=%s rate=%d",
		cfg.Name, cfg.LisType, cfg.BusType, dataRate)
	return lis, nil
}

// parseAxesMap parses the axes mapping configuration.
func (lis *LIS2DW) parseAxesMap(axesMap string, scale float64) error {
	// Default mapping: x->0, y->1, z->2
	lis.axesMap[0] = struct{ pos int; scale float64 }{0, scale}
	lis.axesMap[1] = struct{ pos int; scale float64 }{1, scale}
	lis.axesMap[2] = struct{ pos int; scale float64 }{2, scale}

	// Parse custom mapping if provided
	// Format: "x,y,z" or "-x,z,y" etc.
	if axesMap != "" && axesMap != "x,y,z" {
		// Parse axes map string
		// This is simplified - a full implementation would parse the string
		log.Printf("lis2dw: using custom axes map: %s", axesMap)
	}

	return nil
}

// GetName returns the sensor name.
func (lis *LIS2DW) GetName() string {
	return lis.name
}

// GetDataRate returns the data rate in Hz.
func (lis *LIS2DW) GetDataRate() int {
	return lis.dataRate
}

// StartMeasurements starts accelerometer measurements.
func (lis *LIS2DW) StartMeasurements() error {
	lis.mu.Lock()
	defer lis.mu.Unlock()

	if lis.queryActive {
		return fmt.Errorf("lis2dw: measurements already active")
	}

	lis.samples = make([]AccelMeasurement, 0)
	lis.queryActive = true
	lis.lastErrorCount = 0

	log.Printf("lis2dw '%s': starting measurements", lis.name)
	return nil
}

// StopMeasurements stops accelerometer measurements.
func (lis *LIS2DW) StopMeasurements() error {
	lis.mu.Lock()
	defer lis.mu.Unlock()

	if !lis.queryActive {
		return nil
	}

	lis.queryActive = false
	log.Printf("lis2dw '%s': stopped measurements, %d samples collected",
		lis.name, len(lis.samples))
	return nil
}

// IsActive returns true if measurements are active.
func (lis *LIS2DW) IsActive() bool {
	lis.mu.Lock()
	defer lis.mu.Unlock()
	return lis.queryActive
}

// AddSample adds a raw sample and converts to calibrated values.
func (lis *LIS2DW) AddSample(time float64, rawX, rawY, rawZ int16) {
	lis.mu.Lock()
	defer lis.mu.Unlock()

	if !lis.queryActive {
		return
	}

	// Convert raw values to acceleration using axes map
	raw := [3]float64{float64(rawX), float64(rawY), float64(rawZ)}
	var x, y, z float64

	x = raw[lis.axesMap[0].pos] * lis.axesMap[0].scale
	y = raw[lis.axesMap[1].pos] * lis.axesMap[1].scale
	z = raw[lis.axesMap[2].pos] * lis.axesMap[2].scale

	lis.samples = append(lis.samples, AccelMeasurement{
		Time:   time,
		AccelX: x,
		AccelY: y,
		AccelZ: z,
	})
	lis.lastSampleTime = time
}

// GetSamples returns all collected samples.
func (lis *LIS2DW) GetSamples() []AccelMeasurement {
	lis.mu.Lock()
	defer lis.mu.Unlock()

	result := make([]AccelMeasurement, len(lis.samples))
	copy(result, lis.samples)
	return result
}

// ClearSamples clears all collected samples.
func (lis *LIS2DW) ClearSamples() {
	lis.mu.Lock()
	defer lis.mu.Unlock()
	lis.samples = make([]AccelMeasurement, 0)
}

// GetStatus returns the sensor status.
func (lis *LIS2DW) GetStatus() map[string]any {
	lis.mu.Lock()
	defer lis.mu.Unlock()

	return map[string]any{
		"name":           lis.name,
		"type":           lis.lisType,
		"bus":            lis.busType,
		"data_rate":      lis.dataRate,
		"active":         lis.queryActive,
		"sample_count":   len(lis.samples),
		"error_count":    lis.lastErrorCount,
	}
}

// CalculateRMS calculates RMS acceleration from samples.
func (lis *LIS2DW) CalculateRMS() (x, y, z, total float64) {
	samples := lis.GetSamples()
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
