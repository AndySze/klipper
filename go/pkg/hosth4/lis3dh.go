// LIS3DH - port of klippy/extras/lis3dh.py
//
// Support for reading acceleration data from an LIS3DH chip
//
// Copyright (C) 2024 Luke Vuksta <wulfstawulfsta@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"log"
)

// LIS3DH is an alias for LIS2DW configured in LIS3DH mode.
// The LIS3DH accelerometer shares the same driver as LIS2DW with different
// registers and scaling factors.
type LIS3DH = LIS2DW

// LIS3DHConfig holds configuration for LIS3DH accelerometer.
type LIS3DHConfig struct {
	Name    string
	BusType string // "spi" or "i2c"
	AxesMap string // e.g., "x,y,z" or "-x,z,y"
}

// DefaultLIS3DHConfig returns default LIS3DH configuration.
func DefaultLIS3DHConfig() LIS3DHConfig {
	return LIS3DHConfig{
		BusType: spiBusType,
		AxesMap: "x,y,z",
	}
}

// newLIS3DH creates a new LIS3DH accelerometer.
// LIS3DH uses the same driver as LIS2DW but with different scaling and data rate.
func newLIS3DH(rt *runtime, cfg LIS3DHConfig) (*LIS3DH, error) {
	lis2dwCfg := LIS2DWConfig{
		Name:    cfg.Name,
		LisType: LIS3DHType,
		BusType: cfg.BusType,
		AxesMap: cfg.AxesMap,
	}

	lis, err := newLIS2DW(rt, lis2dwCfg)
	if err != nil {
		return nil, err
	}

	log.Printf("lis3dh: initialized '%s' (using LIS3DH mode)", cfg.Name)
	return lis, nil
}

// AccelerometerManager manages multiple accelerometer chips.
type AccelerometerManager struct {
	rt           *runtime
	adxl345      map[string]*ADXL345
	lis2dw       map[string]*LIS2DW
	icm20948     map[string]*ICM20948
	mpu9250      map[string]*MPU9250
}

// newAccelerometerManager creates a new accelerometer manager.
func newAccelerometerManager(rt *runtime) *AccelerometerManager {
	return &AccelerometerManager{
		rt:       rt,
		adxl345:  make(map[string]*ADXL345),
		lis2dw:   make(map[string]*LIS2DW),
		icm20948: make(map[string]*ICM20948),
		mpu9250:  make(map[string]*MPU9250),
	}
}

// RegisterADXL345 registers an ADXL345 accelerometer.
func (am *AccelerometerManager) RegisterADXL345(cfg ADXL345Config) (*ADXL345, error) {
	accel, err := newADXL345(am.rt, cfg)
	if err != nil {
		return nil, err
	}
	am.adxl345[cfg.Name] = accel
	return accel, nil
}

// RegisterLIS2DW registers an LIS2DW accelerometer.
func (am *AccelerometerManager) RegisterLIS2DW(cfg LIS2DWConfig) (*LIS2DW, error) {
	accel, err := newLIS2DW(am.rt, cfg)
	if err != nil {
		return nil, err
	}
	am.lis2dw[cfg.Name] = accel
	return accel, nil
}

// RegisterLIS3DH registers an LIS3DH accelerometer.
func (am *AccelerometerManager) RegisterLIS3DH(cfg LIS3DHConfig) (*LIS3DH, error) {
	accel, err := newLIS3DH(am.rt, cfg)
	if err != nil {
		return nil, err
	}
	am.lis2dw[cfg.Name] = accel // LIS3DH uses LIS2DW struct
	return accel, nil
}

// RegisterICM20948 registers an ICM20948 accelerometer.
func (am *AccelerometerManager) RegisterICM20948(cfg ICM20948Config) (*ICM20948, error) {
	accel, err := newICM20948(am.rt, cfg)
	if err != nil {
		return nil, err
	}
	am.icm20948[cfg.Name] = accel
	return accel, nil
}

// RegisterMPU9250 registers an MPU9250 accelerometer.
func (am *AccelerometerManager) RegisterMPU9250(cfg MPU9250Config) (*MPU9250, error) {
	accel, err := newMPU9250(am.rt, cfg)
	if err != nil {
		return nil, err
	}
	am.mpu9250[cfg.Name] = accel
	return accel, nil
}

// GetADXL345 returns an ADXL345 by name.
func (am *AccelerometerManager) GetADXL345(name string) *ADXL345 {
	return am.adxl345[name]
}

// GetLIS2DW returns an LIS2DW/LIS3DH by name.
func (am *AccelerometerManager) GetLIS2DW(name string) *LIS2DW {
	return am.lis2dw[name]
}

// GetICM20948 returns an ICM20948 by name.
func (am *AccelerometerManager) GetICM20948(name string) *ICM20948 {
	return am.icm20948[name]
}

// GetMPU9250 returns an MPU9250 by name.
func (am *AccelerometerManager) GetMPU9250(name string) *MPU9250 {
	return am.mpu9250[name]
}

// GetAllAccelerometers returns info about all registered accelerometers.
func (am *AccelerometerManager) GetAllAccelerometers() []string {
	var names []string
	for name := range am.adxl345 {
		names = append(names, "adxl345:"+name)
	}
	for name := range am.lis2dw {
		names = append(names, "lis2dw:"+name)
	}
	for name := range am.icm20948 {
		names = append(names, "icm20948:"+name)
	}
	for name := range am.mpu9250 {
		names = append(names, "mpu9250:"+name)
	}
	return names
}

// GetStatus returns status of all accelerometers.
func (am *AccelerometerManager) GetStatus() map[string]any {
	status := make(map[string]any)

	adxlStatus := make(map[string]any)
	for name, accel := range am.adxl345 {
		adxlStatus[name] = accel.GetStatus()
	}
	if len(adxlStatus) > 0 {
		status["adxl345"] = adxlStatus
	}

	lisStatus := make(map[string]any)
	for name, accel := range am.lis2dw {
		lisStatus[name] = accel.GetStatus()
	}
	if len(lisStatus) > 0 {
		status["lis2dw"] = lisStatus
	}

	icmStatus := make(map[string]any)
	for name, accel := range am.icm20948 {
		icmStatus[name] = accel.GetStatus()
	}
	if len(icmStatus) > 0 {
		status["icm20948"] = icmStatus
	}

	mpuStatus := make(map[string]any)
	for name, accel := range am.mpu9250 {
		mpuStatus[name] = accel.GetStatus()
	}
	if len(mpuStatus) > 0 {
		status["mpu9250"] = mpuStatus
	}

	return status
}
