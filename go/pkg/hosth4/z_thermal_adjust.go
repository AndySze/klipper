// Z Thermal Adjust - port of klippy/extras/z_thermal_adjust.py
//
// Adjusts Z position in real-time using a thermal probe to compensate
// for thermal expansion of the printer frame.
//
// Copyright (C) 2022 Robert Pazdzior <robertp@norbital.com>
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

const (
	kelvinToCelsius = -273.15
)

// ZThermalAdjust provides Z adjustment based on temperature.
type ZThermalAdjust struct {
	rt *runtime

	// Configuration
	tempCoeff     float64 // Temperature coefficient (mm/degC)
	offAboveZ     float64 // Z height above which adjustment is disabled
	maxZAdjustMM  float64 // Maximum Z adjustment
	smoothTime    float64 // Temperature smoothing time
	invSmoothTime float64 // 1/smoothTime

	// Temperature state
	lastTemp       float64
	measuredMin    float64
	measuredMax    float64
	smoothedTemp   float64
	lastTempTime   float64
	refTemperature float64
	refTempOverride bool

	// Z adjustment state
	zAdjustMM       float64
	lastZAdjustMM   float64
	adjustEnable    bool
	lastPosition    []float64
	zStepDist       float64

	mu sync.Mutex
}

// ZThermalAdjustConfig holds configuration for Z thermal adjust.
type ZThermalAdjustConfig struct {
	TempCoeff    float64 // mm/degC, default 0
	OffAboveZ    float64 // Z height threshold, default 99999999
	MaxZAdjust   float64 // Maximum adjustment, default 99999999
	SmoothTime   float64 // Smoothing time in seconds, default 2.0
	MinTemp      float64 // Minimum valid temperature
	MaxTemp      float64 // Maximum valid temperature
}

// DefaultZThermalAdjustConfig returns default Z thermal adjust configuration.
func DefaultZThermalAdjustConfig() ZThermalAdjustConfig {
	return ZThermalAdjustConfig{
		TempCoeff:  0,
		OffAboveZ:  99999999,
		MaxZAdjust: 99999999,
		SmoothTime: 2.0,
		MinTemp:    kelvinToCelsius,
		MaxTemp:    300,
	}
}

// newZThermalAdjust creates a new Z thermal adjuster.
func newZThermalAdjust(rt *runtime, cfg ZThermalAdjustConfig) (*ZThermalAdjust, error) {
	if cfg.TempCoeff < -1 || cfg.TempCoeff > 1 {
		return nil, fmt.Errorf("temp_coeff must be between -1 and 1")
	}
	if cfg.SmoothTime <= 0 {
		cfg.SmoothTime = 2.0
	}

	zta := &ZThermalAdjust{
		rt:            rt,
		tempCoeff:     cfg.TempCoeff,
		offAboveZ:     cfg.OffAboveZ,
		maxZAdjustMM:  cfg.MaxZAdjust,
		smoothTime:    cfg.SmoothTime,
		invSmoothTime: 1.0 / cfg.SmoothTime,
		adjustEnable:  true,
		lastPosition:  make([]float64, 4),
		zStepDist:     0.01, // Default, will be updated from stepper
	}

	return zta, nil
}

// TemperatureCallback is called when temperature is read.
func (zta *ZThermalAdjust) TemperatureCallback(readTime, temp float64) {
	zta.mu.Lock()
	defer zta.mu.Unlock()

	timeDiff := readTime - zta.lastTempTime
	zta.lastTemp = temp
	zta.lastTempTime = readTime

	tempDiff := temp - zta.smoothedTemp
	adjTime := timeDiff * zta.invSmoothTime
	if adjTime > 1 {
		adjTime = 1
	}
	zta.smoothedTemp += tempDiff * adjTime

	if zta.smoothedTemp < zta.measuredMin || zta.measuredMin == 0 {
		zta.measuredMin = zta.smoothedTemp
	}
	if zta.smoothedTemp > zta.measuredMax {
		zta.measuredMax = zta.smoothedTemp
	}
}

// GetTemp returns the current smoothed temperature.
func (zta *ZThermalAdjust) GetTemp(eventtime float64) (float64, float64) {
	zta.mu.Lock()
	defer zta.mu.Unlock()
	return zta.smoothedTemp, 0
}

// HandleHomingEnd sets reference temperature after Z homing.
func (zta *ZThermalAdjust) HandleHomingEnd(axes []int) {
	for _, axis := range axes {
		if axis == 2 { // Z axis
			zta.mu.Lock()
			zta.refTemperature = zta.smoothedTemp
			zta.refTempOverride = false
			zta.zAdjustMM = 0
			zta.mu.Unlock()
			break
		}
	}
}

// calcAdjust calculates Z adjustment for a position.
func (zta *ZThermalAdjust) calcAdjust(pos []float64) []float64 {
	if len(pos) < 3 {
		return pos
	}

	result := make([]float64, len(pos))
	copy(result, pos)

	if pos[2] < zta.offAboveZ {
		deltaT := zta.smoothedTemp - zta.refTemperature
		adjust := -1 * zta.tempCoeff * deltaT

		// Compute sign for maximum offset
		sign := 1.0
		if adjust <= 0 {
			sign = -1.0
		}

		// Only apply if adjustment exceeds step distance
		if math.Abs(adjust-zta.zAdjustMM) > zta.zStepDist {
			maxAdj := zta.maxZAdjustMM * sign
			if math.Abs(adjust) < math.Abs(maxAdj) {
				zta.zAdjustMM = adjust
			} else {
				zta.zAdjustMM = maxAdj
			}
		}
	}

	result[2] = pos[2] + zta.zAdjustMM
	zta.lastZAdjustMM = zta.zAdjustMM
	return result
}

// calcUnadjust removes Z adjustment from position.
func (zta *ZThermalAdjust) calcUnadjust(pos []float64) []float64 {
	if len(pos) < 3 {
		return pos
	}

	result := make([]float64, len(pos))
	copy(result, pos)
	result[2] = pos[2] - zta.zAdjustMM
	return result
}

// GetPosition returns the current position with thermal adjustment removed.
func (zta *ZThermalAdjust) GetPosition(pos []float64) []float64 {
	zta.mu.Lock()
	defer zta.mu.Unlock()

	position := zta.calcUnadjust(pos)
	zta.lastPosition = zta.calcAdjust(position)
	return position
}

// Move applies thermal adjustment and performs the move.
func (zta *ZThermalAdjust) Move(newpos []float64, speed float64) error {
	zta.mu.Lock()
	defer zta.mu.Unlock()

	if zta.rt == nil || zta.rt.toolhead == nil {
		return fmt.Errorf("toolhead not available")
	}

	if len(newpos) < 4 {
		return fmt.Errorf("invalid position")
	}

	var adjustedPos []float64

	// Don't apply to extrude-only moves or when disabled
	isExtrudeOnly := len(zta.lastPosition) >= 2 &&
		newpos[0] == zta.lastPosition[0] && newpos[1] == zta.lastPosition[1]

	if isExtrudeOnly || !zta.adjustEnable {
		z := newpos[2] + zta.lastZAdjustMM
		adjustedPos = []float64{newpos[0], newpos[1], z, newpos[3]}
	} else {
		adjustedPos = zta.calcAdjust(newpos)
	}

	copy(zta.lastPosition, newpos)
	return zta.rt.toolhead.move(adjustedPos, speed)
}

// SetEnable enables or disables thermal adjustment.
func (zta *ZThermalAdjust) SetEnable(enable bool) {
	zta.mu.Lock()
	defer zta.mu.Unlock()
	zta.adjustEnable = enable
}

// SetTempCoeff sets the temperature coefficient.
func (zta *ZThermalAdjust) SetTempCoeff(coeff float64) error {
	if coeff < -1 || coeff > 1 {
		return fmt.Errorf("temp_coeff must be between -1 and 1")
	}
	zta.mu.Lock()
	defer zta.mu.Unlock()
	zta.tempCoeff = coeff
	return nil
}

// SetRefTemperature sets the reference temperature manually.
func (zta *ZThermalAdjust) SetRefTemperature(temp float64) {
	zta.mu.Lock()
	defer zta.mu.Unlock()
	zta.refTemperature = temp
	zta.refTempOverride = true
}

// GetStatus returns the Z thermal adjust status.
func (zta *ZThermalAdjust) GetStatus() map[string]any {
	zta.mu.Lock()
	defer zta.mu.Unlock()
	return map[string]any{
		"temperature":              zta.smoothedTemp,
		"measured_min_temp":        zta.measuredMin,
		"measured_max_temp":        zta.measuredMax,
		"current_z_adjust":         zta.zAdjustMM,
		"z_adjust_ref_temperature": zta.refTemperature,
		"enabled":                  zta.adjustEnable,
	}
}

// cmdSetZThermalAdjust handles the SET_Z_THERMAL_ADJUST command.
func (zta *ZThermalAdjust) cmdSetZThermalAdjust(enable *bool, tempCoeff, refTemp *float64) string {
	zta.mu.Lock()
	defer zta.mu.Unlock()

	if refTemp != nil {
		zta.refTemperature = *refTemp
		zta.refTempOverride = true
	}
	if tempCoeff != nil {
		if *tempCoeff >= -1 && *tempCoeff <= 1 {
			zta.tempCoeff = *tempCoeff
		}
	}
	if enable != nil {
		zta.adjustEnable = *enable
	}

	state := "0 (disabled)"
	if zta.adjustEnable {
		state = "1 (enabled)"
	}
	override := ""
	if zta.refTempOverride {
		override = " (manual)"
	}

	msg := fmt.Sprintf("enable: %s\ntemp_coeff: %f mm/degC\nref_temp: %.2f degC%s\n-------------------\nCurrent Z temp: %.2f degC\nApplied Z adjustment: %.4f mm",
		state, zta.tempCoeff, zta.refTemperature, override, zta.smoothedTemp, zta.zAdjustMM)

	log.Printf("z_thermal_adjust: %s", msg)
	return msg
}

