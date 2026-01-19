// PID Calibrate - port of klippy/extras/pid_calibrate.py
//
// Calibration of heater PID settings
//
// Copyright (C) 2016-2018 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"math"
	"sort"
	"sync"
)

const (
	tunePIDDelta = 5.0
)

// pidParamBase is defined in temperature_fan.go

// PIDCalibrate provides PID auto-tuning functionality.
type PIDCalibrate struct {
	rt *runtime
	mu sync.Mutex
}

// newPIDCalibrate creates a new PID calibrator.
func newPIDCalibrate(rt *runtime) *PIDCalibrate {
	return &PIDCalibrate{rt: rt}
}

// Calibrate runs PID auto-tuning for a heater.
func (pc *PIDCalibrate) Calibrate(heaterName string, target float64, writeFile bool) (kp, ki, kd float64, err error) {
	pc.mu.Lock()
	defer pc.mu.Unlock()

	log.Printf("pid_calibrate: starting calibration for '%s' at target %.1f", heaterName, target)

	// Create auto-tuning controller
	_ = newControlAutoTune(1.0, target) // Assume max power = 1.0

	// Note: Full implementation would replace heater control and run heating cycles
	// This is a simplified placeholder

	// Wait for calibration to complete (would be driven by temperature callbacks)
	// For now, return error indicating not fully implemented

	return 0, 0, 0, fmt.Errorf("PID calibration requires runtime heater integration")
}

// ControlAutoTune performs PID auto-tuning using Ziegler-Nichols method.
type ControlAutoTune struct {
	heaterMaxPower  float64
	calibrateTemp   float64
	heating         bool
	peak            float64
	peakTime        float64
	peaks           []peakEntry
	lastPWM         float64
	pwmSamples      []pwmSample
	tempSamples     []tempSample
	mu              sync.Mutex
}

type peakEntry struct {
	temp float64
	time float64
}

type pwmSample struct {
	time  float64
	value float64
}

type tempSample struct {
	time float64
	temp float64
}

// newControlAutoTune creates a new auto-tune controller.
func newControlAutoTune(heaterMaxPower, target float64) *ControlAutoTune {
	return &ControlAutoTune{
		heaterMaxPower: heaterMaxPower,
		calibrateTemp:  target,
		heating:        false,
		peak:           9999999.0,
	}
}

// SetPWM sets the heater PWM value.
func (ca *ControlAutoTune) SetPWM(readTime, value float64, setPWM func(float64, float64)) {
	ca.mu.Lock()
	defer ca.mu.Unlock()

	if value != ca.lastPWM {
		ca.pwmSamples = append(ca.pwmSamples, pwmSample{time: readTime, value: value})
		ca.lastPWM = value
	}
	setPWM(readTime, value)
}

// TemperatureUpdate processes a temperature reading.
func (ca *ControlAutoTune) TemperatureUpdate(readTime, temp, targetTemp float64,
	setTarget func(float64), setPWM func(float64, float64)) {

	ca.mu.Lock()
	defer ca.mu.Unlock()

	ca.tempSamples = append(ca.tempSamples, tempSample{time: readTime, temp: temp})

	// Check if the temperature has crossed the target
	if ca.heating && temp >= targetTemp {
		ca.heating = false
		ca.checkPeaks()
		setTarget(ca.calibrateTemp - tunePIDDelta)
	} else if !ca.heating && temp <= targetTemp {
		ca.heating = true
		ca.checkPeaks()
		setTarget(ca.calibrateTemp)
	}

	// Record peaks and set PWM
	if ca.heating {
		setPWM(readTime, ca.heaterMaxPower)
		if temp < ca.peak {
			ca.peak = temp
			ca.peakTime = readTime
		}
	} else {
		setPWM(readTime, 0)
		if temp > ca.peak {
			ca.peak = temp
			ca.peakTime = readTime
		}
	}
}

// CheckBusy returns true if calibration is still in progress.
func (ca *ControlAutoTune) CheckBusy() bool {
	ca.mu.Lock()
	defer ca.mu.Unlock()
	return ca.heating || len(ca.peaks) < 12
}

// checkPeaks records a peak and calculates PID if enough peaks are collected.
func (ca *ControlAutoTune) checkPeaks() {
	ca.peaks = append(ca.peaks, peakEntry{temp: ca.peak, time: ca.peakTime})
	if ca.heating {
		ca.peak = 9999999.0
	} else {
		ca.peak = -9999999.0
	}

	if len(ca.peaks) >= 4 {
		kp, ki, kd := ca.calcPID(len(ca.peaks) - 1)
		log.Printf("pid_calibrate: intermediate Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd)
	}
}

// calcPID calculates PID parameters using Astrom-Hagglund and Ziegler-Nichols methods.
func (ca *ControlAutoTune) calcPID(pos int) (kp, ki, kd float64) {
	if pos < 2 {
		return 0, 0, 0
	}

	tempDiff := ca.peaks[pos].temp - ca.peaks[pos-1].temp
	timeDiff := ca.peaks[pos].time - ca.peaks[pos-2].time

	// Astrom-Hagglund method to estimate Ku and Tu
	amplitude := 0.5 * math.Abs(tempDiff)
	if amplitude == 0 {
		return 0, 0, 0
	}

	Ku := 4.0 * ca.heaterMaxPower / (math.Pi * amplitude)
	Tu := timeDiff

	// Ziegler-Nichols method to generate PID parameters
	Ti := 0.5 * Tu
	Td := 0.125 * Tu

	kp = 0.6 * Ku * pidParamBase
	if Ti > 0 {
		ki = kp / Ti
	}
	kd = kp * Td

	log.Printf("pid_calibrate: raw=%f/%f Ku=%f Tu=%f Kp=%f Ki=%f Kd=%f",
		tempDiff, ca.heaterMaxPower, Ku, Tu, kp, ki, kd)

	return kp, ki, kd
}

// CalcFinalPID calculates the final PID parameters from recorded peaks.
func (ca *ControlAutoTune) CalcFinalPID() (kp, ki, kd float64) {
	ca.mu.Lock()
	defer ca.mu.Unlock()

	if len(ca.peaks) < 4 {
		return 0, 0, 0
	}

	// Find median cycle time
	type cycleEntry struct {
		time float64
		pos  int
	}

	cycleTimes := make([]cycleEntry, 0)
	for pos := 4; pos < len(ca.peaks); pos++ {
		cycleTime := ca.peaks[pos].time - ca.peaks[pos-2].time
		cycleTimes = append(cycleTimes, cycleEntry{time: cycleTime, pos: pos})
	}

	if len(cycleTimes) == 0 {
		return ca.calcPID(len(ca.peaks) - 1)
	}

	sort.Slice(cycleTimes, func(i, j int) bool {
		return cycleTimes[i].time < cycleTimes[j].time
	})

	midpointPos := cycleTimes[len(cycleTimes)/2].pos
	return ca.calcPID(midpointPos)
}

// GetSamples returns collected PWM and temperature samples.
func (ca *ControlAutoTune) GetSamples() ([]pwmSample, []tempSample) {
	ca.mu.Lock()
	defer ca.mu.Unlock()
	return ca.pwmSamples, ca.tempSamples
}
