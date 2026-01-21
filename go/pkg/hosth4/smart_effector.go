// SmartEffector - port of klippy/extras/smart_effector.py
//
// Support for SmartEffector probe with programmable sensitivity
//
// Copyright (C) 2021 Dmitry Butyugin <dmbutyugin@google.com>
// Copyright (C) 2026 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// SmartEffector communication protocol baud rate
const smartEffectorBitsPerSecond = 1000.0

// SmartEffectorProbe implements the SmartEffector probe with programmable sensitivity.
// This module manages probe acceleration, recovery time, and sensitivity programming
// via a control pin that sends a bit-stream protocol to the SmartEffector device.
type SmartEffectorProbe struct {
	name         string
	probeAccel   float64
	recoveryTime float64
	controlPin   string
	invert       bool
	mcuName      string

	// OID for the control pin
	oid int

	// State
	oldMaxAccel float64

	mu sync.Mutex
}

// SmartEffectorConfig holds configuration for SmartEffector
type SmartEffectorConfig struct {
	Name         string
	ProbeAccel   float64
	RecoveryTime float64
	ControlPin   string
	Invert       bool
	MCUName      string // MCU name (default "mcu")
}

// NewSmartEffectorProbe creates a new SmartEffector probe
func NewSmartEffectorProbe(cfg SmartEffectorConfig) *SmartEffectorProbe {
	mcuName := cfg.MCUName
	if mcuName == "" {
		mcuName = "mcu"
	}

	se := &SmartEffectorProbe{
		name:         cfg.Name,
		probeAccel:   cfg.ProbeAccel,
		recoveryTime: cfg.RecoveryTime,
		controlPin:   cfg.ControlPin,
		invert:       cfg.Invert,
		mcuName:      mcuName,
	}

	log.Printf("smart_effector: initialized name=%s control_pin=%s probe_accel=%.1f recovery_time=%.3f",
		cfg.Name, cfg.ControlPin, cfg.ProbeAccel, cfg.RecoveryTime)

	return se
}

// GetProbeParams returns probe parameters
func (se *SmartEffectorProbe) GetProbeParams() map[string]interface{} {
	return map[string]interface{}{
		"probe_accel":   se.probeAccel,
		"recovery_time": se.recoveryTime,
	}
}

// GetStatus returns probe status
func (se *SmartEffectorProbe) GetStatus() map[string]interface{} {
	se.mu.Lock()
	defer se.mu.Unlock()

	return map[string]interface{}{
		"name":                   se.name,
		"probe_accel":            se.probeAccel,
		"recovery_time":          se.recoveryTime,
		"control_pin":            se.controlPin,
		"control_pin_configured": se.controlPin != "",
		"mcu_name":               se.mcuName,
	}
}

// GetConfigCommands returns the MCU config commands for this SmartEffector
func (se *SmartEffectorProbe) GetConfigCommands() []string {
	se.mu.Lock()
	defer se.mu.Unlock()

	if se.controlPin == "" {
		return nil
	}

	startVal := 0
	if se.invert {
		startVal = 1
	}

	return []string{
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			se.oid, se.controlPin, startVal, startVal, 0),
	}
}

// SetOID sets the OID for this device
func (se *SmartEffectorProbe) SetOID(oid int) {
	se.mu.Lock()
	defer se.mu.Unlock()
	se.oid = oid
}

// GetOID returns the OID
func (se *SmartEffectorProbe) GetOID() int {
	se.mu.Lock()
	defer se.mu.Unlock()
	return se.oid
}

// GetMCUName returns the MCU name
func (se *SmartEffectorProbe) GetMCUName() string {
	return se.mcuName
}

// SetProbeAccel sets the probe acceleration
func (se *SmartEffectorProbe) SetProbeAccel(accel float64) {
	se.mu.Lock()
	defer se.mu.Unlock()
	se.probeAccel = accel
	log.Printf("smart_effector: probe_accel set to %.3f", accel)
}

// GetProbeAccel returns the probe acceleration
func (se *SmartEffectorProbe) GetProbeAccel() float64 {
	se.mu.Lock()
	defer se.mu.Unlock()
	return se.probeAccel
}

// SetRecoveryTime sets the recovery time
func (se *SmartEffectorProbe) SetRecoveryTime(t float64) {
	se.mu.Lock()
	defer se.mu.Unlock()
	se.recoveryTime = t
	log.Printf("smart_effector: recovery_time set to %.3f", t)
}

// GetRecoveryTime returns the recovery time
func (se *SmartEffectorProbe) GetRecoveryTime() float64 {
	se.mu.Lock()
	defer se.mu.Unlock()
	return se.recoveryTime
}

// EncodeSensitivityCommand encodes a sensitivity value into the SmartEffector protocol.
// Returns the bit stream that should be sent via the control pin.
func EncodeSensitivityCommand(sensitivity int) ([]int, error) {
	if sensitivity < 0 || sensitivity > 255 {
		return nil, fmt.Errorf("sensitivity must be between 0 and 255")
	}

	buf := []byte{105, byte(sensitivity), byte(255 - sensitivity)}
	return encodeSmartEffectorBytes(buf), nil
}

// EncodeResetCommand encodes the reset command for SmartEffector.
// Returns the bit stream that should be sent via the control pin.
func EncodeResetCommand() []int {
	buf := []byte{131, 131}
	return encodeSmartEffectorBytes(buf)
}

// encodeSmartEffectorBytes encodes bytes into the SmartEffector bit-stream protocol.
// Each byte is sent as:
// [0 0 1 0 b7 b6 b5 b4 !b4 b3 b2 b1 b0 !b0]
func encodeSmartEffectorBytes(buf []byte) []int {
	bitStream := make([]int, 0, len(buf)*14)
	for _, b := range buf {
		// Start bits
		bitStream = append(bitStream, 0, 0, 1, 0)
		// High nibble
		bitStream = append(bitStream,
			int((b>>7)&1),
			int((b>>6)&1),
			int((b>>5)&1),
			int((b>>4)&1))
		// Inverted bit 4
		bitStream = append(bitStream, int((^b>>4)&1))
		// Low nibble
		bitStream = append(bitStream,
			int((b>>3)&1),
			int((b>>2)&1),
			int((b>>1)&1),
			int(b&1))
		// Inverted bit 0
		bitStream = append(bitStream, int((^b)&1))
	}
	return bitStream
}

// GetBitStepDuration returns the duration of each bit in seconds
func GetBitStepDuration() float64 {
	return 1.0 / smartEffectorBitsPerSecond
}
