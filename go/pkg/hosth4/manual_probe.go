// Manual Probe - port of klippy/extras/manual_probe.py
//
// Helper script for manual z height probing
//
// Copyright (C) 2019 Kevin O'Connor <kevin@koconnor.net>
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
	zBobMinimum = 0.500
	bisectMax   = 0.200
)

// ManualProbe provides manual Z probing functionality.
type ManualProbe struct {
	rt                   *runtime
	zPositionEndstop     *float64
	zEndstopConfigName   string
	aPositionEndstop     *float64
	bPositionEndstop     *float64
	cPositionEndstop     *float64
	status               ManualProbeStatus
	activeHelper         *ManualProbeHelper
	mu                   sync.Mutex
}

// ManualProbeStatus holds the current probe status.
type ManualProbeStatus struct {
	IsActive       bool
	ZPosition      *float64
	ZPositionLower *float64
	ZPositionUpper *float64
}

// ManualProbeConfig holds configuration for manual probe.
type ManualProbeConfig struct {
	ZPositionEndstop   *float64 // Z endstop position
	ZEndstopConfigName string   // Config section name for Z endstop
}

// newManualProbe creates a new manual probe handler.
func newManualProbe(rt *runtime, cfg ManualProbeConfig) *ManualProbe {
	mp := &ManualProbe{
		rt:                 rt,
		zPositionEndstop:   cfg.ZPositionEndstop,
		zEndstopConfigName: cfg.ZEndstopConfigName,
	}

	mp.ResetStatus()
	log.Printf("manual_probe: initialized")
	return mp
}

// ResetStatus resets the probe status.
func (mp *ManualProbe) ResetStatus() {
	mp.mu.Lock()
	defer mp.mu.Unlock()

	mp.status = ManualProbeStatus{
		IsActive:       false,
		ZPosition:      nil,
		ZPositionLower: nil,
		ZPositionUpper: nil,
	}
}

// GetStatus returns the current probe status.
func (mp *ManualProbe) GetStatus() map[string]any {
	mp.mu.Lock()
	defer mp.mu.Unlock()

	return map[string]any{
		"is_active":        mp.status.IsActive,
		"z_position":       mp.status.ZPosition,
		"z_position_lower": mp.status.ZPositionLower,
		"z_position_upper": mp.status.ZPositionUpper,
	}
}

// StartProbe starts a manual probe session.
func (mp *ManualProbe) StartProbe(speed float64, callback func([]float64)) error {
	mp.mu.Lock()
	defer mp.mu.Unlock()

	if mp.activeHelper != nil {
		return fmt.Errorf("already in a manual Z probe")
	}

	mp.activeHelper = newManualProbeHelper(mp, speed, callback)
	mp.status.IsActive = true

	log.Printf("manual_probe: starting probe session")
	return nil
}

// UpdateStatus updates the probe status from the helper.
func (mp *ManualProbe) UpdateStatus(zPos float64, lower, upper *float64) {
	mp.mu.Lock()
	defer mp.mu.Unlock()

	mp.status.ZPosition = &zPos
	mp.status.ZPositionLower = lower
	mp.status.ZPositionUpper = upper
}

// Finalize ends the probe session.
func (mp *ManualProbe) Finalize() {
	mp.mu.Lock()
	defer mp.mu.Unlock()

	mp.activeHelper = nil
	mp.status.IsActive = false
}

// IsActive returns true if a probe session is active.
func (mp *ManualProbe) IsActive() bool {
	mp.mu.Lock()
	defer mp.mu.Unlock()
	return mp.activeHelper != nil
}

// GetHelper returns the active probe helper.
func (mp *ManualProbe) GetHelper() *ManualProbeHelper {
	mp.mu.Lock()
	defer mp.mu.Unlock()
	return mp.activeHelper
}

// ManualProbeHelper assists with manual Z probing.
type ManualProbeHelper struct {
	manualProbe      *ManualProbe
	speed            float64
	callback         func([]float64)
	pastPositions    []float64
	startPosition    []float64
	lastToolheadPos  []float64
	lastKinematicsPos []float64
}

// newManualProbeHelper creates a new probe helper.
func newManualProbeHelper(mp *ManualProbe, speed float64, cb func([]float64)) *ManualProbeHelper {
	return &ManualProbeHelper{
		manualProbe:   mp,
		speed:         speed,
		callback:      cb,
		pastPositions: make([]float64, 0),
	}
}

// SetStartPosition sets the starting position.
func (mph *ManualProbeHelper) SetStartPosition(pos []float64) {
	mph.startPosition = make([]float64, len(pos))
	copy(mph.startPosition, pos)
}

// TestZ moves to a new Z position based on the command.
func (mph *ManualProbeHelper) TestZ(cmd string, currentZ float64) (float64, error) {
	// Store current position
	insertPos := sort.SearchFloat64s(mph.pastPositions, currentZ)
	if insertPos >= len(mph.pastPositions) || mph.pastPositions[insertPos] != currentZ {
		mph.pastPositions = append(mph.pastPositions, 0)
		copy(mph.pastPositions[insertPos+1:], mph.pastPositions[insertPos:])
		mph.pastPositions[insertPos] = currentZ
	}

	var nextZ float64

	switch cmd {
	case "++":
		checkZ := 9999999999999.9
		if insertPos < len(mph.pastPositions)-1 {
			checkZ = mph.pastPositions[insertPos+1]
		}
		nextZ = min(checkZ, currentZ+bisectMax)

	case "+":
		checkZ := 9999999999999.9
		if insertPos < len(mph.pastPositions)-1 {
			checkZ = mph.pastPositions[insertPos+1]
		}
		nextZ = min((checkZ+currentZ)/2.0, currentZ+bisectMax)

	case "--":
		checkZ := -9999999999999.9
		if insertPos > 0 {
			checkZ = mph.pastPositions[insertPos-1]
		}
		nextZ = max(checkZ, currentZ-bisectMax)

	case "-":
		checkZ := -9999999999999.9
		if insertPos > 0 {
			checkZ = mph.pastPositions[insertPos-1]
		}
		nextZ = max((checkZ+currentZ)/2.0, currentZ-bisectMax)

	default:
		return currentZ, fmt.Errorf("invalid TESTZ command: %s", cmd)
	}

	return nextZ, nil
}

// ReportZStatus reports the current Z status.
func (mph *ManualProbeHelper) ReportZStatus(zPos float64) (lower, upper *float64) {
	pp := mph.pastPositions
	nextPos := sort.SearchFloat64s(pp, zPos)
	prevPos := nextPos - 1

	if nextPos < len(pp) && pp[nextPos] == zPos {
		nextPos++
	}

	if prevPos >= 0 {
		val := pp[prevPos]
		lower = &val
	}
	if nextPos < len(pp) {
		val := pp[nextPos]
		upper = &val
	}

	mph.manualProbe.UpdateStatus(zPos, lower, upper)
	return lower, upper
}

// Accept accepts the current Z position.
func (mph *ManualProbeHelper) Accept(currentPos []float64) error {
	if len(mph.startPosition) < 3 || len(currentPos) < 3 {
		return fmt.Errorf("invalid position data")
	}

	// Check if position is valid
	if currentPos[0] != mph.startPosition[0] ||
		currentPos[1] != mph.startPosition[1] ||
		currentPos[2] >= mph.startPosition[2] {
		return fmt.Errorf("invalid position - use TESTZ to adjust before ACCEPT")
	}

	mph.finalize(currentPos)
	return nil
}

// Abort aborts the probe session.
func (mph *ManualProbeHelper) Abort() {
	mph.finalize(nil)
}

// finalize ends the probe session.
func (mph *ManualProbeHelper) finalize(kinPos []float64) {
	mph.manualProbe.ResetStatus()
	mph.manualProbe.Finalize()

	if mph.callback != nil {
		mph.callback(kinPos)
	}
}

// GetSpeed returns the configured probe speed.
func (mph *ManualProbeHelper) GetSpeed() float64 {
	return mph.speed
}
