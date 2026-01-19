// Bed tilt compensation - port of klippy/extras/bed_tilt.py
//
// Copyright (C) 2018 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// BedTilt provides bed tilt compensation.
type BedTilt struct {
	rt *runtime

	xAdjust float64 // X tilt adjustment (mm/mm)
	yAdjust float64 // Y tilt adjustment (mm/mm)
	zAdjust float64 // Z offset adjustment

	mu sync.Mutex
}

// BedTiltConfig holds configuration for bed tilt.
type BedTiltConfig struct {
	XAdjust float64
	YAdjust float64
	ZAdjust float64
}

// DefaultBedTiltConfig returns default bed tilt configuration.
func DefaultBedTiltConfig() BedTiltConfig {
	return BedTiltConfig{
		XAdjust: 0,
		YAdjust: 0,
		ZAdjust: 0,
	}
}

// newBedTilt creates a new bed tilt compensator.
func newBedTilt(rt *runtime, cfg BedTiltConfig) *BedTilt {
	return &BedTilt{
		rt:      rt,
		xAdjust: cfg.XAdjust,
		yAdjust: cfg.YAdjust,
		zAdjust: cfg.ZAdjust,
	}
}

// GetPosition returns the current position with tilt compensation removed.
// This transforms from machine coordinates to logical coordinates.
func (bt *BedTilt) GetPosition(pos []float64) []float64 {
	bt.mu.Lock()
	defer bt.mu.Unlock()

	if len(pos) < 3 {
		return pos
	}

	x, y, z := pos[0], pos[1], pos[2]
	// Remove tilt adjustment to get logical Z
	z -= x*bt.xAdjust + y*bt.yAdjust + bt.zAdjust

	result := make([]float64, len(pos))
	copy(result, pos)
	result[2] = z
	return result
}

// Move transforms the target position and performs the move.
// This transforms from logical coordinates to machine coordinates.
func (bt *BedTilt) Move(newpos []float64, speed float64) error {
	bt.mu.Lock()
	defer bt.mu.Unlock()

	if bt.rt == nil || bt.rt.toolhead == nil {
		return fmt.Errorf("toolhead not available")
	}

	if len(newpos) < 3 {
		return fmt.Errorf("invalid position")
	}

	x, y, z := newpos[0], newpos[1], newpos[2]
	// Apply tilt adjustment to get machine Z
	z += x*bt.xAdjust + y*bt.yAdjust + bt.zAdjust

	machinePos := make([]float64, len(newpos))
	copy(machinePos, newpos)
	machinePos[2] = z

	return bt.rt.toolhead.move(machinePos, speed)
}

// UpdateAdjust updates the tilt compensation values.
func (bt *BedTilt) UpdateAdjust(xAdjust, yAdjust, zAdjust float64) {
	bt.mu.Lock()
	defer bt.mu.Unlock()

	bt.xAdjust = xAdjust
	bt.yAdjust = yAdjust
	bt.zAdjust = zAdjust

	log.Printf("bed_tilt: updated adjust x=%.6f y=%.6f z=%.6f",
		xAdjust, yAdjust, zAdjust)
}

// GetAdjust returns the current tilt adjustment values.
func (bt *BedTilt) GetAdjust() (float64, float64, float64) {
	bt.mu.Lock()
	defer bt.mu.Unlock()
	return bt.xAdjust, bt.yAdjust, bt.zAdjust
}

// GetStatus returns the bed tilt status.
func (bt *BedTilt) GetStatus() map[string]any {
	bt.mu.Lock()
	defer bt.mu.Unlock()
	return map[string]any{
		"x_adjust": bt.xAdjust,
		"y_adjust": bt.yAdjust,
		"z_adjust": bt.zAdjust,
	}
}

// BedTiltCalibrate handles bed tilt calibration.
type BedTiltCalibrate struct {
	rt      *runtime
	bedTilt *BedTilt
	points  [][]float64 // Calibration points [[x, y], ...]
}

// BedTiltCalibrateConfig holds calibration configuration.
type BedTiltCalibrateConfig struct {
	Points [][]float64 // [[x1, y1], [x2, y2], [x3, y3], ...]
}

// newBedTiltCalibrate creates a new bed tilt calibrator.
func newBedTiltCalibrate(rt *runtime, bt *BedTilt, cfg BedTiltCalibrateConfig) (*BedTiltCalibrate, error) {
	if len(cfg.Points) < 3 {
		return nil, fmt.Errorf("bed_tilt calibration requires at least 3 points")
	}

	return &BedTiltCalibrate{
		rt:      rt,
		bedTilt: bt,
		points:  cfg.Points,
	}, nil
}

// Calibrate performs bed tilt calibration using the configured points.
// Returns the calculated x_adjust, y_adjust, z_adjust values.
func (btc *BedTiltCalibrate) Calibrate(probedPositions [][]float64) (float64, float64, float64, error) {
	if len(probedPositions) < 3 {
		return 0, 0, 0, fmt.Errorf("need at least 3 probed positions")
	}

	// Use coordinate descent to find optimal tilt parameters
	xAdj, yAdj, zAdj := btc.bedTilt.GetAdjust()

	// Simple gradient descent implementation
	learningRate := 0.001
	iterations := 1000

	for i := 0; i < iterations; i++ {
		// Calculate gradients
		var dX, dY, dZ float64
		for _, pos := range probedPositions {
			if len(pos) < 3 {
				continue
			}
			x, y, z := pos[0], pos[1], pos[2]
			adjusted := z - x*xAdj - y*yAdj - zAdj
			dX += -2 * adjusted * x
			dY += -2 * adjusted * y
			dZ += -2 * adjusted
		}

		// Update parameters
		n := float64(len(probedPositions))
		xAdj -= learningRate * dX / n
		yAdj -= learningRate * dY / n
		zAdj -= learningRate * dZ / n
	}

	return xAdj, yAdj, zAdj, nil
}

// cmdBedTiltCalibrate handles the BED_TILT_CALIBRATE command.
func (btc *BedTiltCalibrate) cmdBedTiltCalibrate() (string, error) {
	// In a full implementation, this would:
	// 1. Move to each calibration point
	// 2. Probe at each point
	// 3. Calculate optimal tilt parameters
	// 4. Update bed_tilt with new values

	return "BED_TILT_CALIBRATE: probe sequence would start here", nil
}

// ProbeFinalize processes probed positions and updates bed tilt.
func (btc *BedTiltCalibrate) ProbeFinalize(offsets []float64, positions [][]float64) error {
	if len(offsets) < 3 {
		return fmt.Errorf("invalid offsets")
	}

	xAdj, yAdj, zAdj, err := btc.Calibrate(positions)
	if err != nil {
		return err
	}

	// Adjust z_adjust based on probe offsets
	zOffset := offsets[2]
	zAdj = zAdj - zOffset - xAdj*offsets[0] - yAdj*offsets[1]

	// Update bed tilt
	btc.bedTilt.UpdateAdjust(xAdj, yAdj, zAdj)

	log.Printf("bed_tilt calibrate: x_adjust=%.6f y_adjust=%.6f z_adjust=%.6f",
		xAdj, yAdj, zAdj)

	return nil
}
