// z_tilt implements mechanical bed tilt calibration with multiple Z steppers.
// This module adjusts the Z steppers to level the bed based on probed points.
package hosth4

import (
	"fmt"
	"math"
	"sort"
	"strings"
)

// zTilt implements the Z_TILT_ADJUST command for bed leveling.
type zTilt struct {
	rt          *runtime
	zPositions  [][2]float64 // Z stepper positions [x, y]
	probePoints [][2]float64 // Probe points from config
	speed       float64
	retries     int
	tolerance   float64
	applied     bool
}

// zTiltConfig holds configuration for z_tilt.
type zTiltConfig struct {
	zPositions    [][2]float64
	probePoints   [][2]float64
	speed         float64
	horizontalMoveZ float64
	retries       int
	retryTolerance float64
}

// newZTilt creates a new z_tilt instance from configuration.
func newZTilt(rt *runtime, cfg *configWrapper) (*zTilt, error) {
	sec, ok := cfg.section("z_tilt")
	if !ok {
		return nil, nil // z_tilt not configured
	}

	// Parse z_positions (required)
	zPosStr, ok := sec["z_positions"]
	if !ok {
		return nil, fmt.Errorf("z_tilt: missing z_positions")
	}
	zPositions, err := parsePointList(zPosStr)
	if err != nil {
		return nil, fmt.Errorf("z_tilt: invalid z_positions: %w", err)
	}
	if len(zPositions) < 2 {
		return nil, fmt.Errorf("z_tilt: z_positions requires at least 2 points")
	}

	// Parse probe points (required)
	probeStr, ok := sec["points"]
	if !ok {
		return nil, fmt.Errorf("z_tilt: missing points")
	}
	probePoints, err := parsePointList(probeStr)
	if err != nil {
		return nil, fmt.Errorf("z_tilt: invalid points: %w", err)
	}
	if len(probePoints) < 2 {
		return nil, fmt.Errorf("z_tilt: points requires at least 2 probe points")
	}

	// Parse optional parameters
	speed := 50.0
	if s, ok := sec["speed"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &speed); err != nil {
			return nil, fmt.Errorf("z_tilt: invalid speed: %s", s)
		}
	}

	retries := 0
	if r, ok := sec["retries"]; ok {
		if _, err := fmt.Sscanf(r, "%d", &retries); err != nil {
			return nil, fmt.Errorf("z_tilt: invalid retries: %s", r)
		}
	}

	tolerance := 0.0
	if t, ok := sec["retry_tolerance"]; ok {
		if _, err := fmt.Sscanf(t, "%f", &tolerance); err != nil {
			return nil, fmt.Errorf("z_tilt: invalid retry_tolerance: %s", t)
		}
	}

	return &zTilt{
		rt:          rt,
		zPositions:  zPositions,
		probePoints: probePoints,
		speed:       speed,
		retries:     retries,
		tolerance:   tolerance,
		applied:     false,
	}, nil
}

// cmdZTiltAdjust handles the Z_TILT_ADJUST command.
func (zt *zTilt) cmdZTiltAdjust(args map[string]string) error {
	zt.applied = false

	// Get retries from args or use default
	retries := zt.retries
	if r, ok := args["RETRIES"]; ok {
		if _, err := fmt.Sscanf(r, "%d", &retries); err != nil {
			return fmt.Errorf("invalid RETRIES: %s", r)
		}
	}

	tolerance := zt.tolerance
	if t, ok := args["RETRY_TOLERANCE"]; ok {
		if _, err := fmt.Sscanf(t, "%f", &tolerance); err != nil {
			return fmt.Errorf("invalid RETRY_TOLERANCE: %s", t)
		}
	}

	// Probe all points
	positions := make([][3]float64, len(zt.probePoints))
	for i, point := range zt.probePoints {
		// Move to probe point
		targetPos := make([]float64, len(zt.rt.toolhead.commandedPos))
		copy(targetPos, zt.rt.toolhead.commandedPos)
		targetPos[0] = point[0]
		targetPos[1] = point[1]

		if err := zt.rt.toolhead.move(targetPos, zt.speed); err != nil {
			return fmt.Errorf("failed to move to probe point %d: %w", i, err)
		}

		// Simulate probe (in real implementation, this would trigger the probe)
		// For now, we just record the current position
		z := zt.rt.toolhead.commandedPos[2]
		positions[i] = [3]float64{point[0], point[1], z}
	}

	// Calculate adjustments using coordinate descent
	adjustments, err := zt.calculateAdjustments(positions)
	if err != nil {
		return fmt.Errorf("failed to calculate adjustments: %w", err)
	}

	// Apply adjustments to Z steppers
	if err := zt.applyAdjustments(adjustments); err != nil {
		return fmt.Errorf("failed to apply adjustments: %w", err)
	}

	// Check if we need to retry
	zValues := make([]float64, len(positions))
	for i, pos := range positions {
		zValues[i] = pos[2]
	}

	errorRange := maxFloat(zValues) - minFloat(zValues)
	if retries > 0 && errorRange > tolerance {
		zt.rt.gcodeRespond(fmt.Sprintf("Z_TILT_ADJUST: error range %.6f exceeds tolerance %.6f, retry needed", errorRange, tolerance))
	} else {
		zt.applied = true
		zt.rt.gcodeRespond(fmt.Sprintf("Z_TILT_ADJUST: adjustments applied, error range: %.6f", errorRange))
	}

	return nil
}

// calculateAdjustments uses coordinate descent to find optimal Z adjustments.
func (zt *zTilt) calculateAdjustments(positions [][3]float64) ([]float64, error) {
	if len(positions) < 2 {
		return nil, fmt.Errorf("need at least 2 positions")
	}

	// Initial parameters: x_adjust, y_adjust, z_adjust
	params := map[string]float64{
		"x_adjust": 0.0,
		"y_adjust": 0.0,
		"z_adjust": positions[0][2], // Use first probe Z as starting offset
	}

	// Adjusted height function
	adjustedHeight := func(pos [3]float64, p map[string]float64) float64 {
		return pos[2] - pos[0]*p["x_adjust"] - pos[1]*p["y_adjust"] - p["z_adjust"]
	}

	// Error function
	errorFunc := func(p map[string]float64) float64 {
		totalError := 0.0
		for _, pos := range positions {
			h := adjustedHeight(pos, p)
			totalError += h * h
		}
		return totalError
	}

	// Simple coordinate descent
	newParams := coordinateDescent([]string{"x_adjust", "y_adjust", "z_adjust"}, params, errorFunc)

	// Calculate adjustments for each Z stepper position
	xAdjust := newParams["x_adjust"]
	yAdjust := newParams["y_adjust"]
	zAdjust := newParams["z_adjust"] - positions[0][2]

	adjustments := make([]float64, len(zt.zPositions))
	for i, zPos := range zt.zPositions {
		adjustments[i] = zPos[0]*xAdjust + zPos[1]*yAdjust + zAdjust
	}

	return adjustments, nil
}

// applyAdjustments applies the calculated adjustments to the Z steppers.
func (zt *zTilt) applyAdjustments(adjustments []float64) error {
	// Report adjustments
	var msgs []string
	for i, adj := range adjustments {
		msgs = append(msgs, fmt.Sprintf("stepper_z%d = %.6f", i, adj))
	}
	zt.rt.gcodeRespond("Making the following Z adjustments:\n" + strings.Join(msgs, "\n"))

	// Sort steppers by adjustment (lowest first)
	type stepperAdj struct {
		index int
		adj   float64
	}
	sorted := make([]stepperAdj, len(adjustments))
	for i, adj := range adjustments {
		sorted[i] = stepperAdj{i, -adj}
	}
	sort.Slice(sorted, func(i, j int) bool {
		return sorted[i].adj < sorted[j].adj
	})

	// Move each stepper to level the bed
	curPos := zt.rt.toolhead.commandedPos[2]
	zLow := curPos - sorted[0].adj

	for i := 0; i < len(sorted)-1; i++ {
		nextZ := zLow + sorted[i+1].adj
		targetPos := make([]float64, len(zt.rt.toolhead.commandedPos))
		copy(targetPos, zt.rt.toolhead.commandedPos)
		targetPos[2] = nextZ

		if err := zt.rt.toolhead.move(targetPos, zt.speed); err != nil {
			return fmt.Errorf("failed to adjust Z stepper %d: %w", sorted[i].index, err)
		}
	}

	// Update final position
	targetPos := make([]float64, len(zt.rt.toolhead.commandedPos))
	copy(targetPos, zt.rt.toolhead.commandedPos)
	targetPos[2] = curPos + sorted[0].adj
	zt.rt.toolhead.setPosition(targetPos, "")

	return nil
}

// getStatus returns the current status of z_tilt.
func (zt *zTilt) getStatus() map[string]interface{} {
	return map[string]interface{}{
		"applied": zt.applied,
	}
}

// reset resets the applied state (called on motor off).
func (zt *zTilt) reset() {
	zt.applied = false
}

// parsePointList parses a list of x,y points from a config string.
func parsePointList(s string) ([][2]float64, error) {
	var points [][2]float64

	// Split by newlines first, then by commas
	lines := strings.Split(s, "\n")
	for _, line := range lines {
		line = strings.TrimSpace(line)
		if line == "" {
			continue
		}

		// Parse x, y from the line
		var x, y float64
		parts := strings.Split(line, ",")
		if len(parts) == 2 {
			if _, err := fmt.Sscanf(strings.TrimSpace(parts[0]), "%f", &x); err != nil {
				return nil, fmt.Errorf("invalid x coordinate: %s", parts[0])
			}
			if _, err := fmt.Sscanf(strings.TrimSpace(parts[1]), "%f", &y); err != nil {
				return nil, fmt.Errorf("invalid y coordinate: %s", parts[1])
			}
			points = append(points, [2]float64{x, y})
		}
	}

	return points, nil
}

// coordinateDescent performs simple coordinate descent optimization.
func coordinateDescent(keys []string, params map[string]float64, errorFunc func(map[string]float64) float64) map[string]float64 {
	result := make(map[string]float64)
	for k, v := range params {
		result[k] = v
	}

	stepSize := 0.01
	minStep := 1e-8

	for stepSize > minStep {
		improved := false
		for _, key := range keys {
			origValue := result[key]
			origError := errorFunc(result)

			// Try increasing
			result[key] = origValue + stepSize
			newError := errorFunc(result)
			if newError < origError {
				improved = true
				continue
			}

			// Try decreasing
			result[key] = origValue - stepSize
			newError = errorFunc(result)
			if newError < origError {
				improved = true
				continue
			}

			// No improvement, restore original
			result[key] = origValue
		}

		if !improved {
			stepSize *= 0.5
		}
	}

	return result
}

func maxFloat(vals []float64) float64 {
	if len(vals) == 0 {
		return 0
	}
	max := vals[0]
	for _, v := range vals[1:] {
		if v > max {
			max = v
		}
	}
	return max
}

func minFloat(vals []float64) float64 {
	if len(vals) == 0 {
		return 0
	}
	min := vals[0]
	for _, v := range vals[1:] {
		if v < min {
			min = v
		}
	}
	return min
}

// Ensure math is used
var _ = math.Abs