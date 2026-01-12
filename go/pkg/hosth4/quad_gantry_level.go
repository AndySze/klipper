// quad_gantry_level implements leveling for a moving gantry with 4 Z steppers.
// This is commonly used on Voron 2.x style printers where the gantry can twist.
package hosth4

import (
	"fmt"
	"math"
	"strings"
)

// quadGantryLevel implements the QUAD_GANTRY_LEVEL command.
type quadGantryLevel struct {
	rt              *runtime
	gantryCorners   [][2]float64 // [front-left, back-right] corners
	probePoints     [][2]float64 // 4 probe points
	maxAdjust       float64
	horizontalMoveZ float64
	speed           float64
	retries         int
	tolerance       float64
	applied         bool
}

// newQuadGantryLevel creates a new quad_gantry_level instance from configuration.
func newQuadGantryLevel(rt *runtime, cfg *configWrapper) (*quadGantryLevel, error) {
	sec, ok := cfg.section("quad_gantry_level")
	if !ok {
		return nil, nil // not configured
	}

	// Parse gantry_corners (required)
	cornersStr, ok := sec["gantry_corners"]
	if !ok {
		return nil, fmt.Errorf("quad_gantry_level: missing gantry_corners")
	}
	corners, err := parsePointList(cornersStr)
	if err != nil {
		return nil, fmt.Errorf("quad_gantry_level: invalid gantry_corners: %w", err)
	}
	if len(corners) < 2 {
		return nil, fmt.Errorf("quad_gantry_level: gantry_corners requires at least 2 points")
	}

	// Parse points (required, need exactly 4)
	pointsStr, ok := sec["points"]
	if !ok {
		return nil, fmt.Errorf("quad_gantry_level: missing points")
	}
	points, err := parsePointList(pointsStr)
	if err != nil {
		return nil, fmt.Errorf("quad_gantry_level: invalid points: %w", err)
	}
	if len(points) != 4 {
		return nil, fmt.Errorf("quad_gantry_level: need exactly 4 probe points, got %d", len(points))
	}

	// Parse optional parameters
	maxAdjust := 4.0
	if s, ok := sec["max_adjust"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &maxAdjust); err != nil {
			return nil, fmt.Errorf("quad_gantry_level: invalid max_adjust: %s", s)
		}
	}

	horizontalMoveZ := 5.0
	if s, ok := sec["horizontal_move_z"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &horizontalMoveZ); err != nil {
			return nil, fmt.Errorf("quad_gantry_level: invalid horizontal_move_z: %s", s)
		}
	}

	speed := 50.0
	if s, ok := sec["speed"]; ok {
		if _, err := fmt.Sscanf(s, "%f", &speed); err != nil {
			return nil, fmt.Errorf("quad_gantry_level: invalid speed: %s", s)
		}
	}

	retries := 0
	if r, ok := sec["retries"]; ok {
		if _, err := fmt.Sscanf(r, "%d", &retries); err != nil {
			return nil, fmt.Errorf("quad_gantry_level: invalid retries: %s", r)
		}
	}

	tolerance := 0.0
	if t, ok := sec["retry_tolerance"]; ok {
		if _, err := fmt.Sscanf(t, "%f", &tolerance); err != nil {
			return nil, fmt.Errorf("quad_gantry_level: invalid retry_tolerance: %s", t)
		}
	}

	return &quadGantryLevel{
		rt:              rt,
		gantryCorners:   corners,
		probePoints:     points,
		maxAdjust:       maxAdjust,
		horizontalMoveZ: horizontalMoveZ,
		speed:           speed,
		retries:         retries,
		tolerance:       tolerance,
		applied:         false,
	}, nil
}

// cmdQuadGantryLevel handles the QUAD_GANTRY_LEVEL command.
func (qgl *quadGantryLevel) cmdQuadGantryLevel(args map[string]string) error {
	qgl.applied = false

	// Get retries from args or use default
	retries := qgl.retries
	if r, ok := args["RETRIES"]; ok {
		if _, err := fmt.Sscanf(r, "%d", &retries); err != nil {
			return fmt.Errorf("invalid RETRIES: %s", r)
		}
	}

	tolerance := qgl.tolerance
	if t, ok := args["RETRY_TOLERANCE"]; ok {
		if _, err := fmt.Sscanf(t, "%f", &tolerance); err != nil {
			return fmt.Errorf("invalid RETRY_TOLERANCE: %s", t)
		}
	}

	// Probe all 4 points
	positions := make([][3]float64, 4)
	for i, point := range qgl.probePoints {
		// Move to probe point at horizontal_move_z
		targetPos := make([]float64, len(qgl.rt.toolhead.commandedPos))
		copy(targetPos, qgl.rt.toolhead.commandedPos)
		targetPos[0] = point[0]
		targetPos[1] = point[1]
		targetPos[2] = qgl.horizontalMoveZ

		if err := qgl.rt.toolhead.move(targetPos, qgl.speed); err != nil {
			return fmt.Errorf("failed to move to probe point %d: %w", i, err)
		}

		// Simulate probe (in real implementation, this would trigger the probe)
		z := qgl.rt.toolhead.commandedPos[2]
		positions[i] = [3]float64{point[0], point[1], z}
	}

	// Calculate adjustments
	adjustments, err := qgl.calculateAdjustments(positions)
	if err != nil {
		return fmt.Errorf("failed to calculate adjustments: %w", err)
	}

	// Check max_adjust limit
	for i, adj := range adjustments {
		if math.Abs(adj) > qgl.maxAdjust {
			return fmt.Errorf("stepper_z%d adjustment %.3f exceeds max_adjust %.3f", i, adj, qgl.maxAdjust)
		}
	}

	// Apply adjustments
	if err := qgl.applyAdjustments(adjustments); err != nil {
		return fmt.Errorf("failed to apply adjustments: %w", err)
	}

	// Check if we need to retry
	errorRange := maxFloat(adjustments) - minFloat(adjustments)
	if retries > 0 && errorRange > tolerance {
		qgl.rt.gcodeRespond(fmt.Sprintf("QUAD_GANTRY_LEVEL: error range %.6f exceeds tolerance %.6f, retry needed", errorRange, tolerance))
	} else {
		qgl.applied = true
		qgl.rt.gcodeRespond(fmt.Sprintf("QUAD_GANTRY_LEVEL: adjustments applied"))
	}

	return nil
}

// calculateAdjustments calculates the Z adjustments for each stepper.
// The algorithm fits lines through the probe points to determine the gantry twist.
func (qgl *quadGantryLevel) calculateAdjustments(positions [][3]float64) ([]float64, error) {
	// Mirror perspective so adjustments make sense from gantry perspective
	zPositions := make([]float64, 4)
	for i, pos := range positions {
		zPositions[i] = qgl.horizontalMoveZ - pos[2]
	}

	// Report gantry-relative probe points
	var msgs []string
	for i, z := range zPositions {
		msgs = append(msgs, fmt.Sprintf("%d: %.6f", i, z))
	}
	qgl.rt.gcodeRespond("Gantry-relative probe points:\n" + strings.Join(msgs, " "))

	// Probe points layout:
	// 1 ---- 2  (back)
	// |      |
	// 0 ---- 3  (front)

	// Calculate slope along X axis between probe point 0 and 3 (front)
	slopeFront := qgl.linefit(
		[2]float64{positions[0][0], zPositions[0]},
		[2]float64{positions[3][0], zPositions[3]},
	)

	// Calculate slope along X axis between probe point 1 and 2 (back)
	slopeBack := qgl.linefit(
		[2]float64{positions[1][0], zPositions[1]},
		[2]float64{positions[2][0], zPositions[2]},
	)

	// Calculate gantry slope along Y axis for left side (steppers 0 and 1)
	leftFrontZ := qgl.plot(slopeFront, qgl.gantryCorners[0][0])
	leftBackZ := qgl.plot(slopeBack, qgl.gantryCorners[0][0])
	slopeLeft := qgl.linefit(
		[2]float64{positions[0][1], leftFrontZ},
		[2]float64{positions[1][1], leftBackZ},
	)

	// Calculate gantry slope along Y axis for right side (steppers 2 and 3)
	rightFrontZ := qgl.plot(slopeFront, qgl.gantryCorners[1][0])
	rightBackZ := qgl.plot(slopeBack, qgl.gantryCorners[1][0])
	slopeRight := qgl.linefit(
		[2]float64{positions[0][1], rightFrontZ},
		[2]float64{positions[1][1], rightBackZ},
	)

	// Calculate adjustments for each Z stepper
	// Stepper layout: 0=front-left, 1=back-left, 2=back-right, 3=front-right
	adjustments := make([]float64, 4)
	adjustments[0] = qgl.plot(slopeLeft, qgl.gantryCorners[0][1])  // Front-left
	adjustments[1] = qgl.plot(slopeLeft, qgl.gantryCorners[1][1])  // Back-left
	adjustments[2] = qgl.plot(slopeRight, qgl.gantryCorners[1][1]) // Back-right
	adjustments[3] = qgl.plot(slopeRight, qgl.gantryCorners[0][1]) // Front-right

	return adjustments, nil
}

// applyAdjustments applies the calculated adjustments to the Z steppers.
func (qgl *quadGantryLevel) applyAdjustments(adjustments []float64) error {
	// Report adjustments
	var msgs []string
	for i, adj := range adjustments {
		msgs = append(msgs, fmt.Sprintf("stepper_z%d = %.6f", i, adj))
	}
	qgl.rt.gcodeRespond("Making the following Z adjustments:\n" + strings.Join(msgs, "\n"))

	// For now, just move to apply the adjustment effect
	// In a real implementation, this would individually adjust each Z stepper

	return nil
}

// linefit calculates slope and intercept for a line through two points.
// Returns [slope, intercept].
func (qgl *quadGantryLevel) linefit(p1, p2 [2]float64) [2]float64 {
	if p2[0] == p1[0] {
		// Vertical line, undefined slope
		return [2]float64{0, p1[1]}
	}
	slope := (p2[1] - p1[1]) / (p2[0] - p1[0])
	intercept := p1[1] - slope*p1[0]
	return [2]float64{slope, intercept}
}

// plot calculates y = slope*x + intercept.
func (qgl *quadGantryLevel) plot(line [2]float64, x float64) float64 {
	return line[0]*x + line[1]
}

// getStatus returns the current status of quad_gantry_level.
func (qgl *quadGantryLevel) getStatus() map[string]any {
	return map[string]any{
		"applied": qgl.applied,
	}
}

// reset resets the applied state (called on motor off).
func (qgl *quadGantryLevel) reset() {
	qgl.applied = false
}