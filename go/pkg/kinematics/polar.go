// Polar kinematics implementation for rotating bed printers.
package kinematics

import (
	"math"
)

// PolarKinematics implements polar kinematics with a rotating bed and linear arm.
type PolarKinematics struct {
	*BaseKinematics
	maxRadius float64 // Maximum arm extension
}

// NewPolarKinematics creates a new polar kinematics instance.
// For polar, rails[0] is the arm (radius), rails[1] is the z-axis.
// The bed rotation is not bounded (infinite rotation).
func NewPolarKinematics(rails []Rail, maxZVelocity, maxZAccel float64) *PolarKinematics {
	// Create base kinematics with X/Y bounds based on arm radius
	maxRadius := rails[0].PositionMax

	// Create synthetic X/Y rails based on radius
	syntheticRails := []Rail{
		{
			Name:        "stepper_x",
			PositionMin: -maxRadius,
			PositionMax: maxRadius,
		},
		{
			Name:        "stepper_y",
			PositionMin: -maxRadius,
			PositionMax: maxRadius,
		},
	}

	// Add the Z rail as-is
	if len(rails) > 1 {
		syntheticRails = append(syntheticRails, rails[1])
	} else {
		// Default Z rail if not provided
		syntheticRails = append(syntheticRails, Rail{
			Name:        "stepper_z",
			PositionMin: 0,
			PositionMax: 200,
		})
	}

	return &PolarKinematics{
		BaseKinematics: NewBaseKinematics(syntheticRails, maxZVelocity, maxZAccel),
		maxRadius:      maxRadius,
	}
}

// GetType returns the kinematic type name.
func (pk *PolarKinematics) GetType() string {
	return "polar"
}

// CalcPosition calculates the cartesian position from stepper positions.
// For polar: arm position is radius, bed position is angle.
// X = radius * cos(angle), Y = radius * sin(angle)
func (pk *PolarKinematics) CalcPosition(stepperPositions map[string]float64) []float64 {
	pos := make([]float64, 3)

	var radius, angle float64
	if r, ok := stepperPositions["stepper_arm"]; ok {
		radius = r
	}
	if a, ok := stepperPositions["stepper_bed"]; ok {
		// Bed position is in degrees, convert to radians
		angle = a * math.Pi / 180.0
	}

	pos[0] = radius * math.Cos(angle)
	pos[1] = radius * math.Sin(angle)

	if z, ok := stepperPositions["stepper_z"]; ok {
		pos[2] = z
	}

	return pos
}

// CheckMove validates a move and applies any necessary speed limits.
func (pk *PolarKinematics) CheckMove(move *Move) error {
	// Check that the end position is within the polar bounds
	if len(move.EndPos) >= 2 {
		x, y := move.EndPos[0], move.EndPos[1]
		radius := math.Sqrt(x*x + y*y)

		// Check radius bounds
		if radius > pk.maxRadius {
			if err := pk.CheckEndstops(move); err != nil {
				return err
			}
		}
	}

	// If no Z movement, we're done
	if len(move.AxesD) <= 2 || move.AxesD[2] == 0.0 {
		return nil
	}

	// Check all endstops for Z movement
	if err := pk.CheckEndstops(move); err != nil {
		return err
	}

	// Apply Z-axis speed limits
	pk.CheckZMove(move)

	return nil
}
