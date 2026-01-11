// CoreXY kinematics implementation for CoreXY/CoreXZ 3D printers.
package kinematics

import (
	"math"
)

// CoreXYKinematics implements CoreXY kinematics.
// In CoreXY, the X and Y motors (A and B) work together:
// - Motor A (stepper_x): moves carriage diagonally (X+Y direction)
// - Motor B (stepper_y): moves carriage diagonally (X-Y direction)
// - X position = 0.5 * (A + B)
// - Y position = 0.5 * (A - B)
// - A position = X + Y
// - B position = X - Y
type CoreXYKinematics struct {
	*BaseKinematics
}

// NewCoreXYKinematics creates a new CoreXY kinematics instance.
func NewCoreXYKinematics(rails []Rail, maxZVelocity, maxZAccel float64) *CoreXYKinematics {
	return &CoreXYKinematics{
		BaseKinematics: NewBaseKinematics(rails, maxZVelocity, maxZAccel),
	}
}

// GetType returns the kinematic type name.
func (ck *CoreXYKinematics) GetType() string {
	return "corexy"
}

// CalcPosition calculates the cartesian position from stepper positions.
// For CoreXY:
// - X = 0.5 * (A + B)
// - Y = 0.5 * (A - B)
// - Z = Z (direct mapping)
func (ck *CoreXYKinematics) CalcPosition(stepperPositions map[string]float64) []float64 {
	pos := make([]float64, 3)

	// Get A and B motor positions (mapped to stepper_x and stepper_y)
	aPos := 0.0
	bPos := 0.0
	if a, ok := stepperPositions["stepper_x"]; ok {
		aPos = a
	}
	if b, ok := stepperPositions["stepper_y"]; ok {
		bPos = b
	}

	// Calculate X and Y from A and B
	pos[0] = 0.5 * (aPos + bPos) // X = 0.5 * (A + B)
	pos[1] = 0.5 * (aPos - bPos) // Y = 0.5 * (A - B)

	// Z is direct mapping
	if z, ok := stepperPositions["stepper_z"]; ok {
		pos[2] = z
	}

	return pos
}

// CheckMove validates a move and applies any necessary speed limits.
func (ck *CoreXYKinematics) CheckMove(move *Move) error {
	// Check X and Y limits
	if len(move.EndPos) >= 2 {
		xpos, ypos := move.EndPos[0], move.EndPos[1]
		if xpos < ck.Limits[0][0] || xpos > ck.Limits[0][1] ||
			ypos < ck.Limits[1][0] || ypos > ck.Limits[1][1] {
			if err := ck.CheckEndstops(move); err != nil {
				return err
			}
		}
	}

	// If no Z movement, we're done
	if len(move.AxesD) <= 2 || move.AxesD[2] == 0.0 {
		return nil
	}

	// Check all endstops for Z movement
	if err := ck.CheckEndstops(move); err != nil {
		return err
	}

	// Apply Z-axis speed limits
	ck.CheckZMove(move)

	return nil
}

// CalcStepperPosition calculates stepper motor positions from cartesian coordinates.
// This is needed for move planning.
// For CoreXY:
// - A = X + Y
// - B = X - Y
// - Z = Z (direct mapping)
func (ck *CoreXYKinematics) CalcStepperPosition(cartesianPos []float64) map[string]float64 {
	stepperPos := make(map[string]float64)

	if len(cartesianPos) >= 2 {
		x := cartesianPos[0]
		y := cartesianPos[1]

		// Calculate A and B motor positions
		stepperPos["stepper_x"] = x + y // A = X + Y
		stepperPos["stepper_y"] = x - y // B = X - Y
	}

	// Z is direct mapping
	if len(cartesianPos) >= 3 {
		stepperPos["stepper_z"] = cartesianPos[2]
	}

	return stepperPos
}

// GetStepperVectors returns the movement vectors for each stepper.
// This is used by the motion planner to calculate step timing.
// For CoreXY:
// - Motor A (stepper_x): [1, 1, 0] in cartesian space
// - Motor B (stepper_y): [1, -1, 0] in cartesian space
// - Motor Z (stepper_z): [0, 0, 1] in cartesian space
func (ck *CoreXYKinematics) GetStepperVectors() map[string][]float64 {
	return map[string][]float64{
		"stepper_x": {1.0, 1.0, 0.0},  // A motor moves in X+Y direction
		"stepper_y": {1.0, -1.0, 0.0}, // B motor moves in X-Y direction
		"stepper_z": {0.0, 0.0, 1.0},  // Z motor moves in Z direction only
	}
}

// CoreXZKinematics implements CoreXZ kinematics.
// Similar to CoreXY but for X and Z axes.
type CoreXZKinematics struct {
	*BaseKinematics
}

// NewCoreXZKinematics creates a new CoreXZ kinematics instance.
func NewCoreXZKinematics(rails []Rail, maxZVelocity, maxZAccel float64) *CoreXZKinematics {
	return &CoreXZKinematics{
		BaseKinematics: NewBaseKinematics(rails, maxZVelocity, maxZAccel),
	}
}

// GetType returns the kinematic type name.
func (ck *CoreXZKinematics) GetType() string {
	return "corexz"
}

// CalcPosition calculates the cartesian position from stepper positions.
// For CoreXZ:
// - X = 0.5 * (A + B)
// - Z = 0.5 * (A - B)
// - Y = Y (direct mapping)
func (ck *CoreXZKinematics) CalcPosition(stepperPositions map[string]float64) []float64 {
	pos := make([]float64, 3)

	// Get A and B motor positions (mapped to stepper_x and stepper_z)
	aPos := 0.0
	bPos := 0.0
	if a, ok := stepperPositions["stepper_x"]; ok {
		aPos = a
	}
	if b, ok := stepperPositions["stepper_z"]; ok {
		bPos = b
	}

	// Calculate X and Z from A and B
	pos[0] = 0.5 * (aPos + bPos) // X = 0.5 * (A + B)
	pos[2] = 0.5 * (aPos - bPos) // Z = 0.5 * (A - B)

	// Y is direct mapping
	if y, ok := stepperPositions["stepper_y"]; ok {
		pos[1] = y
	}

	return pos
}

// CheckMove validates a move and applies any necessary speed limits.
func (ck *CoreXZKinematics) CheckMove(move *Move) error {
	// For CoreXZ, we need to apply speed limits differently since X and Z are coupled
	// Check all limits first
	if err := ck.CheckEndstops(move); err != nil {
		return err
	}

	// Apply speed limits based on the coupled motion
	if len(move.AxesD) >= 3 && (move.AxesD[0] != 0.0 || move.AxesD[2] != 0.0) {
		// Calculate the effective speed limit for coupled X-Z motion
		// The motors need to move sqrt(x^2 + z^2) distance
		xzDist := math.Sqrt(move.AxesD[0]*move.AxesD[0] + move.AxesD[2]*move.AxesD[2])
		if xzDist > 0 {
			// Apply Z speed limits to the coupled motion
			ratio := move.MoveD / xzDist
			move.LimitSpeed(ck.MaxZVelocity*ratio, ck.MaxZAccel*ratio)
		}
	}

	return nil
}