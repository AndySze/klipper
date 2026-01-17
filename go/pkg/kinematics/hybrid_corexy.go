// Hybrid CoreXY kinematics (also known as Markforged kinematics).
// The hybrid-corexy kinematic uses:
// - stepper_x: corexy_stepper_alloc with '-' (combined X)
// - stepper_y: cartesian_stepper_alloc (direct Y)
// - stepper_z: cartesian_stepper_alloc (direct Z)
package kinematics

import (
	"math"
)

// HybridCoreXYKinematics implements hybrid CoreXY kinematics.
type HybridCoreXYKinematics struct {
	*BaseKinematics
}

// NewHybridCoreXYKinematics creates a new hybrid CoreXY kinematics instance.
func NewHybridCoreXYKinematics(rails []Rail, maxZVelocity, maxZAccel float64) *HybridCoreXYKinematics {
	return &HybridCoreXYKinematics{
		BaseKinematics: NewBaseKinematics(rails, maxZVelocity, maxZAccel),
	}
}

// GetType returns the kinematic type name.
func (hcxy *HybridCoreXYKinematics) GetType() string {
	return "hybrid_corexy"
}

// CalcPosition calculates the cartesian position from stepper positions.
// For hybrid CoreXY: X = stepper_x + stepper_y, Y = stepper_y, Z = stepper_z
func (hcxy *HybridCoreXYKinematics) CalcPosition(stepperPositions map[string]float64) []float64 {
	// Hybrid CoreXY calculation:
	// The X stepper contributes to both X and Y movement (CoreXY coupling)
	// while Y and Z steppers are direct Cartesian
	posX := stepperPositions["stepper_x"]
	posY := stepperPositions["stepper_y"]
	posZ := stepperPositions["stepper_z"]

	return []float64{posX + posY, posY, posZ}
}

// SetPosition sets the current position and updates homing state.
func (hcxy *HybridCoreXYKinematics) SetPosition(newPos []float64, homingAxes string) {
	hcxy.BaseKinematics.SetPosition(newPos, homingAxes)
}

// ClearHomingState clears the homing state for specified axes.
func (hcxy *HybridCoreXYKinematics) ClearHomingState(clearAxes string) {
	hcxy.BaseKinematics.ClearHomingState(clearAxes)
}

// CheckMove validates a move and applies any necessary speed limits.
func (hcxy *HybridCoreXYKinematics) CheckMove(move *Move) error {
	// Check if move is within limits
	limits := hcxy.Limits
	endPos := move.EndPos

	if len(endPos) >= 2 {
		xpos, ypos := endPos[0], endPos[1]
		if xpos < limits[0][0] || xpos > limits[0][1] ||
			ypos < limits[1][0] || ypos > limits[1][1] {
			if err := hcxy.CheckEndstops(move); err != nil {
				return err
			}
		}
	}

	// Apply Z velocity limits if Z movement
	if len(move.AxesD) > 2 && move.AxesD[2] != 0.0 {
		if err := hcxy.CheckEndstops(move); err != nil {
			return err
		}
		zRatio := move.MoveD / math.Abs(move.AxesD[2])
		move.LimitSpeed(hcxy.MaxZVelocity*zRatio, hcxy.MaxZAccel*zRatio)
	}

	return nil
}

// GetStatus returns the current status of the kinematics.
func (hcxy *HybridCoreXYKinematics) GetStatus() map[string]interface{} {
	return hcxy.BaseKinematics.GetStatus()
}
