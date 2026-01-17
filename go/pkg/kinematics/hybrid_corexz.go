// Hybrid CoreXZ kinematics.
// The hybrid-corexz kinematic uses:
// - stepper_x: corexz_stepper_alloc with '-' (combined X)
// - stepper_y: cartesian_stepper_alloc (direct Y)
// - stepper_z: cartesian_stepper_alloc (direct Z)
package kinematics

import (
	"fmt"
	"math"
)

// HybridCoreXZKinematics implements hybrid CoreXZ kinematics.
type HybridCoreXZKinematics struct {
	*BaseKinematics
}

// NewHybridCoreXZKinematics creates a new hybrid CoreXZ kinematics instance.
func NewHybridCoreXZKinematics(rails []Rail, maxZVelocity, maxZAccel float64) *HybridCoreXZKinematics {
	return &HybridCoreXZKinematics{
		BaseKinematics: NewBaseKinematics(rails, maxZVelocity, maxZAccel),
	}
}

// GetType returns the kinematic type name.
func (hcxz *HybridCoreXZKinematics) GetType() string {
	return "hybrid_corexz"
}

// CalcPosition calculates the cartesian position from stepper positions.
// For hybrid CoreXZ: X = stepper_x + stepper_z, Y = stepper_y, Z = stepper_z
func (hcxz *HybridCoreXZKinematics) CalcPosition(stepperPositions map[string]float64) []float64 {
	// Hybrid CoreXZ calculation:
	// The X stepper contributes to both X and Z movement (CoreXZ coupling)
	// while Y and Z steppers are direct Cartesian
	posX := stepperPositions["stepper_x"]
	posY := stepperPositions["stepper_y"]
	posZ := stepperPositions["stepper_z"]

	return []float64{posX + posZ, posY, posZ}
}

// SetPosition sets the current position and updates homing state.
func (hcxz *HybridCoreXZKinematics) SetPosition(newPos []float64, homingAxes string) {
	hcxz.BaseKinematics.SetPosition(newPos, homingAxes)
}

// ClearHomingState clears the homing state for specified axes.
func (hcxz *HybridCoreXZKinematics) ClearHomingState(clearAxes string) {
	hcxz.BaseKinematics.ClearHomingState(clearAxes)
}

// CheckMove validates a move and applies any necessary speed limits.
func (hcxz *HybridCoreXZKinematics) CheckMove(move *Move) error {
	// Check if move is within limits
	limits := hcxz.Limits
	endPos := move.EndPos

	if len(endPos) >= 2 {
		xpos, ypos := endPos[0], endPos[1]
		if xpos < limits[0][0] || xpos > limits[0][1] ||
			ypos < limits[1][0] || ypos > limits[1][1] {
			if err := hcxz.CheckEndstops(move); err != nil {
				return err
			}
		}
	}

	// Apply Z velocity limits if Z movement
	if len(move.AxesD) > 2 && move.AxesD[2] != 0.0 {
		if err := hcxz.CheckEndstops(move); err != nil {
			return err
		}
		zRatio := move.MoveD / math.Abs(move.AxesD[2])
		move.LimitSpeed(hcxz.MaxZVelocity*zRatio, hcxz.MaxZAccel*zRatio)
	}

	return nil
}

// GetStatus returns the current status of the kinematics.
func (hcxz *HybridCoreXZKinematics) GetStatus() map[string]interface{} {
	return hcxz.BaseKinematics.GetStatus()
}

// Ensure fmt is used
var _ = fmt.Sprintf
