// Cartesian kinematics implementation for standard 3D printers.
package kinematics

// CartesianKinematics implements standard cartesian kinematics.
type CartesianKinematics struct {
	*BaseKinematics
}

// NewCartesianKinematics creates a new cartesian kinematics instance.
func NewCartesianKinematics(rails []Rail, maxZVelocity, maxZAccel float64) *CartesianKinematics {
	return &CartesianKinematics{
		BaseKinematics: NewBaseKinematics(rails, maxZVelocity, maxZAccel),
	}
}

// GetType returns the kinematic type name.
func (ck *CartesianKinematics) GetType() string {
	return "cartesian"
}

// CalcPosition calculates the cartesian position from stepper positions.
// For cartesian, the mapping is direct: stepper_x -> x, stepper_y -> y, stepper_z -> z
func (ck *CartesianKinematics) CalcPosition(stepperPositions map[string]float64) []float64 {
	pos := make([]float64, 3)

	// Direct mapping for cartesian
	if x, ok := stepperPositions["stepper_x"]; ok {
		pos[0] = x
	}
	if y, ok := stepperPositions["stepper_y"]; ok {
		pos[1] = y
	}
	if z, ok := stepperPositions["stepper_z"]; ok {
		pos[2] = z
	}

	return pos
}

// CheckMove validates a move and applies any necessary speed limits.
func (ck *CartesianKinematics) CheckMove(move *Move) error {
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