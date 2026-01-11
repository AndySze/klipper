// Package kinematics provides kinematic transformation implementations for various printer types.
package kinematics

import (
	"fmt"
	"math"
)

// Move represents a movement command with position and velocity information.
type Move struct {
	StartPos     []float64 // Starting position [x, y, z, e...]
	EndPos       []float64 // Ending position [x, y, z, e...]
	AxesD        []float64 // Distance moved per axis
	MoveD        float64   // Total movement distance
	MinMoveTime  float64   // Minimum time for move
	MaxCruiseV   float64   // Maximum cruise velocity
	AccelT       float64   // Acceleration time
	CruiseT      float64   // Cruise time
	DecelT       float64   // Deceleration time
	StartV       float64   // Starting velocity
	CruiseV      float64   // Cruise velocity
	AccelR       float64   // Acceleration rate
	DecelR       float64   // Deceleration rate
	DeltaV2      float64   // Velocity squared delta
	SmoothDeltaV2 float64  // Smoothed velocity squared delta
}

// LimitSpeed reduces the maximum speed and acceleration of the move.
func (m *Move) LimitSpeed(maxV, maxA float64) {
	if m.MaxCruiseV > maxV {
		m.MaxCruiseV = maxV
	}
	// Additional speed limiting logic would go here
}

// Rail represents a stepper motor rail configuration.
type Rail struct {
	Name         string
	StepDist     float64
	PositionMin  float64
	PositionMax  float64
	HomingSpeed  float64
	SecondHoming float64
	HomingRetract float64
	PositionEndstop float64
	HomingPositive bool
}

// Kinematics is the interface for all kinematic implementations.
type Kinematics interface {
	// GetType returns the kinematic type name (e.g., "cartesian", "corexy", "delta").
	GetType() string

	// CalcPosition calculates the cartesian position from stepper positions.
	CalcPosition(stepperPositions map[string]float64) []float64

	// SetPosition sets the current position and updates homing state.
	SetPosition(newPos []float64, homingAxes string)

	// ClearHomingState clears the homing state for specified axes.
	ClearHomingState(clearAxes string)

	// CheckMove validates a move and applies any necessary speed limits.
	CheckMove(move *Move) error

	// GetStatus returns the current status of the kinematics.
	GetStatus() map[string]interface{}

	// GetRails returns the rails configuration.
	GetRails() []Rail

	// GetLimits returns the current axis limits.
	GetLimits() [][2]float64

	// GetMaxZVelocity returns the maximum Z axis velocity.
	GetMaxZVelocity() float64

	// GetMaxZAccel returns the maximum Z axis acceleration.
	GetMaxZAccel() float64
}

// BaseKinematics provides common functionality for all kinematic implementations.
type BaseKinematics struct {
	Rails        []Rail
	Limits       [][2]float64
	MaxZVelocity float64
	MaxZAccel    float64
	AxesMin      []float64
	AxesMax      []float64
}

// NewBaseKinematics creates a new base kinematics instance.
func NewBaseKinematics(rails []Rail, maxZVelocity, maxZAccel float64) *BaseKinematics {
	bk := &BaseKinematics{
		Rails:        rails,
		MaxZVelocity: maxZVelocity,
		MaxZAccel:    maxZAccel,
		Limits:       make([][2]float64, len(rails)),
		AxesMin:      make([]float64, len(rails)),
		AxesMax:      make([]float64, len(rails)),
	}

	// Initialize limits to unhomed state and axes bounds
	for i := range rails {
		bk.Limits[i] = [2]float64{1.0, -1.0} // Unhomed state
		bk.AxesMin[i] = rails[i].PositionMin
		bk.AxesMax[i] = rails[i].PositionMax
	}

	return bk
}

// SetPosition updates the position for homed axes.
func (bk *BaseKinematics) SetPosition(newPos []float64, homingAxes string) {
	for _, axisName := range homingAxes {
		axis := axisIndex(axisName)
		if axis >= 0 && axis < len(bk.Rails) {
			bk.Limits[axis] = [2]float64{bk.Rails[axis].PositionMin, bk.Rails[axis].PositionMax}
		}
	}
}

// ClearHomingState clears the homing state for specified axes.
func (bk *BaseKinematics) ClearHomingState(clearAxes string) {
	for _, axisName := range clearAxes {
		axis := axisIndex(axisName)
		if axis >= 0 && axis < len(bk.Limits) {
			bk.Limits[axis] = [2]float64{1.0, -1.0}
		}
	}
}

// CheckEndstops checks if the move is within the endstop limits.
func (bk *BaseKinematics) CheckEndstops(move *Move) error {
	for i := 0; i < len(move.EndPos) && i < len(bk.Limits); i++ {
		if move.AxesD[i] == 0.0 {
			continue
		}
		if move.EndPos[i] >= bk.Limits[i][0] && move.EndPos[i] <= bk.Limits[i][1] {
			continue
		}
		if bk.Limits[i][0] > bk.Limits[i][1] {
			return fmt.Errorf("must home axis first")
		}
		return fmt.Errorf("move out of range")
	}
	return nil
}

// CheckZMove applies Z-axis speed limits if the move includes Z movement.
func (bk *BaseKinematics) CheckZMove(move *Move) {
	if len(move.AxesD) > 2 && move.AxesD[2] != 0.0 {
		zRatio := move.MoveD / math.Abs(move.AxesD[2])
		move.LimitSpeed(bk.MaxZVelocity*zRatio, bk.MaxZAccel*zRatio)
	}
}

// GetRails returns the rails configuration.
func (bk *BaseKinematics) GetRails() []Rail {
	return bk.Rails
}

// GetLimits returns the current axis limits.
func (bk *BaseKinematics) GetLimits() [][2]float64 {
	return bk.Limits
}

// GetMaxZVelocity returns the maximum Z axis velocity.
func (bk *BaseKinematics) GetMaxZVelocity() float64 {
	return bk.MaxZVelocity
}

// GetMaxZAccel returns the maximum Z axis acceleration.
func (bk *BaseKinematics) GetMaxZAccel() float64 {
	return bk.MaxZAccel
}

// GetStatus returns the current status of the kinematics.
func (bk *BaseKinematics) GetStatus() map[string]interface{} {
	homedAxes := ""
	for i, limit := range bk.Limits {
		if limit[0] <= limit[1] && i < 3 {
			homedAxes += string(rune('x' + i))
		}
	}

	return map[string]interface{}{
		"homed_axes":   homedAxes,
		"axis_minimum": bk.AxesMin,
		"axis_maximum": bk.AxesMax,
	}
}

// axisIndex returns the index for a given axis name.
func axisIndex(axisName rune) int {
	switch axisName {
	case 'x', 'X':
		return 0
	case 'y', 'Y':
		return 1
	case 'z', 'Z':
		return 2
	case 'e', 'E':
		return 3
	default:
		return -1
	}
}