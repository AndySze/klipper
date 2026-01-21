package hosth4

import (
	"fmt"
	"math"

	"klipper-go-migration/pkg/chelper"
)

type extraAxis interface {
	GetName() string
	GetAxisGcodeID() string
	GetTrapQ() *chelper.TrapQ
	ProcessMove(printTime float64, mv *move, axisIndex int) error
	CheckMove(mv *move, axisIndex int) error
	CalcJunction(prev *move, mv *move, axisIndex int) float64
}

type extruderAxis struct {
	name string

	trapq *chelper.TrapQ

	lastPosition   float64
	instantCornerV float64

	nozzleDiameter  float64
	filamentArea    float64
	maxExtrudeRatio float64
	maxEVelocity    float64
	maxEAccel       float64
	maxEDist        float64
}

func newExtruderAxis(name string, tq *chelper.TrapQ) *extruderAxis {
	return &extruderAxis{name: name, trapq: tq, lastPosition: 0.0, instantCornerV: 1.0, maxEDist: 50.0}
}

func (e *extruderAxis) GetName() string          { return e.name }
func (e *extruderAxis) GetAxisGcodeID() string   { return "E" }
func (e *extruderAxis) GetTrapQ() *chelper.TrapQ { return e.trapq }

// FindPastPosition returns the extruder position at a given print time.
// Note: This is a simplified implementation that returns the current position.
// A more accurate implementation would use itersolve to calculate the position
// at the specific print time.
func (e *extruderAxis) FindPastPosition(printTime float64) float64 {
	// Return the last commanded position (simplified)
	return e.lastPosition
}

func (e *extruderAxis) ProcessMove(printTime float64, mv *move, axisIndex int) error {
	// Add bounds check for axisIndex
	if axisIndex >= len(mv.axesR) || axisIndex >= len(mv.startPos) || axisIndex >= len(mv.endPos) {
		// axisIndex is out of range, skip this move for this axis
		return nil
	}

	// Only process moves if there's actual extrusion/retraction (axisR != 0)
	axisR := mv.axesR[axisIndex]
	if axisR == 0.0 {
		return nil
	}

	accel := mv.accel * axisR
	startV := mv.startV * axisR
	cruiseV := mv.cruiseV * axisR
	canPressureAdvance := 0.0
	if axisR > 0.0 && (mv.axesD[0] != 0.0 || mv.axesD[1] != 0.0) {
		canPressureAdvance = 1.0
	}
	e.trapq.Append(
		printTime,
		mv.accelT,
		mv.cruiseT,
		mv.decelT,
		mv.startPos[axisIndex],
		0.0,
		0.0,
		1.0,
		canPressureAdvance,
		0.0,
		startV,
		cruiseV,
		accel,
	)
	e.lastPosition = mv.endPos[axisIndex]
	return nil
}

func (e *extruderAxis) CheckMove(mv *move, axisIndex int) error {
	if mv == nil {
		return nil
	}
	if axisIndex >= len(mv.axesR) || axisIndex >= len(mv.axesD) {
		return nil
	}
	axisR := mv.axesR[axisIndex]
	axisD := mv.axesD[axisIndex]
	if axisR == 0.0 {
		return nil
	}

	// Mirror klippy/kinematics/extruder.py:PrinterExtruder.check_move.
	// Note: It checks XY only (not Z) for "extrude only" mode.
	isExtrudeOnlyOrRetract := (len(mv.axesD) >= 2 && mv.axesD[0] == 0.0 && mv.axesD[1] == 0.0) || axisR < 0.0
	if isExtrudeOnlyOrRetract {
		if e.maxEDist > 0.0 && math.Abs(axisD) > e.maxEDist {
			return fmt.Errorf("extrude only move too long")
		}
		if e.maxEVelocity > 0.0 && e.maxEAccel > 0.0 {
			invExtrudeR := 1.0 / math.Abs(axisR)
			mv.limitSpeed(e.maxEVelocity*invExtrudeR, e.maxEAccel*invExtrudeR)
		}
		return nil
	}

	if e.maxExtrudeRatio > 0.0 && axisR > e.maxExtrudeRatio {
		if e.nozzleDiameter > 0.0 && axisD <= e.nozzleDiameter*e.maxExtrudeRatio {
			// Permit extrusion if amount extruded is tiny.
			return nil
		}
		return fmt.Errorf("move exceeds maximum extrusion")
	}
	return nil
}

func (e *extruderAxis) CalcJunction(prev *move, mv *move, axisIndex int) float64 {
	// Add bounds check for axisIndex
	if axisIndex >= len(mv.axesR) || axisIndex >= len(prev.axesR) {
		// axisIndex is out of range, return max cruise velocity as fallback
		return mv.maxCruiseV2
	}
	diffR := mv.axesR[axisIndex] - prev.axesR[axisIndex]
	if diffR != 0.0 {
		v := e.instantCornerV / math.Abs(diffR)
		return v * v
	}
	return mv.maxCruiseV2
}
