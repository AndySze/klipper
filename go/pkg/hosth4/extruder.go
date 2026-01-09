package hosth4

import (
    "math"

    "klipper-go-migration/pkg/chelper"
)

type extraAxis interface {
    GetName() string
    GetAxisGcodeID() string
    ProcessMove(printTime float64, mv *move, axisIndex int) error
    CheckMove(mv *move, axisIndex int) error
    CalcJunction(prev *move, mv *move, axisIndex int) float64
}

type extruderAxis struct {
    name string

    trapq *chelper.TrapQ

    lastPosition float64
    instantCornerV float64
}

func newExtruderAxis(name string, tq *chelper.TrapQ) *extruderAxis {
    return &extruderAxis{name: name, trapq: tq, lastPosition: 0.0, instantCornerV: 1.0}
}

func (e *extruderAxis) GetName() string { return e.name }
func (e *extruderAxis) GetAxisGcodeID() string { return "E" }

func (e *extruderAxis) ProcessMove(printTime float64, mv *move, axisIndex int) error {
    // Add bounds check for axisIndex
    if axisIndex >= len(mv.axesR) || axisIndex >= len(mv.startPos) || axisIndex >= len(mv.endPos) {
        // axisIndex is out of range, skip this move for this axis
        return nil
    }

    // Only process moves if there's actual extrusion (axisR > 0)
    axisR := mv.axesR[axisIndex]
    if axisR <= 0.0 {
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

func (e *extruderAxis) CheckMove(_ *move, _ int) error {
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

