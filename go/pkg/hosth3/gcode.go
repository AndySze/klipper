package hosth3

import (
    "fmt"
    "strconv"
    "strings"
)

type gcodeMove struct {
    toolhead *toolhead

    absoluteCoord   bool
    absoluteExtrude bool

    basePosition []float64
    lastPosition []float64

    axisMap map[string]int

    speed        float64
    speedFactor  float64
    extrudeFactor float64
}

func newGCodeMove(th *toolhead) *gcodeMove {
    gm := &gcodeMove{
        toolhead:       th,
        absoluteCoord:   true,
        absoluteExtrude: true,
        axisMap:         map[string]int{"X": 0, "Y": 1, "Z": 2, "E": 3},
        speed:           25.0,
        speedFactor:     1.0 / 60.0,
        extrudeFactor:   1.0,
    }
    gm.basePosition = make([]float64, len(th.commandedPos))
    gm.lastPosition = append([]float64{}, th.commandedPos...)
    return gm
}

func (gm *gcodeMove) updateExtraAxes() {
    extra := gm.toolhead.getExtraAxes()
    axisMap := map[string]int{"X": 0, "Y": 1, "Z": 2, "E": 3}
    for idx, ea := range extra {
        if ea == nil {
            continue
        }
        axis, ok := ea.(interface{ GetAxisGcodeID() string })
        if !ok {
            continue
        }
        gcodeID := axis.GetAxisGcodeID()
        if len(gcodeID) != 1 {
            continue
        }
        if _, exists := axisMap[gcodeID]; exists {
            continue
        }
        if strings.Contains("FN", gcodeID) {
            continue
        }
        axisMap[gcodeID] = idx
    }
    gm.axisMap = axisMap

    wantLen := len(gm.toolhead.commandedPos)
    if len(gm.basePosition) != wantLen {
        nb := make([]float64, wantLen)
        copy(nb, gm.basePosition)
        gm.basePosition = nb
    }
    gm.lastPosition = append([]float64{}, gm.toolhead.commandedPos...)
}

func (gm *gcodeMove) cmdG1(args map[string]string) error {
    for axis, pos := range gm.axisMap {
        raw, ok := args[axis]
        if !ok {
            continue
        }
        v, err := strconv.ParseFloat(raw, 64)
        if err != nil {
            return fmt.Errorf("bad %s=%q", axis, raw)
        }
        absolute := gm.absoluteCoord
        if axis == "E" {
            v *= gm.extrudeFactor
            if !gm.absoluteExtrude {
                absolute = false
            }
        }
        if !absolute {
            gm.lastPosition[pos] += v
        } else {
            gm.lastPosition[pos] = v + gm.basePosition[pos]
        }
    }
    if raw, ok := args["F"]; ok {
        f, err := strconv.ParseFloat(raw, 64)
        if err != nil {
            return fmt.Errorf("bad F=%q", raw)
        }
        if f <= 0.0 {
            return fmt.Errorf("invalid speed F=%q", raw)
        }
        gm.speed = f * gm.speedFactor
    }
    return gm.toolhead.move(gm.lastPosition, gm.speed)
}

func (gm *gcodeMove) cmdG28() error {
    return nil
}
