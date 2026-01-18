package hosth4

import (
	"fmt"
	"math"
	"strconv"
)

const (
	arcPlaneXY = 0
	arcPlaneXZ = 1
	arcPlaneYZ = 2
)

// gcodeState holds a saved gcode state for SAVE/RESTORE_GCODE_STATE.
type gcodeState struct {
	absoluteCoord   bool
	absoluteExtrude bool
	basePosition    []float64
	lastPosition    []float64
	speed           float64
	speedFactor     float64
	extrudeFactor   float64
	arcPlane        int
}

type gcodeMove struct {
	toolhead *toolhead

	moveWithTransform     func(newPos []float64, speed float64) error
	positionWithTransform func() []float64

	absoluteCoord   bool
	absoluteExtrude bool

	basePosition []float64
	lastPosition []float64

	speed         float64
	speedFactor   float64
	extrudeFactor float64

	arcPlane        int
	mmPerArcSegment float64

	// Saved gcode states by name
	savedStates map[string]*gcodeState
}

func newGCodeMove(th *toolhead, mmPerArcSegment float64) *gcodeMove {
	gm := &gcodeMove{
		toolhead: th,
		moveWithTransform: func(newPos []float64, speed float64) error {
			return th.move(newPos, speed)
		},
		positionWithTransform: func() []float64 {
			return append([]float64{}, th.commandedPos...)
		},
		absoluteCoord:   true,
		absoluteExtrude: true,
		speed:           25.0,
		speedFactor:     1.0 / 60.0,
		extrudeFactor:   1.0,
		arcPlane:        arcPlaneXY,
		mmPerArcSegment: mmPerArcSegment,
		savedStates:     make(map[string]*gcodeState),
	}
	gm.basePosition = make([]float64, len(th.commandedPos))
	gm.lastPosition = append([]float64{}, th.commandedPos...)
	return gm
}

func (gm *gcodeMove) setMoveTransform(transform moveTransform) {
	if transform == nil {
		gm.moveWithTransform = func(newPos []float64, speed float64) error {
			return gm.toolhead.move(newPos, speed)
		}
		gm.positionWithTransform = func() []float64 {
			return append([]float64{}, gm.toolhead.commandedPos...)
		}
		return
	}
	gm.moveWithTransform = transform.Move
	gm.positionWithTransform = transform.GetPosition
}

func (gm *gcodeMove) resetFromToolhead() {
	if gm.positionWithTransform != nil {
		gm.lastPosition = gm.positionWithTransform()
	} else {
		gm.lastPosition = append([]float64{}, gm.toolhead.commandedPos...)
	}
	if len(gm.basePosition) != len(gm.lastPosition) {
		nb := make([]float64, len(gm.lastPosition))
		copy(nb, gm.basePosition)
		gm.basePosition = nb
	}
}

func (gm *gcodeMove) getGcodePosition() []float64 {
	out := make([]float64, len(gm.lastPosition))
	for i := 0; i < len(out) && i < len(gm.basePosition); i++ {
		out[i] = gm.lastPosition[i] - gm.basePosition[i]
	}
	if len(out) > 3 {
		out[3] /= gm.extrudeFactor
	}
	return out
}

func (gm *gcodeMove) cmdG90() { gm.absoluteCoord = true }
func (gm *gcodeMove) cmdG91() { gm.absoluteCoord = false }

func (gm *gcodeMove) cmdG17() { gm.arcPlane = arcPlaneXY }
func (gm *gcodeMove) cmdG18() { gm.arcPlane = arcPlaneXZ }
func (gm *gcodeMove) cmdG19() { gm.arcPlane = arcPlaneYZ }

// cmdG92 sets the current gcode position by adjusting the base position offset.
// This doesn't move the toolhead - it just changes how gcode coordinates map to toolhead coordinates.
func (gm *gcodeMove) cmdG92(args map[string]string) error {
	axisMap := map[string]int{"X": 0, "Y": 1, "Z": 2, "E": 3}
	anySet := false
	for axis, pos := range axisMap {
		raw, ok := args[axis]
		if !ok {
			continue
		}
		v, err := strconv.ParseFloat(raw, 64)
		if err != nil {
			return fmt.Errorf("bad %s=%q", axis, raw)
		}
		anySet = true
		if axis == "E" {
			v *= gm.extrudeFactor
		}
		// Set base_position so that gcode_pos becomes v
		// gcode_pos = last_position - base_position
		// Therefore: base_position = last_position - v
		if pos < len(gm.basePosition) && pos < len(gm.lastPosition) {
			gm.basePosition[pos] = gm.lastPosition[pos] - v
		}
	}
	// If no arguments, reset all base positions (like G92 with no args)
	if !anySet {
		for i := range gm.basePosition {
			if i < len(gm.lastPosition) {
				gm.basePosition[i] = gm.lastPosition[i]
			}
		}
	}
	return nil
}

func (gm *gcodeMove) cmdG1(args map[string]string) error {
	axisMap := map[string]int{"X": 0, "Y": 1, "Z": 2, "E": 3}
	for axis, pos := range axisMap {
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
	if gm.moveWithTransform == nil {
		return gm.toolhead.move(gm.lastPosition, gm.speed)
	}
	return gm.moveWithTransform(gm.lastPosition, gm.speed)
}

func (gm *gcodeMove) moveGcodePosition(gpos []float64, ePos *float64, feed *float64) error {
	if len(gpos) < 3 {
		return fmt.Errorf("gcode pos missing xyz")
	}
	if feed != nil {
		if *feed <= 0.0 {
			return fmt.Errorf("invalid speed F=%v", *feed)
		}
		gm.speed = *feed * gm.speedFactor
	}
	target := append([]float64{}, gm.lastPosition...)
	for i := 0; i < 3; i++ {
		target[i] = gpos[i] + gm.basePosition[i]
	}
	if ePos != nil && len(target) > 3 {
		v := *ePos * gm.extrudeFactor
		if gm.absoluteExtrude {
			target[3] = v + gm.basePosition[3]
		} else {
			target[3] = target[3] + v
		}
	}
	gm.lastPosition = target
	if gm.moveWithTransform == nil {
		return gm.toolhead.move(target, gm.speed)
	}
	return gm.moveWithTransform(target, gm.speed)
}

func (gm *gcodeMove) cmdG2G3(args map[string]string, clockwise bool) error {
	if !gm.absoluteCoord {
		return fmt.Errorf("G2/G3 does not support relative move mode")
	}
	current := gm.getGcodePosition()
	if len(current) < 4 {
		return fmt.Errorf("missing E axis")
	}

	target := []float64{
		floatArgOr(args, "X", current[0]),
		floatArgOr(args, "Y", current[1]),
		floatArgOr(args, "Z", current[2]),
	}
	if _, ok := args["R"]; ok {
		return fmt.Errorf("G2/G3 does not support R moves")
	}

	offsetI := floatArgOr(args, "I", 0.0)
	offsetJ := floatArgOr(args, "J", 0.0)
	offsetK := floatArgOr(args, "K", 0.0)

	alpha := 0
	beta := 1
	helical := 2
	planar0 := offsetI
	planar1 := offsetJ
	if gm.arcPlane == arcPlaneXZ {
		alpha = 0
		beta = 2
		helical = 1
		planar0 = offsetI
		planar1 = offsetK
	} else if gm.arcPlane == arcPlaneYZ {
		alpha = 1
		beta = 2
		helical = 0
		planar0 = offsetJ
		planar1 = offsetK
	}
	if planar0 == 0.0 && planar1 == 0.0 {
		return fmt.Errorf("G2/G3 requires IJ, IK or JK parameters")
	}

	rP := -planar0
	rQ := -planar1

	centerP := current[alpha] - rP
	centerQ := current[beta] - rQ
	rtAlpha := target[alpha] - centerP
	rtBeta := target[beta] - centerQ
	angularTravel := math.Atan2(rP*rtBeta-rQ*rtAlpha, rP*rtAlpha+rQ*rtBeta)
	if angularTravel < 0.0 {
		angularTravel += 2.0 * math.Pi
	}
	if clockwise {
		angularTravel -= 2.0 * math.Pi
	}

	if angularTravel == 0.0 && current[alpha] == target[alpha] && current[beta] == target[beta] {
		angularTravel = 2.0 * math.Pi
	}

	linearTravel := target[helical] - current[helical]
	radius := math.Hypot(rP, rQ)
	flatMM := radius * angularTravel
	mmOfTravel := math.Abs(flatMM)
	if linearTravel != 0.0 {
		mmOfTravel = math.Hypot(flatMM, linearTravel)
	}
	segLen := gm.mmPerArcSegment
	if segLen <= 0.0 {
		segLen = 1.0
	}
	segments := math.Max(1.0, math.Floor(mmOfTravel/segLen))
	thetaPerSegment := angularTravel / segments
	linearPerSegment := linearTravel / segments

	var asE *float64
	if raw, ok := args["E"]; ok {
		v, err := strconv.ParseFloat(raw, 64)
		if err != nil {
			return fmt.Errorf("bad E=%q", raw)
		}
		asE = &v
	}
	var asF *float64
	if raw, ok := args["F"]; ok {
		v, err := strconv.ParseFloat(raw, 64)
		if err != nil {
			return fmt.Errorf("bad F=%q", raw)
		}
		asF = &v
	}

	ePerMove := 0.0
	eBase := 0.0
	if asE != nil {
		if gm.absoluteExtrude {
			eBase = current[3]
		}
		ePerMove = (*asE - eBase) / segments
	}

	for i := 1; i <= int(segments); i++ {
		distHelical := float64(i) * linearPerSegment
		cTheta := float64(i) * thetaPerSegment
		cosTi := math.Cos(cTheta)
		sinTi := math.Sin(cTheta)
		rp := -planar0*cosTi + planar1*sinTi
		rq := -planar0*sinTi - planar1*cosTi

		seg := []float64{current[0], current[1], current[2]}
		seg[alpha] = centerP + rp
		seg[beta] = centerQ + rq
		seg[helical] = current[helical] + distHelical
		if float64(i) == segments {
			seg = target
		}

		var ePos *float64
		if ePerMove != 0.0 {
			v := eBase + ePerMove
			ePos = &v
			if gm.absoluteExtrude {
				eBase += ePerMove
			}
		}
		if err := gm.moveGcodePosition(seg, ePos, asF); err != nil {
			return err
		}
	}
	return nil
}

func floatArgOr(args map[string]string, key string, def float64) float64 {
	raw, ok := args[key]
	if !ok {
		return def
	}
	if raw == "" {
		return def
	}
	v, err := strconv.ParseFloat(raw, 64)
	if err != nil {
		return def
	}
	return v
}

// saveState saves the current gcode state with the given name.
func (gm *gcodeMove) saveState(name string) {
	if name == "" {
		name = "default"
	}
	state := &gcodeState{
		absoluteCoord:   gm.absoluteCoord,
		absoluteExtrude: gm.absoluteExtrude,
		basePosition:    append([]float64{}, gm.basePosition...),
		lastPosition:    append([]float64{}, gm.lastPosition...),
		speed:           gm.speed,
		speedFactor:     gm.speedFactor,
		extrudeFactor:   gm.extrudeFactor,
		arcPlane:        gm.arcPlane,
	}
	gm.savedStates[name] = state
}

// restoreState restores a previously saved gcode state.
// If move is true, moves to the saved position at the given speed.
func (gm *gcodeMove) restoreState(name string, move bool, moveSpeed float64) error {
	if name == "" {
		name = "default"
	}
	state, ok := gm.savedStates[name]
	if !ok {
		return fmt.Errorf("gcode state '%s' not found", name)
	}

	gm.absoluteCoord = state.absoluteCoord
	gm.absoluteExtrude = state.absoluteExtrude
	gm.speed = state.speed
	gm.speedFactor = state.speedFactor
	gm.extrudeFactor = state.extrudeFactor
	gm.arcPlane = state.arcPlane

	// Copy positions
	gm.basePosition = append([]float64{}, state.basePosition...)

	if move && moveSpeed > 0 {
		// Move to saved position
		targetPos := append([]float64{}, state.lastPosition...)
		// Restore XYZ but not E
		if len(targetPos) > 3 {
			targetPos[3] = gm.lastPosition[3]
		}
		gm.lastPosition = targetPos
		if gm.moveWithTransform != nil {
			if err := gm.moveWithTransform(targetPos, moveSpeed); err != nil {
				return err
			}
		} else if gm.toolhead != nil {
			if err := gm.toolhead.move(targetPos, moveSpeed); err != nil {
				return err
			}
		}
	} else {
		gm.lastPosition = append([]float64{}, state.lastPosition...)
	}

	return nil
}

// GetStatus returns the current gcode move status for API queries.
func (gm *gcodeMove) GetStatus() map[string]any {
	gcodePos := gm.getGcodePosition()
	return map[string]any{
		"speed_factor":       gm.speedFactor * 60.0,
		"speed":              gm.speed,
		"extrude_factor":     gm.extrudeFactor,
		"absolute_coordinates": gm.absoluteCoord,
		"absolute_extrude":   gm.absoluteExtrude,
		"position":           gcodePos,
		"gcode_position":     gcodePos,
	}
}
