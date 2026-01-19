// GCode arcs - port of klippy/extras/gcode_arcs.py
//
// Adds support for ARC commands via G2/G3
//
// Copyright (C) 2019 Aleksej Vasiljkovic <achmed21@gmail.com>
// function planArc() originates from https://github.com/MarlinFirmware/Marlin
// Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"math"
	"strconv"
)

// Arc plane constants.
const (
	ArcPlaneXY = iota
	ArcPlaneXZ
	ArcPlaneYZ
)

// Axis constants.
const (
	XAxis = 0
	YAxis = 1
	ZAxis = 2
	EAxis = 3
)

// GCodeArcs provides G2/G3 arc support.
type GCodeArcs struct {
	rt *runtime

	mmPerArcSegment float64 // arc segment resolution
	plane           int     // current arc plane

	// GCode move handler
	gcodeMove *gcodeMove
}

// GCodeArcsConfig holds configuration for gcode arcs.
type GCodeArcsConfig struct {
	Resolution float64 // mm per arc segment (default 1.0)
}

// DefaultGCodeArcsConfig returns default gcode arcs configuration.
func DefaultGCodeArcsConfig() GCodeArcsConfig {
	return GCodeArcsConfig{
		Resolution: 1.0,
	}
}

// newGCodeArcs creates a new gcode arcs handler.
func newGCodeArcs(rt *runtime, cfg GCodeArcsConfig) *GCodeArcs {
	if cfg.Resolution <= 0 {
		cfg.Resolution = 1.0
	}
	return &GCodeArcs{
		rt:              rt,
		mmPerArcSegment: cfg.Resolution,
		plane:           ArcPlaneXY,
	}
}

// SetGCodeMove sets the gcode move handler.
func (ga *GCodeArcs) SetGCodeMove(gm *gcodeMove) {
	ga.gcodeMove = gm
}

// cmdG2 handles the G2 (clockwise arc) command.
// G2 [X<pos>] [Y<pos>] [Z<pos>] [I<offset>] [J<offset>] [K<offset>] [E<pos>] [F<speed>]
func (ga *GCodeArcs) cmdG2(args map[string]string) error {
	return ga.cmdArc(args, true)
}

// cmdG3 handles the G3 (counter-clockwise arc) command.
// G3 [X<pos>] [Y<pos>] [Z<pos>] [I<offset>] [J<offset>] [K<offset>] [E<pos>] [F<speed>]
func (ga *GCodeArcs) cmdG3(args map[string]string) error {
	return ga.cmdArc(args, false)
}

// cmdG17 selects XY plane.
func (ga *GCodeArcs) cmdG17() {
	ga.plane = ArcPlaneXY
}

// cmdG18 selects XZ plane.
func (ga *GCodeArcs) cmdG18() {
	ga.plane = ArcPlaneXZ
}

// cmdG19 selects YZ plane.
func (ga *GCodeArcs) cmdG19() {
	ga.plane = ArcPlaneYZ
}

// cmdArc handles G2/G3 arc commands.
func (ga *GCodeArcs) cmdArc(args map[string]string, clockwise bool) error {
	if ga.gcodeMove == nil {
		return fmt.Errorf("gcode_move not available")
	}

	// Get current state
	status := ga.gcodeMove.GetStatus()
	absCoords, ok := status["absolute_coordinates"].(bool)
	if !ok || !absCoords {
		return fmt.Errorf("G2/G3 does not support relative move mode")
	}

	currentPos, ok := status["gcode_position"].([]float64)
	if !ok || len(currentPos) < 4 {
		return fmt.Errorf("invalid gcode position")
	}

	absExtrude, _ := status["absolute_extrude"].(bool)

	// Parse target position (default to current position)
	targetPos := []float64{currentPos[0], currentPos[1], currentPos[2]}

	if xStr, ok := args["X"]; ok {
		v, err := strconv.ParseFloat(xStr, 64)
		if err != nil {
			return fmt.Errorf("invalid X value: %w", err)
		}
		targetPos[0] = v
	}
	if yStr, ok := args["Y"]; ok {
		v, err := strconv.ParseFloat(yStr, 64)
		if err != nil {
			return fmt.Errorf("invalid Y value: %w", err)
		}
		targetPos[1] = v
	}
	if zStr, ok := args["Z"]; ok {
		v, err := strconv.ParseFloat(zStr, 64)
		if err != nil {
			return fmt.Errorf("invalid Z value: %w", err)
		}
		targetPos[2] = v
	}

	// Check for R parameter (not supported)
	if _, ok := args["R"]; ok {
		return fmt.Errorf("G2/G3 does not support R moves")
	}

	// Parse I, J, K offsets
	iVal := 0.0
	jVal := 0.0
	kVal := 0.0

	if iStr, ok := args["I"]; ok {
		v, err := strconv.ParseFloat(iStr, 64)
		if err != nil {
			return fmt.Errorf("invalid I value: %w", err)
		}
		iVal = v
	}
	if jStr, ok := args["J"]; ok {
		v, err := strconv.ParseFloat(jStr, 64)
		if err != nil {
			return fmt.Errorf("invalid J value: %w", err)
		}
		jVal = v
	}
	if kStr, ok := args["K"]; ok {
		v, err := strconv.ParseFloat(kStr, 64)
		if err != nil {
			return fmt.Errorf("invalid K value: %w", err)
		}
		kVal = v
	}

	// Determine plane coordinates and helical axis
	var planar [2]float64
	var alphaAxis, betaAxis, helicalAxis int

	switch ga.plane {
	case ArcPlaneXY:
		planar = [2]float64{iVal, jVal}
		alphaAxis = XAxis
		betaAxis = YAxis
		helicalAxis = ZAxis
	case ArcPlaneXZ:
		planar = [2]float64{iVal, kVal}
		alphaAxis = XAxis
		betaAxis = ZAxis
		helicalAxis = YAxis
	case ArcPlaneYZ:
		planar = [2]float64{jVal, kVal}
		alphaAxis = YAxis
		betaAxis = ZAxis
		helicalAxis = XAxis
	default:
		return fmt.Errorf("unknown arc plane")
	}

	// Require at least one planar offset
	if planar[0] == 0 && planar[1] == 0 {
		return fmt.Errorf("G2/G3 requires IJ, IK or JK parameters")
	}

	// Parse E and F parameters
	var eVal *float64
	var fVal *float64

	if eStr, ok := args["E"]; ok {
		v, err := strconv.ParseFloat(eStr, 64)
		if err != nil {
			return fmt.Errorf("invalid E value: %w", err)
		}
		eVal = &v
	}
	if fStr, ok := args["F"]; ok {
		v, err := strconv.ParseFloat(fStr, 64)
		if err != nil {
			return fmt.Errorf("invalid F value: %w", err)
		}
		fVal = &v
	}

	// Plan and execute the arc
	return ga.planArc(currentPos, targetPos, planar, clockwise,
		absExtrude, eVal, fVal, alphaAxis, betaAxis, helicalAxis)
}

// planArc generates line segments to approximate an arc.
// This function originates from Marlin plan_arc().
func (ga *GCodeArcs) planArc(currentPos, targetPos []float64, offset [2]float64,
	clockwise bool, absoluteExtrude bool, eVal, fVal *float64,
	alphaAxis, betaAxis, helicalAxis int) error {

	// Radius vector from center to current location
	rP := -offset[0]
	rQ := -offset[1]

	// Determine angular travel
	centerP := currentPos[alphaAxis] - rP
	centerQ := currentPos[betaAxis] - rQ
	rtAlpha := targetPos[alphaAxis] - centerP
	rtBeta := targetPos[betaAxis] - centerQ

	angularTravel := math.Atan2(rP*rtBeta-rQ*rtAlpha, rP*rtAlpha+rQ*rtBeta)
	if angularTravel < 0 {
		angularTravel += 2 * math.Pi
	}
	if clockwise {
		angularTravel -= 2 * math.Pi
	}

	// Make a full circle if angular rotation is 0 and target equals current
	if angularTravel == 0 &&
		currentPos[alphaAxis] == targetPos[alphaAxis] &&
		currentPos[betaAxis] == targetPos[betaAxis] {
		angularTravel = 2 * math.Pi
	}

	// Determine number of segments
	linearTravel := targetPos[helicalAxis] - currentPos[helicalAxis]
	radius := math.Hypot(rP, rQ)
	flatMM := radius * angularTravel
	var mmOfTravel float64
	if linearTravel != 0 {
		mmOfTravel = math.Hypot(flatMM, linearTravel)
	} else {
		mmOfTravel = math.Abs(flatMM)
	}

	segments := math.Max(1, math.Floor(mmOfTravel/ga.mmPerArcSegment))

	// Generate coordinates
	thetaPerSegment := angularTravel / segments
	linearPerSegment := linearTravel / segments

	ePerMove := 0.0
	eBase := 0.0
	if eVal != nil {
		if absoluteExtrude {
			eBase = currentPos[EAxis]
		}
		ePerMove = (*eVal - eBase) / segments
	}

	// Generate line segments
	for i := 1; i <= int(segments); i++ {
		distHelical := float64(i) * linearPerSegment
		cTheta := float64(i) * thetaPerSegment
		cosTi := math.Cos(cTheta)
		sinTi := math.Sin(cTheta)
		rP = -offset[0]*cosTi + offset[1]*sinTi
		rQ = -offset[0]*sinTi - offset[1]*cosTi

		c := make([]float64, 3)
		c[alphaAxis] = centerP + rP
		c[betaAxis] = centerQ + rQ
		c[helicalAxis] = currentPos[helicalAxis] + distHelical

		// Last segment uses exact target position
		if i == int(segments) {
			c = targetPos
		}

		// Build G1 parameters
		g1Params := map[string]string{
			"X": strconv.FormatFloat(c[0], 'f', -1, 64),
			"Y": strconv.FormatFloat(c[1], 'f', -1, 64),
			"Z": strconv.FormatFloat(c[2], 'f', -1, 64),
		}

		if ePerMove != 0 {
			e := eBase + ePerMove
			g1Params["E"] = strconv.FormatFloat(e, 'f', -1, 64)
			if absoluteExtrude {
				eBase += ePerMove
			}
		}

		if fVal != nil {
			g1Params["F"] = strconv.FormatFloat(*fVal, 'f', -1, 64)
		}

		// Execute G1 move
		if err := ga.gcodeMove.cmdG1(g1Params); err != nil {
			return fmt.Errorf("arc segment %d: %w", i, err)
		}
	}

	return nil
}

// GetPlane returns the current arc plane.
func (ga *GCodeArcs) GetPlane() int {
	return ga.plane
}

// GetPlaneName returns the name of the current arc plane.
func (ga *GCodeArcs) GetPlaneName() string {
	switch ga.plane {
	case ArcPlaneXY:
		return "XY"
	case ArcPlaneXZ:
		return "XZ"
	case ArcPlaneYZ:
		return "YZ"
	default:
		return "unknown"
	}
}

// GetStatus returns the gcode arcs status.
func (ga *GCodeArcs) GetStatus() map[string]any {
	return map[string]any{
		"plane":               ga.GetPlaneName(),
		"mm_per_arc_segment": ga.mmPerArcSegment,
	}
}
