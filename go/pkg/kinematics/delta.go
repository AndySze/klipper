// Delta kinematics implementation for linear delta 3D printers.
package kinematics

import (
	"fmt"
	"math"
)

// SLOW_RATIO determines when moves are slowed near the build envelope edge
const SLOW_RATIO = 3.0

// DeltaKinematics implements linear delta kinematics.
// Delta printers have three towers (A, B, C) at 120-degree intervals.
// Each tower has a carriage that moves up/down, connected to the effector by an arm.
type DeltaKinematics struct {
	*BaseKinematics

	// Configuration
	radius      float64   // Delta radius (distance from center to tower)
	armLengths  []float64 // Arm lengths for each tower [A, B, C]
	arm2        []float64 // Squared arm lengths
	angles      []float64 // Tower angles in degrees [210, 330, 90]
	towers      [][2]float64 // Tower XY positions
	absEndstops []float64 // Absolute Z height of each tower endstop

	// Boundaries
	maxZ       float64
	minZ       float64
	limitZ     float64 // Z height where radius starts to taper
	maxXY2     float64 // Maximum XY distance squared
	slowXY2    float64 // XY distance where moves start slowing
	verySlowXY2 float64 // XY distance where moves slow significantly
	limitXY2   float64 // Current XY limit squared (for move checking)
	needHome   bool    // Whether homing is required

	// Home position
	homePosition []float64

	// Speed limits
	maxVelocity  float64
	maxAccel     float64
}

// DeltaConfig contains configuration for delta kinematics
type DeltaConfig struct {
	Radius        float64   // Delta radius
	ArmLengths    []float64 // Arm lengths [A, B, C]
	Angles        []float64 // Tower angles in degrees [A, B, C], default [210, 330, 90]
	Endstops      []float64 // Endstop positions for each tower
	PrintRadius   float64   // Maximum print radius (optional, defaults to radius)
	MinZ          float64   // Minimum Z position
	MaxVelocity   float64   // Maximum velocity
	MaxAccel      float64   // Maximum acceleration
	MaxZVelocity  float64   // Maximum Z velocity
	MaxZAccel     float64   // Maximum Z acceleration
	StepDistances []float64 // Step distances for each stepper
}

// NewDeltaKinematics creates a new delta kinematics instance.
func NewDeltaKinematics(cfg DeltaConfig) (*DeltaKinematics, error) {
	// Validate configuration
	if cfg.Radius <= 0 {
		return nil, fmt.Errorf("delta_radius must be positive")
	}
	if len(cfg.ArmLengths) != 3 {
		return nil, fmt.Errorf("delta requires exactly 3 arm lengths")
	}
	for i, arm := range cfg.ArmLengths {
		if arm <= cfg.Radius {
			return nil, fmt.Errorf("arm_length[%d] must be greater than radius", i)
		}
	}
	if len(cfg.Endstops) != 3 {
		return nil, fmt.Errorf("delta requires exactly 3 endstop positions")
	}

	// Set default angles if not provided
	angles := cfg.Angles
	if len(angles) != 3 {
		angles = []float64{210.0, 330.0, 90.0}
	}

	// Calculate squared arm lengths
	arm2 := make([]float64, 3)
	for i, arm := range cfg.ArmLengths {
		arm2[i] = arm * arm
	}

	// Calculate tower positions in cartesian space
	towers := make([][2]float64, 3)
	for i, angle := range angles {
		rad := angle * math.Pi / 180.0
		towers[i] = [2]float64{
			math.Cos(rad) * cfg.Radius,
			math.Sin(rad) * cfg.Radius,
		}
	}

	// Calculate absolute endstop positions
	radius2 := cfg.Radius * cfg.Radius
	absEndstops := make([]float64, 3)
	for i := range cfg.Endstops {
		absEndstops[i] = cfg.Endstops[i] + math.Sqrt(arm2[i]-radius2)
	}

	// Calculate home position
	homePosition := trilateration(towers, absEndstops, arm2)

	// Calculate max Z
	maxZ := cfg.Endstops[0]
	for _, ep := range cfg.Endstops[1:] {
		if ep < maxZ {
			maxZ = ep
		}
	}

	// Calculate limit Z (where radius starts to taper)
	minArmLength := cfg.ArmLengths[0]
	for _, arm := range cfg.ArmLengths[1:] {
		if arm < minArmLength {
			minArmLength = arm
		}
	}
	limitZ := absEndstops[0] - cfg.ArmLengths[0]
	for i := 1; i < 3; i++ {
		lz := absEndstops[i] - cfg.ArmLengths[i]
		if lz < limitZ {
			limitZ = lz
		}
	}

	// Calculate XY limits
	printRadius := cfg.PrintRadius
	if printRadius <= 0 {
		printRadius = cfg.Radius
	}

	halfMinStepDist := 0.01 // Default, would come from stepper config
	if len(cfg.StepDistances) >= 3 {
		halfMinStepDist = cfg.StepDistances[0]
		for _, sd := range cfg.StepDistances[1:] {
			if sd < halfMinStepDist {
				halfMinStepDist = sd
			}
		}
		halfMinStepDist *= 0.5
	}

	minArm2 := minArmLength * minArmLength

	// Calculate the point where an XY move could result in excessive tower movement
	ratioToXY := func(ratio float64) float64 {
		return ratio*math.Sqrt(minArm2/(ratio*ratio+1.0)-halfMinStepDist*halfMinStepDist) +
			halfMinStepDist - cfg.Radius
	}

	slowXY2 := ratioToXY(SLOW_RATIO)
	slowXY2 = slowXY2 * slowXY2
	verySlowXY2 := ratioToXY(2.0 * SLOW_RATIO)
	verySlowXY2 = verySlowXY2 * verySlowXY2

	maxXY := printRadius
	if minArmLength-cfg.Radius < maxXY {
		maxXY = minArmLength - cfg.Radius
	}
	r4 := ratioToXY(4.0 * SLOW_RATIO)
	if r4 < maxXY {
		maxXY = r4
	}
	maxXY2 := maxXY * maxXY

	// Create base kinematics with 3 rails
	rails := make([]Rail, 3)
	for i, axis := range []string{"a", "b", "c"} {
		rails[i] = Rail{
			Name:            "stepper_" + axis,
			PositionMin:     cfg.MinZ,
			PositionMax:     maxZ,
			PositionEndstop: cfg.Endstops[i],
		}
	}

	dk := &DeltaKinematics{
		BaseKinematics: NewBaseKinematics(rails, cfg.MaxZVelocity, cfg.MaxZAccel),
		radius:         cfg.Radius,
		armLengths:     cfg.ArmLengths,
		arm2:           arm2,
		angles:         angles,
		towers:         towers,
		absEndstops:    absEndstops,
		maxZ:           maxZ,
		minZ:           cfg.MinZ,
		limitZ:         limitZ,
		maxXY2:         maxXY2,
		slowXY2:        slowXY2,
		verySlowXY2:    verySlowXY2,
		limitXY2:       -1.0,
		needHome:       true,
		homePosition:   homePosition,
		maxVelocity:    cfg.MaxVelocity,
		maxAccel:       cfg.MaxAccel,
	}

	// Update axes bounds
	dk.AxesMin = []float64{-maxXY, -maxXY, cfg.MinZ}
	dk.AxesMax = []float64{maxXY, maxXY, maxZ}

	return dk, nil
}

// GetType returns the kinematic type name.
func (dk *DeltaKinematics) GetType() string {
	return "delta"
}

// CalcPosition calculates the cartesian position from stepper positions.
// Uses trilateration to find the point where all three arm spheres intersect.
func (dk *DeltaKinematics) CalcPosition(stepperPositions map[string]float64) []float64 {
	spos := make([]float64, 3)

	for i, axis := range []string{"a", "b", "c"} {
		if pos, ok := stepperPositions["stepper_"+axis]; ok {
			spos[i] = pos
		}
	}

	return trilateration(dk.towers, spos, dk.arm2)
}

// SetPosition sets the current position and updates homing state.
func (dk *DeltaKinematics) SetPosition(newPos []float64, homingAxes string) {
	dk.BaseKinematics.SetPosition(newPos, homingAxes)
	dk.limitXY2 = -1.0
	if homingAxes == "xyz" {
		dk.needHome = false
	}
}

// ClearHomingState clears the homing state for specified axes.
func (dk *DeltaKinematics) ClearHomingState(clearAxes string) {
	if clearAxes != "" {
		dk.limitXY2 = -1.0
		dk.needHome = true
	}
}

// CheckMove validates a move and applies any necessary speed limits.
func (dk *DeltaKinematics) CheckMove(move *Move) error {
	endPos := move.EndPos
	endXY2 := endPos[0]*endPos[0] + endPos[1]*endPos[1]

	// Check if this is a normal XY move within limits
	if endXY2 <= dk.limitXY2 && (len(move.AxesD) <= 2 || move.AxesD[2] == 0) {
		return nil
	}

	// Check if homing is needed
	if dk.needHome {
		return fmt.Errorf("must home first")
	}

	endZ := 0.0
	if len(endPos) > 2 {
		endZ = endPos[2]
	}

	// Calculate XY limit based on Z position
	limitXY2 := dk.maxXY2
	if endZ > dk.limitZ {
		// Above limit_z, the printable radius tapers
		aboveZLimit := endZ - dk.limitZ
		minArmLength := dk.armLengths[0]
		for _, arm := range dk.armLengths[1:] {
			if arm < minArmLength {
				minArmLength = arm
			}
		}
		minArm2 := minArmLength * minArmLength
		allowedRadius := dk.radius - math.Sqrt(minArm2-(minArmLength-aboveZLimit)*(minArmLength-aboveZLimit))
		newLimit := allowedRadius * allowedRadius
		if newLimit < limitXY2 {
			limitXY2 = newLimit
		}
	}

	// Check if move is out of range
	if endXY2 > limitXY2 || endZ > dk.maxZ || endZ < dk.minZ {
		// Check if this is a homing move
		homeX, homeY := dk.homePosition[0], dk.homePosition[1]
		if endPos[0] != homeX || endPos[1] != homeY ||
			endZ < dk.minZ || endZ > dk.homePosition[2] {
			return fmt.Errorf("move out of range")
		}
		limitXY2 = -1.0
	}

	// Apply Z velocity limits if there's Z movement
	if len(move.AxesD) > 2 && move.AxesD[2] != 0 {
		zRatio := move.MoveD / math.Abs(move.AxesD[2])
		move.LimitSpeed(dk.MaxZVelocity*zRatio, dk.MaxZAccel*zRatio)
		limitXY2 = -1.0
	}

	// Slow down moves near the build envelope edge
	startXY2 := 0.0
	if len(move.StartPos) >= 2 {
		startXY2 = move.StartPos[0]*move.StartPos[0] + move.StartPos[1]*move.StartPos[1]
	}
	extremeXY2 := endXY2
	if startXY2 > extremeXY2 {
		extremeXY2 = startXY2
	}

	if extremeXY2 > dk.slowXY2 {
		r := 0.5
		if extremeXY2 > dk.verySlowXY2 {
			r = 0.25
		}
		move.LimitSpeed(dk.maxVelocity*r, dk.maxAccel*r)
		limitXY2 = -1.0
	}

	// Update the limit for next move check
	if limitXY2 > dk.slowXY2 {
		dk.limitXY2 = dk.slowXY2
	} else {
		dk.limitXY2 = limitXY2
	}

	return nil
}

// GetStatus returns the current status of the kinematics.
func (dk *DeltaKinematics) GetStatus() map[string]interface{} {
	homedAxes := ""
	if !dk.needHome {
		homedAxes = "xyz"
	}

	return map[string]interface{}{
		"homed_axes":   homedAxes,
		"axis_minimum": dk.AxesMin,
		"axis_maximum": dk.AxesMax,
		"cone_start_z": dk.limitZ,
	}
}

// GetHomePosition returns the home position for delta.
func (dk *DeltaKinematics) GetHomePosition() []float64 {
	return dk.homePosition
}

// CalcStepperPosition calculates stepper positions from cartesian coordinates.
// For delta, this is the inverse kinematics.
func (dk *DeltaKinematics) CalcStepperPosition(cartesianPos []float64) map[string]float64 {
	stepperPos := make(map[string]float64)

	x, y, z := 0.0, 0.0, 0.0
	if len(cartesianPos) >= 1 {
		x = cartesianPos[0]
	}
	if len(cartesianPos) >= 2 {
		y = cartesianPos[1]
	}
	if len(cartesianPos) >= 3 {
		z = cartesianPos[2]
	}

	for i, axis := range []string{"a", "b", "c"} {
		// Calculate the stepper position for this tower
		// stepper_pos = sqrt(arm^2 - (tower_x - x)^2 - (tower_y - y)^2) + z
		dx := dk.towers[i][0] - x
		dy := dk.towers[i][1] - y
		spos := math.Sqrt(dk.arm2[i]-dx*dx-dy*dy) + z
		stepperPos["stepper_"+axis] = spos
	}

	return stepperPos
}

// trilateration calculates the intersection point of three spheres.
// towers: XY positions of the three tower bases
// spos: Z positions of the three carriages
// arm2: squared arm lengths
func trilateration(towers [][2]float64, spos []float64, arm2 []float64) []float64 {
	// Create sphere coordinates (tower x, tower y, carriage z)
	sphereCoord1 := []float64{towers[0][0], towers[0][1], spos[0]}
	sphereCoord2 := []float64{towers[1][0], towers[1][1], spos[1]}
	sphereCoord3 := []float64{towers[2][0], towers[2][1], spos[2]}

	// s21 = sphere2 - sphere1
	s21 := []float64{
		sphereCoord2[0] - sphereCoord1[0],
		sphereCoord2[1] - sphereCoord1[1],
		sphereCoord2[2] - sphereCoord1[2],
	}

	// s31 = sphere3 - sphere1
	s31 := []float64{
		sphereCoord3[0] - sphereCoord1[0],
		sphereCoord3[1] - sphereCoord1[1],
		sphereCoord3[2] - sphereCoord1[2],
	}

	// d = |s21|
	d := math.Sqrt(s21[0]*s21[0] + s21[1]*s21[1] + s21[2]*s21[2])

	// ex = s21 / d (unit vector)
	ex := []float64{s21[0] / d, s21[1] / d, s21[2] / d}

	// i = ex . s31
	i := ex[0]*s31[0] + ex[1]*s31[1] + ex[2]*s31[2]

	// vect_ey = s31 - ex * i
	vectEy := []float64{
		s31[0] - ex[0]*i,
		s31[1] - ex[1]*i,
		s31[2] - ex[2]*i,
	}

	// ey = vect_ey / |vect_ey|
	eyMag := math.Sqrt(vectEy[0]*vectEy[0] + vectEy[1]*vectEy[1] + vectEy[2]*vectEy[2])
	ey := []float64{vectEy[0] / eyMag, vectEy[1] / eyMag, vectEy[2] / eyMag}

	// ez = ex x ey (cross product)
	ez := []float64{
		ex[1]*ey[2] - ex[2]*ey[1],
		ex[2]*ey[0] - ex[0]*ey[2],
		ex[0]*ey[1] - ex[1]*ey[0],
	}

	// j = ey . s31
	j := ey[0]*s31[0] + ey[1]*s31[1] + ey[2]*s31[2]

	// Calculate x, y, z in the new coordinate system
	x := (arm2[0] - arm2[1] + d*d) / (2.0 * d)
	y := (arm2[0] - arm2[2] - x*x + (x-i)*(x-i) + j*j) / (2.0 * j)
	z := -math.Sqrt(arm2[0] - x*x - y*y)

	// Transform back to original coordinate system
	// result = sphere1 + ex*x + ey*y + ez*z
	return []float64{
		sphereCoord1[0] + ex[0]*x + ey[0]*y + ez[0]*z,
		sphereCoord1[1] + ex[1]*x + ey[1]*y + ez[1]*z,
		sphereCoord1[2] + ex[2]*x + ey[2]*y + ez[2]*z,
	}
}