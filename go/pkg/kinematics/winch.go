// Winch kinematics for cable-driven robots.
// The winch kinematics allows for 3+ cable-driven robots where each cable
// is controlled by a separate stepper motor.
package kinematics

import (
	"fmt"
	"math"
)

// WinchKinematics implements kinematics for cable winch robots.
// Each stepper controls a cable attached to an anchor point.
type WinchKinematics struct {
	*BaseKinematics
	anchors [][3]float64 // Anchor positions for each stepper [x, y, z]
}

// WinchConfig holds configuration for winch kinematics.
type WinchConfig struct {
	Anchors      [][3]float64 // Anchor positions for each stepper
	MaxZVelocity float64
	MaxZAccel    float64
}

// NewWinchKinematics creates a new winch kinematics instance.
func NewWinchKinematics(cfg WinchConfig) (*WinchKinematics, error) {
	if len(cfg.Anchors) < 3 {
		return nil, fmt.Errorf("winch kinematics requires at least 3 anchors")
	}

	// Create rails from anchor configuration
	rails := make([]Rail, len(cfg.Anchors))
	for i := range cfg.Anchors {
		rails[i] = Rail{
			Name:        fmt.Sprintf("stepper_%c", 'a'+i),
			PositionMin: 0,
			PositionMax: 10000, // Cable length maximum
		}
	}

	// Calculate bounds from anchor positions
	minX, maxX := cfg.Anchors[0][0], cfg.Anchors[0][0]
	minY, maxY := cfg.Anchors[0][1], cfg.Anchors[0][1]
	minZ, maxZ := cfg.Anchors[0][2], cfg.Anchors[0][2]
	for _, a := range cfg.Anchors[1:] {
		minX = math.Min(minX, a[0])
		maxX = math.Max(maxX, a[0])
		minY = math.Min(minY, a[1])
		maxY = math.Max(maxY, a[1])
		minZ = math.Min(minZ, a[2])
		maxZ = math.Max(maxZ, a[2])
	}

	wk := &WinchKinematics{
		BaseKinematics: NewBaseKinematics(rails, cfg.MaxZVelocity, cfg.MaxZAccel),
		anchors:        cfg.Anchors,
	}

	// Update bounds based on anchor positions
	wk.AxesMin = []float64{minX, minY, minZ}
	wk.AxesMax = []float64{maxX, maxY, maxZ}

	// Winch always starts as "homed" (position 0,0,0)
	wk.SetPosition([]float64{0, 0, 0}, "xyz")

	return wk, nil
}

// GetType returns the kinematic type name.
func (wk *WinchKinematics) GetType() string {
	return "winch"
}

// CalcPosition calculates the cartesian position from stepper positions (cable lengths).
// Uses trilateration to determine position from the first 3 cable lengths.
func (wk *WinchKinematics) CalcPosition(stepperPositions map[string]float64) []float64 {
	// Get cable lengths from stepper positions
	lengths := make([]float64, 3)
	for i := 0; i < 3; i++ {
		name := fmt.Sprintf("stepper_%c", 'a'+i)
		lengths[i] = stepperPositions[name]
	}

	// Trilaterate position from first 3 anchors and cable lengths
	return winchTrilateration(wk.anchors[:3], lengths)
}

// winchTrilateration calculates the position from 3 anchor points and distances.
// This is the same algorithm used in Python's mathutil.trilateration for winch.
func winchTrilateration(anchors [][3]float64, distances []float64) []float64 {
	// Use the squared distances for the calculation
	sqDistances := make([]float64, 3)
	for i := range distances {
		sqDistances[i] = distances[i] * distances[i]
	}

	// Get anchor coordinates
	p1, p2, p3 := anchors[0], anchors[1], anchors[2]

	// Translate so p1 is at origin
	ex := [3]float64{p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]}

	// Calculate distance from p1 to p2
	d := math.Sqrt(ex[0]*ex[0] + ex[1]*ex[1] + ex[2]*ex[2])
	if d == 0 {
		return []float64{0, 0, 0}
	}

	// Normalize ex
	ex[0] /= d
	ex[1] /= d
	ex[2] /= d

	// Calculate vector to p3
	p13 := [3]float64{p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]}

	// i = ex . p13
	i := ex[0]*p13[0] + ex[1]*p13[1] + ex[2]*p13[2]

	// ey = p13 - i*ex, normalized
	ey := [3]float64{p13[0] - i*ex[0], p13[1] - i*ex[1], p13[2] - i*ex[2]}
	j := math.Sqrt(ey[0]*ey[0] + ey[1]*ey[1] + ey[2]*ey[2])
	if j == 0 {
		return []float64{0, 0, 0}
	}
	ey[0] /= j
	ey[1] /= j
	ey[2] /= j

	// ez = ex x ey
	ez := [3]float64{
		ex[1]*ey[2] - ex[2]*ey[1],
		ex[2]*ey[0] - ex[0]*ey[2],
		ex[0]*ey[1] - ex[1]*ey[0],
	}

	// Calculate position
	x := (sqDistances[0] - sqDistances[1] + d*d) / (2 * d)
	y := (sqDistances[0] - sqDistances[2] + i*i + j*j - 2*i*x) / (2 * j)
	z2 := sqDistances[0] - x*x - y*y
	z := 0.0
	if z2 > 0 {
		z = math.Sqrt(z2)
	}

	// Transform back to original coordinate system
	return []float64{
		p1[0] + x*ex[0] + y*ey[0] + z*ez[0],
		p1[1] + x*ex[1] + y*ey[1] + z*ez[1],
		p1[2] + x*ex[2] + y*ey[2] + z*ez[2],
	}
}

// SetPosition sets the current position (winch doesn't have traditional homing).
func (wk *WinchKinematics) SetPosition(newPos []float64, homingAxes string) {
	// Winch kinematics doesn't use traditional homing.
	// It just sets position directly.
	wk.BaseKinematics.SetPosition(newPos, homingAxes)
}

// ClearHomingState clears the homing state for specified axes.
func (wk *WinchKinematics) ClearHomingState(clearAxes string) {
	// XXX - homing not implemented for winch
	// Just delegate to base
	wk.BaseKinematics.ClearHomingState(clearAxes)
}

// CheckMove validates a move (winch has minimal boundary checking).
func (wk *WinchKinematics) CheckMove(move *Move) error {
	// XXX - boundary checks and speed limits not implemented for winch
	// Just pass through for now
	return nil
}

// GetStatus returns the current status.
func (wk *WinchKinematics) GetStatus() map[string]interface{} {
	// Winch always reports as homed (XXX - homing not fully implemented)
	return map[string]interface{}{
		"homed_axes":   "xyz",
		"axis_minimum": wk.AxesMin,
		"axis_maximum": wk.AxesMax,
	}
}
