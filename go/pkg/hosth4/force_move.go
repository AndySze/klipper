// Force move - port of klippy/extras/force_move.py
//
// Utility for manually moving a stepper for diagnostic purposes
//
// Copyright (C) 2018-2025 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"math"
	"strconv"
)

const (
	buzzDistance        = 1.0
	buzzVelocity        = buzzDistance / 0.250
	buzzRadiansDistance = math.Pi / 180.0 // 1 degree in radians
	buzzRadiansVelocity = buzzRadiansDistance / 0.250
)

// calcMoveTime calculates a move's accel_t, cruise_t, and cruise_v.
// Returns: axis_r (direction), accel_t, cruise_t, cruise_v
func calcMoveTime(dist, speed, accel float64) (axisR, accelT, cruiseT, cruiseV float64) {
	axisR = 1.0
	if dist < 0 {
		axisR = -1.0
		dist = -dist
	}
	if accel == 0 || dist == 0 {
		return axisR, 0.0, dist / speed, speed
	}
	maxCruiseV2 := dist * accel
	if maxCruiseV2 < speed*speed {
		speed = math.Sqrt(maxCruiseV2)
	}
	accelT = speed / accel
	accelDecelD := accelT * speed
	cruiseT = (dist - accelDecelD) / speed
	return axisR, accelT, cruiseT, speed
}

// registeredStepper holds a stepper and its metadata for force_move operations.
type registeredStepper struct {
	name           string
	stepper        *stepper
	unitsInRadians bool
}

// ForceMove provides manual stepper control functionality.
type ForceMove struct {
	rt              *runtime
	steppers        map[string]*registeredStepper
	enableForceMove bool
}

// ForceMoveConfig holds configuration for force_move.
type ForceMoveConfig struct {
	EnableForceMove bool
}

// DefaultForceMoveConfig returns the default force_move configuration.
func DefaultForceMoveConfig() ForceMoveConfig {
	return ForceMoveConfig{
		EnableForceMove: false,
	}
}

// newForceMove creates a new force move manager.
func newForceMove(rt *runtime, cfg ForceMoveConfig) *ForceMove {
	return &ForceMove{
		rt:              rt,
		steppers:        make(map[string]*registeredStepper),
		enableForceMove: cfg.EnableForceMove,
	}
}

// RegisterStepper registers a stepper for force move operations.
func (fm *ForceMove) RegisterStepper(name string, s *stepper, unitsInRadians bool) {
	fm.steppers[name] = &registeredStepper{
		name:           name,
		stepper:        s,
		unitsInRadians: unitsInRadians,
	}
}

// LookupStepper returns a registered stepper by name.
func (fm *ForceMove) LookupStepper(name string) (*registeredStepper, error) {
	s, ok := fm.steppers[name]
	if !ok {
		return nil, fmt.Errorf("unknown stepper %s", name)
	}
	return s, nil
}

// forceEnable enables a stepper and returns whether it was previously disabled.
func (fm *ForceMove) forceEnable(rs *registeredStepper) bool {
	if fm.rt == nil || fm.rt.stepperEnable == nil {
		return false
	}
	// Enable the stepper through stepperEnable
	// In a full implementation, this would call the enable method
	_ = rs.name
	return true
}

// restoreEnable restores the previous enable state.
func (fm *ForceMove) restoreEnable(rs *registeredStepper, didEnable bool) {
	if !didEnable {
		return
	}
	if fm.rt == nil || fm.rt.stepperEnable == nil {
		return
	}
	// Disable the stepper
	_ = rs.name
}

// ManualMove performs a manual move on a stepper.
func (fm *ForceMove) ManualMove(rs *registeredStepper, dist, speed, accel float64) error {
	if fm.rt == nil || fm.rt.toolhead == nil {
		return fmt.Errorf("toolhead not available")
	}

	th := fm.rt.toolhead

	// Flush step generation
	if err := th.flushStepGeneration(); err != nil {
		return fmt.Errorf("flush step generation: %w", err)
	}

	// Calculate move timing
	axisR, accelT, cruiseT, cruiseV := calcMoveTime(dist, speed, accel)

	// Get print time
	printTime, err := th.getLastMoveTime()
	if err != nil {
		return fmt.Errorf("get last move time: %w", err)
	}

	// In a full implementation, this would:
	// 1. Set stepper kinematics to a cartesian stepper
	// 2. Set stepper trapq to force_move's trapq
	// 3. Append move to trapq
	// 4. Dwell for move duration
	// 5. Restore original trapq and kinematics

	moveTime := accelT + cruiseT + accelT
	log.Printf("force_move: manual_move stepper=%s dist=%.3f speed=%.3f accel=%.3f axis_r=%.1f time=%.3f",
		rs.name, dist, speed, accel, axisR, moveTime)

	// Dwell for move duration
	if err := th.dwell(moveTime); err != nil {
		return fmt.Errorf("dwell: %w", err)
	}

	// Flush step generation again
	if err := th.flushStepGeneration(); err != nil {
		return fmt.Errorf("flush step generation: %w", err)
	}

	_ = printTime
	_ = cruiseV

	return nil
}

// cmdStepperBuzz handles the STEPPER_BUZZ command.
// STEPPER_BUZZ STEPPER=<name>
func (fm *ForceMove) cmdStepperBuzz(args map[string]string) error {
	stepperName, ok := args["STEPPER"]
	if !ok {
		return fmt.Errorf("missing STEPPER parameter")
	}

	rs, err := fm.LookupStepper(stepperName)
	if err != nil {
		return err
	}

	log.Printf("Stepper buzz %s", rs.name)

	didEnable := fm.forceEnable(rs)

	dist := buzzDistance
	speed := buzzVelocity

	// Check if stepper uses radians
	if rs.unitsInRadians {
		dist = buzzRadiansDistance
		speed = buzzRadiansVelocity
	}

	// Buzz 10 times
	for i := 0; i < 10; i++ {
		if err := fm.ManualMove(rs, dist, speed, 0); err != nil {
			fm.restoreEnable(rs, didEnable)
			return err
		}
		if fm.rt != nil && fm.rt.toolhead != nil {
			fm.rt.toolhead.dwell(0.050)
		}
		if err := fm.ManualMove(rs, -dist, speed, 0); err != nil {
			fm.restoreEnable(rs, didEnable)
			return err
		}
		if fm.rt != nil && fm.rt.toolhead != nil {
			fm.rt.toolhead.dwell(0.450)
		}
	}

	fm.restoreEnable(rs, didEnable)
	return nil
}

// cmdForceMove handles the FORCE_MOVE command.
// FORCE_MOVE STEPPER=<name> DISTANCE=<value> VELOCITY=<value> [ACCEL=<value>]
func (fm *ForceMove) cmdForceMove(args map[string]string) error {
	if !fm.enableForceMove {
		return fmt.Errorf("FORCE_MOVE not enabled in config")
	}

	stepperName, ok := args["STEPPER"]
	if !ok {
		return fmt.Errorf("missing STEPPER parameter")
	}

	rs, err := fm.LookupStepper(stepperName)
	if err != nil {
		return err
	}

	distStr, ok := args["DISTANCE"]
	if !ok {
		return fmt.Errorf("missing DISTANCE parameter")
	}
	distance, err := strconv.ParseFloat(distStr, 64)
	if err != nil {
		return fmt.Errorf("invalid DISTANCE: %w", err)
	}

	velStr, ok := args["VELOCITY"]
	if !ok {
		return fmt.Errorf("missing VELOCITY parameter")
	}
	velocity, err := strconv.ParseFloat(velStr, 64)
	if err != nil {
		return fmt.Errorf("invalid VELOCITY: %w", err)
	}
	if velocity <= 0 {
		return fmt.Errorf("VELOCITY must be > 0")
	}

	accel := 0.0
	if accelStr, ok := args["ACCEL"]; ok {
		accel, err = strconv.ParseFloat(accelStr, 64)
		if err != nil {
			return fmt.Errorf("invalid ACCEL: %w", err)
		}
		if accel < 0 {
			return fmt.Errorf("ACCEL must be >= 0")
		}
	}

	log.Printf("FORCE_MOVE %s distance=%.3f velocity=%.3f accel=%.3f",
		rs.name, distance, velocity, accel)

	fm.forceEnable(rs)
	return fm.ManualMove(rs, distance, velocity, accel)
}

// cmdSetKinematicPosition handles the SET_KINEMATIC_POSITION command.
// SET_KINEMATIC_POSITION [X=<value>] [Y=<value>] [Z=<value>] [SET_HOMED=<axes>] [CLEAR_HOMED=<axes>]
func (fm *ForceMove) cmdSetKinematicPosition(args map[string]string) error {
	if !fm.enableForceMove {
		return fmt.Errorf("SET_KINEMATIC_POSITION not enabled in config")
	}

	if fm.rt == nil || fm.rt.toolhead == nil {
		return fmt.Errorf("toolhead not available")
	}

	th := fm.rt.toolhead

	// Get current position
	curPos := make([]float64, len(th.commandedPos))
	copy(curPos, th.commandedPos)

	// Parse X, Y, Z
	x := curPos[0]
	if xStr, ok := args["X"]; ok {
		v, err := strconv.ParseFloat(xStr, 64)
		if err != nil {
			return fmt.Errorf("invalid X: %w", err)
		}
		x = v
	}

	y := curPos[1]
	if yStr, ok := args["Y"]; ok {
		v, err := strconv.ParseFloat(yStr, 64)
		if err != nil {
			return fmt.Errorf("invalid Y: %w", err)
		}
		y = v
	}

	z := curPos[2]
	if zStr, ok := args["Z"]; ok {
		v, err := strconv.ParseFloat(zStr, 64)
		if err != nil {
			return fmt.Errorf("invalid Z: %w", err)
		}
		z = v
	}

	// Parse SET_HOMED
	setHomed := "xyz"
	if sh, ok := args["SET_HOMED"]; ok {
		setHomed = sh
	}
	setHomedAxes := ""
	for _, c := range setHomed {
		if c == 'x' || c == 'X' || c == 'y' || c == 'Y' || c == 'z' || c == 'Z' {
			setHomedAxes += string(c)
		}
	}

	// Parse CLEAR_HOMED (or CLEAR for backwards compatibility)
	clearHomed := ""
	if ch, ok := args["CLEAR_HOMED"]; ok {
		clearHomed = ch
	} else if ch, ok := args["CLEAR"]; ok {
		clearHomed = ch
	}
	clearHomedAxes := ""
	for _, c := range clearHomed {
		if c == 'x' || c == 'X' || c == 'y' || c == 'Y' || c == 'z' || c == 'Z' {
			clearHomedAxes += string(c)
		}
	}

	log.Printf("SET_KINEMATIC_POSITION pos=%.3f,%.3f,%.3f set_homed=%s clear_homed=%s",
		x, y, z, setHomedAxes, clearHomedAxes)

	// Set position
	newPos := []float64{x, y, z}
	if len(curPos) > 3 {
		newPos = append(newPos, curPos[3:]...)
	}
	if err := th.setPosition(newPos, setHomedAxes); err != nil {
		return fmt.Errorf("set position: %w", err)
	}

	// Clear homing state
	if clearHomedAxes != "" && th.kin != nil {
		th.kin.ClearHomingState(clearHomedAxes)
	}

	return nil
}

// GetStatus returns the force_move status.
func (fm *ForceMove) GetStatus() map[string]any {
	stepperNames := make([]string, 0, len(fm.steppers))
	for name := range fm.steppers {
		stepperNames = append(stepperNames, name)
	}
	return map[string]any{
		"steppers":          stepperNames,
		"enable_force_move": fm.enableForceMove,
	}
}
