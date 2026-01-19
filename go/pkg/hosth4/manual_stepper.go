// Manual stepper - port of klippy/extras/manual_stepper.py
//
// Support for a manual controlled stepper
//
// Copyright (C) 2019-2025 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"strconv"
)

// ManualStepper provides manual control of a single stepper motor.
type ManualStepper struct {
	rt   *runtime
	name string

	stepper     *stepper
	canHome     bool    // true if endstop is configured
	velocity    float64 // default velocity
	accel       float64 // default acceleration
	homingAccel float64 // acceleration for homing moves

	nextCmdTime  float64 // next command time
	commandedPos float64 // current commanded position

	posMin *float64 // optional minimum position
	posMax *float64 // optional maximum position

	// Extra axis support (for gcode axis registration)
	axisGcodeID       *string // nil if not registered as gcode axis
	instantCornerV    float64 // instantaneous corner velocity
	gaxisLimitVel     float64 // limit velocity when used as gcode axis
	gaxisLimitAccel   float64 // limit accel when used as gcode axis
}

// ManualStepperConfig holds configuration for manual stepper.
type ManualStepperConfig struct {
	Name        string
	Stepper     *stepper
	HasEndstop  bool
	Velocity    float64
	Accel       float64
	PositionMin *float64
	PositionMax *float64
}

// DefaultManualStepperConfig returns default manual stepper configuration.
func DefaultManualStepperConfig() ManualStepperConfig {
	return ManualStepperConfig{
		Velocity: 5.0,
		Accel:    0.0,
	}
}

// newManualStepper creates a new manual stepper controller.
func newManualStepper(rt *runtime, cfg ManualStepperConfig) *ManualStepper {
	return &ManualStepper{
		rt:          rt,
		name:        cfg.Name,
		stepper:     cfg.Stepper,
		canHome:     cfg.HasEndstop,
		velocity:    cfg.Velocity,
		accel:       cfg.Accel,
		homingAccel: cfg.Accel,
		posMin:      cfg.PositionMin,
		posMax:      cfg.PositionMax,
	}
}

// GetName returns the name of this manual stepper.
func (ms *ManualStepper) GetName() string {
	return ms.name
}

// syncPrintTime synchronizes with toolhead print time.
func (ms *ManualStepper) syncPrintTime() error {
	if ms.rt == nil || ms.rt.toolhead == nil {
		return fmt.Errorf("toolhead not available")
	}

	th := ms.rt.toolhead
	printTime, err := th.getLastMoveTime()
	if err != nil {
		return err
	}

	if ms.nextCmdTime > printTime {
		if err := th.dwell(ms.nextCmdTime - printTime); err != nil {
			return err
		}
	} else {
		ms.nextCmdTime = printTime
	}
	return nil
}

// DoEnable enables or disables the stepper motor.
func (ms *ManualStepper) DoEnable(enable bool) error {
	if ms.rt == nil || ms.rt.stepperEnable == nil {
		return nil
	}
	// In a full implementation, this would call stepper_enable.set_motors_enable
	_ = enable
	return nil
}

// DoSetPosition sets the current position without moving.
func (ms *ManualStepper) DoSetPosition(pos float64) error {
	if ms.rt == nil || ms.rt.toolhead == nil {
		return fmt.Errorf("toolhead not available")
	}

	if err := ms.rt.toolhead.flushStepGeneration(); err != nil {
		return err
	}
	ms.commandedPos = pos
	// In a full implementation, this would set the stepper position
	return nil
}

// submitMove submits a move to the motion queue.
func (ms *ManualStepper) submitMove(moveTime, movePos, speed, accel float64) float64 {
	cp := ms.commandedPos
	dist := movePos - cp
	axisR, accelT, cruiseT, cruiseV := calcMoveTime(dist, speed, accel)

	// In a full implementation, this would call trapq_append
	log.Printf("manual_stepper: submit_move pos=%.3f dist=%.3f axis_r=%.1f accel_t=%.3f cruise_t=%.3f cruise_v=%.3f",
		movePos, dist, axisR, accelT, cruiseT, cruiseV)

	ms.commandedPos = movePos
	return moveTime + accelT + cruiseT + accelT
}

// DoMove performs a manual move.
func (ms *ManualStepper) DoMove(movePos, speed, accel float64, sync bool) error {
	if err := ms.syncPrintTime(); err != nil {
		return err
	}
	ms.nextCmdTime = ms.submitMove(ms.nextCmdTime, movePos, speed, accel)

	// In a full implementation, this would note mcu movequeue activity

	if sync {
		return ms.syncPrintTime()
	}
	return nil
}

// DoHomingMove performs a homing move.
func (ms *ManualStepper) DoHomingMove(movePos, speed, accel float64, triggered bool, checkTrigger bool) error {
	if !ms.canHome {
		return fmt.Errorf("no endstop for this manual stepper")
	}
	ms.homingAccel = accel

	// In a full implementation, this would call homing.manual_home
	log.Printf("manual_stepper: homing_move pos=%.3f speed=%.3f accel=%.3f triggered=%v check=%v",
		movePos, speed, accel, triggered, checkTrigger)

	return nil
}

// cmdManualStepper handles the MANUAL_STEPPER command.
// MANUAL_STEPPER STEPPER=<name> [ENABLE=<0|1>] [SET_POSITION=<pos>]
//   [MOVE=<pos>] [SPEED=<speed>] [ACCEL=<accel>] [SYNC=<0|1>]
//   [STOP_ON_ENDSTOP=<-1|0|1>]
//   [GCODE_AXIS=<axis>] [INSTANTANEOUS_CORNER_VELOCITY=<vel>]
//   [LIMIT_VELOCITY=<vel>] [LIMIT_ACCEL=<accel>]
func (ms *ManualStepper) cmdManualStepper(args map[string]string) error {
	// Check for gcode axis registration
	if _, ok := args["GCODE_AXIS"]; ok {
		return ms.commandWithGcodeAxis(args)
	}

	// Check if already registered as gcode axis
	if ms.axisGcodeID != nil {
		return fmt.Errorf("must unregister from gcode axis first")
	}

	// Handle ENABLE
	if enableStr, ok := args["ENABLE"]; ok {
		enable, err := strconv.Atoi(enableStr)
		if err != nil {
			return fmt.Errorf("invalid ENABLE value: %w", err)
		}
		if err := ms.DoEnable(enable != 0); err != nil {
			return err
		}
	}

	// Handle SET_POSITION
	if setPosStr, ok := args["SET_POSITION"]; ok {
		setPos, err := strconv.ParseFloat(setPosStr, 64)
		if err != nil {
			return fmt.Errorf("invalid SET_POSITION value: %w", err)
		}
		if err := ms.DoSetPosition(setPos); err != nil {
			return err
		}
	}

	// Get speed and accel (use defaults if not specified)
	speed := ms.velocity
	if speedStr, ok := args["SPEED"]; ok {
		v, err := strconv.ParseFloat(speedStr, 64)
		if err != nil {
			return fmt.Errorf("invalid SPEED value: %w", err)
		}
		if v <= 0 {
			return fmt.Errorf("SPEED must be > 0")
		}
		speed = v
	}

	accel := ms.accel
	if accelStr, ok := args["ACCEL"]; ok {
		v, err := strconv.ParseFloat(accelStr, 64)
		if err != nil {
			return fmt.Errorf("invalid ACCEL value: %w", err)
		}
		if v < 0 {
			return fmt.Errorf("ACCEL must be >= 0")
		}
		accel = v
	}

	// Handle STOP_ON_ENDSTOP (homing move)
	homingMove := 0
	if homingStr, ok := args["STOP_ON_ENDSTOP"]; ok {
		v, err := strconv.Atoi(homingStr)
		if err != nil {
			return fmt.Errorf("invalid STOP_ON_ENDSTOP value: %w", err)
		}
		homingMove = v
	}

	if homingMove != 0 {
		// Homing move requires MOVE parameter
		movePosStr, ok := args["MOVE"]
		if !ok {
			return fmt.Errorf("missing MOVE parameter for homing move")
		}
		movePos, err := strconv.ParseFloat(movePosStr, 64)
		if err != nil {
			return fmt.Errorf("invalid MOVE value: %w", err)
		}

		// Check bounds
		if err := ms.checkBounds(movePos); err != nil {
			return err
		}

		triggered := homingMove > 0
		checkTrigger := abs(homingMove) == 1
		return ms.DoHomingMove(movePos, speed, accel, triggered, checkTrigger)
	}

	// Handle regular MOVE
	if movePosStr, ok := args["MOVE"]; ok {
		movePos, err := strconv.ParseFloat(movePosStr, 64)
		if err != nil {
			return fmt.Errorf("invalid MOVE value: %w", err)
		}

		// Check bounds
		if err := ms.checkBounds(movePos); err != nil {
			return err
		}

		sync := true
		if syncStr, ok := args["SYNC"]; ok {
			v, err := strconv.Atoi(syncStr)
			if err != nil {
				return fmt.Errorf("invalid SYNC value: %w", err)
			}
			sync = v != 0
		}

		return ms.DoMove(movePos, speed, accel, sync)
	}

	// Handle SYNC only
	if syncStr, ok := args["SYNC"]; ok {
		v, err := strconv.Atoi(syncStr)
		if err != nil {
			return fmt.Errorf("invalid SYNC value: %w", err)
		}
		if v != 0 {
			return ms.syncPrintTime()
		}
	}

	return nil
}

// checkBounds checks if a position is within bounds.
func (ms *ManualStepper) checkBounds(pos float64) error {
	if ms.posMin != nil && pos < *ms.posMin {
		return fmt.Errorf("move out of range: %.3f < min %.3f", pos, *ms.posMin)
	}
	if ms.posMax != nil && pos > *ms.posMax {
		return fmt.Errorf("move out of range: %.3f > max %.3f", pos, *ms.posMax)
	}
	return nil
}

// abs returns absolute value of an int.
func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// commandWithGcodeAxis handles gcode axis registration/unregistration.
func (ms *ManualStepper) commandWithGcodeAxis(args map[string]string) error {
	gcodeAxis := args["GCODE_AXIS"]

	// Get optional parameters
	instantCornerV := 1.0
	if icvStr, ok := args["INSTANTANEOUS_CORNER_VELOCITY"]; ok {
		v, err := strconv.ParseFloat(icvStr, 64)
		if err != nil {
			return fmt.Errorf("invalid INSTANTANEOUS_CORNER_VELOCITY: %w", err)
		}
		if v < 0 {
			return fmt.Errorf("INSTANTANEOUS_CORNER_VELOCITY must be >= 0")
		}
		instantCornerV = v
	}

	limitVelocity := 999999.9
	if lvStr, ok := args["LIMIT_VELOCITY"]; ok {
		v, err := strconv.ParseFloat(lvStr, 64)
		if err != nil {
			return fmt.Errorf("invalid LIMIT_VELOCITY: %w", err)
		}
		if v <= 0 {
			return fmt.Errorf("LIMIT_VELOCITY must be > 0")
		}
		limitVelocity = v
	}

	limitAccel := 999999.9
	if laStr, ok := args["LIMIT_ACCEL"]; ok {
		v, err := strconv.ParseFloat(laStr, 64)
		if err != nil {
			return fmt.Errorf("invalid LIMIT_ACCEL: %w", err)
		}
		if v <= 0 {
			return fmt.Errorf("LIMIT_ACCEL must be > 0")
		}
		limitAccel = v
	}

	// Handle unregistration
	if ms.axisGcodeID != nil {
		if gcodeAxis != "" {
			return fmt.Errorf("must unregister axis first")
		}
		// Unregister from toolhead
		// In a full implementation, this would call toolhead.remove_extra_axis
		ms.axisGcodeID = nil
		return nil
	}

	// Validate axis name
	if len(gcodeAxis) != 1 {
		if gcodeAxis == "" {
			// Request to unregister already unregistered axis - OK
			return nil
		}
		return fmt.Errorf("not a valid GCODE_AXIS")
	}
	axis := gcodeAxis[0]
	if axis < 'A' || axis > 'Z' {
		return fmt.Errorf("not a valid GCODE_AXIS")
	}
	// Reserved axes
	if axis == 'X' || axis == 'Y' || axis == 'Z' || axis == 'E' || axis == 'F' || axis == 'N' {
		return fmt.Errorf("not a valid GCODE_AXIS")
	}

	// In a full implementation, this would check for duplicate axis registration
	// and call toolhead.add_extra_axis

	ms.axisGcodeID = &gcodeAxis
	ms.instantCornerV = instantCornerV
	ms.gaxisLimitVel = limitVelocity
	ms.gaxisLimitAccel = limitAccel

	log.Printf("manual_stepper: registered as gcode axis %s", gcodeAxis)
	return nil
}

// GetAxisGcodeID returns the gcode axis ID if registered.
func (ms *ManualStepper) GetAxisGcodeID() *string {
	return ms.axisGcodeID
}

// GetCommandedPos returns the current commanded position.
func (ms *ManualStepper) GetCommandedPos() float64 {
	return ms.commandedPos
}

// GetStatus returns the manual stepper status.
func (ms *ManualStepper) GetStatus() map[string]any {
	result := map[string]any{
		"is_homed": false,          // In a full implementation, this would track homing state
		"position": ms.commandedPos,
	}
	if ms.axisGcodeID != nil {
		result["gcode_axis"] = *ms.axisGcodeID
	}
	return result
}

// ManualStepperManager manages multiple manual stepper instances.
type ManualStepperManager struct {
	rt       *runtime
	steppers map[string]*ManualStepper
}

// newManualStepperManager creates a new manual stepper manager.
func newManualStepperManager(rt *runtime) *ManualStepperManager {
	return &ManualStepperManager{
		rt:       rt,
		steppers: make(map[string]*ManualStepper),
	}
}

// Register registers a new manual stepper.
func (msm *ManualStepperManager) Register(cfg ManualStepperConfig) (*ManualStepper, error) {
	if _, exists := msm.steppers[cfg.Name]; exists {
		return nil, fmt.Errorf("manual_stepper %s already registered", cfg.Name)
	}
	ms := newManualStepper(msm.rt, cfg)
	msm.steppers[cfg.Name] = ms
	return ms, nil
}

// Get returns a manual stepper by name.
func (msm *ManualStepperManager) Get(name string) *ManualStepper {
	return msm.steppers[name]
}

// List returns all registered manual steppers.
func (msm *ManualStepperManager) List() []*ManualStepper {
	result := make([]*ManualStepper, 0, len(msm.steppers))
	for _, ms := range msm.steppers {
		result = append(result, ms)
	}
	return result
}
