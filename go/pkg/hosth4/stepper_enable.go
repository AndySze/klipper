// Stepper Enable - port of klippy/extras/stepper_enable.py
//
// Support for enable pins on stepper motor drivers
//
// Copyright (C) 2019-2025 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

const (
	// DisableStallTime is the delay before/after disabling steppers
	DisableStallTime = 0.100
)

// StepperEnableManager provides high-level stepper enable management.
// This wraps the internal stepperEnable type to provide a cleaner API.
type StepperEnableManager struct {
	rt *runtime

	mu sync.RWMutex
}

// newStepperEnableManager creates a new stepper enable manager.
func newStepperEnableManager(rt *runtime) *StepperEnableManager {
	return &StepperEnableManager{rt: rt}
}

// GetSteppers returns a list of all stepper names.
func (sem *StepperEnableManager) GetSteppers() []string {
	sem.mu.RLock()
	defer sem.mu.RUnlock()

	if sem.rt == nil || sem.rt.stepperEnable == nil {
		return nil
	}

	names := make([]string, 0, len(sem.rt.stepperEnable.lines))
	for name := range sem.rt.stepperEnable.lines {
		names = append(names, name)
	}
	return names
}

// IsMotorEnabled returns whether a specific stepper is enabled.
func (sem *StepperEnableManager) IsMotorEnabled(stepperName string) (bool, error) {
	sem.mu.RLock()
	defer sem.mu.RUnlock()

	if sem.rt == nil || sem.rt.stepperEnable == nil {
		return false, fmt.Errorf("stepper enable not initialized")
	}

	et, ok := sem.rt.stepperEnable.lines[stepperName]
	if !ok {
		return false, fmt.Errorf("unknown stepper '%s'", stepperName)
	}

	return et.isEnabled, nil
}

// SetMotorsEnable enables or disables specific steppers.
func (sem *StepperEnableManager) SetMotorsEnable(stepperNames []string, enable bool) (bool, error) {
	sem.mu.Lock()
	defer sem.mu.Unlock()

	if sem.rt == nil || sem.rt.stepperEnable == nil || sem.rt.toolhead == nil {
		return false, fmt.Errorf("stepper enable not initialized")
	}

	th := sem.rt.toolhead

	// Flush steps to ensure all auto enable callbacks invoked
	if err := th.flushStepGeneration(); err != nil {
		return false, err
	}

	printTime := 0.0
	didChange := false

	for _, stepperName := range stepperNames {
		et, ok := sem.rt.stepperEnable.lines[stepperName]
		if !ok {
			continue
		}
		if et.isEnabled == enable {
			continue
		}

		if printTime == 0 {
			// Dwell for sufficient delay from any previous auto enable
			if !enable {
				th.advanceMoveTime(th.printTime + DisableStallTime)
			}
			var err error
			printTime, err = th.getLastMoveTime()
			if err != nil {
				return false, err
			}
		}

		if enable {
			et.motorEnable(printTime)
		} else {
			et.motorDisable(printTime)
		}
		didChange = true
	}

	// Dwell to ensure sufficient delay prior to a future auto enable
	if didChange && !enable {
		th.advanceMoveTime(th.printTime + DisableStallTime)
	}

	return didChange, nil
}

// MotorOff turns off all motors.
func (sem *StepperEnableManager) MotorOff() error {
	sem.mu.Lock()
	defer sem.mu.Unlock()

	if sem.rt == nil || sem.rt.stepperEnable == nil {
		return fmt.Errorf("stepper enable not initialized")
	}

	return sem.rt.stepperEnable.motorOff()
}

// LookupEnable returns the enable tracking for a stepper.
func (sem *StepperEnableManager) LookupEnable(name string) (*enableTracking, error) {
	sem.mu.RLock()
	defer sem.mu.RUnlock()

	if sem.rt == nil || sem.rt.stepperEnable == nil {
		return nil, fmt.Errorf("stepper enable not initialized")
	}

	et, ok := sem.rt.stepperEnable.lines[name]
	if !ok {
		return nil, fmt.Errorf("unknown stepper '%s'", name)
	}

	return et, nil
}

// GetStatus returns the status of all steppers.
func (sem *StepperEnableManager) GetStatus() map[string]any {
	sem.mu.RLock()
	defer sem.mu.RUnlock()

	if sem.rt == nil || sem.rt.stepperEnable == nil {
		return map[string]any{"steppers": map[string]bool{}}
	}

	steppers := make(map[string]bool)
	for name, et := range sem.rt.stepperEnable.lines {
		steppers[name] = et.isEnabled
	}

	return map[string]any{"steppers": steppers}
}

// cmdM18 handles M18/M84 (disable motors) command.
func (sem *StepperEnableManager) cmdM18() error {
	if err := sem.MotorOff(); err != nil {
		return err
	}
	log.Printf("stepper_enable: M18/M84 motors disabled")
	return nil
}

// cmdSetStepperEnable handles SET_STEPPER_ENABLE command.
func (sem *StepperEnableManager) cmdSetStepperEnable(stepperName string, enable bool) error {
	if sem.rt == nil || sem.rt.stepperEnable == nil {
		return fmt.Errorf("stepper enable not initialized")
	}

	_, ok := sem.rt.stepperEnable.lines[stepperName]
	if !ok {
		return fmt.Errorf("SET_STEPPER_ENABLE: invalid stepper '%s'", stepperName)
	}

	_, err := sem.SetMotorsEnable([]string{stepperName}, enable)
	if err != nil {
		return err
	}

	if enable {
		log.Printf("stepper_enable: %s has been manually enabled", stepperName)
	} else {
		log.Printf("stepper_enable: %s has been manually disabled", stepperName)
	}

	return nil
}

// HasDedicatedEnable returns whether a stepper has a dedicated enable pin.
// A stepper has a dedicated enable pin if it has an enablePin configured.
func (sem *StepperEnableManager) HasDedicatedEnable(stepperName string) (bool, error) {
	et, err := sem.LookupEnable(stepperName)
	if err != nil {
		return false, err
	}

	// If the stepper has an enablePin, it's considered to have a dedicated enable
	return et.enablePin != nil, nil
}
