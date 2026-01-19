// TMC - Common TMC driver support
//
// Common infrastructure for Trinamic TMC stepper drivers
//
// Copyright (C) 2018-2024 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// TMCFieldHelper manages TMC register field values.
type TMCFieldHelper struct {
	fields        map[string]map[string]uint32 // register -> field -> mask
	signedFields  []string
	fieldValues   map[string]uint32 // field -> value
	mu            sync.RWMutex
}

// newTMCFieldHelper creates a new field helper.
func newTMCFieldHelper(fields map[string]map[string]uint32, signedFields []string) *TMCFieldHelper {
	return &TMCFieldHelper{
		fields:       fields,
		signedFields: signedFields,
		fieldValues:  make(map[string]uint32),
	}
}

// SetField sets a field value.
func (fh *TMCFieldHelper) SetField(name string, value uint32) {
	fh.mu.Lock()
	defer fh.mu.Unlock()
	fh.fieldValues[name] = value
}

// GetField returns a field value.
func (fh *TMCFieldHelper) GetField(name string) uint32 {
	fh.mu.RLock()
	defer fh.mu.RUnlock()
	return fh.fieldValues[name]
}

// GetRegisterValue builds the full register value from field values.
func (fh *TMCFieldHelper) GetRegisterValue(regName string) uint32 {
	fh.mu.RLock()
	defer fh.mu.RUnlock()

	fields, ok := fh.fields[regName]
	if !ok {
		return 0
	}

	var value uint32
	for fieldName, mask := range fields {
		if fieldValue, ok := fh.fieldValues[fieldName]; ok {
			// Find shift from mask
			shift := uint(0)
			for (mask>>shift)&1 == 0 && shift < 32 {
				shift++
			}
			value |= (fieldValue << shift) & mask
		}
	}
	return value
}

// TMCDriver is the common interface for TMC stepper drivers.
type TMCDriver interface {
	GetName() string
	GetPhaseOffset() (int, int)
	GetStatus() map[string]any
}

// TMCCurrentHelper manages driver current settings.
type TMCCurrentHelper struct {
	runCurrent      float64
	holdCurrent     float64
	senseResistor   float64
	idlePercent     int
}

// NewTMCCurrentHelper creates a new current helper.
func NewTMCCurrentHelper(runCurrent, holdCurrent, senseResistor float64) *TMCCurrentHelper {
	if holdCurrent <= 0 {
		holdCurrent = runCurrent
	}
	return &TMCCurrentHelper{
		runCurrent:    runCurrent,
		holdCurrent:   holdCurrent,
		senseResistor: senseResistor,
		idlePercent:   100,
	}
}

// GetRunCurrent returns the run current.
func (ch *TMCCurrentHelper) GetRunCurrent() float64 {
	return ch.runCurrent
}

// GetHoldCurrent returns the hold current.
func (ch *TMCCurrentHelper) GetHoldCurrent() float64 {
	return ch.holdCurrent
}

// SetIdlePercent sets the idle current percentage.
func (ch *TMCCurrentHelper) SetIdlePercent(percent int) {
	ch.idlePercent = percent
}

// TMCRegister represents a TMC register.
type TMCRegister struct {
	Name    string
	Address uint8
	Value   uint32
}

// TMCConfig holds common TMC driver configuration.
type TMCConfig struct {
	Name           string
	RunCurrent     float64
	HoldCurrent    float64
	SenseResistor  float64
	Microsteps     int
	Interpolate    bool
	StealthChop    bool
}

// DefaultTMCConfig returns default TMC configuration.
func DefaultTMCConfig() TMCConfig {
	return TMCConfig{
		Microsteps:    16,
		Interpolate:   true,
		SenseResistor: 0.110,
	}
}

// TMCBase provides common TMC driver functionality.
type TMCBase struct {
	rt            *runtime
	name          string
	fields        *TMCFieldHelper
	currentHelper *TMCCurrentHelper
	microsteps    int
	mstepOffset   int
	phases        int
	mu            sync.RWMutex
}

// NewTMCBase creates a new TMC base.
func NewTMCBase(rt *runtime, name string, fields *TMCFieldHelper, currentHelper *TMCCurrentHelper, microsteps int) *TMCBase {
	return &TMCBase{
		rt:            rt,
		name:          name,
		fields:        fields,
		currentHelper: currentHelper,
		microsteps:    microsteps,
		phases:        microsteps * 4, // Full steps per electrical cycle
	}
}

// GetName returns the driver name.
func (tb *TMCBase) GetName() string {
	return tb.name
}

// GetPhaseOffset returns the phase offset and total phases.
func (tb *TMCBase) GetPhaseOffset() (int, int) {
	tb.mu.RLock()
	defer tb.mu.RUnlock()
	return tb.mstepOffset, tb.phases
}

// GetMicrosteps returns the configured microsteps.
func (tb *TMCBase) GetMicrosteps() int {
	return tb.microsteps
}

// GetStatus returns the driver status.
func (tb *TMCBase) GetStatus() map[string]any {
	tb.mu.RLock()
	defer tb.mu.RUnlock()

	return map[string]any{
		"name":       tb.name,
		"microsteps": tb.microsteps,
		"phases":     tb.phases,
	}
}

// SetMstepOffset sets the microstep offset for phase tracking.
func (tb *TMCBase) SetMstepOffset(offset int) {
	tb.mu.Lock()
	defer tb.mu.Unlock()
	tb.mstepOffset = offset
}

// logInit logs driver initialization.
func logTMCInit(driverType, name string, microsteps int, runCurrent float64) {
	log.Printf("tmc: initialized %s '%s' with %d microsteps, run_current=%.3fA",
		driverType, name, microsteps, runCurrent)
}

// TMCUARTConfig holds UART-based TMC driver configuration.
type TMCUARTConfig struct {
	TMCConfig
	UARTPin  string
	UARTAddr uint8
}

// TMCSPIConfig holds SPI-based TMC driver configuration.
type TMCSPIConfig struct {
	TMCConfig
	SPIBus      string
	SPIChipSel  string
	SPIMode     int
	SPISpeed    int
}

// MicrostepsToMRES converts microsteps to MRES register value.
func MicrostepsToMRES(microsteps int) (uint32, error) {
	switch microsteps {
	case 256:
		return 0, nil
	case 128:
		return 1, nil
	case 64:
		return 2, nil
	case 32:
		return 3, nil
	case 16:
		return 4, nil
	case 8:
		return 5, nil
	case 4:
		return 6, nil
	case 2:
		return 7, nil
	case 1:
		return 8, nil
	default:
		return 0, fmt.Errorf("invalid microsteps: %d", microsteps)
	}
}

// MRESToMicrosteps converts MRES register value to microsteps.
func MRESToMicrosteps(mres uint32) int {
	if mres > 8 {
		return 256
	}
	return 256 >> mres
}
