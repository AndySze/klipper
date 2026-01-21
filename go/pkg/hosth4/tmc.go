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

// TMCDriverManager manages all TMC drivers in the system.
type TMCDriverManager struct {
	rt      *runtime
	drivers map[string]TMCDriver
	mu      sync.RWMutex
}

// newTMCDriverManager creates a new TMC driver manager.
func newTMCDriverManager(rt *runtime) *TMCDriverManager {
	return &TMCDriverManager{
		rt:      rt,
		drivers: make(map[string]TMCDriver),
	}
}

// RegisterDriver registers a TMC driver.
func (m *TMCDriverManager) RegisterDriver(name string, driver TMCDriver) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.drivers[name] = driver
}

// GetDriver returns a driver by name.
func (m *TMCDriverManager) GetDriver(name string) (TMCDriver, bool) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	d, ok := m.drivers[name]
	return d, ok
}

// GetAllDrivers returns all registered drivers.
func (m *TMCDriverManager) GetAllDrivers() map[string]TMCDriver {
	m.mu.RLock()
	defer m.mu.RUnlock()
	result := make(map[string]TMCDriver, len(m.drivers))
	for k, v := range m.drivers {
		result[k] = v
	}
	return result
}

// cmdSetTMCCurrent handles the SET_TMC_CURRENT command.
// Usage: SET_TMC_CURRENT STEPPER=<name> [CURRENT=<amps>] [HOLDCURRENT=<amps>]
func (m *TMCDriverManager) cmdSetTMCCurrent(args map[string]string) (string, error) {
	stepperName, ok := args["STEPPER"]
	if !ok || stepperName == "" {
		return "", fmt.Errorf("SET_TMC_CURRENT requires STEPPER parameter")
	}

	m.mu.RLock()
	driver, exists := m.drivers[stepperName]
	m.mu.RUnlock()

	if !exists {
		return "", fmt.Errorf("unknown stepper: %s", stepperName)
	}

	// Get current values for reporting
	status := driver.GetStatus()
	runCurrent := 0.0
	holdCurrent := 0.0
	if rc, ok := status["run_current"].(float64); ok {
		runCurrent = rc
	}
	if hc, ok := status["hold_current"].(float64); ok {
		holdCurrent = hc
	}

	// Parse optional new current values
	newRunCurrent := runCurrent
	newHoldCurrent := holdCurrent
	hasChange := false

	if currentStr, ok := args["CURRENT"]; ok && currentStr != "" {
		var err error
		newRunCurrent, err = tmcParseFloat(currentStr)
		if err != nil {
			return "", fmt.Errorf("invalid CURRENT value: %s", currentStr)
		}
		if newRunCurrent <= 0 {
			return "", fmt.Errorf("CURRENT must be positive")
		}
		hasChange = true
	}

	if holdStr, ok := args["HOLDCURRENT"]; ok && holdStr != "" {
		var err error
		newHoldCurrent, err = tmcParseFloat(holdStr)
		if err != nil {
			return "", fmt.Errorf("invalid HOLDCURRENT value: %s", holdStr)
		}
		if newHoldCurrent <= 0 {
			return "", fmt.Errorf("HOLDCURRENT must be positive")
		}
		hasChange = true
	}

	// Apply changes if any
	if hasChange {
		// Try to set current on TMC2208/TMC2209 drivers
		switch d := driver.(type) {
		case *TMC2208Driver:
			if err := d.SetCurrent(newRunCurrent, newHoldCurrent); err != nil {
				return "", err
			}
			runCurrent = newRunCurrent
			holdCurrent = newHoldCurrent
		case *TMC2209Driver:
			if err := d.SetCurrent(newRunCurrent, newHoldCurrent); err != nil {
				return "", err
			}
			runCurrent = newRunCurrent
			holdCurrent = newHoldCurrent
		default:
			return "", fmt.Errorf("driver %s does not support SetCurrent", stepperName)
		}
	}

	// Return current status
	if holdCurrent > 0 {
		return fmt.Sprintf("Run Current: %.2fA Hold Current: %.2fA", runCurrent, holdCurrent), nil
	}
	return fmt.Sprintf("Run Current: %.2fA", runCurrent), nil
}

// cmdDumpTMC handles the DUMP_TMC command.
// Usage: DUMP_TMC STEPPER=<name>
func (m *TMCDriverManager) cmdDumpTMC(args map[string]string) (string, error) {
	stepperName, ok := args["STEPPER"]
	if !ok || stepperName == "" {
		return "", fmt.Errorf("DUMP_TMC requires STEPPER parameter")
	}

	m.mu.RLock()
	driver, exists := m.drivers[stepperName]
	m.mu.RUnlock()

	if !exists {
		return "", fmt.Errorf("unknown stepper: %s", stepperName)
	}

	// Get driver status
	status := driver.GetStatus()

	// Format output
	result := fmt.Sprintf("========== Dump TMC %s ==========\n", stepperName)
	for key, val := range status {
		result += fmt.Sprintf("  %s: %v\n", key, val)
	}

	return result, nil
}

// cmdInitTMC handles the INIT_TMC command.
// Usage: INIT_TMC STEPPER=<name>
func (m *TMCDriverManager) cmdInitTMC(args map[string]string) (string, error) {
	stepperName, ok := args["STEPPER"]
	if !ok || stepperName == "" {
		return "", fmt.Errorf("INIT_TMC requires STEPPER parameter")
	}

	m.mu.RLock()
	_, exists := m.drivers[stepperName]
	m.mu.RUnlock()

	if !exists {
		return "", fmt.Errorf("unknown stepper: %s", stepperName)
	}

	// In file output mode, we just log the init request
	// A live implementation would reinitialize the TMC registers via UART/SPI
	log.Printf("INIT_TMC %s", stepperName)

	return fmt.Sprintf("TMC %s initialized", stepperName), nil
}

// cmdSetTMCField handles the SET_TMC_FIELD command.
// Usage: SET_TMC_FIELD STEPPER=<name> FIELD=<field_name> VALUE=<value>
func (m *TMCDriverManager) cmdSetTMCField(args map[string]string) (string, error) {
	stepperName, ok := args["STEPPER"]
	if !ok || stepperName == "" {
		return "", fmt.Errorf("SET_TMC_FIELD requires STEPPER parameter")
	}

	fieldName, ok := args["FIELD"]
	if !ok || fieldName == "" {
		return "", fmt.Errorf("SET_TMC_FIELD requires FIELD parameter")
	}

	valueStr, ok := args["VALUE"]
	if !ok || valueStr == "" {
		return "", fmt.Errorf("SET_TMC_FIELD requires VALUE parameter")
	}

	m.mu.RLock()
	_, exists := m.drivers[stepperName]
	m.mu.RUnlock()

	if !exists {
		return "", fmt.Errorf("unknown stepper: %s", stepperName)
	}

	value, err := tmcParseInt(valueStr)
	if err != nil {
		return "", fmt.Errorf("invalid VALUE: %s", valueStr)
	}

	// In file output mode, we just log the field set request
	// A live implementation would update the register via UART/SPI
	log.Printf("SET_TMC_FIELD %s FIELD=%s VALUE=%d", stepperName, fieldName, value)

	return fmt.Sprintf("TMC %s field %s set to %d", stepperName, fieldName, value), nil
}

// GetStatus returns status for all TMC drivers.
func (m *TMCDriverManager) GetStatus() map[string]any {
	m.mu.RLock()
	defer m.mu.RUnlock()

	result := make(map[string]any)
	for name, driver := range m.drivers {
		result[name] = driver.GetStatus()
	}
	return result
}

// tmcParseFloat parses a string to float64.
func tmcParseFloat(s string) (float64, error) {
	var f float64
	_, err := fmt.Sscanf(s, "%f", &f)
	return f, err
}

// tmcParseInt parses a string to int.
func tmcParseInt(s string) (int, error) {
	var i int
	_, err := fmt.Sscanf(s, "%d", &i)
	return i, err
}
