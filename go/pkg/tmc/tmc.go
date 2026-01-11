// TMC stepper driver support - port of klippy/extras/tmc.py
//
// Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025  Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package tmc

import (
	"fmt"
	"sort"
	"strings"
)

// ffs returns the position of the first bit set in a mask.
func ffs(mask uint32) int {
	if mask == 0 {
		return 0
	}
	pos := 0
	for (mask & 1) == 0 {
		mask >>= 1
		pos++
	}
	return pos
}

// FieldHelper handles TMC register field manipulation.
type FieldHelper struct {
	AllFields       map[string]map[string]uint32 // register -> field -> mask
	SignedFields    map[string]bool
	FieldFormatters map[string]func(int32) string
	Registers       map[string]uint32
	FieldToRegister map[string]string
}

// NewFieldHelper creates a new field helper.
func NewFieldHelper(allFields map[string]map[string]uint32, signedFields []string, formatters map[string]func(int32) string) *FieldHelper {
	fh := &FieldHelper{
		AllFields:       allFields,
		SignedFields:    make(map[string]bool),
		FieldFormatters: formatters,
		Registers:       make(map[string]uint32),
		FieldToRegister: make(map[string]string),
	}

	for _, sf := range signedFields {
		fh.SignedFields[sf] = true
	}

	if fh.FieldFormatters == nil {
		fh.FieldFormatters = make(map[string]func(int32) string)
	}

	// Build field to register mapping
	for regName, fields := range allFields {
		for fieldName := range fields {
			fh.FieldToRegister[fieldName] = regName
		}
	}

	return fh
}

// LookupRegister returns the register name for a field.
func (fh *FieldHelper) LookupRegister(fieldName string) (string, bool) {
	reg, ok := fh.FieldToRegister[fieldName]
	return reg, ok
}

// GetField returns the value of a register field.
func (fh *FieldHelper) GetField(fieldName string, regValue *uint32, regName string) int32 {
	if regName == "" {
		regName = fh.FieldToRegister[fieldName]
	}

	var val uint32
	if regValue != nil {
		val = *regValue
	} else {
		val = fh.Registers[regName]
	}

	mask := fh.AllFields[regName][fieldName]
	shift := ffs(mask)
	fieldValue := int32((val & mask) >> shift)

	// Handle signed fields
	if fh.SignedFields[fieldName] {
		if ((val & mask) << 1) > mask {
			bits := bitLength(uint32(fieldValue))
			fieldValue -= (1 << bits)
		}
	}

	return fieldValue
}

// SetField sets a field value and returns the new register value.
func (fh *FieldHelper) SetField(fieldName string, fieldValue int32, regValue *uint32, regName string) uint32 {
	if regName == "" {
		regName = fh.FieldToRegister[fieldName]
	}

	var val uint32
	if regValue != nil {
		val = *regValue
	} else {
		val = fh.Registers[regName]
	}

	mask := fh.AllFields[regName][fieldName]
	shift := ffs(mask)
	newValue := (val & ^mask) | ((uint32(fieldValue) << shift) & mask)
	fh.Registers[regName] = newValue
	return newValue
}

// PrettyFormat returns a string description of a register.
func (fh *FieldHelper) PrettyFormat(regName string, regValue uint32) string {
	regFields, ok := fh.AllFields[regName]
	if !ok {
		return fmt.Sprintf("%s: %08x", regName, regValue)
	}

	// Sort fields by mask
	type maskField struct {
		mask uint32
		name string
	}
	fields := make([]maskField, 0, len(regFields))
	for name, mask := range regFields {
		fields = append(fields, maskField{mask, name})
	}
	sort.Slice(fields, func(i, j int) bool {
		return fields[i].mask < fields[j].mask
	})

	var parts []string
	for _, f := range fields {
		v := fh.GetField(f.name, &regValue, regName)
		var sval string
		if formatter, ok := fh.FieldFormatters[f.name]; ok {
			sval = formatter(v)
		} else {
			sval = fmt.Sprintf("%d", v)
		}
		if sval != "" && sval != "0" {
			parts = append(parts, fmt.Sprintf("%s=%s", f.name, sval))
		}
	}

	return fmt.Sprintf("%-11s %08x %s", regName+":", regValue, strings.Join(parts, " "))
}

// GetRegFields returns all fields in a register.
func (fh *FieldHelper) GetRegFields(regName string, regValue uint32) map[string]int32 {
	result := make(map[string]int32)
	regFields, ok := fh.AllFields[regName]
	if !ok {
		return result
	}

	for fieldName := range regFields {
		result[fieldName] = fh.GetField(fieldName, &regValue, regName)
	}
	return result
}

// bitLength returns the number of bits needed to represent a value.
func bitLength(v uint32) int {
	bits := 0
	for v > 0 {
		bits++
		v >>= 1
	}
	return bits
}

// TMCDriver represents a TMC stepper driver.
type TMCDriver interface {
	GetName() string
	GetFields() *FieldHelper
	GetRegister(regName string) (uint32, error)
	SetRegister(regName string, value uint32) error
	GetMicrosteps() int
	SetMicrosteps(ms int) error
	GetCurrent() float64
	SetCurrent(current, holdMultiplier float64) error
}

// TMCConfig holds common TMC configuration.
type TMCConfig struct {
	Name             string
	RunCurrent       float64
	HoldCurrent      float64
	SenseResistor    float64
	StealthChop      bool
	Microsteps       int
	Interpolate      bool
	DiagPin          string
	StallThreshold   int
}

// DefaultTMCConfig returns default TMC configuration.
func DefaultTMCConfig() *TMCConfig {
	return &TMCConfig{
		RunCurrent:     0.8,
		HoldCurrent:    0.5,
		SenseResistor:  0.110,
		StealthChop:    true,
		Microsteps:     16,
		Interpolate:    true,
		StallThreshold: 10,
	}
}

// CurrentCalculator calculates TMC current settings.
type CurrentCalculator struct {
	SenseResistor float64
	VSense        bool
}

// NewCurrentCalculator creates a new current calculator.
func NewCurrentCalculator(senseResistor float64) *CurrentCalculator {
	return &CurrentCalculator{
		SenseResistor: senseResistor,
		VSense:        true,
	}
}

// CalcCurrentBits calculates the IRUN/IHOLD bits for a given current.
func (cc *CurrentCalculator) CalcCurrentBits(current float64) (int, bool) {
	// VSense=1: CS = current * 32 * sqrt(2) * Rs / 0.180 - 1
	// VSense=0: CS = current * 32 * sqrt(2) * Rs / 0.325 - 1
	const sqrt2 = 1.41421356237

	// Try VSense=1 first
	cs := int(current*32*sqrt2*cc.SenseResistor/0.180 - 1)
	if cs >= 0 && cs <= 31 {
		return cs, true
	}

	// Use VSense=0 for higher currents
	cs = int(current*32*sqrt2*cc.SenseResistor/0.325 - 1)
	if cs < 0 {
		cs = 0
	}
	if cs > 31 {
		cs = 31
	}
	return cs, false
}

// CalcCurrentFromBits calculates current from IRUN/IHOLD bits.
func (cc *CurrentCalculator) CalcCurrentFromBits(cs int, vsense bool) float64 {
	const sqrt2 = 1.41421356237

	var vref float64
	if vsense {
		vref = 0.180
	} else {
		vref = 0.325
	}

	return float64(cs+1) * vref / (32 * sqrt2 * cc.SenseResistor)
}

// TMCStatus holds TMC driver status.
type TMCStatus struct {
	Phase           int
	MStep           int
	DriverError     bool
	OverTemp        bool
	OverTempPreWarn bool
	ShortToGndA     bool
	ShortToGndB     bool
	OpenLoadA       bool
	OpenLoadB       bool
	StandStill      bool
	StallGuard      bool
}

// ParseDRVStatus parses DRV_STATUS register value.
func ParseDRVStatus(value uint32, fields *FieldHelper) *TMCStatus {
	status := &TMCStatus{}

	if phase, ok := fields.AllFields["DRV_STATUS"]["mstep"]; ok {
		status.MStep = int((value & phase) >> ffs(phase))
	}

	status.OverTemp = (value & fields.AllFields["DRV_STATUS"]["ot"]) != 0
	status.OverTempPreWarn = (value & fields.AllFields["DRV_STATUS"]["otpw"]) != 0
	status.ShortToGndA = (value & fields.AllFields["DRV_STATUS"]["s2ga"]) != 0
	status.ShortToGndB = (value & fields.AllFields["DRV_STATUS"]["s2gb"]) != 0
	status.OpenLoadA = (value & fields.AllFields["DRV_STATUS"]["ola"]) != 0
	status.OpenLoadB = (value & fields.AllFields["DRV_STATUS"]["olb"]) != 0
	status.StandStill = (value & fields.AllFields["DRV_STATUS"]["stst"]) != 0

	// Check for driver error
	status.DriverError = status.OverTemp || status.ShortToGndA || status.ShortToGndB

	return status
}

// MicrostepTable maps microstep setting to MRES value.
var MicrostepTable = map[int]int{
	256: 0,
	128: 1,
	64:  2,
	32:  3,
	16:  4,
	8:   5,
	4:   6,
	2:   7,
	1:   8,
}

// GetMRES returns the MRES value for a microstep setting.
func GetMRES(microsteps int) (int, error) {
	mres, ok := MicrostepTable[microsteps]
	if !ok {
		return 0, fmt.Errorf("invalid microsteps %d", microsteps)
	}
	return mres, nil
}

// GetMicrostepsFromMRES returns microsteps from MRES value.
func GetMicrostepsFromMRES(mres int) int {
	for ms, m := range MicrostepTable {
		if m == mres {
			return ms
		}
	}
	return 16 // default
}
