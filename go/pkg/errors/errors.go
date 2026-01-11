// Unified error handling for Klipper Go migration
//
// Copyright (C) 2026  Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package errors

import (
	"fmt"
	"runtime"
)

// ErrorCode represents the category of error
type ErrorCode string

const (
	// Configuration errors
	ErrConfigSection    ErrorCode = "CONFIG_SECTION"
	ErrConfigOption     ErrorCode = "CONFIG_OPTION"
	ErrConfigValidation ErrorCode = "CONFIG_VALIDATION"
	ErrConfigType       ErrorCode = "CONFIG_TYPE"

	// G-code parsing errors
	ErrGCodeParse        ErrorCode = "GCODE_PARSE"
	ErrGCodeUnknownCmd   ErrorCode = "GCODE_UNKNOWN_CMD"
	ErrGCodeMissingParam ErrorCode = "GCODE_MISSING_PARAM"
	ErrGCodeInvalidParam ErrorCode = "GCODE_INVALID_PARAM"

	// Kinematics errors
	ErrKinematics       ErrorCode = "KINEMATICS"
	ErrKinematicsBounds ErrorCode = "KINEMATICS_BOUNDS"
	ErrKinematicsCalc   ErrorCode = "KINEMATICS_CALC"

	// Runtime errors
	ErrRuntime      ErrorCode = "RUNTIME"
	ErrRuntimeInit  ErrorCode = "RUNTIME_INIT"
	ErrRuntimeQueue ErrorCode = "RUNTIME_QUEUE"
	ErrRuntimeMCU   ErrorCode = "RUNTIME_MCU"

	// Module-specific errors
	ErrModuleBedMesh   ErrorCode = "MODULE_BED_MESH"
	ErrModuleBedScrews ErrorCode = "MODULE_BED_SCREWS"
	ErrModuleBLTouch   ErrorCode = "MODULE_BLTOUCH"
	ErrModuleExtruder  ErrorCode = "MODULE_EXTRUDER"
	ErrModuleHeater    ErrorCode = "MODULE_HEATER"
	ErrModuleMacro     ErrorCode = "MODULE_MACRO"
	ErrModuleProbe     ErrorCode = "MODULE_PROBE"
)

// HostError is the unified error type for the host system
type HostError struct {
	// Code is the error category
	Code ErrorCode

	// Message is a human-readable error description
	Message string

	// File is the source file (if available)
	File string

	// Line is the line number in the source file (if available)
	Line int

	// Section is the config section or context
	Section string

	// Option is the config option name (if applicable)
	Option string

	// Err wraps the underlying error
	Err error

	// Context provides additional context
	Context map[string]interface{}
}

// Error implements the error interface
func (e *HostError) Error() string {
	if e.Err != nil {
		return fmt.Sprintf("[%s:%s] %s", e.Code, e.Option, e.Message)
	}
	return fmt.Sprintf("[%s:%s] %s", e.Code, e.Section, e.Message)
}

// Unwrap returns the underlying error
func (e *HostError) Unwrap() error {
	return e.Err
}

// SetFile sets the source file
func (e *HostError) SetFile(file string) *HostError {
	e.File = file
	return e
}

// SetLine sets the line number
func (e *HostError) SetLine(line int) *HostError {
	e.Line = line
	return e
}

// SetSection sets the context section
func (e *HostError) SetSection(section string) *HostError {
	e.Section = section
	return e
}

// SetOption sets the config option
func (e *HostError) SetOption(option string) *HostError {
	e.Option = option
	return e
}

// SetContext adds additional context
func (e *HostError) SetContext(key string, value interface{}) *HostError {
	if e.Context == nil {
		e.Context = make(map[string]interface{})
	}
	e.Context[key] = value
	return e
}

// Wrap wraps an existing error with additional context
func Wrap(err error, code ErrorCode, message string) *HostError {
	return &HostError{
		Code:    code,
		Message: message,
		Err:     err,
	}
}

// New creates a new HostError
func New(code ErrorCode, message string) *HostError {
	return &HostError{
		Code:    code,
		Message: message,
	}
}

// Config errors

// ConfigSectionError creates an error for missing config section
func ConfigSectionError(section string) *HostError {
	return New(ErrConfigSection, fmt.Sprintf("section '%s' not found", section)).
		SetSection(section)
}

// ConfigOptionError creates an error for missing or invalid config option
func ConfigOptionError(section, option string) *HostError {
	return New(ErrConfigOption, fmt.Sprintf("option '%s' not found in section '%s'", option, section)).
		SetSection(section).
		SetOption(option)
}

// ConfigValidationError creates an error for config validation failure
func ConfigValidationError(section, option string, reason string) *HostError {
	return New(ErrConfigValidation, fmt.Sprintf("option '%s' in section '%s': %s", option, section, reason)).
		SetSection(section).
		SetOption(option)
}

// ConfigTypeError creates an error for config type conversion failure
func ConfigTypeError(section, option, value string, targetType string, err error) *HostError {
	return Wrap(err, ErrConfigType, fmt.Sprintf("option '%s' in section '%s': failed to parse '%s' as %s", option, section, value, targetType)).
		SetSection(section).
		SetOption(option)
}

// G-code errors

// GCodeParseError creates an error for G-code parsing failure
func GCodeParseError(line string, reason string) *HostError {
	return New(ErrGCodeParse, fmt.Sprintf("failed to parse G-code: %s (reason: %s)", line, reason))
}

// GCodeUnknownCommandError creates an error for unknown G-code command
func GCodeUnknownCommandError(command string) *HostError {
	return New(ErrGCodeUnknownCmd, fmt.Sprintf("unknown G-code command: %s", command))
}

// GCodeMissingParameterError creates an error for missing G-code parameter
func GCodeMissingParameterError(command, param string) *HostError {
	return New(ErrGCodeMissingParam, fmt.Sprintf("G-code command '%s' missing required parameter: %s", command, param))
}

// GCodeInvalidParameterError creates an error for invalid G-code parameter
func GCodeInvalidParameterError(command, param, value string, reason string) *HostError {
	return New(ErrGCodeInvalidParam, fmt.Sprintf("G-code command '%s': invalid parameter '%s=%s' (%s)", command, param, value, reason))
}

// Kinematics errors

// KinematicsError creates a general kinematics error
func KinematicsError(message string) *HostError {
	return New(ErrKinematics, message)
}

// KinematicsBoundsError creates an error for kinematics bounds violation
func KinematicsBoundsError(axis string, coord, min, max float64) *HostError {
	return New(ErrKinematicsBounds, fmt.Sprintf("%s coordinate %.3f out of bounds [%.3f, %.3f]", axis, coord, min, max))
}

// Runtime errors

// RuntimeError creates a general runtime error
func RuntimeError(message string) *HostError {
	return New(ErrRuntime, message)
}

// RuntimeErrorInit creates an error for initialization failure
func RuntimeErrorInit(component string, reason string) *HostError {
	return New(ErrRuntimeInit, fmt.Sprintf("failed to initialize %s: %s", component, reason))
}

// RuntimeErrorQueue creates an error for queue operation failure
func RuntimeErrorQueue(operation string, reason string) *HostError {
	return New(ErrRuntimeQueue, fmt.Sprintf("queue %s failed: %s", operation, reason))
}

// RuntimeErrorMCU creates an error for MCU communication failure
func RuntimeErrorMCU(operation string, reason string) *HostError {
	return New(ErrRuntimeMCU, fmt.Sprintf("MCU %s failed: %s", operation, reason))
}

// Module-specific errors

// BedMeshError creates a bed_mesh module error
func BedMeshError(message string) *HostError {
	return New(ErrModuleBedMesh, message)
}

// BedScrewsError creates a bed_screws module error
func BedScrewsError(message string) *HostError {
	return New(ErrModuleBedScrews, message)
}

// BLTouchError creates a bltouch module error
func BLTouchError(message string) *HostError {
	return New(ErrModuleBLTouch, message)
}

// ExtruderError creates an extruder module error
func ExtruderError(message string) *HostError {
	return New(ErrModuleExtruder, message)
}

// HeaterError creates a heater module error
func HeaterError(message string) *HostError {
	return New(ErrModuleHeater, message)
}

// MacroError creates a macro module error
func MacroError(message string) *HostError {
	return New(ErrModuleMacro, message)
}

// ProbeError creates a probe module error
func ProbeError(message string) *HostError {
	return New(ErrModuleProbe, message)
}

// Helper functions for adding context

// WithConfigPath adds config file path to error context
func WithConfigPath(err *HostError, path string) *HostError {
	if err == nil {
		return nil
	}
	err.SetContext("config_path", path)
	return err
}

// WithLineNumber adds line number to error context
func WithLineNumber(err *HostError, line int) *HostError {
	if err == nil {
		return nil
	}
	err.SetLine(line)
	return err
}

// RecoverPanic safely recovers from panic and converts to error
func RecoverPanic() *HostError {
	if r := recover(); r != nil {
		// Convert panic to HostError
		var err error
		switch x := r.(type) {
		case string:
			err = RuntimeError(fmt.Sprintf("panic: %s", x))
		case error:
			err = RuntimeError(x.Error())
		case runtime.Error:
			err = RuntimeError(x.Error())
		default:
			err = RuntimeError(fmt.Sprintf("panic: %v", x))
		}
		return err.(*HostError)
	}
	return nil
}

// Is checks if error matches given error code
func Is(err error, code ErrorCode) bool {
	if hostErr, ok := err.(*HostError); ok {
		return hostErr.Code == code
	}
	return false
}

// IsConfig checks if error is a config error
func IsConfig(err error) bool {
	return Is(err, ErrConfigSection) ||
		Is(err, ErrConfigOption) ||
		Is(err, ErrConfigValidation) ||
		Is(err, ErrConfigType)
}

// IsGCode checks if error is a G-code error
func IsGCode(err error) bool {
	return Is(err, ErrGCodeParse) ||
		Is(err, ErrGCodeUnknownCmd) ||
		Is(err, ErrGCodeMissingParam) ||
		Is(err, ErrGCodeInvalidParam)
}

// IsRuntime checks if error is a runtime error
func IsRuntime(err error) bool {
	return Is(err, ErrRuntime) ||
		Is(err, ErrRuntimeInit) ||
		Is(err, ErrRuntimeQueue) ||
		Is(err, ErrRuntimeMCU)
}
