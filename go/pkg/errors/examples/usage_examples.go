// Examples of using the new error handling and logging systems
//
// Copyright (C) 2026  Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package examples

import (
	"fmt"
	"os"
	"strings"

	"klipper-go-migration/pkg/errors"
	"klipper-go-migration/pkg/log"
)

func Example1_BasicErrorHandling() {
	// Example 1: Basic error creation
	fmt.Println("=== Example 1: Basic Error Handling ===")

	// Create different types of errors
	err := errors.ConfigSectionError("[stepper_x]")
	fmt.Printf("Error: %v\n", err)

	err = errors.ConfigOptionError("[stepper_x]", "step_pin")
	fmt.Printf("Error: %v\n", err)

	err = errors.GCodeParseError("G28", "missing coordinates")
	fmt.Printf("Error: %v\n", err)

	fmt.Println()
}

func Example2_ConfigValidation() {
	fmt.Println("=== Example 2: Config Validation ===")

	// Simulate config parsing with validation
	section := "[heater_bed]"
	option := "min_temp"

	// Missing required option
	err := errors.ConfigOptionError(section, option)
	fmt.Printf("Missing option error: %v\n", err)

	// Invalid range
	err = errors.ConfigValidationError(section, option, "must be greater than 0")
	fmt.Printf("Validation error: %v\n", err)

	// Type conversion error
	value := "invalid"
	err = errors.ConfigTypeError(section, option, value, "float", fmt.Errorf("not a number"))
	fmt.Printf("Type error: %v\n", err)

	// Add additional context
	fmt.Printf("With line number: %v\n", err.SetLine(42))
	fmt.Printf("With option context: %v\n", err.SetOption("min_temp"))

	fmt.Println()
}

func Example3_GCodeErrors() {
	fmt.Println("=== Example 3: G-Code Errors ===")

	// Unknown command
	err := errors.GCodeUnknownCommandError("G999")
	fmt.Printf("Unknown command: %v\n", err)

	// Missing parameter
	err = errors.GCodeMissingParameterError("G28", "X")
	fmt.Printf("Missing param: %v\n", err)

	// Invalid parameter
	err = errors.GCodeInvalidParameterError("G28", "X", "invalid", "must be a number")
	fmt.Printf("Invalid param: %v\n", err)

	fmt.Println()
}

func Example4_KinematicsErrors() {
	fmt.Println("=== Example 4: Kinematics Errors ===")

	// General kinematics error
	err := errors.KinematicsError("forward kinematics not available")
	fmt.Printf("Kinematics error: %v\n", err)

	// Bounds violation
	err = errors.KinematicsBoundsError("X", 250.5, 0.0, 200.0)
	fmt.Printf("Bounds error: %v\n", err)

	fmt.Println()
}

func Example5_RuntimeErrors() {
	fmt.Println("=== Example 5: Runtime Errors ===")

	// Initialization failure
	err := errors.RuntimeErrorInit("MCU", "communication timeout")
	fmt.Printf("Init error: %v\n", err)

	// Queue error
	err = errors.RuntimeErrorQueue("step", "buffer full")
	fmt.Printf("Queue error: %v\n", err)

	// MCU error
	err = errors.RuntimeErrorMCU("send", "checksum mismatch")
	fmt.Printf("MCU error: %v\n", err)

	fmt.Println()
}

func Example6_ModuleSpecificErrors() {
	fmt.Println("=== Example 6: Module-Specific Errors ===")

	// Different module errors
	errs := []*errors.HostError{
		errors.BedMeshError("invalid mesh size"),
		errors.BedScrewsError("screw position out of range"),
		errors.BLTouchError("probe not triggered"),
		errors.ExtruderError("pressure advance too high"),
		errors.HeaterError("temperature too low"),
		errors.MacroError("undefined variable"),
		errors.ProbeError("probe not homed"),
	}

	for _, e := range errs {
		fmt.Printf("Module error: %v\n", e)
	}

	fmt.Println()
}

func Example7_WrappingErrors() {
	fmt.Println("=== Example 7: Wrapping Errors ===")

	// Wrap an existing error
	baseErr := fmt.Errorf("file not found")
	wrappedErr := errors.Wrap(baseErr, errors.ErrConfigSection, "failed to load config")
	fmt.Printf("Wrapped error: %v\n", wrappedErr)

	// Add context - these return new errors, not modify in place
	fmt.Printf("With section: %v\n", wrappedErr.SetSection("printer"))
	fmt.Printf("With option: %v\n", wrappedErr.SetOption("config_file"))
	fmt.Printf("With line: %v\n", wrappedErr.SetLine(10))

	fmt.Println()
}

func Example8_ErrorChecking() {
	fmt.Println("=== Example 8: Error Checking ===")

	// Create different errors
	configErr := errors.ConfigOptionError("[printer]", "kinematics")
	gcodeErr := errors.GCodeParseError("G1", "missing coordinates")
	runtimeErr := errors.RuntimeError("unexpected shutdown")

	// Check error types
	fmt.Println("Checking error types:")
	fmt.Printf("Is config error? %v\n", errors.IsConfig(configErr))
	fmt.Printf("Is G-code error? %v\n", errors.IsGCode(gcodeErr))
	fmt.Printf("Is runtime error? %v\n", errors.IsRuntime(runtimeErr))
	fmt.Printf("Is config error? %v\n", errors.IsConfig(gcodeErr))

	fmt.Println()
}

func Example9_LoggingUsage() {
	fmt.Println("=== Example 9: Logging Usage ===")

	// Create logger with different prefixes
	configLogger := log.New("[config]")
	mcuLogger := log.New("[mcu]")
	motionLogger := log.New("[motion]")

	// Log at different levels
	configLogger.Info("Loading configuration file")
	configLogger.Debug("Parsed section [stepper_x]")
	configLogger.Warn("Option 'step_pin' not set, using default")
	configLogger.Error("Failed to parse option")

	fmt.Println()

	mcuLogger.Info("Connecting to MCU")
	mcuLogger.Debug("Sending command: alloc oid=0")
	mcuLogger.Warn("Retry count: 3")

	fmt.Println()

	motionLogger.Info("Planning move to (100, 50, 10)")
	motionLogger.Debug("Lookahead buffer size: 3")
	motionLogger.Error("Junction deviation exceeded limit")

	fmt.Println()
}

func Example10_CustomLogLevels() {
	fmt.Println("=== Example 10: Custom Log Levels ===")

	// Set custom log level
	logger := log.New("[test]")

	// Only messages at or above INFO level will be shown
	logger.SetLevel(log.INFO)
	logger.Debug("This DEBUG message won't show")
	logger.Info("This INFO message will show")
	logger.Warn("This WARN message will show")
	logger.Error("This ERROR message will show")

	fmt.Println()

	// Set to DEBUG to see all messages
	logger.SetLevel(log.DEBUG)
	logger.Debug("Now DEBUG messages will show")
	logger.Info("INFO message")
	logger.Warn("WARN message")
	logger.Error("ERROR message")

	fmt.Println()
}

func Example11_LoggingInFunctions() {
	fmt.Println("=== Example 11: Logging in Functions ===")

	processConfig := func() {
		logger := log.GetLogger("config")
		logger.Info("Processing configuration")
		logger.Debug("Checking validation rules")

		if err := validateConfig(); err != nil {
			logger.Error("Configuration validation failed: %v", err)
			return
		}

		logger.Info("Configuration loaded successfully")
	}

	processGCode := func(command string) error {
		logger := log.GetLogger("gcode")
		logger.Debug("Parsing command: %s", command)

		if err := parseCommand(command); err != nil {
			logger.Error("Failed to parse command: %v", err)
			return err
		}

		logger.Info("Command parsed successfully")
		return nil
	}

	processConfig()
	processGCode("G28")
	processGCode("G1 X100 Y100 Z10")

	fmt.Println()
}

func Example12_ErrorRecovery() {
	fmt.Println("=== Example 12: Error Recovery ===")

	// Simulate function that might panic
	riskyOperation := func() {
		panic("something went wrong")
	}

	// Recover from panic using defer
	defer func() {
		if err := errors.RecoverPanic(); err != nil {
			logger := log.GetLogger("recovery")
			logger.Error("Recovered from panic: %v", err)
		}
	}()

	logger := log.GetLogger("app")
	logger.Info("Starting risky operation")

	riskyOperation()

	logger.Info("Operation completed")
}

func Example13_WithContext() {
	fmt.Println("=== Example 13: Error with Context ===")

	err := errors.ConfigValidationError("[stepper_x]", "max_speed", "too high")
	fmt.Printf("With max_speed context: %v\n", err.SetContext("max_speed", 1000.0))
	fmt.Printf("With measured_speed context: %v\n", err.SetContext("measured_speed", 1200.0))
	fmt.Printf("With line number: %v\n", err.SetLine(42))

	fmt.Println()
}

func Example14_EnvironmentVariables() {
	fmt.Println("=== Example 14: Environment Variables ===")

	// Set log level via environment variable
	os.Setenv("KLIPPER_LOG_LEVEL", "DEBUG")

	// Get logger (will pick up the env var)
	logger := log.GetLogger("env")

	logger.Debug("This will show because DEBUG is set")
	logger.Info("This will also show")
	logger.Warn("This will also show")
	logger.Error("This will also show")

	fmt.Println()

	// Disable color output
	os.Setenv("NO_COLOR", "1")
	colorDisabledLogger := log.New("no-color")
	colorDisabledLogger.Info("Color should be disabled")

	fmt.Println()
}

// Helper functions for examples
func validateConfig() error {
	return errors.ConfigValidationError("[stepper_x]", "step_pin", "invalid format")
}

func parseCommand(command string) error {
	return errors.GCodeParseError(command, "not implemented")
}

func Example15_CompleteWorkflow() {
	fmt.Println("=== Example 15: Complete Workflow ===")

	logger := log.GetLogger("workflow")
	logger.Info("Starting complete workflow example")

	// Step 1: Load config
	logger.Info("Step 1: Loading configuration")
	config, err := loadConfig()
	if err != nil {
		logger.Error("Failed to load config: %v", err)
		return
	}

	// Step 2: Validate config
	logger.Info("Step 2: Validating configuration")
	if err := validateConfigFile(config); err != nil {
		logger.Error("Config validation failed: %v", err)
		return
	}

	// Step 3: Initialize components
	logger.Info("Step 3: Initializing components")
	if err := initializeComponents(config); err != nil {
		logger.Error("Initialization failed: %v", err)
		return
	}

	// Step 4: Start processing
	logger.Info("Step 4: Starting processing")
	if err := startProcessing(); err != nil {
		logger.Error("Processing failed: %v", err)
		return
	}

	logger.Info("Workflow completed successfully")
}

func loadConfig() (map[string]string, error) {
	logger := log.GetLogger("config")
	logger.Debug("Reading config file")

	config := map[string]string{
		"stepper_x": "PA1",
		"stepper_y": "PA2",
		"stepper_z": "PA3",
	}

	logger.Info("Config loaded")
	return config, nil
}

func validateConfigFile(config map[string]string) error {
	logger := log.GetLogger("config")
	logger.Debug("Validating config values")

	if _, ok := config["stepper_x"]; !ok {
		return errors.ConfigOptionError("[stepper_x]", "pin")
	}

	logger.Debug("Config validation passed")
	return nil
}

func initializeComponents(config map[string]string) error {
	logger := log.GetLogger("init")
	logger.Info("Initializing MCU connection")

	// Simulate initialization
	logger.Debug("Allocating OIDs")
	logger.Info("MCU initialized")
	return nil
}

func startProcessing() error {
	logger := log.GetLogger("process")
	logger.Info("Starting main loop")

	// Simulate processing
	logger.Debug("Waiting for G-code")
	logger.Info("Processing G-code commands")
	return nil
}

func main() {
	fmt.Println("Klipper Go Migration - Error Handling & Logging Examples")
	fmt.Println("=" + strings.Repeat("=", 60))
	fmt.Println()

	Example1_BasicErrorHandling()
	Example2_ConfigValidation()
	Example3_GCodeErrors()
	Example4_KinematicsErrors()
	Example5_RuntimeErrors()
	Example6_ModuleSpecificErrors()
	Example7_WrappingErrors()
	Example8_ErrorChecking()
	Example9_LoggingUsage()
	Example10_CustomLogLevels()
	Example11_LoggingInFunctions()
	Example12_ErrorRecovery()
	Example13_WithContext()
	Example14_EnvironmentVariables()
	Example15_CompleteWorkflow()

	fmt.Println("=" + strings.Repeat("=", 60))
	fmt.Println("All examples completed!")
}
