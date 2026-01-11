# Error Handling & Logging Packages

This document describes the new error handling and logging packages created for the Klipper Go migration.

## Packages

### 1. Error Handling (`go/pkg/errors/`)

Unified error handling system that replaces ad-hoc error handling across the codebase.

#### Key Features:
- **Error Categories**: Config, G-Code, Kinematics, Runtime, Module-specific
- **Context Rich**: Each error can include file, line, section, option, and custom context
- **Type Safety**: Compile-time error checking
- **Unwrapping**: Support for wrapping underlying errors
- **Error Checking**: Helper functions to check error categories (IsConfig, IsGCode, IsRuntime, etc.)

#### API Overview:

```go
// Creating errors
err := errors.ConfigSectionError("[stepper_x]")
err = errors.GCodeParseError("G28", "missing coordinates")
err = errors.KinematicsError("forward kinematics not available")

// Wrapping errors
err = errors.Wrap(baseErr, errors.ErrConfigSection, "failed to load config")

// Adding context
err.SetSection("printer")
err.SetOption("config_file")
err.SetLine(42)
err.SetContext("key", value)

// Checking error types
if errors.IsConfig(err) {
    // Handle config error
}
```

#### Error Categories:
- **Config errors**: Missing sections, invalid options, type conversion failures
- **G-Code errors**: Parse failures, unknown commands, missing/invalid parameters
- **Kinematics errors**: General kinematics failures, bounds violations, calculation errors
- **Runtime errors**: Initialization failures, queue errors, MCU communication errors
- **Module errors**: Bed mesh, bed screws, BLTouch, extruder, heater, macro, probe

#### Examples:
See `go/pkg/errors/examples/usage_examples.go` for comprehensive examples of:
- Basic error creation
- Config validation
- G-Code errors
- Kinematics errors
- Runtime errors
- Module-specific errors
- Error wrapping
- Error checking
- Panic recovery

### 2. Logging (`go/pkg/log/`)

Structured logging system with configurable levels and colored output.

#### Key Features:
- **Log Levels**: DEBUG, INFO, WARN, ERROR
- **Thread-Safe**: Mutex-protected operations
- **Flexible Writers**: Can output to any io.Writer (default: stderr)
- **Colorized Output**: ANSI colors for different log levels (can be disabled)
- **Timestamps**: Configurable time format
- **Environment Variables**: Control log level via KLIPPER_LOG_LEVEL
- **NO_COLOR Support**: Disable colors via environment variable

#### API Overview:

```go
// Create logger with prefix
logger := log.New("[config]")

// Set log level
logger.SetLevel(log.DEBUG)  // DEBUG, INFO, WARN, ERROR

// Log at different levels
logger.Debug("Detailed debugging info")
logger.Info("General information")
logger.Warn("Warning message")
logger.Error("Error message")
logger.Errorf("Formatted %s error", message)

// Create logger with custom prefix
motionLogger := logger.WithPrefix("[motion]")

// Package-level convenience functions
log.Info("This uses default logger")
log.GetLogger("custom").Info("This uses custom logger")
```

#### Environment Variables:
- `KLIPPER_LOG_LEVEL`: Set to DEBUG, INFO, WARN, or ERROR (default: INFO)
- `NO_COLOR`: Set to any value to disable colored output

#### Log Levels:
- **DEBUG**: Detailed debugging information (not shown by default)
- **INFO**: General informational messages (default)
- **WARN**: Warning messages that don't prevent operation
- **ERROR**: Error messages that affect operation

#### Color Scheme:
- DEBUG: Cyan
- INFO: Green
- WARN: Yellow
- ERROR: Red

## Usage in Host Code

### Replacing Ad-Hoc Errors:

**Before**:
```go
return nil, fmt.Errorf("option '%s' not found", key)
```

**After**:
```go
import "klipper-go-migration/pkg/errors"

return errors.ConfigOptionError(section, key)
```

### Adding Logging:

**Before**:
```go
fmt.Printf("Loading config: %s\n", path)
```

**After**:
```go
import "klipper-go-migration/pkg/log"

logger := log.GetLogger("config")
logger.Info("Loading config: %s", path)
logger.Debug("Parsing sections")
```

### Error Recovery:

```go
import "klipper-go-migration/pkg/errors"

defer func() {
    if err := errors.RecoverPanic(); err != nil {
        logger := log.GetLogger("recovery")
        logger.Error("Recovered from panic: %v", err)
    }
}()

// Your risky code here
```

## Testing

Run the examples to see the new error handling and logging in action:

```bash
cd go/pkg/errors/examples
go run usage_examples.go
```

This will demonstrate all error types, logging levels, and usage patterns.

## Migration Guide

### Step 1: Update Imports

Replace ad-hoc error handling:
```go
// Add import
import "klipper-go-migration/pkg/errors"
import "klipper-go-migration/pkg/log"
```

### Step 2: Replace Error Creation

Replace:
```go
// Old
return nil, fmt.Errorf("message")

// New
return errors.RuntimeError("message")
```

### Step 3: Add Logging

Add logging where appropriate:
```go
logger := log.GetLogger("module_name")
logger.Info("Starting operation")

// Use appropriate log levels
logger.Debug("Detailed info: %v", data)
logger.Info("General info")
logger.Warn("Warning condition")
logger.Error("Error occurred: %v", err)
```

### Step 4: Return Errors

Ensure functions return errors properly:
```go
func (c *config) GetFloat(...) (float64, error) {
    value, err := parse()
    if err != nil {
        return 0, errors.ConfigTypeError(...)
    }
    // Validate
    if value < minVal {
        return 0, errors.ConfigValidationError(...)
    }
    return value, nil
}
```

## Benefits

1. **Consistency**: All errors follow the same structure
2. **Debuggability**: Rich context makes debugging easier
3. **Type Safety**: Compile-time checking for error types
4. **Maintainability**: Centralized error definitions
5. **Observability**: Structured logging provides better insight
6. **Flexibility**: Easy to add new error types and log levels

## Future Enhancements

- [ ] Add stack trace capture for errors
- [ ] Add log file rotation
- [ ] Add structured logging (JSON format)
- [ ] Add metrics collection
- [ ] Add log aggregation support
