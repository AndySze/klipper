// Package config provides configuration file parsing with access tracking
// and validation, matching Python's ConfigWrapper behavior.
package config

import "fmt"

// ConfigError represents a configuration error with context.
type ConfigError struct {
	Section string
	Option  string
	Message string
	Cause   error
}

func (e *ConfigError) Error() string {
	if e.Option != "" {
		return fmt.Sprintf("Option '%s' in section '%s': %s", e.Option, e.Section, e.Message)
	}
	if e.Section != "" {
		return fmt.Sprintf("Section '%s': %s", e.Section, e.Message)
	}
	return e.Message
}

func (e *ConfigError) Unwrap() error {
	return e.Cause
}

// NewConfigError creates a new ConfigError.
func NewConfigError(section, option, message string) *ConfigError {
	return &ConfigError{
		Section: section,
		Option:  option,
		Message: message,
	}
}

// WrapError wraps an existing error with config context.
func WrapError(section, option string, err error) *ConfigError {
	return &ConfigError{
		Section: section,
		Option:  option,
		Message: err.Error(),
		Cause:   err,
	}
}

// ErrMissingOption returns an error for a required but missing option.
func ErrMissingOption(section, option string) *ConfigError {
	return &ConfigError{
		Section: section,
		Option:  option,
		Message: "must be specified",
	}
}

// ErrMissingSection returns an error for a missing section.
func ErrMissingSection(section string) *ConfigError {
	return &ConfigError{
		Section: section,
		Message: "section not found",
	}
}

// ErrInvalidValue returns an error for an invalid value.
func ErrInvalidValue(section, option, value, expected string) *ConfigError {
	return &ConfigError{
		Section: section,
		Option:  option,
		Message: fmt.Sprintf("invalid value '%s', expected %s", value, expected),
	}
}

// ErrOutOfRange returns an error for a value outside the allowed range.
func ErrOutOfRange(section, option string, value float64, constraint string) *ConfigError {
	return &ConfigError{
		Section: section,
		Option:  option,
		Message: fmt.Sprintf("value %v %s", value, constraint),
	}
}

// ErrInvalidChoice returns an error for an invalid choice value.
func ErrInvalidChoice(section, option, value string, choices []string) *ConfigError {
	return &ConfigError{
		Section: section,
		Option:  option,
		Message: fmt.Sprintf("'%s' is not a valid choice (valid: %v)", value, choices),
	}
}
