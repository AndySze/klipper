// Structured logging for Klipper Go migration
//
// Copyright (C) 2026  Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package log

import (
	"fmt"
	"io"
	"os"
	"sync"
	"time"
)

// LogLevel represents the severity of a log message
type LogLevel int

const (
	// DEBUG level for detailed debugging information
	DEBUG LogLevel = iota

	// INFO level for general informational messages
	INFO

	// WARN level for warning messages
	WARN

	// ERROR level for error messages
	ERROR
)

// String returns the string representation of the log level
func (l LogLevel) String() string {
	switch l {
	case DEBUG:
		return "DEBUG"
	case INFO:
		return "INFO"
	case WARN:
		return "WARN"
	case ERROR:
		return "ERROR"
	default:
		return "UNKNOWN"
	}
}

// Logger is the main logging interface
type Logger struct {
	mu         sync.Mutex
	prefix     string
	writer     io.Writer
	level      LogLevel
	timeFormat string
	colorize   bool
}

var (
	// Default logger instance
	defaultLogger *Logger

	// ANSI color codes for terminal output
	ansiColors = map[LogLevel]string{
		DEBUG: "\x1b[36m", // Cyan
		INFO:  "\x1b[32m", // Green
		WARN:  "\x1b[33m", // Yellow
		ERROR: "\x1b[31m", // Red
	}
	ansiReset = "\x1b[0m"
)

// New creates a new logger with the given prefix
func New(prefix string) *Logger {
	return &Logger{
		prefix:     prefix,
		writer:     os.Stderr,
		level:      INFO,
		timeFormat: "2006-01-02 15:04:05.000",
		colorize:   os.Getenv("NO_COLOR") == "",
	}
}

// SetLevel sets the minimum log level
func (l *Logger) SetLevel(level LogLevel) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.level = level
}

// SetWriter sets the output writer (e.g., for testing)
func (l *Logger) SetWriter(w io.Writer) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.writer = w
}

// SetTimeFormat sets the time format string
func (l *Logger) SetTimeFormat(format string) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.timeFormat = format
}

// SetColorize enables or disables colorized output
func (l *Logger) SetColorize(enable bool) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.colorize = enable
}

// format formats the log message
func (l *Logger) format(level LogLevel, msg string, args []interface{}) string {
	timestamp := time.Now().Format(l.timeFormat)
	formattedMsg := fmt.Sprintf(msg, args...)

	if l.colorize {
		color := ansiColors[level]
		return fmt.Sprintf("%s [%-5s] %s%s%s: %s\n",
			timestamp, level.String(), color, l.prefix, ansiReset, formattedMsg)
	}

	return fmt.Sprintf("%s [%-5s] %s: %s\n",
		timestamp, level.String(), l.prefix, formattedMsg)
}

// log writes a message at the given level
func (l *Logger) log(level LogLevel, msg string, args ...interface{}) {
	l.mu.Lock()
	defer l.mu.Unlock()

	if level < l.level {
		return
	}

	fmt.Fprint(l.writer, l.format(level, msg, args))
}

// Debug logs a message at DEBUG level
func (l *Logger) Debug(msg string, args ...interface{}) {
	l.log(DEBUG, msg, args...)
}

// Info logs a message at INFO level
func (l *Logger) Info(msg string, args ...interface{}) {
	l.log(INFO, msg, args...)
}

// Warn logs a message at WARN level
func (l *Logger) Warn(msg string, args ...interface{}) {
	l.log(WARN, msg, args...)
}

// Error logs a message at ERROR level
func (l *Logger) Error(msg string, args ...interface{}) {
	l.log(ERROR, msg, args...)
}

// Errorf logs a formatted error message
func (l *Logger) Errorf(msg string, args ...interface{}) {
	l.Error(msg, args...)
}

// WithPrefix returns a new logger with a modified prefix
func (l *Logger) WithPrefix(prefix string) *Logger {
	return &Logger{
		prefix:     prefix,
		writer:     l.writer,
		level:      l.level,
		timeFormat: l.timeFormat,
		colorize:   l.colorize,
	}
}

// Package-level functions using default logger

// SetDefaultLogger sets the global default logger
func SetDefaultLogger(logger *Logger) {
	defaultLogger = logger
}

// GetLogger returns the default logger or creates one if needed
func GetLogger(prefix string) *Logger {
	if defaultLogger == nil {
		defaultLogger = New("klipper")
	}
	return defaultLogger.WithPrefix(prefix)
}

// Debug logs at DEBUG level using default logger
func Debug(msg string, args ...interface{}) {
	GetLogger("").Debug(msg, args...)
}

// Info logs at INFO level using default logger
func Info(msg string, args ...interface{}) {
	GetLogger("").Info(msg, args...)
}

// Warn logs at WARN level using default logger
func Warn(msg string, args ...interface{}) {
	GetLogger("").Warn(msg, args...)
}

// Error logs at ERROR level using default logger
func Error(msg string, args ...interface{}) {
	GetLogger("").Error(msg, args...)
}

// Errorf logs a formatted error at ERROR level using default logger
func Errorf(msg string, args ...interface{}) {
	GetLogger("").Errorf(msg, args...)
}

// Initialize logging system from environment
func init() {
	// Set log level from environment variable
	if levelStr := os.Getenv("KLIPPER_LOG_LEVEL"); levelStr != "" {
		switch levelStr {
		case "DEBUG":
			defaultLogger = New("klipper")
			defaultLogger.SetLevel(DEBUG)
		case "INFO":
			defaultLogger = New("klipper")
			defaultLogger.SetLevel(INFO)
		case "WARN":
			defaultLogger = New("klipper")
			defaultLogger.SetLevel(WARN)
		case "ERROR":
			defaultLogger = New("klipper")
			defaultLogger.SetLevel(ERROR)
		}
	}

	if defaultLogger == nil {
		defaultLogger = New("klipper")
	}
}
