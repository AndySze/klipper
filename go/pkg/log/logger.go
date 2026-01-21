// Structured logging for Klipper Go migration
//
// Provides a flexible logging system with support for:
// - Log levels (DEBUG, INFO, WARN, ERROR)
// - Structured fields (key-value pairs)
// - Multiple output formats (text, JSON)
// - ANSI colors for terminal output
// - Log file rotation
// - Per-component loggers with prefixes
//
// Copyright (C) 2026  Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package log

import (
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"sort"
	"strings"
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

// ParseLevel parses a string into a LogLevel
func ParseLevel(s string) LogLevel {
	switch strings.ToUpper(s) {
	case "DEBUG":
		return DEBUG
	case "INFO":
		return INFO
	case "WARN", "WARNING":
		return WARN
	case "ERROR":
		return ERROR
	default:
		return INFO
	}
}

// OutputFormat specifies the output format for log messages
type OutputFormat int

const (
	// FormatText outputs human-readable text format
	FormatText OutputFormat = iota
	// FormatJSON outputs machine-readable JSON format
	FormatJSON
)

// Fields is a map of structured logging fields
type Fields map[string]interface{}

// Logger is the main logging interface
type Logger struct {
	mu         sync.Mutex
	prefix     string
	writer     io.Writer
	level      LogLevel
	timeFormat string
	colorize   bool
	outFormat  OutputFormat
	fields     Fields         // Persistent fields attached to this logger
	caller     bool           // Include caller info (file:line)
}

// Entry represents a single log entry with fields
type Entry struct {
	logger *Logger
	fields Fields
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
		outFormat:  FormatText,
		fields:     make(Fields),
	}
}

// SetLevel sets the minimum log level
func (l *Logger) SetLevel(level LogLevel) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.level = level
}

// GetLevel returns the current log level
func (l *Logger) GetLevel() LogLevel {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.level
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

// SetFormat sets the output format (FormatText or FormatJSON)
func (l *Logger) SetFormat(format OutputFormat) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.outFormat = format
}

// SetCaller enables or disables caller info in log output
func (l *Logger) SetCaller(enable bool) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.caller = enable
}

// WithField returns an Entry with the given field
func (l *Logger) WithField(key string, value interface{}) *Entry {
	return &Entry{
		logger: l,
		fields: Fields{key: value},
	}
}

// WithFields returns an Entry with the given fields
func (l *Logger) WithFields(fields Fields) *Entry {
	return &Entry{
		logger: l,
		fields: fields,
	}
}

// WithError returns an Entry with the error field set
func (l *Logger) WithError(err error) *Entry {
	return l.WithField("error", err.Error())
}

// getCaller returns the caller file and line number
func getCaller(skip int) string {
	_, file, line, ok := runtime.Caller(skip)
	if !ok {
		return "unknown:0"
	}
	return fmt.Sprintf("%s:%d", filepath.Base(file), line)
}

// formatText formats the log message as text
func (l *Logger) formatText(level LogLevel, msg string, fields Fields) string {
	return l.formatTextInternal(level, msg, fields, 4)
}

// formatTextInternal formats the log message as text with configurable caller skip
func (l *Logger) formatTextInternal(level LogLevel, msg string, fields Fields, callerSkip int) string {
	timestamp := time.Now().Format(l.timeFormat)

	var sb strings.Builder

	// Timestamp and level
	sb.WriteString(timestamp)
	sb.WriteString(" [")
	sb.WriteString(fmt.Sprintf("%-5s", level.String()))
	sb.WriteString("] ")

	// Prefix with optional color
	if l.colorize {
		sb.WriteString(ansiColors[level])
	}
	sb.WriteString(l.prefix)
	if l.colorize {
		sb.WriteString(ansiReset)
	}
	sb.WriteString(": ")

	// Message
	sb.WriteString(msg)

	// Caller info
	if l.caller {
		sb.WriteString(" (")
		sb.WriteString(getCaller(callerSkip))
		sb.WriteString(")")
	}

	// Fields
	if len(fields) > 0 {
		sb.WriteString(" {")
		keys := make([]string, 0, len(fields))
		for k := range fields {
			keys = append(keys, k)
		}
		sort.Strings(keys)
		for i, k := range keys {
			if i > 0 {
				sb.WriteString(", ")
			}
			sb.WriteString(k)
			sb.WriteString("=")
			sb.WriteString(fmt.Sprintf("%v", fields[k]))
		}
		sb.WriteString("}")
	}

	sb.WriteString("\n")
	return sb.String()
}

// JSONLogEntry is the structure for JSON formatted log entries
type JSONLogEntry struct {
	Timestamp string                 `json:"timestamp"`
	Level     string                 `json:"level"`
	Logger    string                 `json:"logger"`
	Message   string                 `json:"message"`
	Caller    string                 `json:"caller,omitempty"`
	Fields    map[string]interface{} `json:"fields,omitempty"`
}

// formatJSON formats the log message as JSON
func (l *Logger) formatJSON(level LogLevel, msg string, fields Fields) string {
	return l.formatJSONInternal(level, msg, fields, 4)
}

// formatJSONInternal formats the log message as JSON with configurable caller skip
func (l *Logger) formatJSONInternal(level LogLevel, msg string, fields Fields, callerSkip int) string {
	entry := JSONLogEntry{
		Timestamp: time.Now().Format(time.RFC3339Nano),
		Level:     level.String(),
		Logger:    l.prefix,
		Message:   msg,
	}

	if l.caller {
		entry.Caller = getCaller(callerSkip)
	}

	// Merge logger fields with entry fields
	if len(l.fields) > 0 || len(fields) > 0 {
		entry.Fields = make(map[string]interface{})
		for k, v := range l.fields {
			entry.Fields[k] = v
		}
		for k, v := range fields {
			entry.Fields[k] = v
		}
	}

	data, err := json.Marshal(entry)
	if err != nil {
		return fmt.Sprintf(`{"error":"failed to marshal log entry: %v"}`+"\n", err)
	}
	return string(data) + "\n"
}

// format formats the log message based on current format setting
func (l *Logger) format(level LogLevel, msg string, args []interface{}) string {
	formattedMsg := msg
	if len(args) > 0 {
		formattedMsg = fmt.Sprintf(msg, args...)
	}

	if l.outFormat == FormatJSON {
		return l.formatJSON(level, formattedMsg, nil)
	}
	return l.formatText(level, formattedMsg, nil)
}

// formatWithFields formats the log message with fields
func (l *Logger) formatWithFields(level LogLevel, msg string, fields Fields) string {
	if l.outFormat == FormatJSON {
		return l.formatJSON(level, msg, fields)
	}
	return l.formatText(level, msg, fields)
}

// log writes a message at the given level
func (l *Logger) log(level LogLevel, msg string, args ...interface{}) {
	l.logInternal(level, msg, args, nil, 4)
}

// logWithFields writes a message with fields at the given level
func (l *Logger) logWithFields(level LogLevel, msg string, fields Fields) {
	l.logInternal(level, msg, nil, fields, 4)
}

// logInternal is the core logging function
func (l *Logger) logInternal(level LogLevel, msg string, args []interface{}, fields Fields, callerSkip int) {
	l.mu.Lock()
	defer l.mu.Unlock()

	if level < l.level {
		return
	}

	formattedMsg := msg
	if len(args) > 0 {
		formattedMsg = fmt.Sprintf(msg, args...)
	}

	var output string
	if l.outFormat == FormatJSON {
		output = l.formatJSONInternal(level, formattedMsg, fields, callerSkip+1)
	} else {
		output = l.formatTextInternal(level, formattedMsg, fields, callerSkip+1)
	}
	fmt.Fprint(l.writer, output)
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
		outFormat:  l.outFormat,
		fields:     l.fields,
		caller:     l.caller,
	}
}

// Entry methods - log with fields

// WithField adds a field to the entry
func (e *Entry) WithField(key string, value interface{}) *Entry {
	newFields := make(Fields, len(e.fields)+1)
	for k, v := range e.fields {
		newFields[k] = v
	}
	newFields[key] = value
	return &Entry{
		logger: e.logger,
		fields: newFields,
	}
}

// WithFields adds multiple fields to the entry
func (e *Entry) WithFields(fields Fields) *Entry {
	newFields := make(Fields, len(e.fields)+len(fields))
	for k, v := range e.fields {
		newFields[k] = v
	}
	for k, v := range fields {
		newFields[k] = v
	}
	return &Entry{
		logger: e.logger,
		fields: newFields,
	}
}

// WithError adds an error field to the entry
func (e *Entry) WithError(err error) *Entry {
	return e.WithField("error", err.Error())
}

// Debug logs at DEBUG level with fields
func (e *Entry) Debug(msg string) {
	e.logger.logWithFields(DEBUG, msg, e.fields)
}

// Info logs at INFO level with fields
func (e *Entry) Info(msg string) {
	e.logger.logWithFields(INFO, msg, e.fields)
}

// Warn logs at WARN level with fields
func (e *Entry) Warn(msg string) {
	e.logger.logWithFields(WARN, msg, e.fields)
}

// Error logs at ERROR level with fields
func (e *Entry) Error(msg string) {
	e.logger.logWithFields(ERROR, msg, e.fields)
}

// Debugf logs formatted message at DEBUG level with fields
func (e *Entry) Debugf(format string, args ...interface{}) {
	e.logger.logWithFields(DEBUG, fmt.Sprintf(format, args...), e.fields)
}

// Infof logs formatted message at INFO level with fields
func (e *Entry) Infof(format string, args ...interface{}) {
	e.logger.logWithFields(INFO, fmt.Sprintf(format, args...), e.fields)
}

// Warnf logs formatted message at WARN level with fields
func (e *Entry) Warnf(format string, args ...interface{}) {
	e.logger.logWithFields(WARN, fmt.Sprintf(format, args...), e.fields)
}

// Errorf logs formatted message at ERROR level with fields
func (e *Entry) Errorf(format string, args ...interface{}) {
	e.logger.logWithFields(ERROR, fmt.Sprintf(format, args...), e.fields)
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
	defaultLogger = New("klipper")

	// Set log level from environment variable
	if levelStr := os.Getenv("KLIPPER_LOG_LEVEL"); levelStr != "" {
		defaultLogger.SetLevel(ParseLevel(levelStr))
	}

	// Set output format from environment variable
	if formatStr := os.Getenv("KLIPPER_LOG_FORMAT"); formatStr != "" {
		switch strings.ToLower(formatStr) {
		case "json":
			defaultLogger.SetFormat(FormatJSON)
		case "text":
			defaultLogger.SetFormat(FormatText)
		}
	}

	// Enable caller info from environment variable
	if os.Getenv("KLIPPER_LOG_CALLER") != "" {
		defaultLogger.SetCaller(true)
	}
}

// ConfigureFromEnv applies environment-based configuration to the logger.
// Environment variables:
//   - KLIPPER_LOG_LEVEL: DEBUG, INFO, WARN, ERROR
//   - KLIPPER_LOG_FORMAT: text, json
//   - KLIPPER_LOG_CALLER: any non-empty value enables caller info
//   - NO_COLOR: any non-empty value disables colors
func ConfigureFromEnv(l *Logger) {
	if levelStr := os.Getenv("KLIPPER_LOG_LEVEL"); levelStr != "" {
		l.SetLevel(ParseLevel(levelStr))
	}
	if formatStr := os.Getenv("KLIPPER_LOG_FORMAT"); formatStr != "" {
		switch strings.ToLower(formatStr) {
		case "json":
			l.SetFormat(FormatJSON)
		case "text":
			l.SetFormat(FormatText)
		}
	}
	if os.Getenv("KLIPPER_LOG_CALLER") != "" {
		l.SetCaller(true)
	}
	if os.Getenv("NO_COLOR") != "" {
		l.SetColorize(false)
	}
}
