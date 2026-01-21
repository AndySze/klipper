// Structured logging tests
//
// Copyright (C) 2026  Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package log

import (
	"bytes"
	"encoding/json"
	"strings"
	"testing"
)

func TestLoggerBasic(t *testing.T) {
	var buf bytes.Buffer
	logger := New("test")
	logger.SetWriter(&buf)
	logger.SetLevel(DEBUG)
	logger.SetColorize(false)

	logger.Info("hello %s", "world")

	output := buf.String()
	if !strings.Contains(output, "[INFO ]") {
		t.Errorf("expected INFO level, got: %s", output)
	}
	if !strings.Contains(output, "test:") {
		t.Errorf("expected prefix 'test:', got: %s", output)
	}
	if !strings.Contains(output, "hello world") {
		t.Errorf("expected message 'hello world', got: %s", output)
	}
}

func TestLoggerLevels(t *testing.T) {
	var buf bytes.Buffer
	logger := New("test")
	logger.SetWriter(&buf)
	logger.SetColorize(false)

	// Default level is INFO, so DEBUG should be filtered
	logger.SetLevel(INFO)
	logger.Debug("debug message")
	if buf.Len() != 0 {
		t.Errorf("expected DEBUG to be filtered, got: %s", buf.String())
	}

	// INFO should pass
	logger.Info("info message")
	if !strings.Contains(buf.String(), "info message") {
		t.Errorf("expected INFO to pass, got: %s", buf.String())
	}

	buf.Reset()

	// WARN should pass
	logger.Warn("warn message")
	if !strings.Contains(buf.String(), "warn message") {
		t.Errorf("expected WARN to pass, got: %s", buf.String())
	}

	buf.Reset()

	// ERROR should pass
	logger.Error("error message")
	if !strings.Contains(buf.String(), "error message") {
		t.Errorf("expected ERROR to pass, got: %s", buf.String())
	}
}

func TestLoggerJSON(t *testing.T) {
	var buf bytes.Buffer
	logger := New("test")
	logger.SetWriter(&buf)
	logger.SetFormat(FormatJSON)
	logger.SetLevel(DEBUG)

	logger.Info("json test")

	var entry JSONLogEntry
	if err := json.Unmarshal(buf.Bytes(), &entry); err != nil {
		t.Fatalf("failed to parse JSON: %v, output: %s", err, buf.String())
	}

	if entry.Level != "INFO" {
		t.Errorf("expected level INFO, got: %s", entry.Level)
	}
	if entry.Logger != "test" {
		t.Errorf("expected logger 'test', got: %s", entry.Logger)
	}
	if entry.Message != "json test" {
		t.Errorf("expected message 'json test', got: %s", entry.Message)
	}
}

func TestLoggerWithFields(t *testing.T) {
	var buf bytes.Buffer
	logger := New("test")
	logger.SetWriter(&buf)
	logger.SetFormat(FormatText)
	logger.SetLevel(DEBUG)
	logger.SetColorize(false)

	logger.WithField("key", "value").Info("with field")

	output := buf.String()
	if !strings.Contains(output, "key=value") {
		t.Errorf("expected field 'key=value', got: %s", output)
	}
}

func TestLoggerWithFieldsJSON(t *testing.T) {
	var buf bytes.Buffer
	logger := New("test")
	logger.SetWriter(&buf)
	logger.SetFormat(FormatJSON)
	logger.SetLevel(DEBUG)

	logger.WithFields(Fields{
		"user":   "alice",
		"action": "login",
	}).Info("user logged in")

	var entry JSONLogEntry
	if err := json.Unmarshal(buf.Bytes(), &entry); err != nil {
		t.Fatalf("failed to parse JSON: %v", err)
	}

	if entry.Fields == nil {
		t.Fatal("expected fields to be set")
	}
	if entry.Fields["user"] != "alice" {
		t.Errorf("expected user=alice, got: %v", entry.Fields["user"])
	}
	if entry.Fields["action"] != "login" {
		t.Errorf("expected action=login, got: %v", entry.Fields["action"])
	}
}

func TestLoggerWithError(t *testing.T) {
	var buf bytes.Buffer
	logger := New("test")
	logger.SetWriter(&buf)
	logger.SetFormat(FormatJSON)
	logger.SetLevel(DEBUG)

	err := &testError{"something went wrong"}
	logger.WithError(err).Error("operation failed")

	var entry JSONLogEntry
	if err := json.Unmarshal(buf.Bytes(), &entry); err != nil {
		t.Fatalf("failed to parse JSON: %v", err)
	}

	if entry.Fields == nil || entry.Fields["error"] != "something went wrong" {
		t.Errorf("expected error field, got: %v", entry.Fields)
	}
}

type testError struct {
	msg string
}

func (e *testError) Error() string {
	return e.msg
}

func TestLoggerWithPrefix(t *testing.T) {
	var buf bytes.Buffer
	logger := New("parent")
	logger.SetWriter(&buf)
	logger.SetLevel(DEBUG)
	logger.SetColorize(false)

	child := logger.WithPrefix("child")
	child.Info("child message")

	output := buf.String()
	if !strings.Contains(output, "child:") {
		t.Errorf("expected prefix 'child:', got: %s", output)
	}
}

func TestLoggerCaller(t *testing.T) {
	var buf bytes.Buffer
	logger := New("test")
	logger.SetWriter(&buf)
	logger.SetLevel(DEBUG)
	logger.SetCaller(true)
	logger.SetColorize(false)

	logger.Info("caller test")

	output := buf.String()
	if !strings.Contains(output, "logger_test.go:") {
		t.Errorf("expected caller info 'logger_test.go:', got: %s", output)
	}
}

func TestLoggerCallerJSON(t *testing.T) {
	var buf bytes.Buffer
	logger := New("test")
	logger.SetWriter(&buf)
	logger.SetFormat(FormatJSON)
	logger.SetLevel(DEBUG)
	logger.SetCaller(true)

	logger.Info("caller test")

	var entry JSONLogEntry
	if err := json.Unmarshal(buf.Bytes(), &entry); err != nil {
		t.Fatalf("failed to parse JSON: %v", err)
	}

	if entry.Caller == "" {
		t.Error("expected caller to be set")
	}
	if !strings.Contains(entry.Caller, "logger_test.go:") {
		t.Errorf("expected caller to contain 'logger_test.go:', got: %s", entry.Caller)
	}
}

func TestParseLevel(t *testing.T) {
	tests := []struct {
		input    string
		expected LogLevel
	}{
		{"DEBUG", DEBUG},
		{"debug", DEBUG},
		{"INFO", INFO},
		{"info", INFO},
		{"WARN", WARN},
		{"warn", WARN},
		{"WARNING", WARN},
		{"ERROR", ERROR},
		{"error", ERROR},
		{"invalid", INFO}, // default
		{"", INFO},        // default
	}

	for _, tt := range tests {
		result := ParseLevel(tt.input)
		if result != tt.expected {
			t.Errorf("ParseLevel(%q) = %v, expected %v", tt.input, result, tt.expected)
		}
	}
}

func TestLogLevelString(t *testing.T) {
	tests := []struct {
		level    LogLevel
		expected string
	}{
		{DEBUG, "DEBUG"},
		{INFO, "INFO"},
		{WARN, "WARN"},
		{ERROR, "ERROR"},
		{LogLevel(99), "UNKNOWN"},
	}

	for _, tt := range tests {
		result := tt.level.String()
		if result != tt.expected {
			t.Errorf("LogLevel(%d).String() = %q, expected %q", tt.level, result, tt.expected)
		}
	}
}

func TestEntryChaining(t *testing.T) {
	var buf bytes.Buffer
	logger := New("test")
	logger.SetWriter(&buf)
	logger.SetFormat(FormatJSON)
	logger.SetLevel(DEBUG)

	logger.
		WithField("a", 1).
		WithField("b", 2).
		WithFields(Fields{"c": 3}).
		Info("chained")

	var entry JSONLogEntry
	if err := json.Unmarshal(buf.Bytes(), &entry); err != nil {
		t.Fatalf("failed to parse JSON: %v", err)
	}

	if len(entry.Fields) != 3 {
		t.Errorf("expected 3 fields, got %d: %v", len(entry.Fields), entry.Fields)
	}
}

func TestGetLogger(t *testing.T) {
	// GetLogger should return a logger with the given prefix
	logger := GetLogger("mycomponent")
	if logger == nil {
		t.Fatal("expected logger, got nil")
	}
	if logger.prefix != "mycomponent" {
		t.Errorf("expected prefix 'mycomponent', got %q", logger.prefix)
	}
}

func BenchmarkLoggerText(b *testing.B) {
	var buf bytes.Buffer
	logger := New("bench")
	logger.SetWriter(&buf)
	logger.SetLevel(INFO)
	logger.SetColorize(false)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		buf.Reset()
		logger.Info("benchmark message %d", i)
	}
}

func BenchmarkLoggerJSON(b *testing.B) {
	var buf bytes.Buffer
	logger := New("bench")
	logger.SetWriter(&buf)
	logger.SetLevel(INFO)
	logger.SetFormat(FormatJSON)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		buf.Reset()
		logger.Info("benchmark message %d", i)
	}
}

func BenchmarkLoggerWithFields(b *testing.B) {
	var buf bytes.Buffer
	logger := New("bench")
	logger.SetWriter(&buf)
	logger.SetLevel(INFO)
	logger.SetFormat(FormatJSON)

	fields := Fields{
		"user":      "alice",
		"action":    "login",
		"sessionID": "abc123",
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		buf.Reset()
		logger.WithFields(fields).Info("user action")
	}
}

func BenchmarkLoggerFiltered(b *testing.B) {
	var buf bytes.Buffer
	logger := New("bench")
	logger.SetWriter(&buf)
	logger.SetLevel(ERROR) // Filter out INFO

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		logger.Info("this should be filtered")
	}
}
