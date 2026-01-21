// Log rotation tests
//
// Copyright (C) 2026  Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package log

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestRotatingFileWriter(t *testing.T) {
	// Create temp directory
	tmpDir, err := os.MkdirTemp("", "log_rotation_test")
	if err != nil {
		t.Fatalf("failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tmpDir)

	logFile := filepath.Join(tmpDir, "test.log")

	writer, err := NewRotatingFileWriter(RotationConfig{
		Filename:   logFile,
		MaxSize:    1, // 1 MB
		MaxBackups: 3,
		Compress:   false,
	})
	if err != nil {
		t.Fatalf("failed to create rotating writer: %v", err)
	}
	defer writer.Close()

	// Write some data
	msg := "test log message\n"
	n, err := writer.Write([]byte(msg))
	if err != nil {
		t.Fatalf("write failed: %v", err)
	}
	if n != len(msg) {
		t.Errorf("expected %d bytes written, got %d", len(msg), n)
	}

	// Verify file exists
	if _, err := os.Stat(logFile); err != nil {
		t.Errorf("log file not created: %v", err)
	}

	// Verify size tracking
	if writer.CurrentSize() != int64(len(msg)) {
		t.Errorf("expected size %d, got %d", len(msg), writer.CurrentSize())
	}
}

func TestRotatingFileWriterRotation(t *testing.T) {
	tmpDir, err := os.MkdirTemp("", "log_rotation_test")
	if err != nil {
		t.Fatalf("failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tmpDir)

	logFile := filepath.Join(tmpDir, "test.log")

	// Use very small max size to trigger rotation
	writer, err := NewRotatingFileWriter(RotationConfig{
		Filename:   logFile,
		MaxSize:    1,    // 1 MB (we'll force rotation manually)
		MaxBackups: 3,
		Compress:   false,
	})
	if err != nil {
		t.Fatalf("failed to create rotating writer: %v", err)
	}
	defer writer.Close()

	// Force rotation by setting current size high
	writer.mu.Lock()
	writer.currentSize = writer.maxSize + 1
	writer.mu.Unlock()

	// Write should trigger rotation
	_, err = writer.Write([]byte("after rotation\n"))
	if err != nil {
		t.Fatalf("write after rotation failed: %v", err)
	}

	// Check that we have rotated files
	entries, _ := os.ReadDir(tmpDir)
	found := false
	for _, e := range entries {
		if strings.HasPrefix(e.Name(), "test.") && e.Name() != "test.log" {
			found = true
			break
		}
	}
	if !found {
		t.Error("expected rotated file to exist")
	}
}

func TestNewFileLogger(t *testing.T) {
	tmpDir, err := os.MkdirTemp("", "log_rotation_test")
	if err != nil {
		t.Fatalf("failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tmpDir)

	logFile := filepath.Join(tmpDir, "app.log")

	logger, writer, err := NewFileLogger("test", RotationConfig{
		Filename:   logFile,
		MaxSize:    10,
		MaxBackups: 5,
	})
	if err != nil {
		t.Fatalf("failed to create file logger: %v", err)
	}
	defer writer.Close()

	logger.SetLevel(DEBUG)
	logger.Info("test message")

	// Verify content was written
	content, err := os.ReadFile(logFile)
	if err != nil {
		t.Fatalf("failed to read log file: %v", err)
	}

	if !strings.Contains(string(content), "test message") {
		t.Errorf("log file missing expected content: %s", content)
	}
}

func TestIsRotatedFile(t *testing.T) {
	tests := []struct {
		name     string
		prefix   string
		ext      string
		expected bool
	}{
		{"test.20260121-153000.log", "test", ".log", true},
		{"test.20260121-153000.log.gz", "test", ".log", true},
		{"test.log", "test", ".log", false},
		{"test.backup.log", "test", ".log", false},
		{"test.12345678-123456.log", "test", ".log", true},
		{"other.20260121-153000.log", "test", ".log", false},
	}

	for _, tt := range tests {
		result := isRotatedFile(tt.name, tt.prefix, tt.ext)
		if result != tt.expected {
			t.Errorf("isRotatedFile(%q, %q, %q) = %v, expected %v",
				tt.name, tt.prefix, tt.ext, result, tt.expected)
		}
	}
}

func TestMultiWriter(t *testing.T) {
	var buf1, buf2 strings.Builder

	mw := NewMultiWriter(&buf1, &buf2)

	msg := "hello world"
	n, err := mw.Write([]byte(msg))
	if err != nil {
		t.Fatalf("write failed: %v", err)
	}
	if n != len(msg) {
		t.Errorf("expected %d bytes, got %d", len(msg), n)
	}

	if buf1.String() != msg {
		t.Errorf("buf1 expected %q, got %q", msg, buf1.String())
	}
	if buf2.String() != msg {
		t.Errorf("buf2 expected %q, got %q", msg, buf2.String())
	}
}

func TestRotationConfigDefaults(t *testing.T) {
	tmpDir, err := os.MkdirTemp("", "log_rotation_test")
	if err != nil {
		t.Fatalf("failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tmpDir)

	logFile := filepath.Join(tmpDir, "test.log")

	// Empty config should use defaults
	writer, err := NewRotatingFileWriter(RotationConfig{
		Filename: logFile,
	})
	if err != nil {
		t.Fatalf("failed to create writer: %v", err)
	}
	defer writer.Close()

	// Check defaults were applied
	if writer.maxSize != 10*1024*1024 {
		t.Errorf("expected maxSize 10MB, got %d", writer.maxSize)
	}
	if writer.maxBackups != 5 {
		t.Errorf("expected maxBackups 5, got %d", writer.maxBackups)
	}
}

func TestRotationConfigEmptyFilename(t *testing.T) {
	_, err := NewRotatingFileWriter(RotationConfig{})
	if err == nil {
		t.Error("expected error for empty filename")
	}
}
