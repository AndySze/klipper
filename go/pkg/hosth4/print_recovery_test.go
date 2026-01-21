// Print Recovery tests
//
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"os"
	"path/filepath"
	"testing"
	"time"
)

func TestPrintRecoveryCreate(t *testing.T) {
	pr := newPrintRecovery(nil, "")
	if pr == nil {
		t.Fatal("newPrintRecovery returned nil")
	}
	if !pr.IsEnabled() {
		t.Error("print recovery should be enabled by default")
	}
}

func TestPrintRecoverySetEnabled(t *testing.T) {
	pr := newPrintRecovery(nil, "")

	pr.SetEnabled(false)
	if pr.IsEnabled() {
		t.Error("expected disabled")
	}

	pr.SetEnabled(true)
	if !pr.IsEnabled() {
		t.Error("expected enabled")
	}
}

func TestPrintRecoveryStateCapture(t *testing.T) {
	state := &PrintRecoveryState{
		Version:         printRecoveryVersion,
		SavedAt:         time.Now(),
		Filename:        "test.gcode",
		FilePosition:    12345,
		FileSize:        100000,
		PositionX:       100.5,
		PositionY:       50.25,
		PositionZ:       10.0,
		PositionE:       150.0,
		AbsoluteCoord:   true,
		AbsoluteExtrude: true,
		Speed:           50.0,
		SpeedFactor:     1.0 / 60.0,
		ExtrudeFactor:   1.0,
		ExtruderTarget:  210.0,
		BedTarget:       60.0,
		FanSpeed:        0.8,
		ActiveExtruder:  "extruder",
		PrintDuration:   3600.0,
		FilamentUsed:    500.0,
		CurrentLayer:    50,
		TotalLayers:     100,
	}

	if state.Version != printRecoveryVersion {
		t.Errorf("unexpected version: %d", state.Version)
	}
	if state.Filename != "test.gcode" {
		t.Errorf("unexpected filename: %s", state.Filename)
	}
	if state.FilePosition != 12345 {
		t.Errorf("unexpected file position: %d", state.FilePosition)
	}
}

func TestPrintRecoverySaveLoad(t *testing.T) {
	// Create temp directory
	tmpDir, err := os.MkdirTemp("", "print_recovery_test")
	if err != nil {
		t.Fatalf("failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tmpDir)

	pr := newPrintRecovery(nil, tmpDir)

	// Create a test state directly
	testState := &PrintRecoveryState{
		Version:         printRecoveryVersion,
		SavedAt:         time.Now(),
		Filename:        "test.gcode",
		FilePosition:    12345,
		FileSize:        100000,
		PositionX:       100.5,
		PositionY:       50.25,
		PositionZ:       10.0,
		PositionE:       150.0,
		AbsoluteCoord:   true,
		AbsoluteExtrude: true,
		Speed:           50.0,
		SpeedFactor:     1.0 / 60.0,
		ExtrudeFactor:   1.0,
		ExtruderTarget:  210.0,
		BedTarget:       60.0,
		FanSpeed:        0.8,
		ActiveExtruder:  "extruder",
	}

	// Save state to file
	pr.mu.Lock()
	err = pr.saveStateToFile(testState)
	pr.mu.Unlock()
	if err != nil {
		t.Fatalf("failed to save state: %v", err)
	}

	// Verify file exists
	if !pr.HasRecoveryState() {
		t.Error("expected recovery state to exist")
	}

	// Load state
	loaded, err := pr.LoadState()
	if err != nil {
		t.Fatalf("failed to load state: %v", err)
	}
	if loaded == nil {
		t.Fatal("loaded state is nil")
	}

	// Verify values
	if loaded.Filename != testState.Filename {
		t.Errorf("filename mismatch: got %s, want %s", loaded.Filename, testState.Filename)
	}
	if loaded.FilePosition != testState.FilePosition {
		t.Errorf("file position mismatch: got %d, want %d", loaded.FilePosition, testState.FilePosition)
	}
	if loaded.PositionX != testState.PositionX {
		t.Errorf("position X mismatch: got %f, want %f", loaded.PositionX, testState.PositionX)
	}
	if loaded.ExtruderTarget != testState.ExtruderTarget {
		t.Errorf("extruder target mismatch: got %f, want %f", loaded.ExtruderTarget, testState.ExtruderTarget)
	}
}

func TestPrintRecoveryClear(t *testing.T) {
	// Create temp directory
	tmpDir, err := os.MkdirTemp("", "print_recovery_test")
	if err != nil {
		t.Fatalf("failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tmpDir)

	pr := newPrintRecovery(nil, tmpDir)

	// Create a test state
	testState := &PrintRecoveryState{
		Version:      printRecoveryVersion,
		SavedAt:      time.Now(),
		Filename:     "test.gcode",
		FilePosition: 12345,
	}

	// Save state
	pr.mu.Lock()
	err = pr.saveStateToFile(testState)
	pr.mu.Unlock()
	if err != nil {
		t.Fatalf("failed to save state: %v", err)
	}

	if !pr.HasRecoveryState() {
		t.Error("expected recovery state to exist before clear")
	}

	// Clear state
	if err := pr.ClearState(); err != nil {
		t.Fatalf("failed to clear state: %v", err)
	}

	if pr.HasRecoveryState() {
		t.Error("expected recovery state to not exist after clear")
	}
}

func TestPrintRecoveryGenerateResumeGCode(t *testing.T) {
	pr := newPrintRecovery(nil, "")

	state := &PrintRecoveryState{
		Version:         printRecoveryVersion,
		Filename:        "test.gcode",
		FilePosition:    12345,
		FileSize:        100000,
		PositionX:       100.0,
		PositionY:       50.0,
		PositionZ:       10.0,
		PositionE:       150.0,
		AbsoluteCoord:   true,
		AbsoluteExtrude: true,
		Speed:           50.0,
		SpeedFactor:     1.0 / 60.0,
		ExtrudeFactor:   1.0,
		ExtruderTarget:  210.0,
		BedTarget:       60.0,
		FanSpeed:        0.8,
	}

	lines := pr.GenerateResumeGCode(state)
	if len(lines) == 0 {
		t.Fatal("expected resume gcode lines")
	}

	// Check for expected commands
	hasG90 := false
	hasM109 := false
	hasM190 := false
	hasM106 := false
	hasG0Z := false
	hasG92E := false

	for _, line := range lines {
		if line == "G90 ; absolute positioning" {
			hasG90 = true
		}
		if len(line) > 4 && line[:4] == "M109" {
			hasM109 = true
		}
		if len(line) > 4 && line[:4] == "M190" {
			hasM190 = true
		}
		if len(line) > 4 && line[:4] == "M106" {
			hasM106 = true
		}
		if len(line) > 3 && line[:2] == "G0" && line[3] == 'Z' {
			hasG0Z = true
		}
		if len(line) > 4 && line[:3] == "G92" {
			hasG92E = true
		}
	}

	if !hasG90 {
		t.Error("expected G90 command for absolute mode")
	}
	if !hasM109 {
		t.Error("expected M109 command for extruder temp wait")
	}
	if !hasM190 {
		t.Error("expected M190 command for bed temp wait")
	}
	if !hasM106 {
		t.Error("expected M106 command for fan speed")
	}
	if !hasG0Z {
		t.Error("expected G0 Z command for Z positioning")
	}
	if !hasG92E {
		t.Error("expected G92 E command for extruder position")
	}
}

func TestPrintRecoveryNoState(t *testing.T) {
	// Create temp directory
	tmpDir, err := os.MkdirTemp("", "print_recovery_test")
	if err != nil {
		t.Fatalf("failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tmpDir)

	pr := newPrintRecovery(nil, tmpDir)

	// Should not have recovery state
	if pr.HasRecoveryState() {
		t.Error("expected no recovery state")
	}

	// Load should return nil, nil
	state, err := pr.LoadState()
	if err != nil {
		t.Errorf("unexpected error: %v", err)
	}
	if state != nil {
		t.Error("expected nil state")
	}
}

func TestPrintRecoveryWriteResumeScript(t *testing.T) {
	// Create temp directory
	tmpDir, err := os.MkdirTemp("", "print_recovery_test")
	if err != nil {
		t.Fatalf("failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tmpDir)

	pr := newPrintRecovery(nil, tmpDir)

	// Create and save a test state
	testState := &PrintRecoveryState{
		Version:         printRecoveryVersion,
		SavedAt:         time.Now(),
		Filename:        "test.gcode",
		FilePosition:    12345,
		FileSize:        100000,
		PositionX:       100.0,
		PositionY:       50.0,
		PositionZ:       10.0,
		PositionE:       150.0,
		AbsoluteCoord:   true,
		AbsoluteExtrude: true,
		ExtruderTarget:  210.0,
		BedTarget:       60.0,
	}

	pr.mu.Lock()
	err = pr.saveStateToFile(testState)
	pr.mu.Unlock()
	if err != nil {
		t.Fatalf("failed to save state: %v", err)
	}

	// Write resume script
	scriptPath := filepath.Join(tmpDir, "resume.gcode")
	if err := pr.WriteResumeScript(scriptPath); err != nil {
		t.Fatalf("failed to write resume script: %v", err)
	}

	// Verify script exists
	if _, err := os.Stat(scriptPath); err != nil {
		t.Errorf("resume script not created: %v", err)
	}

	// Read and verify content
	content, err := os.ReadFile(scriptPath)
	if err != nil {
		t.Fatalf("failed to read resume script: %v", err)
	}
	if len(content) == 0 {
		t.Error("resume script is empty")
	}
}

func TestPrintRecoveryStatus(t *testing.T) {
	pr := newPrintRecovery(nil, "")

	status := pr.GetStatus()
	if status == nil {
		t.Fatal("status is nil")
	}

	if enabled, ok := status["enabled"].(bool); !ok || !enabled {
		t.Error("expected enabled=true")
	}

	if hasState, ok := status["has_recovery_state"].(bool); !ok || hasState {
		t.Error("expected has_recovery_state=false")
	}
}

func BenchmarkPrintRecoverySaveState(b *testing.B) {
	// Create temp directory
	tmpDir, err := os.MkdirTemp("", "print_recovery_bench")
	if err != nil {
		b.Fatalf("failed to create temp dir: %v", err)
	}
	defer os.RemoveAll(tmpDir)

	pr := newPrintRecovery(nil, tmpDir)

	testState := &PrintRecoveryState{
		Version:         printRecoveryVersion,
		SavedAt:         time.Now(),
		Filename:        "test.gcode",
		FilePosition:    12345,
		FileSize:        100000,
		PositionX:       100.0,
		PositionY:       50.0,
		PositionZ:       10.0,
		PositionE:       150.0,
		AbsoluteCoord:   true,
		AbsoluteExtrude: true,
		ExtruderTarget:  210.0,
		BedTarget:       60.0,
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		pr.mu.Lock()
		pr.saveStateToFile(testState)
		pr.mu.Unlock()
	}
}
