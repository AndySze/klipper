package config

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestAutosaveSetOption(t *testing.T) {
	data := `
[printer]
kinematics: cartesian
max_velocity: 300
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	ac := NewAutosaveConfig(cfg, "")

	// Set new option
	err = ac.SetOption("printer", "max_accel", "3000")
	if err != nil {
		t.Fatalf("SetOption failed: %v", err)
	}

	// Verify change tracked
	if !ac.HasChanges() {
		t.Error("expected HasChanges to return true")
	}

	modified := ac.GetModifiedSections()
	if len(modified) != 1 || modified[0] != "printer" {
		t.Errorf("expected ['printer'], got %v", modified)
	}

	// Verify value accessible
	sec, _ := ac.GetSection("printer")
	val, _ := sec.GetInt("max_accel")
	if val != 3000 {
		t.Errorf("expected 3000, got %d", val)
	}
}

func TestAutosaveNewSection(t *testing.T) {
	data := `
[printer]
kinematics: cartesian
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	ac := NewAutosaveConfig(cfg, "")

	// Add option to new section
	err = ac.SetOption("new_section", "key", "value")
	if err != nil {
		t.Fatalf("SetOption failed: %v", err)
	}

	// Verify new section exists
	if !ac.HasSection("new_section") {
		t.Error("expected new_section to exist")
	}

	sec, _ := ac.GetSection("new_section")
	val, _ := sec.Get("key")
	if val != "value" {
		t.Errorf("expected 'value', got %q", val)
	}
}

func TestAutosaveDeleteSection(t *testing.T) {
	data := `
[printer]
kinematics: cartesian

[stepper_x]
step_pin: PA0
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	ac := NewAutosaveConfig(cfg, "")

	// Delete section
	ac.DeleteSection("stepper_x")

	if !ac.HasChanges() {
		t.Error("expected HasChanges to return true")
	}

	deleted := ac.GetDeletedSections()
	if len(deleted) != 1 || deleted[0] != "stepper_x" {
		t.Errorf("expected ['stepper_x'], got %v", deleted)
	}
}

func TestAutosaveClearChanges(t *testing.T) {
	data := `
[printer]
kinematics: cartesian
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	ac := NewAutosaveConfig(cfg, "")

	// Make changes
	ac.SetOption("printer", "new_key", "value")
	ac.DeleteSection("printer")

	if !ac.HasChanges() {
		t.Error("expected changes before clear")
	}

	// Clear changes
	ac.ClearChanges()

	if ac.HasChanges() {
		t.Error("expected no changes after clear")
	}
}

func TestAutosaveSaveChanges(t *testing.T) {
	data := `
[printer]
kinematics: cartesian
max_velocity: 300
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	// Create temp file
	tmpDir := t.TempDir()
	tmpPath := filepath.Join(tmpDir, "test.cfg")

	ac := NewAutosaveConfig(cfg, tmpPath)

	// Modify and save
	ac.SetOption("printer", "max_accel", "3000")
	err = ac.SaveChanges("")
	if err != nil {
		t.Fatalf("SaveChanges failed: %v", err)
	}

	// Verify changes cleared
	if ac.HasChanges() {
		t.Error("expected no changes after save")
	}

	// Read saved file and verify content
	content, err := os.ReadFile(tmpPath)
	if err != nil {
		t.Fatalf("failed to read saved file: %v", err)
	}

	if !strings.Contains(string(content), "max_accel: 3000") {
		t.Error("expected saved file to contain max_accel")
	}
	if !strings.Contains(string(content), "kinematics: cartesian") {
		t.Error("expected saved file to contain kinematics")
	}
}

func TestAutosaveBackup(t *testing.T) {
	// Create initial file
	tmpDir := t.TempDir()
	tmpPath := filepath.Join(tmpDir, "printer.cfg")

	initialContent := `[printer]
kinematics: cartesian
`
	if err := os.WriteFile(tmpPath, []byte(initialContent), 0644); err != nil {
		t.Fatalf("failed to write initial file: %v", err)
	}

	// Load and modify
	ac, err := LoadAutosave(tmpPath)
	if err != nil {
		t.Fatalf("LoadAutosave failed: %v", err)
	}

	ac.SetOption("printer", "new_key", "value")
	err = ac.SaveChanges("")
	if err != nil {
		t.Fatalf("SaveChanges failed: %v", err)
	}

	// Check backup was created
	files, err := filepath.Glob(filepath.Join(tmpDir, "printer-*.cfg"))
	if err != nil {
		t.Fatalf("glob failed: %v", err)
	}

	if len(files) == 0 {
		t.Error("expected backup file to be created")
	}

	// Verify backup contains original content
	if len(files) > 0 {
		backup, err := os.ReadFile(files[0])
		if err != nil {
			t.Fatalf("failed to read backup: %v", err)
		}
		if string(backup) != initialContent {
			t.Error("backup should contain original content")
		}
	}
}

func TestAutosaveReloadFromDisk(t *testing.T) {
	// Create initial file
	tmpDir := t.TempDir()
	tmpPath := filepath.Join(tmpDir, "test.cfg")

	initialContent := `[printer]
kinematics: cartesian
`
	if err := os.WriteFile(tmpPath, []byte(initialContent), 0644); err != nil {
		t.Fatalf("failed to write initial file: %v", err)
	}

	// Load
	ac, err := LoadAutosave(tmpPath)
	if err != nil {
		t.Fatalf("LoadAutosave failed: %v", err)
	}

	// Make changes
	ac.SetOption("printer", "new_key", "value")

	// Write different content to file
	newContent := `[printer]
kinematics: corexy
`
	if err := os.WriteFile(tmpPath, []byte(newContent), 0644); err != nil {
		t.Fatalf("failed to write new content: %v", err)
	}

	// Reload
	err = ac.ReloadFromDisk()
	if err != nil {
		t.Fatalf("ReloadFromDisk failed: %v", err)
	}

	// Verify changes discarded and new content loaded
	if ac.HasChanges() {
		t.Error("expected no changes after reload")
	}

	sec, _ := ac.GetSection("printer")
	val, _ := sec.Get("kinematics")
	if val != "corexy" {
		t.Errorf("expected 'corexy' after reload, got %q", val)
	}
}

func TestBuildConfigContent(t *testing.T) {
	data := `
[printer]
kinematics: cartesian
max_velocity: 300

[stepper_x]
step_pin: PA0
dir_pin: PA1
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	ac := NewAutosaveConfig(cfg, "")
	content := ac.buildConfigContent()

	// Verify sections present
	if !strings.Contains(content, "[printer]") {
		t.Error("expected [printer] section")
	}
	if !strings.Contains(content, "[stepper_x]") {
		t.Error("expected [stepper_x] section")
	}

	// Verify options present
	if !strings.Contains(content, "kinematics: cartesian") {
		t.Error("expected kinematics option")
	}
	if !strings.Contains(content, "step_pin: PA0") {
		t.Error("expected step_pin option")
	}
}
