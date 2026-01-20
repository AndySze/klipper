// Virtual SD card tests
package hosth4

import (
	"os"
	"path/filepath"
	"testing"
)

func TestVirtualSDCardBasic(t *testing.T) {
	// Create temp directory for test files
	tmpDir := t.TempDir()

	// Create a test gcode file
	testGcode := `; Test file
G28 ; home
G1 X10 Y10 F1000
G1 Z5
`
	testFile := filepath.Join(tmpDir, "test.gcode")
	if err := os.WriteFile(testFile, []byte(testGcode), 0644); err != nil {
		t.Fatalf("Failed to create test file: %v", err)
	}

	// Create VirtualSDCard (without runtime, so processing will fail but structure works)
	sd := newVirtualSDCard(nil, tmpDir, nil)

	// Test IsActive before starting
	if sd.IsActive() {
		t.Error("SD card should not be active before print")
	}

	// Test FilePath when no file loaded
	if sd.FilePath() != "" {
		t.Error("FilePath should be empty when no file loaded")
	}

	// Test Progress when no file
	if sd.Progress() != 0.0 {
		t.Error("Progress should be 0 when no file loaded")
	}

	// Test GetFileList
	files, err := sd.GetFileList(false)
	if err != nil {
		t.Fatalf("GetFileList failed: %v", err)
	}
	if len(files) != 1 {
		t.Errorf("Expected 1 file, got %d", len(files))
	}
	if files[0].Name != "test.gcode" {
		t.Errorf("Expected test.gcode, got %s", files[0].Name)
	}

	// Test GetStatus
	status := sd.GetStatus()
	if status["is_active"].(bool) {
		t.Error("Status should show not active")
	}
	if status["progress"].(float64) != 0.0 {
		t.Error("Status progress should be 0")
	}
}

func TestVirtualSDCardFileList(t *testing.T) {
	tmpDir := t.TempDir()

	// Create test files with various extensions
	files := map[string]bool{
		"print1.gcode": true,
		"print2.g":     true,
		"print3.gco":   true,
		"readme.txt":   false,
		".hidden.gcode": false,
	}

	for name := range files {
		path := filepath.Join(tmpDir, name)
		if err := os.WriteFile(path, []byte("test"), 0644); err != nil {
			t.Fatalf("Failed to create test file: %v", err)
		}
	}

	sd := newVirtualSDCard(nil, tmpDir, nil)

	// Test non-recursive listing
	list, err := sd.GetFileList(false)
	if err != nil {
		t.Fatalf("GetFileList failed: %v", err)
	}

	// Should only include gcode files and not hidden files
	foundFiles := make(map[string]bool)
	for _, f := range list {
		foundFiles[f.Name] = true
	}

	// Check expected files
	if !foundFiles["print1.gcode"] {
		t.Error("print1.gcode should be in list")
	}
	if !foundFiles["print2.g"] {
		t.Error("print2.g should be in list")
	}
	if !foundFiles["print3.gco"] {
		t.Error("print3.gco should be in list")
	}
	if foundFiles["readme.txt"] {
		t.Error("readme.txt should not be in list")
	}
}

func TestVirtualSDCardSubdirs(t *testing.T) {
	tmpDir := t.TempDir()

	// Create subdirectory with gcode file
	subDir := filepath.Join(tmpDir, "subfolder")
	if err := os.MkdirAll(subDir, 0755); err != nil {
		t.Fatalf("Failed to create subdir: %v", err)
	}

	rootFile := filepath.Join(tmpDir, "root.gcode")
	subFile := filepath.Join(subDir, "sub.gcode")

	if err := os.WriteFile(rootFile, []byte("root"), 0644); err != nil {
		t.Fatalf("Failed to create root file: %v", err)
	}
	if err := os.WriteFile(subFile, []byte("sub"), 0644); err != nil {
		t.Fatalf("Failed to create sub file: %v", err)
	}

	sd := newVirtualSDCard(nil, tmpDir, nil)

	// Non-recursive should only find root file
	list, err := sd.GetFileList(false)
	if err != nil {
		t.Fatalf("GetFileList(false) failed: %v", err)
	}
	if len(list) != 1 {
		t.Errorf("Non-recursive should find 1 file, found %d", len(list))
	}

	// Recursive should find both
	list, err = sd.GetFileList(true)
	if err != nil {
		t.Fatalf("GetFileList(true) failed: %v", err)
	}
	if len(list) != 2 {
		t.Errorf("Recursive should find 2 files, found %d", len(list))
	}
}

func TestVirtualSDCardLoadFile(t *testing.T) {
	tmpDir := t.TempDir()

	testContent := "G28\nG1 X10\n"
	testFile := filepath.Join(tmpDir, "test.gcode")
	if err := os.WriteFile(testFile, []byte(testContent), 0644); err != nil {
		t.Fatalf("Failed to create test file: %v", err)
	}

	sd := newVirtualSDCard(nil, tmpDir, nil)

	// Load the file
	if err := sd.loadFile("test.gcode", false); err != nil {
		t.Fatalf("loadFile failed: %v", err)
	}

	// Check file state
	if sd.file == nil {
		t.Error("File should be loaded")
	}
	if sd.fileSize != int64(len(testContent)) {
		t.Errorf("File size mismatch: expected %d, got %d", len(testContent), sd.fileSize)
	}
	if sd.filePosition != 0 {
		t.Error("File position should be 0")
	}

	// Check FilePath
	if sd.FilePath() == "" {
		t.Error("FilePath should not be empty after load")
	}

	// Reset
	sd.resetFile()
	if sd.file != nil {
		t.Error("File should be nil after reset")
	}
}

func TestVirtualSDCardCaseInsensitive(t *testing.T) {
	tmpDir := t.TempDir()

	testFile := filepath.Join(tmpDir, "MyPrint.GCODE")
	if err := os.WriteFile(testFile, []byte("G28\n"), 0644); err != nil {
		t.Fatalf("Failed to create test file: %v", err)
	}

	sd := newVirtualSDCard(nil, tmpDir, nil)

	// Should find with different case
	err := sd.loadFile("myprint.gcode", true)
	if err != nil {
		t.Errorf("Case insensitive load failed: %v", err)
	}

	sd.resetFile()
}

func TestVirtualSDCardPauseResumeState(t *testing.T) {
	tmpDir := t.TempDir()

	testFile := filepath.Join(tmpDir, "test.gcode")
	if err := os.WriteFile(testFile, []byte("G28\n"), 0644); err != nil {
		t.Fatalf("Failed to create test file: %v", err)
	}

	sd := newVirtualSDCard(nil, tmpDir, nil)

	// Load file
	if err := sd.loadFile("test.gcode", false); err != nil {
		t.Fatalf("loadFile failed: %v", err)
	}

	// Not active yet
	if sd.IsActive() {
		t.Error("Should not be active before starting")
	}

	// Test DoPause when not active (should be safe)
	sd.DoPause()
	if sd.IsActive() {
		t.Error("Should still not be active after pause when not started")
	}

	// Test DoCancel when not active (should be safe)
	sd.DoCancel()
	if sd.IsActive() {
		t.Error("Should not be active after cancel")
	}

	// Test that DoResume fails when already active
	sd.mu.Lock()
	sd.workActive = true
	sd.mu.Unlock()

	err := sd.DoResume()
	if err == nil {
		t.Error("DoResume should fail when already active")
	}

	// Reset
	sd.mu.Lock()
	sd.workActive = false
	sd.mu.Unlock()
}

func TestVirtualSDCardStats(t *testing.T) {
	tmpDir := t.TempDir()
	sd := newVirtualSDCard(nil, tmpDir, nil)

	// When not active
	active, desc := sd.Stats()
	if active {
		t.Error("Stats should show not active")
	}
	if desc != "" {
		t.Error("Description should be empty when not active")
	}

	// Manually set active state for testing
	sd.mu.Lock()
	sd.workActive = true
	sd.filePosition = 1234
	sd.mu.Unlock()

	active, desc = sd.Stats()
	if !active {
		t.Error("Stats should show active")
	}
	if desc != "sd_pos=1234" {
		t.Errorf("Unexpected description: %s", desc)
	}
}

func TestVirtualSDCardOnErrorGCode(t *testing.T) {
	tmpDir := t.TempDir()
	sd := newVirtualSDCard(nil, tmpDir, nil)

	// Check default error gcode is set
	if sd.onErrorGCode == "" {
		t.Error("Default error gcode should be set")
	}

	// Test setting custom error gcode
	customGcode := "M104 S0\nM140 S0\nM84"
	sd.SetOnErrorGCode(customGcode)
	if sd.onErrorGCode != customGcode {
		t.Error("Custom error gcode not set correctly")
	}
}

func TestVirtualSDCardLoopStack(t *testing.T) {
	tmpDir := t.TempDir()
	sd := newVirtualSDCard(nil, tmpDir, nil)

	// Test LoopDesist clears stack
	sd.loopStack = []loopFrame{{count: 5, position: 100}}
	sd.LoopDesist()
	if len(sd.loopStack) != 0 {
		t.Error("LoopDesist should clear stack")
	}
}
