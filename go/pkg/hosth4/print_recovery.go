// Print Recovery - support for resuming prints after power loss or errors
//
// Provides state persistence and recovery mechanisms similar to Python's
// save_variables.py combined with pause_resume.py functionality.
//
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"bufio"
	"encoding/json"
	"fmt"
	"log"
	"os"
	"path/filepath"
	"sync"
	"time"
)

// PrintRecoveryState holds the complete state for resuming a print.
type PrintRecoveryState struct {
	// File information
	Filename     string `json:"filename"`
	FilePosition int64  `json:"file_position"`
	FileSize     int64  `json:"file_size"`

	// Position information
	PositionX float64 `json:"position_x"`
	PositionY float64 `json:"position_y"`
	PositionZ float64 `json:"position_z"`
	PositionE float64 `json:"position_e"`

	// G-code state
	AbsoluteCoord   bool    `json:"absolute_coord"`
	AbsoluteExtrude bool    `json:"absolute_extrude"`
	Speed           float64 `json:"speed"`
	SpeedFactor     float64 `json:"speed_factor"`
	ExtrudeFactor   float64 `json:"extrude_factor"`

	// Temperature targets
	ExtruderTemp   float64 `json:"extruder_temp"`
	ExtruderTarget float64 `json:"extruder_target"`
	BedTemp        float64 `json:"bed_temp"`
	BedTarget      float64 `json:"bed_target"`

	// Fan speed (0-1)
	FanSpeed float64 `json:"fan_speed"`

	// Active extruder
	ActiveExtruder string `json:"active_extruder"`

	// Print statistics
	PrintDuration  float64 `json:"print_duration"`
	FilamentUsed   float64 `json:"filament_used"`
	CurrentLayer   int     `json:"current_layer"`
	TotalLayers    int     `json:"total_layers"`

	// Timestamp when state was saved
	SavedAt time.Time `json:"saved_at"`

	// Version for compatibility checking
	Version int `json:"version"`
}

const (
	printRecoveryVersion    = 1
	defaultRecoveryFilename = "print_recovery.json"
	saveIntervalLines       = 100  // Save state every N lines
	saveIntervalSeconds     = 30.0 // Save state every N seconds
)

// PrintRecovery manages print state persistence for recovery.
type PrintRecovery struct {
	rt           *runtime
	recoveryPath string
	enabled      bool

	// Current state
	currentState *PrintRecoveryState
	lastSaveTime time.Time
	linesSinceLastSave int

	mu sync.Mutex
}

// newPrintRecovery creates a new print recovery manager.
func newPrintRecovery(rt *runtime, recoveryDir string) *PrintRecovery {
	if recoveryDir == "" {
		recoveryDir = "."
	}
	return &PrintRecovery{
		rt:           rt,
		recoveryPath: filepath.Join(recoveryDir, defaultRecoveryFilename),
		enabled:      true,
		lastSaveTime: time.Now(),
	}
}

// SetEnabled enables or disables print recovery.
func (pr *PrintRecovery) SetEnabled(enabled bool) {
	pr.mu.Lock()
	defer pr.mu.Unlock()
	pr.enabled = enabled
}

// IsEnabled returns whether print recovery is enabled.
func (pr *PrintRecovery) IsEnabled() bool {
	pr.mu.Lock()
	defer pr.mu.Unlock()
	return pr.enabled
}

// SetRecoveryPath sets the path for the recovery state file.
func (pr *PrintRecovery) SetRecoveryPath(path string) {
	pr.mu.Lock()
	defer pr.mu.Unlock()
	pr.recoveryPath = path
}

// CaptureState captures the current print state.
func (pr *PrintRecovery) CaptureState() *PrintRecoveryState {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	if pr.rt == nil {
		return nil
	}

	state := &PrintRecoveryState{
		Version: printRecoveryVersion,
		SavedAt: time.Now(),
	}

	// Capture file position from virtual_sdcard
	if pr.rt.sdcard != nil {
		state.Filename = pr.rt.sdcard.FilePath()
		if state.Filename != "" {
			// Make filename relative
			state.Filename = filepath.Base(state.Filename)
		}
		state.FilePosition = pr.rt.sdcard.GetFilePosition()
		state.FileSize = pr.rt.sdcard.fileSize
	}

	// Capture position from toolhead
	if pr.rt.toolhead != nil && len(pr.rt.toolhead.commandedPos) >= 4 {
		state.PositionX = pr.rt.toolhead.commandedPos[0]
		state.PositionY = pr.rt.toolhead.commandedPos[1]
		state.PositionZ = pr.rt.toolhead.commandedPos[2]
		state.PositionE = pr.rt.toolhead.commandedPos[3]
	}

	// Capture G-code state
	if pr.rt.gm != nil {
		state.AbsoluteCoord = pr.rt.gm.absoluteCoord
		state.AbsoluteExtrude = pr.rt.gm.absoluteExtrude
		state.Speed = pr.rt.gm.speed
		state.SpeedFactor = pr.rt.gm.speedFactor
		state.ExtrudeFactor = pr.rt.gm.extrudeFactor
	}

	// Capture temperature targets from heater manager
	if pr.rt.heaterManager != nil {
		eventtime := float64(time.Now().UnixNano()) / 1e9
		if extruder, err := pr.rt.heaterManager.GetHeater("extruder"); err == nil && extruder != nil {
			state.ExtruderTemp, state.ExtruderTarget = extruder.GetTemp(eventtime)
		}
		if bed, err := pr.rt.heaterManager.GetHeater("heater_bed"); err == nil && bed != nil {
			state.BedTemp, state.BedTarget = bed.GetTemp(eventtime)
		}
	}

	// Capture fan speed from part cooling fan
	if pr.rt.fanManager != nil && pr.rt.fanManager.partFan != nil {
		state.FanSpeed = pr.rt.fanManager.partFan.getSpeed()
	}

	// Capture active extruder
	state.ActiveExtruder = pr.rt.activeExtruder
	if state.ActiveExtruder == "" {
		state.ActiveExtruder = "extruder"
	}

	// Capture print statistics
	if pr.rt.printStats != nil {
		stats := pr.rt.printStats.GetStatus()
		if duration, ok := stats["print_duration"].(float64); ok {
			state.PrintDuration = duration
		}
		if filament, ok := stats["filament_used"].(float64); ok {
			state.FilamentUsed = filament
		}
		if info, ok := stats["info"].(map[string]interface{}); ok {
			if layer, ok := info["current_layer"].(int); ok {
				state.CurrentLayer = layer
			}
			if total, ok := info["total_layer"].(int); ok {
				state.TotalLayers = total
			}
		}
	}

	pr.currentState = state
	return state
}

// SaveState saves the current print state to disk.
func (pr *PrintRecovery) SaveState() error {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	if !pr.enabled {
		return nil
	}

	// Capture current state
	state := pr.captureStateUnlocked()
	if state == nil {
		return fmt.Errorf("failed to capture state")
	}

	// Skip save if no active print
	if state.Filename == "" {
		return nil
	}

	return pr.saveStateToFile(state)
}

// captureStateUnlocked captures state without holding lock (internal use).
func (pr *PrintRecovery) captureStateUnlocked() *PrintRecoveryState {
	if pr.rt == nil {
		return nil
	}

	state := &PrintRecoveryState{
		Version: printRecoveryVersion,
		SavedAt: time.Now(),
	}

	// Capture file position from virtual_sdcard
	if pr.rt.sdcard != nil {
		state.Filename = pr.rt.sdcard.FilePath()
		if state.Filename != "" {
			state.Filename = filepath.Base(state.Filename)
		}
		state.FilePosition = pr.rt.sdcard.GetFilePosition()
		state.FileSize = pr.rt.sdcard.fileSize
	}

	// Capture position from toolhead
	if pr.rt.toolhead != nil && len(pr.rt.toolhead.commandedPos) >= 4 {
		state.PositionX = pr.rt.toolhead.commandedPos[0]
		state.PositionY = pr.rt.toolhead.commandedPos[1]
		state.PositionZ = pr.rt.toolhead.commandedPos[2]
		state.PositionE = pr.rt.toolhead.commandedPos[3]
	}

	// Capture G-code state
	if pr.rt.gm != nil {
		state.AbsoluteCoord = pr.rt.gm.absoluteCoord
		state.AbsoluteExtrude = pr.rt.gm.absoluteExtrude
		state.Speed = pr.rt.gm.speed
		state.SpeedFactor = pr.rt.gm.speedFactor
		state.ExtrudeFactor = pr.rt.gm.extrudeFactor
	}

	// Capture temperature targets
	if pr.rt.heaterManager != nil {
		eventtime := float64(time.Now().UnixNano()) / 1e9
		if extruder, err := pr.rt.heaterManager.GetHeater("extruder"); err == nil && extruder != nil {
			state.ExtruderTemp, state.ExtruderTarget = extruder.GetTemp(eventtime)
		}
		if bed, err := pr.rt.heaterManager.GetHeater("heater_bed"); err == nil && bed != nil {
			state.BedTemp, state.BedTarget = bed.GetTemp(eventtime)
		}
	}

	// Capture fan speed from part cooling fan
	if pr.rt.fanManager != nil && pr.rt.fanManager.partFan != nil {
		state.FanSpeed = pr.rt.fanManager.partFan.getSpeed()
	}

	state.ActiveExtruder = pr.rt.activeExtruder
	if state.ActiveExtruder == "" {
		state.ActiveExtruder = "extruder"
	}

	// Capture print statistics
	if pr.rt.printStats != nil {
		stats := pr.rt.printStats.GetStatus()
		if duration, ok := stats["print_duration"].(float64); ok {
			state.PrintDuration = duration
		}
		if filament, ok := stats["filament_used"].(float64); ok {
			state.FilamentUsed = filament
		}
	}

	pr.currentState = state
	return state
}

// saveStateToFile writes state to the recovery file.
func (pr *PrintRecovery) saveStateToFile(state *PrintRecoveryState) error {
	// Create temp file first
	tempPath := pr.recoveryPath + ".tmp"
	f, err := os.Create(tempPath)
	if err != nil {
		return fmt.Errorf("create recovery file: %w", err)
	}

	// Write JSON
	encoder := json.NewEncoder(f)
	encoder.SetIndent("", "  ")
	if err := encoder.Encode(state); err != nil {
		f.Close()
		os.Remove(tempPath)
		return fmt.Errorf("encode recovery state: %w", err)
	}

	if err := f.Close(); err != nil {
		os.Remove(tempPath)
		return fmt.Errorf("close recovery file: %w", err)
	}

	// Atomic rename
	if err := os.Rename(tempPath, pr.recoveryPath); err != nil {
		os.Remove(tempPath)
		return fmt.Errorf("rename recovery file: %w", err)
	}

	pr.lastSaveTime = time.Now()
	pr.linesSinceLastSave = 0
	log.Printf("print_recovery: saved state to %s (file_pos=%d)", pr.recoveryPath, state.FilePosition)
	return nil
}

// LoadState loads the print recovery state from disk.
func (pr *PrintRecovery) LoadState() (*PrintRecoveryState, error) {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	f, err := os.Open(pr.recoveryPath)
	if err != nil {
		if os.IsNotExist(err) {
			return nil, nil // No recovery state available
		}
		return nil, fmt.Errorf("open recovery file: %w", err)
	}
	defer f.Close()

	var state PrintRecoveryState
	decoder := json.NewDecoder(f)
	if err := decoder.Decode(&state); err != nil {
		return nil, fmt.Errorf("decode recovery state: %w", err)
	}

	// Version check
	if state.Version != printRecoveryVersion {
		log.Printf("print_recovery: version mismatch (got %d, expected %d)", state.Version, printRecoveryVersion)
		// For now, still try to use it
	}

	pr.currentState = &state
	return &state, nil
}

// ClearState removes the recovery state file.
func (pr *PrintRecovery) ClearState() error {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	pr.currentState = nil
	pr.linesSinceLastSave = 0

	if err := os.Remove(pr.recoveryPath); err != nil && !os.IsNotExist(err) {
		return fmt.Errorf("remove recovery file: %w", err)
	}
	log.Printf("print_recovery: cleared recovery state")
	return nil
}

// HasRecoveryState returns true if a recovery state file exists.
func (pr *PrintRecovery) HasRecoveryState() bool {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	_, err := os.Stat(pr.recoveryPath)
	return err == nil
}

// GetCurrentState returns the current (possibly unsaved) recovery state.
func (pr *PrintRecovery) GetCurrentState() *PrintRecoveryState {
	pr.mu.Lock()
	defer pr.mu.Unlock()
	return pr.currentState
}

// NotifyLineExecuted should be called after each G-code line is executed.
// This triggers periodic state saving.
func (pr *PrintRecovery) NotifyLineExecuted() {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	if !pr.enabled {
		return
	}

	pr.linesSinceLastSave++

	// Check if we should save
	shouldSave := false
	if pr.linesSinceLastSave >= saveIntervalLines {
		shouldSave = true
	} else if time.Since(pr.lastSaveTime).Seconds() >= saveIntervalSeconds {
		shouldSave = true
	}

	if shouldSave && pr.rt.sdcard != nil && pr.rt.sdcard.IsActive() {
		state := pr.captureStateUnlocked()
		if state != nil && state.Filename != "" {
			// Save asynchronously to avoid blocking print
			go func() {
				if err := pr.saveStateToFile(state); err != nil {
					log.Printf("print_recovery: save error: %v", err)
				}
			}()
		}
	}
}

// GenerateResumeGCode generates the G-code sequence to resume a print.
func (pr *PrintRecovery) GenerateResumeGCode(state *PrintRecoveryState) []string {
	if state == nil {
		return nil
	}

	var lines []string

	// Add comment header
	lines = append(lines, "; Print recovery resume sequence")
	lines = append(lines, fmt.Sprintf("; Resuming from file position %d", state.FilePosition))
	lines = append(lines, fmt.Sprintf("; Last position: X%.3f Y%.3f Z%.3f E%.3f",
		state.PositionX, state.PositionY, state.PositionZ, state.PositionE))

	// Set coordinate modes
	if state.AbsoluteCoord {
		lines = append(lines, "G90 ; absolute positioning")
	} else {
		lines = append(lines, "G91 ; relative positioning")
	}
	if state.AbsoluteExtrude {
		lines = append(lines, "M82 ; absolute extrusion")
	} else {
		lines = append(lines, "M83 ; relative extrusion")
	}

	// Set temperatures (don't wait yet)
	if state.ExtruderTarget > 0 {
		lines = append(lines, fmt.Sprintf("M104 S%.1f ; set extruder temp", state.ExtruderTarget))
	}
	if state.BedTarget > 0 {
		lines = append(lines, fmt.Sprintf("M140 S%.1f ; set bed temp", state.BedTarget))
	}

	// Wait for temperatures
	if state.ExtruderTarget > 0 {
		lines = append(lines, fmt.Sprintf("M109 S%.1f ; wait for extruder", state.ExtruderTarget))
	}
	if state.BedTarget > 0 {
		lines = append(lines, fmt.Sprintf("M190 S%.1f ; wait for bed", state.BedTarget))
	}

	// Home if needed (user should do this manually for safety, but we include it commented)
	lines = append(lines, "; G28 ; home all axes (uncomment if needed)")

	// Set speed override
	speedPercent := state.SpeedFactor * 60.0 * 100.0
	if speedPercent != 100.0 {
		lines = append(lines, fmt.Sprintf("M220 S%.0f ; speed factor", speedPercent))
	}

	// Set extrusion factor
	if state.ExtrudeFactor != 1.0 {
		lines = append(lines, fmt.Sprintf("M221 S%.0f ; extrusion factor", state.ExtrudeFactor*100.0))
	}

	// Set fan speed
	if state.FanSpeed > 0 {
		fanPWM := int(state.FanSpeed * 255)
		lines = append(lines, fmt.Sprintf("M106 S%d ; fan speed", fanPWM))
	}

	// Move to Z height first (safety)
	lines = append(lines, fmt.Sprintf("G0 Z%.3f F600 ; move to Z height", state.PositionZ+5)) // +5mm for clearance

	// Move to XY position
	lines = append(lines, fmt.Sprintf("G0 X%.3f Y%.3f F3000 ; move to XY position", state.PositionX, state.PositionY))

	// Lower to print Z
	lines = append(lines, fmt.Sprintf("G0 Z%.3f F600 ; lower to print height", state.PositionZ))

	// Set E position
	lines = append(lines, fmt.Sprintf("G92 E%.5f ; set extruder position", state.PositionE))

	// Prime extruder (small amount)
	lines = append(lines, "G1 E2 F300 ; prime extruder")
	lines = append(lines, fmt.Sprintf("G92 E%.5f ; restore extruder position", state.PositionE))

	// Set print speed
	lines = append(lines, fmt.Sprintf("G1 F%.0f ; restore print speed", state.Speed*60.0))

	// Ready to resume
	lines = append(lines, "; Resume printing below this line")

	return lines
}

// cmdSavePrintState handles the SAVE_PRINT_STATE command.
func (pr *PrintRecovery) cmdSavePrintState(args map[string]string) error {
	if err := pr.SaveState(); err != nil {
		return fmt.Errorf("SAVE_PRINT_STATE failed: %w", err)
	}
	return nil
}

// cmdClearPrintState handles the CLEAR_PRINT_STATE command.
func (pr *PrintRecovery) cmdClearPrintState(args map[string]string) error {
	if err := pr.ClearState(); err != nil {
		return fmt.Errorf("CLEAR_PRINT_STATE failed: %w", err)
	}
	return nil
}

// cmdQueryPrintState handles the QUERY_PRINT_STATE command.
func (pr *PrintRecovery) cmdQueryPrintState(args map[string]string) (string, error) {
	state, err := pr.LoadState()
	if err != nil {
		return "", fmt.Errorf("QUERY_PRINT_STATE failed: %w", err)
	}
	if state == nil {
		return "No recovery state available", nil
	}

	return fmt.Sprintf("Recovery state available:\n"+
		"  File: %s\n"+
		"  Position: %d/%d (%.1f%%)\n"+
		"  XYZ: %.3f, %.3f, %.3f\n"+
		"  Saved: %s",
		state.Filename,
		state.FilePosition, state.FileSize,
		float64(state.FilePosition)/float64(state.FileSize)*100,
		state.PositionX, state.PositionY, state.PositionZ,
		state.SavedAt.Format(time.RFC3339)), nil
}

// cmdResumePrint handles the RESUME_PRINT command.
// This loads the recovery state and sets up the printer to resume.
func (pr *PrintRecovery) cmdResumePrint(args map[string]string) error {
	state, err := pr.LoadState()
	if err != nil {
		return fmt.Errorf("RESUME_PRINT: failed to load state: %w", err)
	}
	if state == nil {
		return fmt.Errorf("RESUME_PRINT: no recovery state available")
	}

	// Verify the file still exists
	if pr.rt.sdcard == nil {
		return fmt.Errorf("RESUME_PRINT: virtual_sdcard not configured")
	}

	// Load the file
	if err := pr.rt.sdcard.loadFile(state.Filename, true); err != nil {
		return fmt.Errorf("RESUME_PRINT: failed to load file %s: %w", state.Filename, err)
	}

	// Set file position
	pr.rt.sdcard.filePosition = state.FilePosition
	pr.rt.sdcard.nextFilePosition = state.FilePosition

	// Generate resume gcode
	resumeLines := pr.GenerateResumeGCode(state)

	// Execute resume gcode
	for _, line := range resumeLines {
		if line == "" || line[0] == ';' {
			continue
		}
		cmd, err := parseGCodeLine(line)
		if err != nil {
			log.Printf("RESUME_PRINT: parse error: %v", err)
			continue
		}
		if cmd == nil {
			continue
		}
		if err := pr.rt.exec(cmd); err != nil {
			log.Printf("RESUME_PRINT: exec warning: %v", err)
			// Continue with other commands
		}
	}

	// Start the print
	if err := pr.rt.sdcard.DoResume(); err != nil {
		return fmt.Errorf("RESUME_PRINT: failed to start print: %w", err)
	}

	log.Printf("print_recovery: resumed print from position %d", state.FilePosition)
	return nil
}

// GetStatus returns the current status for API queries.
func (pr *PrintRecovery) GetStatus() map[string]interface{} {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	status := map[string]interface{}{
		"enabled":            pr.enabled,
		"has_recovery_state": pr.HasRecoveryStateUnlocked(),
	}

	if pr.currentState != nil {
		status["current_file"] = pr.currentState.Filename
		status["file_position"] = pr.currentState.FilePosition
		status["last_save_time"] = pr.currentState.SavedAt.Format(time.RFC3339)
	}

	return status
}

// HasRecoveryStateUnlocked checks for recovery state without lock (internal use).
func (pr *PrintRecovery) HasRecoveryStateUnlocked() bool {
	_, err := os.Stat(pr.recoveryPath)
	return err == nil
}

// WriteResumeScript writes a resume script to a file.
func (pr *PrintRecovery) WriteResumeScript(outputPath string) error {
	state, err := pr.LoadState()
	if err != nil {
		return fmt.Errorf("load state: %w", err)
	}
	if state == nil {
		return fmt.Errorf("no recovery state available")
	}

	lines := pr.GenerateResumeGCode(state)

	f, err := os.Create(outputPath)
	if err != nil {
		return fmt.Errorf("create file: %w", err)
	}
	defer f.Close()

	w := bufio.NewWriter(f)
	for _, line := range lines {
		if _, err := w.WriteString(line + "\n"); err != nil {
			return fmt.Errorf("write line: %w", err)
		}
	}
	if err := w.Flush(); err != nil {
		return fmt.Errorf("flush: %w", err)
	}

	log.Printf("print_recovery: wrote resume script to %s", outputPath)
	return nil
}
