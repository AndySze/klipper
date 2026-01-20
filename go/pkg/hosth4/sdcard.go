// Virtual SDCard support - port of klippy/extras/virtual_sdcard.py
//
// Copyright (C) 2018-2024 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"bufio"
	"context"
	"fmt"
	"io"
	"log"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"
)

var validGCodeExts = []string{"gcode", "g", "gco"}

// Default error gcode template - executed on print errors
const defaultErrorGCode = `
; Error occurred - turning off heaters
M104 S0 ; extruder off
M140 S0 ; bed off
`

// VirtualSDCard handles virtual SD card file execution with loop support.
type VirtualSDCard struct {
	rt        *runtime
	basePath  string // base directory for SD card files
	file      *os.File
	reader    *bufio.Reader
	loopStack []loopFrame
	macros    *macroEngine

	// File state
	filePosition     int64
	fileSize         int64
	nextFilePosition int64

	// Print state - protected by mu
	mu            sync.Mutex
	mustPauseWork bool
	cmdFromSD     bool
	workActive    bool

	// Work timer context
	workCtx    context.Context
	workCancel context.CancelFunc

	// Error handling
	onErrorGCode string

	// Stats callback
	statsActive bool
}

// loopFrame tracks a single loop context
type loopFrame struct {
	count    int   // remaining iterations (0 means infinite loop, >0 means finite)
	position int64 // file position to seek back to
}

// newVirtualSDCard creates a virtual SD card with the given base path.
func newVirtualSDCard(rt *runtime, basePath string, macros *macroEngine) *VirtualSDCard {
	return &VirtualSDCard{
		rt:           rt,
		basePath:     basePath,
		macros:       macros,
		onErrorGCode: defaultErrorGCode,
	}
}

// SetOnErrorGCode sets the G-code to run when a print error occurs.
func (sd *VirtualSDCard) SetOnErrorGCode(gcode string) {
	sd.onErrorGCode = gcode
}

// IsActive returns true if currently printing from SD card.
func (sd *VirtualSDCard) IsActive() bool {
	sd.mu.Lock()
	defer sd.mu.Unlock()
	return sd.workActive
}

// FilePath returns the current file path, or empty string if none.
func (sd *VirtualSDCard) FilePath() string {
	if sd.file != nil {
		return sd.file.Name()
	}
	return ""
}

// Progress returns the print progress as a ratio (0.0 to 1.0).
func (sd *VirtualSDCard) Progress() float64 {
	if sd.fileSize > 0 {
		return float64(sd.filePosition) / float64(sd.fileSize)
	}
	return 0.0
}

// GetFilePosition returns the next file position.
func (sd *VirtualSDCard) GetFilePosition() int64 {
	return sd.nextFilePosition
}

// SetFilePosition sets the next file position.
func (sd *VirtualSDCard) SetFilePosition(pos int64) {
	sd.nextFilePosition = pos
}

// IsCmdFromSD returns true if currently executing a command from SD.
func (sd *VirtualSDCard) IsCmdFromSD() bool {
	sd.mu.Lock()
	defer sd.mu.Unlock()
	return sd.cmdFromSD
}

// Stats returns stats information (active, description).
// Compatible with Python's virtual_sdcard.stats() method.
func (sd *VirtualSDCard) Stats() (bool, string) {
	sd.mu.Lock()
	defer sd.mu.Unlock()
	if !sd.workActive {
		return false, ""
	}
	return true, fmt.Sprintf("sd_pos=%d", sd.filePosition)
}

// HandleAnalyzeShutdown logs context around the current file position on shutdown.
// Compatible with Python's virtual_sdcard._handle_analyze_shutdown().
func (sd *VirtualSDCard) HandleAnalyzeShutdown(msg string, details map[string]any) {
	sd.mu.Lock()
	defer sd.mu.Unlock()

	if !sd.workActive || sd.file == nil {
		return
	}

	sd.mustPauseWork = true

	// Read context around current position
	readPos := sd.filePosition - 1024
	if readPos < 0 {
		readPos = 0
	}
	readCount := sd.filePosition - readPos

	data := make([]byte, readCount+128)
	_, err := sd.file.Seek(readPos, io.SeekStart)
	if err != nil {
		log.Printf("virtual_sdcard shutdown read seek error: %v", err)
		return
	}

	n, err := sd.file.Read(data)
	if err != nil && err != io.EOF {
		log.Printf("virtual_sdcard shutdown read error: %v", err)
		return
	}
	data = data[:n]

	// Log the context
	past := data
	upcoming := []byte{}
	if int64(n) > readCount {
		past = data[:readCount]
		upcoming = data[readCount:]
	}

	log.Printf("Virtual sdcard (%d): %q", readPos, past)
	log.Printf("Upcoming (%d): %q", sd.filePosition, upcoming)
}

// GetStatus returns the current status for API queries.
func (sd *VirtualSDCard) GetStatus() map[string]any {
	return map[string]any{
		"file_path":     sd.FilePath(),
		"progress":      sd.Progress(),
		"is_active":     sd.IsActive(),
		"file_position": sd.filePosition,
		"file_size":     sd.fileSize,
	}
}

// DoPause pauses the SD card print.
// This is blocking - it waits until the work timer actually pauses.
func (sd *VirtualSDCard) DoPause() {
	sd.mu.Lock()
	if !sd.workActive {
		sd.mu.Unlock()
		return
	}
	sd.mustPauseWork = true
	sd.mu.Unlock()

	// Wait for work handler to acknowledge pause
	// The work handler checks mustPauseWork and will stop
	for {
		sd.mu.Lock()
		active := sd.workActive
		fromSD := sd.cmdFromSD
		sd.mu.Unlock()

		if !active || !fromSD {
			break
		}
		time.Sleep(1 * time.Millisecond)
	}
}

// DoResume resumes the SD card print.
func (sd *VirtualSDCard) DoResume() error {
	sd.mu.Lock()
	defer sd.mu.Unlock()

	if sd.workActive {
		return fmt.Errorf("SD busy")
	}
	sd.mustPauseWork = false
	sd.workActive = true
	sd.statsActive = true

	// Start the work handler in a goroutine
	sd.workCtx, sd.workCancel = context.WithCancel(context.Background())
	go sd.workHandler()

	return nil
}

// DoCancel cancels the current SD card print.
func (sd *VirtualSDCard) DoCancel() {
	sd.mu.Lock()
	if sd.file != nil {
		sd.mustPauseWork = true
		if sd.workCancel != nil {
			sd.workCancel()
		}
	}
	sd.mu.Unlock()

	// Wait for work handler to stop
	sd.DoPause()

	sd.mu.Lock()
	defer sd.mu.Unlock()

	if sd.file != nil {
		sd.file.Close()
		sd.file = nil
		sd.reader = nil
	}
	sd.filePosition = 0
	sd.fileSize = 0
	sd.workActive = false
	sd.mustPauseWork = false
	sd.statsActive = false
}

// LoopDesist clears all loop state. Safe to call outside of SD card context.
func (sd *VirtualSDCard) LoopDesist() {
	sd.loopStack = nil
}

// GetFileList returns a list of available gcode files.
func (sd *VirtualSDCard) GetFileList(checkSubdirs bool) ([]FileEntry, error) {
	if checkSubdirs {
		var flist []FileEntry
		err := filepath.Walk(sd.basePath, func(path string, info os.FileInfo, err error) error {
			if err != nil {
				return nil // ignore errors
			}
			if info.IsDir() {
				return nil
			}
			ext := strings.ToLower(filepath.Ext(info.Name()))
			if len(ext) > 0 {
				ext = ext[1:] // remove leading dot
			}
			for _, validExt := range validGCodeExts {
				if ext == validExt {
					relPath, _ := filepath.Rel(sd.basePath, path)
					flist = append(flist, FileEntry{
						Name: relPath,
						Size: info.Size(),
					})
					break
				}
			}
			return nil
		})
		if err != nil {
			return nil, fmt.Errorf("unable to get file list: %w", err)
		}
		sort.Slice(flist, func(i, j int) bool {
			return strings.ToLower(flist[i].Name) < strings.ToLower(flist[j].Name)
		})
		return flist, nil
	}

	// Non-recursive listing
	entries, err := os.ReadDir(sd.basePath)
	if err != nil {
		return nil, fmt.Errorf("unable to get file list: %w", err)
	}
	var flist []FileEntry
	for _, entry := range entries {
		if entry.IsDir() || strings.HasPrefix(entry.Name(), ".") {
			continue
		}
		// Filter by valid gcode extensions
		ext := strings.ToLower(filepath.Ext(entry.Name()))
		if len(ext) > 0 {
			ext = ext[1:] // remove leading dot
		}
		isValid := false
		for _, validExt := range validGCodeExts {
			if ext == validExt {
				isValid = true
				break
			}
		}
		if !isValid {
			continue
		}
		info, err := entry.Info()
		if err != nil {
			continue
		}
		flist = append(flist, FileEntry{
			Name: entry.Name(),
			Size: info.Size(),
		})
	}
	sort.Slice(flist, func(i, j int) bool {
		return strings.ToLower(flist[i].Name) < strings.ToLower(flist[j].Name)
	})
	return flist, nil
}

// FileEntry represents a file in the SD card directory.
type FileEntry struct {
	Name string
	Size int64
}

// resetFile resets file state for a new print or cancel.
func (sd *VirtualSDCard) resetFile() {
	if sd.file != nil {
		sd.DoPause()
		sd.file.Close()
		sd.file = nil
		sd.reader = nil
	}
	sd.filePosition = 0
	sd.fileSize = 0
	sd.loopStack = nil
	sd.workActive = false
	sd.mustPauseWork = false
	sd.statsActive = false
}

// loadFile loads a file for printing.
func (sd *VirtualSDCard) loadFile(filename string, checkSubdirs bool) error {
	files, err := sd.GetFileList(checkSubdirs)
	if err != nil {
		return err
	}

	// Build lookup map
	filesByLower := make(map[string]string)
	for _, f := range files {
		filesByLower[strings.ToLower(f.Name)] = f.Name
	}

	// Find the file (case-insensitive)
	fname := filename
	if _, exists := filesByLower[strings.ToLower(filename)]; exists {
		fname = filesByLower[strings.ToLower(filename)]
	}

	fullPath := filepath.Join(sd.basePath, fname)
	f, err := os.Open(fullPath)
	if err != nil {
		return fmt.Errorf("unable to open file: %w", err)
	}

	// Get file size
	info, err := f.Stat()
	if err != nil {
		f.Close()
		return fmt.Errorf("unable to stat file: %w", err)
	}

	sd.file = f
	sd.reader = bufio.NewReader(f)
	sd.filePosition = 0
	sd.fileSize = info.Size()
	sd.nextFilePosition = 0

	// Update print stats
	if sd.rt != nil && sd.rt.printStats != nil {
		sd.rt.printStats.setCurrentFile(filename)
	}

	return nil
}

// PrintFile loads and executes a gcode file from the virtual SD card.
// This is a synchronous version that blocks until print is complete or paused.
func (sd *VirtualSDCard) PrintFile(filename string) error {
	sd.resetFile()

	if err := sd.loadFile(filename, true); err != nil {
		return err
	}

	sd.mu.Lock()
	sd.workActive = true
	sd.mustPauseWork = false
	sd.statsActive = true
	sd.workCtx, sd.workCancel = context.WithCancel(context.Background())
	sd.mu.Unlock()

	// Start print stats
	if sd.rt != nil && sd.rt.printStats != nil {
		sd.rt.printStats.noteStart()
	}

	// Run synchronously (for tests and simple use)
	return sd.runFileSync()
}

// PrintFileAsync loads a file and starts printing in a background goroutine.
// Returns immediately after starting. Use IsActive() to check status.
func (sd *VirtualSDCard) PrintFileAsync(filename string) error {
	sd.resetFile()

	if err := sd.loadFile(filename, true); err != nil {
		return err
	}

	return sd.DoResume()
}

// workHandler is the background work handler that runs print execution.
// Called as a goroutine from DoResume.
func (sd *VirtualSDCard) workHandler() {
	log.Printf("Starting SD card print (position %d)", sd.filePosition)

	err := sd.runFileCore()

	sd.mu.Lock()
	defer sd.mu.Unlock()

	log.Printf("Exiting SD card print (position %d)", sd.filePosition)
	sd.workActive = false
	sd.cmdFromSD = false
	sd.statsActive = false

	if sd.workCancel != nil {
		sd.workCancel()
		sd.workCancel = nil
	}

	// Update print stats based on outcome
	if err != nil {
		if sd.rt != nil && sd.rt.printStats != nil {
			sd.rt.printStats.noteError(err.Error())
		}
	} else if sd.file != nil {
		// Still has file but stopped - must be paused
		if sd.rt != nil && sd.rt.printStats != nil {
			sd.rt.printStats.notePause()
		}
	} else {
		// File closed - complete
		if sd.rt != nil && sd.rt.printStats != nil {
			sd.rt.printStats.noteComplete()
		}
	}
}

// runFileSync executes gcode synchronously (blocking).
// Used by PrintFile for direct synchronous execution.
func (sd *VirtualSDCard) runFileSync() error {
	log.Printf("Starting SD card print (position %d)", sd.filePosition)

	err := sd.runFileCore()

	sd.mu.Lock()
	defer sd.mu.Unlock()

	log.Printf("Exiting SD card print (position %d)", sd.filePosition)

	// Cleanup
	if sd.file != nil {
		sd.file.Close()
		sd.file = nil
		sd.reader = nil
	}
	sd.loopStack = nil
	sd.workActive = false
	sd.cmdFromSD = false
	sd.statsActive = false

	if sd.workCancel != nil {
		sd.workCancel()
		sd.workCancel = nil
	}

	// Update print stats based on outcome
	if err != nil {
		if sd.rt != nil && sd.rt.printStats != nil {
			sd.rt.printStats.noteError(err.Error())
		}
		return err
	} else if sd.mustPauseWork {
		if sd.rt != nil && sd.rt.printStats != nil {
			sd.rt.printStats.notePause()
		}
	} else {
		if sd.rt != nil && sd.rt.printStats != nil {
			sd.rt.printStats.noteComplete()
		}
	}
	return nil
}

// runFileCore executes gcode from the current file position.
// Shared between sync and async execution.
func (sd *VirtualSDCard) runFileCore() error {
	// Seek to file position
	if _, err := sd.file.Seek(sd.filePosition, io.SeekStart); err != nil {
		return fmt.Errorf("virtual_sdcard seek: %w", err)
	}
	sd.reader.Reset(sd.file)

	var partialInput string
	var errorMessage string

	for {
		// Check for pause/cancel
		sd.mu.Lock()
		shouldPause := sd.mustPauseWork
		ctx := sd.workCtx
		sd.mu.Unlock()

		if shouldPause {
			break
		}

		// Check context cancellation
		if ctx != nil {
			select {
			case <-ctx.Done():
				return nil
			default:
			}
		}

		// Read line from file
		line, err := sd.reader.ReadString('\n')
		if err != nil {
			if err == io.EOF {
				// Process any remaining content without newline
				if partialInput != "" || line != "" {
					finalLine := partialInput + line
					if finalLine != "" {
						if err := sd.processLineWithErrorHandling(finalLine, &errorMessage); err != nil {
							return err
						}
					}
				}
				break
			}
			return fmt.Errorf("read SD card file: %w", err)
		}

		// Track file position
		lineLen := int64(len(line))
		nextPos := sd.filePosition + lineLen
		sd.nextFilePosition = nextPos

		// Execute the line
		sd.mu.Lock()
		sd.cmdFromSD = true
		sd.mu.Unlock()

		fullLine := partialInput + strings.TrimRight(line, "\r\n")
		partialInput = ""

		if err := sd.processLineWithErrorHandling(fullLine, &errorMessage); err != nil {
			sd.mu.Lock()
			sd.cmdFromSD = false
			sd.mu.Unlock()
			return err
		}

		sd.mu.Lock()
		sd.cmdFromSD = false
		sd.mu.Unlock()

		sd.filePosition = sd.nextFilePosition

		// Check if we need to seek (file position changed externally)
		if sd.nextFilePosition != nextPos {
			if _, err := sd.file.Seek(sd.filePosition, io.SeekStart); err != nil {
				return fmt.Errorf("virtual_sdcard seek: %w", err)
			}
			sd.reader.Reset(sd.file)
		}
	}

	if errorMessage != "" {
		return fmt.Errorf("%s", errorMessage)
	}
	return nil
}

// processLineWithErrorHandling processes a line and runs error gcode on failure.
func (sd *VirtualSDCard) processLineWithErrorHandling(line string, errorMessage *string) error {
	err := sd.processLine(line)
	if err != nil {
		*errorMessage = err.Error()
		// Run error gcode
		sd.runErrorGCode()
		return err
	}
	return nil
}

// runErrorGCode executes the on_error_gcode template.
func (sd *VirtualSDCard) runErrorGCode() {
	if sd.onErrorGCode == "" {
		return
	}

	log.Printf("virtual_sdcard: running error gcode")
	lines := strings.Split(sd.onErrorGCode, "\n")
	for _, line := range lines {
		line = strings.TrimSpace(line)
		if line == "" || strings.HasPrefix(line, ";") {
			continue
		}
		// Execute error gcode - ignore errors
		if err := sd.processLine(line); err != nil {
			log.Printf("virtual_sdcard on_error: %v", err)
		}
	}
}

// processLine processes a single gcode line.
func (sd *VirtualSDCard) processLine(line string) error {
	// Strip comments and whitespace
	if idx := strings.IndexByte(line, ';'); idx >= 0 {
		line = line[:idx]
	}
	line = strings.TrimSpace(line)
	if line == "" {
		return nil
	}

	return execGCodeWithMacros(sd.rt, sd.macros, line, 0)
}

// LoopBegin starts a new loop with the given count.
// Called from SDCARD_LOOP_BEGIN COUNT=n command.
func (sd *VirtualSDCard) LoopBegin(count int) error {
	if sd.file == nil {
		return fmt.Errorf("SDCARD_LOOP_BEGIN called outside SD card print")
	}

	// Get current file position (after reading the SDCARD_LOOP_BEGIN line)
	pos, err := sd.file.Seek(0, io.SeekCurrent)
	if err != nil {
		return fmt.Errorf("seek current position: %w", err)
	}
	// Adjust for buffered data
	pos -= int64(sd.reader.Buffered())

	frame := loopFrame{
		count:    count,
		position: pos,
	}
	sd.loopStack = append(sd.loopStack, frame)
	return nil
}

// LoopEnd handles end of loop. Decrements counter and seeks back if needed.
// Called from SDCARD_LOOP_END command.
// COUNT=0 means infinite loop (keeps looping until SDCARD_LOOP_DESIST is called).
// COUNT>0 means finite loop (decrements and exits when reaches 0).
func (sd *VirtualSDCard) LoopEnd() error {
	if sd.file == nil {
		return fmt.Errorf("SDCARD_LOOP_END called outside SD card print")
	}

	if len(sd.loopStack) == 0 {
		// No active loop - just continue
		return nil
	}

	// Pop top frame
	idx := len(sd.loopStack) - 1
	frame := sd.loopStack[idx]
	sd.loopStack = sd.loopStack[:idx]

	if frame.count == 0 {
		// Infinite loop (COUNT=0) - keep looping
		if _, err := sd.file.Seek(frame.position, io.SeekStart); err != nil {
			return fmt.Errorf("seek loop start: %w", err)
		}
		sd.reader.Reset(sd.file)
		// Push frame back with count=0 (infinite)
		sd.loopStack = append(sd.loopStack, frame)
	} else if frame.count == 1 {
		// Last iteration - don't loop back, just continue
		// Frame already popped above
	} else {
		// More iterations remaining - decrement and loop back
		if _, err := sd.file.Seek(frame.position, io.SeekStart); err != nil {
			return fmt.Errorf("seek loop start: %w", err)
		}
		sd.reader.Reset(sd.file)
		// Push frame back with decremented count
		frame.count--
		sd.loopStack = append(sd.loopStack, frame)
	}
	return nil
}

// execSDCardLoopDesist handles the SDCARD_LOOP_DESIST command.
func (r *runtime) execSDCardLoopDesist() error {
	if r.sdcard != nil {
		r.sdcard.LoopDesist()
	}
	return nil
}

// execSDCardPrintFile handles the SDCARD_PRINT_FILE command.
func (r *runtime) execSDCardPrintFile(args map[string]string) error {
	filename := args["FILENAME"]
	if filename == "" {
		return fmt.Errorf("SDCARD_PRINT_FILE: missing FILENAME parameter")
	}
	if r.sdcard == nil {
		return fmt.Errorf("SDCARD_PRINT_FILE: virtual_sdcard not configured")
	}
	return r.sdcard.PrintFile(filename)
}

// execSDCardResetFile handles the SDCARD_RESET_FILE command.
func (r *runtime) execSDCardResetFile(args map[string]string) error {
	if r.sdcard == nil {
		return nil
	}
	if r.sdcard.cmdFromSD {
		return fmt.Errorf("SDCARD_RESET_FILE cannot be run from the sdcard")
	}
	r.sdcard.resetFile()
	if r.printStats != nil {
		r.printStats.reset()
	}
	return nil
}

// execSDCardLoopBegin handles the SDCARD_LOOP_BEGIN command.
func (r *runtime) execSDCardLoopBegin(args map[string]string) error {
	if r.sdcard == nil {
		return fmt.Errorf("SDCARD_LOOP_BEGIN: virtual_sdcard not configured")
	}
	countStr := args["COUNT"]
	if countStr == "" {
		return fmt.Errorf("SDCARD_LOOP_BEGIN: missing COUNT parameter")
	}
	count, err := strconv.Atoi(countStr)
	if err != nil {
		return fmt.Errorf("SDCARD_LOOP_BEGIN: invalid COUNT: %w", err)
	}
	return r.sdcard.LoopBegin(count)
}

// execSDCardLoopEnd handles the SDCARD_LOOP_END command.
func (r *runtime) execSDCardLoopEnd() error {
	if r.sdcard == nil {
		return fmt.Errorf("SDCARD_LOOP_END: virtual_sdcard not configured")
	}
	return r.sdcard.LoopEnd()
}

// cmdM20 lists SD card files (M20 command).
func (r *runtime) cmdM20(args map[string]string) error {
	if r.sdcard == nil {
		return fmt.Errorf("virtual_sdcard not configured")
	}
	files, err := r.sdcard.GetFileList(false)
	if err != nil {
		return err
	}
	// Response would go to gcode output
	// Begin file list
	for _, f := range files {
		_ = fmt.Sprintf("%s %d", f.Name, f.Size)
	}
	// End file list
	return nil
}

// cmdM21 initializes SD card (M21 command).
func (r *runtime) cmdM21(args map[string]string) error {
	// SD card ok - always returns success
	return nil
}

// cmdM23 selects SD file (M23 command).
func (r *runtime) cmdM23(args map[string]string) error {
	if r.sdcard == nil {
		return fmt.Errorf("virtual_sdcard not configured")
	}
	if r.sdcard.workActive {
		return fmt.Errorf("SD busy")
	}
	r.sdcard.resetFile()

	// Get raw filename from args
	filename := args["FILENAME"]
	if filename == "" {
		// Try to get from raw command line
		filename = args[""]
	}
	if filename != "" && filename[0] == '/' {
		filename = filename[1:]
	}
	return r.sdcard.loadFile(filename, false)
}

// cmdM24 starts/resumes SD print (M24 command).
func (r *runtime) cmdM24(args map[string]string) error {
	if r.sdcard == nil {
		return fmt.Errorf("virtual_sdcard not configured")
	}
	return r.sdcard.DoResume()
}

// cmdM25 pauses SD print (M25 command).
func (r *runtime) cmdM25(args map[string]string) error {
	if r.sdcard == nil {
		return nil
	}
	r.sdcard.DoPause()
	return nil
}

// cmdM26 sets SD position (M26 command).
func (r *runtime) cmdM26(args map[string]string) error {
	if r.sdcard == nil {
		return fmt.Errorf("virtual_sdcard not configured")
	}
	if r.sdcard.workActive {
		return fmt.Errorf("SD busy")
	}
	posStr := args["S"]
	if posStr == "" {
		return nil
	}
	pos, err := strconv.ParseInt(posStr, 10, 64)
	if err != nil || pos < 0 {
		return fmt.Errorf("invalid position")
	}
	r.sdcard.filePosition = pos
	return nil
}

// cmdM27 reports SD print status (M27 command).
func (r *runtime) cmdM27(args map[string]string) error {
	if r.sdcard == nil || r.sdcard.file == nil {
		// Not SD printing
		return nil
	}
	// SD printing byte X/Y
	_ = fmt.Sprintf("SD printing byte %d/%d", r.sdcard.filePosition, r.sdcard.fileSize)
	return nil
}
