package hosth4

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

// VirtualSDCard handles virtual SD card file execution with loop support.
type VirtualSDCard struct {
	rt        *runtime
	basePath  string // base directory for SD card files
	file      *os.File
	reader    *bufio.Reader
	loopStack []loopFrame
	macros    *macroEngine
}

// loopFrame tracks a single loop context
type loopFrame struct {
	count    int   // remaining iterations (0 means infinite loop, >0 means finite)
	position int64 // file position to seek back to
}

// newVirtualSDCard creates a virtual SD card with the given base path.
func newVirtualSDCard(rt *runtime, basePath string, macros *macroEngine) *VirtualSDCard {
	return &VirtualSDCard{
		rt:       rt,
		basePath: basePath,
		macros:   macros,
	}
}

// LoopDesist clears all loop state. Safe to call outside of SD card context.
func (sd *VirtualSDCard) LoopDesist() {
	sd.loopStack = nil
}

// PrintFile loads and executes a gcode file from the virtual SD card.
func (sd *VirtualSDCard) PrintFile(filename string) error {
	fullPath := filepath.Join(sd.basePath, filename)
	f, err := os.Open(fullPath)
	if err != nil {
		return fmt.Errorf("open SD card file: %w", err)
	}
	sd.file = f
	sd.reader = bufio.NewReader(f)
	sd.loopStack = nil

	defer func() {
		sd.file.Close()
		sd.file = nil
		sd.reader = nil
		sd.loopStack = nil
	}()

	return sd.runFile()
}

// runFile executes gcode from the current file position.
func (sd *VirtualSDCard) runFile() error {
	for {
		// Read line from file
		line, err := sd.reader.ReadString('\n')
		if err != nil {
			if err == io.EOF {
				// Process any remaining content without newline
				if line != "" {
					if err := sd.processLine(line); err != nil {
						return err
					}
				}
				break
			}
			return fmt.Errorf("read SD card file: %w", err)
		}

		if err := sd.processLine(line); err != nil {
			return err
		}
	}
	return nil
}

// processLine processes a single gcode line.
func (sd *VirtualSDCard) processLine(line string) error {
	// Strip comments and whitespace
	line = strings.TrimRight(line, "\r\n")
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
