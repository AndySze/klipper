// Sdcard loop - port of klippy/extras/sdcard_loop.py
//
// Sdcard file looping support for belt printers
//
// Copyright (C) 2021 Jason S. McMullan <jason.mcmullan@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"strconv"
)

// loopEntry represents a single loop on the stack.
type loopEntry struct {
	count    int   // remaining iterations (0 = infinite)
	position int64 // file position to return to
}

// SDCardLoop manages sdcard file looping.
type SDCardLoop struct {
	rt        *runtime
	sdcard    *VirtualSDCard
	loopStack []loopEntry
}

// newSDCardLoop creates a new sdcard loop manager.
func newSDCardLoop(rt *runtime, sdcard *VirtualSDCard) *SDCardLoop {
	return &SDCardLoop{
		rt:        rt,
		sdcard:    sdcard,
		loopStack: make([]loopEntry, 0),
	}
}

// cmdSDCardLoopBegin handles the SDCARD_LOOP_BEGIN command.
// SDCARD_LOOP_BEGIN COUNT=<n>
// Begins a looped section. COUNT=0 means infinite loop.
func (sl *SDCardLoop) cmdSDCardLoopBegin(args map[string]string) error {
	countStr, ok := args["COUNT"]
	if !ok {
		return fmt.Errorf("missing COUNT parameter")
	}

	count, err := strconv.Atoi(countStr)
	if err != nil {
		return fmt.Errorf("invalid COUNT value: %w", err)
	}
	if count < 0 {
		return fmt.Errorf("COUNT must be >= 0")
	}

	if !sl.loopBegin(count) {
		return fmt.Errorf("only permitted in SD file")
	}
	return nil
}

// cmdSDCardLoopEnd handles the SDCARD_LOOP_END command.
// Ends a looped section.
func (sl *SDCardLoop) cmdSDCardLoopEnd() error {
	if !sl.loopEnd() {
		return fmt.Errorf("only permitted in SD file")
	}
	return nil
}

// cmdSDCardLoopDesist handles the SDCARD_LOOP_DESIST command.
// Stops iterating the current loop stack.
func (sl *SDCardLoop) cmdSDCardLoopDesist() error {
	if !sl.loopDesist() {
		return fmt.Errorf("only permitted outside of a SD file")
	}
	return nil
}

// loopBegin starts a new loop.
func (sl *SDCardLoop) loopBegin(count int) bool {
	if sl.sdcard == nil || !sl.sdcard.IsCmdFromSD() {
		// Can only run inside of an SD file
		return false
	}

	position := sl.sdcard.GetFilePosition()
	sl.loopStack = append(sl.loopStack, loopEntry{
		count:    count,
		position: position,
	})
	return true
}

// loopEnd ends the current loop iteration.
func (sl *SDCardLoop) loopEnd() bool {
	if sl.sdcard == nil || !sl.sdcard.IsCmdFromSD() {
		// Can only run inside of an SD file
		return false
	}

	// If the stack is empty, no need to skip back
	if len(sl.loopStack) == 0 {
		return true
	}

	// Get iteration count and return position
	idx := len(sl.loopStack) - 1
	entry := sl.loopStack[idx]
	sl.loopStack = sl.loopStack[:idx] // pop

	if entry.count == 0 {
		// Infinite loop - always jump back
		sl.sdcard.SetFilePosition(entry.position)
		sl.loopStack = append(sl.loopStack, loopEntry{
			count:    0,
			position: entry.position,
		})
	} else if entry.count == 1 {
		// Last repeat - nothing to do
		// Just let execution continue
	} else {
		// Decrement count and jump back
		sl.sdcard.SetFilePosition(entry.position)
		sl.loopStack = append(sl.loopStack, loopEntry{
			count:    entry.count - 1,
			position: entry.position,
		})
	}

	return true
}

// loopDesist clears the loop stack (only allowed outside SD printing).
func (sl *SDCardLoop) loopDesist() bool {
	if sl.sdcard != nil && sl.sdcard.IsCmdFromSD() {
		// Can only run outside of an SD file
		return false
	}

	log.Println("sdcard_loop: Desisting existing SD loops")
	sl.loopStack = make([]loopEntry, 0)
	return true
}

// GetLoopDepth returns the current loop nesting depth.
func (sl *SDCardLoop) GetLoopDepth() int {
	return len(sl.loopStack)
}

// IsInInfiniteLoop returns true if currently in an infinite loop.
func (sl *SDCardLoop) IsInInfiniteLoop() bool {
	for _, entry := range sl.loopStack {
		if entry.count == 0 {
			return true
		}
	}
	return false
}

// GetStatus returns the sdcard loop status.
func (sl *SDCardLoop) GetStatus() map[string]any {
	return map[string]any{
		"loop_depth":       len(sl.loopStack),
		"in_infinite_loop": sl.IsInInfiniteLoop(),
	}
}
