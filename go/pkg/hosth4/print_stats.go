// Print statistics tracking - port of klippy/extras/print_stats.py
//
// Copyright (C) 2020 Eric Callahan <arksine.code@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"strconv"
	"time"
)

// PrintState represents the current state of a print job.
type PrintState string

const (
	PrintStateStandby   PrintState = "standby"
	PrintStatePrinting  PrintState = "printing"
	PrintStatePaused    PrintState = "paused"
	PrintStateComplete  PrintState = "complete"
	PrintStateCancelled PrintState = "cancelled"
	PrintStateError     PrintState = "error"
)

// PrintStats tracks print job statistics compatible with Moonraker.
type PrintStats struct {
	rt *runtime
	gm *gcodeMove

	// Current print state
	filename      string
	state         PrintState
	errorMessage  string
	totalDuration float64
	printDuration float64
	filamentUsed  float64

	// Internal timing
	printStartTime    *time.Time
	lastPauseTime     *time.Time
	prevPauseDuration float64
	initDuration      float64
	lastEPos          float64

	// Layer tracking
	infoTotalLayer   *int
	infoCurrentLayer *int
}

// newPrintStats creates a new print statistics tracker.
func newPrintStats(rt *runtime, gm *gcodeMove) *PrintStats {
	ps := &PrintStats{
		rt: rt,
		gm: gm,
	}
	ps.reset()
	return ps
}

// reset clears all print statistics.
func (ps *PrintStats) reset() {
	ps.filename = ""
	ps.state = PrintStateStandby
	ps.errorMessage = ""
	ps.totalDuration = 0
	ps.printDuration = 0
	ps.filamentUsed = 0
	ps.printStartTime = nil
	ps.lastPauseTime = nil
	ps.prevPauseDuration = 0
	ps.initDuration = 0
	ps.lastEPos = 0
	ps.infoTotalLayer = nil
	ps.infoCurrentLayer = nil
}

// setCurrentFile sets the current filename and resets statistics.
func (ps *PrintStats) setCurrentFile(filename string) {
	ps.reset()
	ps.filename = filename
}

// noteStart marks the start (or resume) of printing.
func (ps *PrintStats) noteStart() {
	now := time.Now()
	if ps.printStartTime == nil {
		ps.printStartTime = &now
	} else if ps.lastPauseTime != nil {
		// Resume from pause - update pause duration
		pauseDuration := now.Sub(*ps.lastPauseTime).Seconds()
		ps.prevPauseDuration += pauseDuration
		ps.lastPauseTime = nil
	}
	// Reset last e-position
	if ps.gm != nil {
		ps.lastEPos = ps.gm.lastPosition[3]
	}
	ps.state = PrintStatePrinting
	ps.errorMessage = ""
}

// notePause marks a pause in printing.
func (ps *PrintStats) notePause() {
	if ps.lastPauseTime == nil {
		now := time.Now()
		ps.lastPauseTime = &now
		// Update filament usage
		ps.updateFilamentUsage()
	}
	if ps.state != PrintStateError {
		ps.state = PrintStatePaused
	}
}

// noteComplete marks successful completion.
func (ps *PrintStats) noteComplete() {
	ps.noteFinish(PrintStateComplete, "")
}

// noteError marks an error during printing.
func (ps *PrintStats) noteError(message string) {
	ps.noteFinish(PrintStateError, message)
}

// noteCancel marks a cancelled print.
func (ps *PrintStats) noteCancel() {
	ps.noteFinish(PrintStateCancelled, "")
}

// noteFinish handles print job completion.
func (ps *PrintStats) noteFinish(state PrintState, errorMessage string) {
	if ps.printStartTime == nil {
		return
	}
	ps.state = state
	ps.errorMessage = errorMessage
	now := time.Now()
	ps.totalDuration = now.Sub(*ps.printStartTime).Seconds()
	if ps.filamentUsed < 0.0000001 {
		// No positive extrusion detected during print
		ps.initDuration = ps.totalDuration - ps.prevPauseDuration
	}
	ps.printStartTime = nil
}

// updateFilamentUsage updates the filament used counter.
func (ps *PrintStats) updateFilamentUsage() {
	if ps.gm == nil {
		return
	}
	curEPos := ps.gm.lastPosition[3]
	// Assume extrude_factor = 1.0 for now (could be extended later)
	ps.filamentUsed += curEPos - ps.lastEPos
	ps.lastEPos = curEPos
}

// GetStatus returns the current print statistics for status queries.
func (ps *PrintStats) GetStatus() map[string]interface{} {
	timePaused := ps.prevPauseDuration
	now := time.Now()

	if ps.printStartTime != nil {
		if ps.lastPauseTime != nil {
			// Calculate total time spent paused during print
			timePaused += now.Sub(*ps.lastPauseTime).Seconds()
		} else {
			// Accumulate filament if not paused
			ps.updateFilamentUsage()
		}
		ps.totalDuration = now.Sub(*ps.printStartTime).Seconds()
		if ps.filamentUsed < 0.0000001 {
			// Track duration prior to extrusion
			ps.initDuration = ps.totalDuration - timePaused
		}
	}
	printDuration := ps.totalDuration - ps.initDuration - timePaused

	info := map[string]interface{}{
		"total_layer":   nil,
		"current_layer": nil,
	}
	if ps.infoTotalLayer != nil {
		info["total_layer"] = *ps.infoTotalLayer
	}
	if ps.infoCurrentLayer != nil {
		info["current_layer"] = *ps.infoCurrentLayer
	}

	return map[string]interface{}{
		"filename":       ps.filename,
		"total_duration": ps.totalDuration,
		"print_duration": printDuration,
		"filament_used":  ps.filamentUsed,
		"state":          string(ps.state),
		"message":        ps.errorMessage,
		"info":           info,
	}
}

// cmdSetPrintStatsInfo handles the SET_PRINT_STATS_INFO command.
func (ps *PrintStats) cmdSetPrintStatsInfo(args map[string]string) error {
	if totalLayerStr, ok := args["TOTAL_LAYER"]; ok {
		totalLayer, err := strconv.Atoi(totalLayerStr)
		if err != nil {
			return err
		}
		if totalLayer == 0 {
			ps.infoTotalLayer = nil
			ps.infoCurrentLayer = nil
		} else if ps.infoTotalLayer == nil || totalLayer != *ps.infoTotalLayer {
			ps.infoTotalLayer = &totalLayer
			zero := 0
			ps.infoCurrentLayer = &zero
		}
	}

	if currentLayerStr, ok := args["CURRENT_LAYER"]; ok {
		currentLayer, err := strconv.Atoi(currentLayerStr)
		if err != nil {
			return err
		}
		if ps.infoTotalLayer != nil && currentLayer != *ps.infoCurrentLayer {
			minLayer := currentLayer
			if minLayer > *ps.infoTotalLayer {
				minLayer = *ps.infoTotalLayer
			}
			ps.infoCurrentLayer = &minLayer
		}
	}
	return nil
}
