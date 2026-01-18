// Pause/Resume functionality - port of klippy/extras/pause_resume.py
//
// Copyright (C) 2019 Eric Callahan <arksine.code@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"strconv"
)

// PauseResume handles pausing and resuming print jobs.
type PauseResume struct {
	rt              *runtime
	recoverVelocity float64

	isPaused          bool
	sdPaused          bool
	pauseCommandSent  bool
	savedGCodeState   string // name of saved gcode state
}

// newPauseResume creates a new pause/resume handler.
func newPauseResume(rt *runtime, recoverVelocity float64) *PauseResume {
	if recoverVelocity <= 0 {
		recoverVelocity = 50.0 // default
	}
	return &PauseResume{
		rt:              rt,
		recoverVelocity: recoverVelocity,
		savedGCodeState: "PAUSE_STATE",
	}
}

// GetStatus returns the current pause state.
func (pr *PauseResume) GetStatus() map[string]interface{} {
	return map[string]interface{}{
		"is_paused": pr.isPaused,
	}
}

// isSDActive checks if virtual_sdcard is actively printing.
func (pr *PauseResume) isSDActive() bool {
	if pr.rt.sdcard == nil {
		return false
	}
	return pr.rt.sdcard.IsActive()
}

// sendPauseCommand sends the appropriate pause command.
func (pr *PauseResume) sendPauseCommand() {
	if pr.pauseCommandSent {
		return
	}
	if pr.isSDActive() {
		// Printing from virtual sd, pause it
		pr.sdPaused = true
		pr.rt.sdcard.DoPause()
	} else {
		pr.sdPaused = false
		// For non-SD prints, send action:paused to host
		// (Moonraker handles this via webhooks)
	}
	pr.pauseCommandSent = true
}

// sendResumeCommand sends the appropriate resume command.
func (pr *PauseResume) sendResumeCommand() {
	if pr.sdPaused {
		// Resume virtual sd printing
		pr.rt.sdcard.DoResume()
		pr.sdPaused = false
	}
	pr.pauseCommandSent = false
}

// cmdPause handles the PAUSE command.
func (pr *PauseResume) cmdPause(args map[string]string) error {
	if pr.isPaused {
		// Print already paused
		return nil
	}
	pr.sendPauseCommand()
	// Save current gcode state
	if pr.rt.gm != nil {
		pr.rt.gm.saveState(pr.savedGCodeState)
	}
	pr.isPaused = true
	// Update print_stats
	if pr.rt.printStats != nil {
		pr.rt.printStats.notePause()
	}
	return nil
}

// cmdResume handles the RESUME command.
func (pr *PauseResume) cmdResume(args map[string]string) error {
	if !pr.isPaused {
		// Print is not paused
		return nil
	}
	velocity := pr.recoverVelocity
	if v, ok := args["VELOCITY"]; ok {
		vel, err := strconv.ParseFloat(v, 64)
		if err == nil && vel > 0 {
			velocity = vel
		}
	}
	// Restore gcode state with movement
	if pr.rt.gm != nil {
		if err := pr.rt.gm.restoreState(pr.savedGCodeState, true, velocity); err != nil {
			return fmt.Errorf("failed to restore gcode state: %w", err)
		}
	}
	pr.sendResumeCommand()
	pr.isPaused = false
	// Update print_stats
	if pr.rt.printStats != nil {
		pr.rt.printStats.noteStart()
	}
	return nil
}

// cmdClearPause handles the CLEAR_PAUSE command.
func (pr *PauseResume) cmdClearPause(args map[string]string) error {
	pr.isPaused = false
	pr.pauseCommandSent = false
	return nil
}

// cmdCancelPrint handles the CANCEL_PRINT command.
func (pr *PauseResume) cmdCancelPrint(args map[string]string) error {
	if pr.isSDActive() || pr.sdPaused {
		pr.rt.sdcard.DoCancel()
	}
	pr.isPaused = false
	pr.pauseCommandSent = false
	// Update print_stats
	if pr.rt.printStats != nil {
		pr.rt.printStats.noteCancel()
	}
	return nil
}
