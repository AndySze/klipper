// Display status - port of klippy/extras/display_status.py
//
// Module to handle M73 and M117 display status commands
//
// Copyright (C) 2018-2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2018 Eric Callahan <arksine.code@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"strconv"
	"strings"
	"time"
)

const (
	m73Timeout = 5.0 // seconds before M73 progress expires
)

// DisplayStatus manages display status for the printer.
type DisplayStatus struct {
	rt *runtime

	progress       *float64  // nil if not set via M73
	expireProgress time.Time // when M73 progress expires
	message        *string   // nil if no message set
}

// newDisplayStatus creates a new display status manager.
func newDisplayStatus(rt *runtime) *DisplayStatus {
	return &DisplayStatus{
		rt: rt,
	}
}

// GetStatus returns the current display status.
func (ds *DisplayStatus) GetStatus() map[string]any {
	progress := ds.getProgress()
	return map[string]any{
		"progress": progress,
		"message":  ds.getMessage(),
	}
}

// getProgress returns the current progress value (0.0 to 1.0).
func (ds *DisplayStatus) getProgress() float64 {
	// If M73 progress is set and not expired, use it
	if ds.progress != nil {
		if time.Now().Before(ds.expireProgress) {
			return *ds.progress
		}
		// Progress expired - clear it
		ds.progress = nil
	}

	// Fall back to virtual_sdcard progress
	if ds.progress == nil {
		if ds.rt != nil && ds.rt.sdcard != nil {
			sdStatus := ds.rt.sdcard.GetStatus()
			if p, ok := sdStatus["progress"].(float64); ok {
				return p
			}
		}
		return 0.0
	}

	return *ds.progress
}

// getMessage returns the current display message.
func (ds *DisplayStatus) getMessage() *string {
	return ds.message
}

// cmdM73 handles the M73 progress command.
// M73 P<percent> - Set print progress percentage
func (ds *DisplayStatus) cmdM73(args map[string]string) {
	if pStr, ok := args["P"]; ok {
		p, err := strconv.ParseFloat(pStr, 64)
		if err == nil {
			// Convert percentage (0-100) to fraction (0.0-1.0)
			progress := p / 100.0
			// Clamp to valid range
			if progress < 0 {
				progress = 0
			} else if progress > 1 {
				progress = 1
			}
			ds.progress = &progress
			ds.expireProgress = time.Now().Add(time.Duration(m73Timeout * float64(time.Second)))
		}
	}
}

// cmdM117 handles the M117 display message command.
// M117 <message> - Set display message
func (ds *DisplayStatus) cmdM117(rawParams string) {
	msg := strings.TrimSpace(rawParams)
	if msg == "" {
		ds.message = nil
	} else {
		ds.message = &msg
	}
}

// cmdSetDisplayText handles the SET_DISPLAY_TEXT command.
// SET_DISPLAY_TEXT [MSG=<message>]
func (ds *DisplayStatus) cmdSetDisplayText(args map[string]string) {
	if msg, ok := args["MSG"]; ok && msg != "" {
		ds.message = &msg
	} else {
		ds.message = nil
	}
}

// SetProgress sets the progress value directly (0.0 to 1.0).
func (ds *DisplayStatus) SetProgress(progress float64) {
	if progress < 0 {
		progress = 0
	} else if progress > 1 {
		progress = 1
	}
	ds.progress = &progress
	ds.expireProgress = time.Now().Add(time.Duration(m73Timeout * float64(time.Second)))
}

// ClearProgress clears the M73 progress value.
func (ds *DisplayStatus) ClearProgress() {
	ds.progress = nil
}

// SetMessage sets the display message.
func (ds *DisplayStatus) SetMessage(msg string) {
	if msg == "" {
		ds.message = nil
	} else {
		ds.message = &msg
	}
}

// ClearMessage clears the display message.
func (ds *DisplayStatus) ClearMessage() {
	ds.message = nil
}
