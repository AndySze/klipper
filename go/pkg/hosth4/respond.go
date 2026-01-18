// Respond module - port of klippy/extras/respond.py
//
// Add 'RESPOND' and 'M118' commands for sending messages to the host
//
// Copyright (C) 2018 Alec Plumb <alec@etherwalker.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"strings"
)

// Response types and their prefixes
var respondTypes = map[string]string{
	"echo":    "echo:",
	"command": "//",
	"error":   "!!",
}

var respondTypesNoSpace = map[string]string{
	"echo_no_space": "echo:",
}

// HostResponder handles RESPOND and M118 commands.
type HostResponder struct {
	rt            *runtime
	defaultPrefix string
	outputFunc    func(string) // function to output messages
}

// HostResponderConfig holds configuration for the host responder.
type HostResponderConfig struct {
	DefaultType   string // "echo", "command", or "error"
	DefaultPrefix string // custom prefix (overrides DefaultType)
}

// DefaultHostResponderConfig returns the default host responder configuration.
func DefaultHostResponderConfig() HostResponderConfig {
	return HostResponderConfig{
		DefaultType:   "echo",
		DefaultPrefix: "",
	}
}

// newHostResponder creates a new host responder.
func newHostResponder(rt *runtime, cfg HostResponderConfig) *HostResponder {
	hr := &HostResponder{
		rt:         rt,
		outputFunc: func(s string) { log.Println(s) },
	}

	// Set default prefix
	if cfg.DefaultPrefix != "" {
		hr.defaultPrefix = cfg.DefaultPrefix
	} else if prefix, ok := respondTypes[cfg.DefaultType]; ok {
		hr.defaultPrefix = prefix
	} else {
		hr.defaultPrefix = respondTypes["echo"]
	}

	return hr
}

// SetOutputFunc sets the function used to output messages.
func (hr *HostResponder) SetOutputFunc(f func(string)) {
	hr.outputFunc = f
}

// cmdM118 handles the M118 command.
// M118 outputs a message with the default prefix.
func (hr *HostResponder) cmdM118(rawParams string) {
	msg := strings.TrimSpace(rawParams)
	hr.output(fmt.Sprintf("%s %s", hr.defaultPrefix, msg))
}

// cmdRespond handles the RESPOND command.
// RESPOND [TYPE=<type>] [PREFIX=<prefix>] MSG=<message>
func (hr *HostResponder) cmdRespond(args map[string]string) error {
	noSpace := false
	prefix := hr.defaultPrefix

	// Check for TYPE parameter
	if respondType, ok := args["TYPE"]; ok {
		respondType = strings.ToLower(respondType)
		if p, found := respondTypes[respondType]; found {
			prefix = p
		} else if p, found := respondTypesNoSpace[respondType]; found {
			prefix = p
			noSpace = true
		} else {
			return fmt.Errorf("RESPOND TYPE '%s' is invalid. Must be one of 'echo', 'command', or 'error'", respondType)
		}
	}

	// Check for PREFIX override
	if p, ok := args["PREFIX"]; ok {
		prefix = p
	}

	// Get message
	msg := args["MSG"]

	// Output the message
	if noSpace {
		hr.output(fmt.Sprintf("%s%s", prefix, msg))
	} else {
		hr.output(fmt.Sprintf("%s %s", prefix, msg))
	}

	return nil
}

// output sends a message through the output function.
func (hr *HostResponder) output(msg string) {
	if hr.outputFunc != nil {
		hr.outputFunc(msg)
	}
}

// RespondEcho outputs an echo message.
func (hr *HostResponder) RespondEcho(msg string) {
	hr.output(fmt.Sprintf("echo: %s", msg))
}

// RespondCommand outputs a command response.
func (hr *HostResponder) RespondCommand(msg string) {
	hr.output(fmt.Sprintf("// %s", msg))
}

// RespondError outputs an error message.
func (hr *HostResponder) RespondError(msg string) {
	hr.output(fmt.Sprintf("!! %s", msg))
}
