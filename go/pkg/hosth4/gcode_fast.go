// Fast G-code parsing with reduced allocations
//
// Provides optimized parsing for high-frequency G-code commands.
// Uses object pools and avoids string allocations where possible.
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"strconv"
	"strings"

	"klipper-go-migration/pkg/pool"
)

// GCodeCommandFast is a poolable G-code command structure
type GCodeCommandFast struct {
	Name string
	Args map[string]string
	Raw  string
}

// Reset clears the command for reuse
func (c *GCodeCommandFast) Reset() {
	c.Name = ""
	c.Raw = ""
	// Clear args map
	for k := range c.Args {
		delete(c.Args, k)
	}
}

// parseGCodeLineFast parses a G-code line with reduced allocations.
// The returned command's Args map is borrowed from a pool and must be
// returned via PutArgsMap when done.
func parseGCodeLineFast(line string) (*gcodeCommand, error) {
	// Fast path for empty lines
	ln := line
	if len(ln) == 0 {
		return nil, nil
	}

	// Trim leading whitespace manually to avoid allocation
	start := 0
	for start < len(ln) && (ln[start] == ' ' || ln[start] == '\t') {
		start++
	}
	if start == len(ln) {
		return nil, nil
	}
	ln = ln[start:]

	// Handle semicolon comments - find index without allocation
	if idx := strings.IndexByte(ln, ';'); idx >= 0 {
		ln = ln[:idx]
		// Trim trailing whitespace
		for len(ln) > 0 && (ln[len(ln)-1] == ' ' || ln[len(ln)-1] == '\t') {
			ln = ln[:len(ln)-1]
		}
	}
	if len(ln) == 0 {
		return nil, nil
	}

	// Check for parenthesis comments - only use regex if needed
	if strings.IndexByte(ln, '(') >= 0 {
		ln = strings.TrimSpace(reParenComment.ReplaceAllString(ln, " "))
		if len(ln) == 0 {
			return nil, nil
		}
	}

	// Get args map from pool
	args := pool.GetArgsMap()

	// Parse manually to reduce allocations
	// Find first field (command name)
	end := 0
	for end < len(ln) && ln[end] != ' ' && ln[end] != '\t' {
		end++
	}
	if end == 0 {
		pool.PutArgsMap(args)
		return nil, nil
	}

	name := strings.ToUpper(ln[:end])

	// Parse remaining fields
	pos := end
	for pos < len(ln) {
		// Skip whitespace
		for pos < len(ln) && (ln[pos] == ' ' || ln[pos] == '\t') {
			pos++
		}
		if pos >= len(ln) {
			break
		}

		// Find end of field
		fieldStart := pos
		for pos < len(ln) && ln[pos] != ' ' && ln[pos] != '\t' {
			pos++
		}
		field := ln[fieldStart:pos]

		if len(field) == 0 {
			continue
		}

		// Parse key=value or Xvalue format
		if eqIdx := strings.IndexByte(field, '='); eqIdx >= 0 {
			// key=value format
			k := strings.ToUpper(field[:eqIdx])
			v := ""
			if eqIdx+1 < len(field) {
				v = field[eqIdx+1:]
			}
			if k != "" {
				args[k] = v
			}
		} else if len(field) >= 2 {
			// Xvalue format (e.g., X100)
			k := strings.ToUpper(field[:1])
			v := field[1:]
			if k != "" {
				args[k] = v
			}
		}
	}

	return &gcodeCommand{Name: name, Args: args, Raw: line}, nil
}

// ReleaseGCodeCommand releases the args map back to the pool
func ReleaseGCodeCommand(cmd *gcodeCommand) {
	if cmd != nil && cmd.Args != nil {
		pool.PutArgsMap(cmd.Args)
		cmd.Args = nil
	}
}

// floatArgFast is an optimized version of floatArg
// that avoids the strings.ToUpper call when key is already uppercase
func floatArgFast(args map[string]string, key string, def float64) (float64, error) {
	raw, ok := args[key]
	if !ok {
		return def, nil
	}
	if raw == "" {
		return def, nil
	}
	f, err := strconv.ParseFloat(raw, 64)
	if err != nil {
		return def, err
	}
	return f, nil
}

// intArgFast gets an integer argument
func intArgFast(args map[string]string, key string, def int) (int, error) {
	raw, ok := args[key]
	if !ok {
		return def, nil
	}
	if raw == "" {
		return def, nil
	}
	i, err := strconv.Atoi(raw)
	if err != nil {
		return def, err
	}
	return i, nil
}

// hasArg checks if an argument exists
func hasArg(args map[string]string, key string) bool {
	_, ok := args[key]
	return ok
}

// stringArg gets a string argument
func stringArg(args map[string]string, key string, def string) string {
	if v, ok := args[key]; ok {
		return v
	}
	return def
}

// FastG1Parser is specialized for parsing G1 commands
// which are by far the most frequent during printing
type FastG1Parser struct {
	// Cached results to avoid allocations
	hasX, hasY, hasZ, hasE, hasF bool
	x, y, z, e, f                float64
}

// Parse parses a G1 command line directly without creating intermediate map
// Returns false if not a G1 command
func (p *FastG1Parser) Parse(line string) bool {
	// Reset
	p.hasX, p.hasY, p.hasZ, p.hasE, p.hasF = false, false, false, false, false

	// Quick check for G1 or G0
	ln := line
	// Skip leading whitespace
	for len(ln) > 0 && (ln[0] == ' ' || ln[0] == '\t') {
		ln = ln[1:]
	}
	if len(ln) < 2 {
		return false
	}

	// Check for G0 or G1
	if (ln[0] != 'G' && ln[0] != 'g') || (ln[1] != '0' && ln[1] != '1') {
		return false
	}

	// Ensure it's actually G0 or G1 (not G10, G28, etc.)
	if len(ln) > 2 && ln[2] != ' ' && ln[2] != '\t' && ln[2] != ';' {
		return false
	}

	// Parse parameters
	pos := 2
	for pos < len(ln) {
		// Skip whitespace
		for pos < len(ln) && (ln[pos] == ' ' || ln[pos] == '\t') {
			pos++
		}
		if pos >= len(ln) || ln[pos] == ';' {
			break
		}

		key := ln[pos]
		pos++

		// Find value end
		start := pos
		for pos < len(ln) && ln[pos] != ' ' && ln[pos] != '\t' && ln[pos] != ';' {
			pos++
		}

		if pos > start {
			val := ln[start:pos]
			switch key {
			case 'X', 'x':
				if v, err := strconv.ParseFloat(val, 64); err == nil {
					p.x = v
					p.hasX = true
				}
			case 'Y', 'y':
				if v, err := strconv.ParseFloat(val, 64); err == nil {
					p.y = v
					p.hasY = true
				}
			case 'Z', 'z':
				if v, err := strconv.ParseFloat(val, 64); err == nil {
					p.z = v
					p.hasZ = true
				}
			case 'E', 'e':
				if v, err := strconv.ParseFloat(val, 64); err == nil {
					p.e = v
					p.hasE = true
				}
			case 'F', 'f':
				if v, err := strconv.ParseFloat(val, 64); err == nil {
					p.f = v
					p.hasF = true
				}
			}
		}
	}

	return true
}

// X returns X coordinate if present
func (p *FastG1Parser) X() (float64, bool) { return p.x, p.hasX }

// Y returns Y coordinate if present
func (p *FastG1Parser) Y() (float64, bool) { return p.y, p.hasY }

// Z returns Z coordinate if present
func (p *FastG1Parser) Z() (float64, bool) { return p.z, p.hasZ }

// E returns E coordinate if present
func (p *FastG1Parser) E() (float64, bool) { return p.e, p.hasE }

// F returns feedrate if present
func (p *FastG1Parser) F() (float64, bool) { return p.f, p.hasF }
