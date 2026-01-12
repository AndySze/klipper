package config

import (
	"strings"
)

// Pin represents a parsed pin specification.
type Pin struct {
	Name   string // Pin name (e.g., "PA5", "gpio25")
	Chip   string // MCU chip name (default: "mcu")
	Invert bool   // Inverted logic (! prefix)
	Pullup int    // Pullup: 1 = up (^), -1 = down (~), 0 = none
}

// FullName returns the full pin name including chip prefix if not "mcu".
func (p Pin) FullName() string {
	if p.Chip != "" && p.Chip != "mcu" {
		return p.Chip + ":" + p.Name
	}
	return p.Name
}

// PinOptions specifies parsing options for pin specifications.
type PinOptions struct {
	CanInvert bool // Allow ! prefix for inverted logic
	CanPullup bool // Allow ^ and ~ prefixes for pullup/pulldown
}

// ParsePin parses a pin specification string.
// Format: [chip:][[!][^|~]]pin_name
// Examples: "PA5", "!PA5", "^PA5", "mcu:PA5", "probe:z_virtual_endstop"
func ParsePin(desc string, opts PinOptions) (Pin, error) {
	d := strings.TrimSpace(desc)
	if d == "" {
		return Pin{}, NewConfigError("", "", "empty pin specification")
	}

	p := Pin{Chip: "mcu"}

	// Parse pullup prefix (^ or ~)
	if opts.CanPullup && len(d) > 0 {
		if d[0] == '^' {
			p.Pullup = 1
			d = strings.TrimSpace(d[1:])
		} else if d[0] == '~' {
			p.Pullup = -1
			d = strings.TrimSpace(d[1:])
		}
	}

	// Parse invert prefix (!)
	if opts.CanInvert && len(d) > 0 && d[0] == '!' {
		p.Invert = true
		d = strings.TrimSpace(d[1:])
	}

	// Parse chip:pin format
	if idx := strings.Index(d, ":"); idx >= 0 {
		p.Chip = strings.TrimSpace(d[:idx])
		d = strings.TrimSpace(d[idx+1:])
	}

	// Validate pin name
	if d == "" {
		return Pin{}, NewConfigError("", "", "empty pin name in specification: "+desc)
	}
	if strings.ContainsAny(d, "^~!:") {
		return Pin{}, NewConfigError("", "", "invalid characters in pin name: "+desc)
	}

	p.Name = d
	return p, nil
}

// GetPin returns a Pin option value from the section.
func (s *Section) GetPin(option string, opts PinOptions, fallback ...Pin) (Pin, error) {
	key := strings.ToLower(option)
	if v, ok := s.options[key]; ok {
		s.markAccessed(option)
		pin, err := ParsePin(v, opts)
		if err != nil {
			return Pin{}, WrapError(s.name, option, err)
		}
		return pin, nil
	}
	if len(fallback) > 0 {
		s.markAccessed(option)
		return fallback[0], nil
	}
	return Pin{}, ErrMissingOption(s.name, option)
}

// GetPinOptional returns a Pin option value, or nil if not present.
func (s *Section) GetPinOptional(option string, opts PinOptions) (*Pin, error) {
	key := strings.ToLower(option)
	if v, ok := s.options[key]; ok {
		s.markAccessed(option)
		pin, err := ParsePin(v, opts)
		if err != nil {
			return nil, WrapError(s.name, option, err)
		}
		return &pin, nil
	}
	return nil, nil
}
