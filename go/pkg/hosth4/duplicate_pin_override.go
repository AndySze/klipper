// Duplicate Pin Override - port of klippy/extras/duplicate_pin_override.py
//
// Allow duplicate pin definitions
//
// Copyright (C) 2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"log"
	"sync"
)

// DuplicatePinOverride allows duplicate pin definitions.
type DuplicatePinOverride struct {
	rt            *runtime
	pins          map[string]int // pin -> usage count
	allowedPins   map[string]bool
	mu            sync.Mutex
}

// DuplicatePinOverrideConfig holds configuration.
type DuplicatePinOverrideConfig struct {
	Pins []string // Pins allowed to be duplicated
}

// newDuplicatePinOverride creates a new duplicate pin override manager.
func newDuplicatePinOverride(rt *runtime, cfg DuplicatePinOverrideConfig) *DuplicatePinOverride {
	dpo := &DuplicatePinOverride{
		rt:          rt,
		pins:        make(map[string]int),
		allowedPins: make(map[string]bool),
	}

	for _, pin := range cfg.Pins {
		dpo.allowedPins[pin] = true
	}

	log.Printf("duplicate_pin_override: initialized with %d allowed pins", len(cfg.Pins))
	return dpo
}

// IsDuplicateAllowed checks if a pin is allowed to be duplicated.
func (dpo *DuplicatePinOverride) IsDuplicateAllowed(pin string) bool {
	dpo.mu.Lock()
	defer dpo.mu.Unlock()
	return dpo.allowedPins[pin]
}

// RegisterPin registers a pin usage.
func (dpo *DuplicatePinOverride) RegisterPin(pin string) bool {
	dpo.mu.Lock()
	defer dpo.mu.Unlock()

	dpo.pins[pin]++

	// Check if this is a duplicate
	if dpo.pins[pin] > 1 {
		if dpo.allowedPins[pin] {
			log.Printf("duplicate_pin_override: allowing duplicate use of pin %s", pin)
			return true
		}
		return false
	}
	return true
}

// GetPinUsage returns the number of times a pin has been registered.
func (dpo *DuplicatePinOverride) GetPinUsage(pin string) int {
	dpo.mu.Lock()
	defer dpo.mu.Unlock()
	return dpo.pins[pin]
}

// GetAllowedPins returns the list of pins allowed to be duplicated.
func (dpo *DuplicatePinOverride) GetAllowedPins() []string {
	dpo.mu.Lock()
	defer dpo.mu.Unlock()

	pins := make([]string, 0, len(dpo.allowedPins))
	for pin := range dpo.allowedPins {
		pins = append(pins, pin)
	}
	return pins
}

// GetStatus returns the duplicate pin override status.
func (dpo *DuplicatePinOverride) GetStatus() map[string]any {
	dpo.mu.Lock()
	defer dpo.mu.Unlock()

	duplicates := make(map[string]int)
	for pin, count := range dpo.pins {
		if count > 1 {
			duplicates[pin] = count
		}
	}

	return map[string]any{
		"allowed_pins": dpo.allowedPins,
		"duplicates":   duplicates,
	}
}
