// Board Pins - port of klippy/extras/board_pins.py
//
// Support for custom board pin aliases
//
// Copyright (C) 2019-2021 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"strings"
	"sync"
)

// PinResolver provides pin alias resolution and reservation.
type PinResolver struct {
	mcuName  string
	aliases  map[string]string // alias -> actual pin
	reserved map[string]string // pin -> reservation reason
	mu       sync.RWMutex
}

// newPinResolver creates a new pin resolver for an MCU.
func newPinResolver(mcuName string) *PinResolver {
	return &PinResolver{
		mcuName:  mcuName,
		aliases:  make(map[string]string),
		reserved: make(map[string]string),
	}
}

// AliasPin creates an alias for a pin.
func (pr *PinResolver) AliasPin(alias, actualPin string) error {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	if _, exists := pr.aliases[alias]; exists {
		return fmt.Errorf("pin alias '%s' already defined", alias)
	}
	pr.aliases[alias] = actualPin
	return nil
}

// ReservePin reserves a pin with a reason.
func (pr *PinResolver) ReservePin(pin, reason string) error {
	pr.mu.Lock()
	defer pr.mu.Unlock()

	if existing, exists := pr.reserved[pin]; exists {
		return fmt.Errorf("pin '%s' already reserved: %s", pin, existing)
	}
	pr.reserved[pin] = reason
	return nil
}

// ResolveAlias resolves a pin alias to actual pin name.
func (pr *PinResolver) ResolveAlias(pinName string) string {
	pr.mu.RLock()
	defer pr.mu.RUnlock()

	if actual, ok := pr.aliases[pinName]; ok {
		return actual
	}
	return pinName
}

// IsReserved checks if a pin is reserved.
func (pr *PinResolver) IsReserved(pinName string) (bool, string) {
	pr.mu.RLock()
	defer pr.mu.RUnlock()

	reason, reserved := pr.reserved[pinName]
	return reserved, reason
}

// GetAliases returns all pin aliases.
func (pr *PinResolver) GetAliases() map[string]string {
	pr.mu.RLock()
	defer pr.mu.RUnlock()

	result := make(map[string]string, len(pr.aliases))
	for k, v := range pr.aliases {
		result[k] = v
	}
	return result
}

// PrinterBoardAliases manages board pin aliases.
type PrinterBoardAliases struct {
	rt           *runtime
	mcuNames     []string
	pinResolvers map[string]*PinResolver
	mu           sync.RWMutex
}

// BoardPinsConfig holds configuration for board pins.
type BoardPinsConfig struct {
	MCUNames []string            // List of MCU names (default: ["mcu"])
	Aliases  map[string]string   // Pin alias -> actual pin mappings
}

// DefaultBoardPinsConfig returns default board pins configuration.
func DefaultBoardPinsConfig() BoardPinsConfig {
	return BoardPinsConfig{
		MCUNames: []string{"mcu"},
		Aliases:  make(map[string]string),
	}
}

// newPrinterBoardAliases creates a new board aliases manager.
func newPrinterBoardAliases(rt *runtime, cfg BoardPinsConfig) *PrinterBoardAliases {
	pba := &PrinterBoardAliases{
		rt:           rt,
		mcuNames:     cfg.MCUNames,
		pinResolvers: make(map[string]*PinResolver),
	}

	if len(pba.mcuNames) == 0 {
		pba.mcuNames = []string{"mcu"}
	}

	// Create pin resolvers for each MCU
	for _, mcuName := range pba.mcuNames {
		pba.pinResolvers[mcuName] = newPinResolver(mcuName)
	}

	// Apply aliases
	for alias, value := range cfg.Aliases {
		pba.applyAlias(alias, value)
	}

	log.Printf("board_pins: initialized for MCUs: %v", pba.mcuNames)
	return pba
}

// applyAlias applies an alias to all configured MCU pin resolvers.
func (pba *PrinterBoardAliases) applyAlias(name, value string) {
	pba.mu.Lock()
	defer pba.mu.Unlock()

	// Check if it's a reservation (value starts with '<' and ends with '>')
	if strings.HasPrefix(value, "<") && strings.HasSuffix(value, ">") {
		reason := value[1 : len(value)-1]
		for _, pr := range pba.pinResolvers {
			if err := pr.ReservePin(name, reason); err != nil {
				log.Printf("board_pins: warning: %v", err)
			}
		}
	} else {
		// Regular alias
		for _, pr := range pba.pinResolvers {
			if err := pr.AliasPin(name, value); err != nil {
				log.Printf("board_pins: warning: %v", err)
			}
		}
	}
}

// GetPinResolver returns the pin resolver for an MCU.
func (pba *PrinterBoardAliases) GetPinResolver(mcuName string) *PinResolver {
	pba.mu.RLock()
	defer pba.mu.RUnlock()
	return pba.pinResolvers[mcuName]
}

// ResolvePin resolves a pin specification to actual pin.
// Pin spec format: "mcu:pin" or just "pin" (defaults to first MCU)
func (pba *PrinterBoardAliases) ResolvePin(pinSpec string) (mcuName, pinName string, err error) {
	pba.mu.RLock()
	defer pba.mu.RUnlock()

	// Parse pin specification
	parts := strings.SplitN(pinSpec, ":", 2)
	if len(parts) == 2 {
		mcuName = parts[0]
		pinName = parts[1]
	} else {
		mcuName = pba.mcuNames[0]
		pinName = parts[0]
	}

	// Get pin resolver
	pr, ok := pba.pinResolvers[mcuName]
	if !ok {
		return "", "", fmt.Errorf("unknown MCU '%s'", mcuName)
	}

	// Check if reserved
	if reserved, reason := pr.IsReserved(pinName); reserved {
		return "", "", fmt.Errorf("pin '%s' is reserved: %s", pinName, reason)
	}

	// Resolve alias
	actualPin := pr.ResolveAlias(pinName)
	return mcuName, actualPin, nil
}

// GetStatus returns the board pins status.
func (pba *PrinterBoardAliases) GetStatus() map[string]any {
	pba.mu.RLock()
	defer pba.mu.RUnlock()

	aliases := make(map[string]map[string]string)
	for mcuName, pr := range pba.pinResolvers {
		aliases[mcuName] = pr.GetAliases()
	}

	return map[string]any{
		"mcu_names": pba.mcuNames,
		"aliases":   aliases,
	}
}

// ParseBoardPinsConfig parses board_pins section from config.
func ParseBoardPinsConfig(section map[string]string) BoardPinsConfig {
	cfg := DefaultBoardPinsConfig()

	// Parse MCU names
	if mcuList, ok := section["mcu"]; ok {
		mcus := strings.Split(mcuList, ",")
		cfg.MCUNames = make([]string, 0, len(mcus))
		for _, mcu := range mcus {
			mcu = strings.TrimSpace(mcu)
			if mcu != "" {
				cfg.MCUNames = append(cfg.MCUNames, mcu)
			}
		}
	}

	// Parse aliases (format: "alias1=pin1, alias2=pin2")
	if aliasStr, ok := section["aliases"]; ok {
		parseAliases(aliasStr, cfg.Aliases)
	}

	// Parse aliases_ prefixed options
	for key, value := range section {
		if strings.HasPrefix(key, "aliases_") {
			parseAliases(value, cfg.Aliases)
		}
	}

	return cfg
}

// parseAliases parses alias string into map.
func parseAliases(aliasStr string, aliases map[string]string) {
	// Split by comma
	pairs := strings.Split(aliasStr, ",")
	for _, pair := range pairs {
		pair = strings.TrimSpace(pair)
		if pair == "" {
			continue
		}
		// Split by '='
		kv := strings.SplitN(pair, "=", 2)
		if len(kv) != 2 {
			continue
		}
		name := strings.TrimSpace(kv[0])
		value := strings.TrimSpace(kv[1])
		if name != "" && value != "" {
			aliases[name] = value
		}
	}
}
