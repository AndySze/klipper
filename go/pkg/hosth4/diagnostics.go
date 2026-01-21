// Diagnostics commands for Klipper Go migration
//
// Provides debugging and status commands:
// - DEBUG_READ: Read MCU memory/registers
// - DEBUG_WRITE: Write MCU memory/registers
// - STATUS: System status report
// - HELP: List available commands
// - GET_UPTIME: Report host uptime
// - M115: Firmware info
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	goruntime "runtime"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"

	"klipper-go-migration/pkg/log"
)

// CommandManager provides debugging and status commands.
// It manages G-code command registration and the HELP/STATUS commands.
type CommandManager struct {
	rt *runtime

	// Registered command handlers and help text
	commands     map[string]CommandHandler
	commandHelp  map[string]string
	commandOrder []string // For stable ordering in HELP output

	// Host state
	startTime   time.Time
	isReady     bool
	stateMsg    string
	printerName string

	// Logger
	logger *log.Logger

	mu sync.RWMutex
}

// CommandHandler is a function that handles a G-code command.
type CommandHandler func(args map[string]string) (string, error)

// newCommandManager creates a new command manager.
func newCommandManager(rt *runtime) *CommandManager {
	cm := &CommandManager{
		rt:          rt,
		commands:    make(map[string]CommandHandler),
		commandHelp: make(map[string]string),
		startTime:   time.Now(),
		isReady:     true,
		stateMsg:    "Ready",
		printerName: "Klipper Go Host",
		logger:      log.GetLogger("commands"),
	}

	// Register built-in commands
	cm.registerBuiltinCommands()

	return cm
}

// registerBuiltinCommands registers the built-in diagnostic commands.
func (cm *CommandManager) registerBuiltinCommands() {
	cm.RegisterCommand("STATUS", cm.cmdStatus, "Report the printer status")
	cm.RegisterCommand("HELP", cm.cmdHelp, "Report available extended G-Code commands")
	cm.RegisterCommand("M115", cm.cmdM115, "Report firmware info")
	cm.RegisterCommand("GET_UPTIME", cm.cmdGetUptime, "Report host uptime")
	cm.RegisterCommand("DEBUG_READ", cm.cmdDebugRead, "Read from MCU memory/register (CHIP=<name> ADDR=<address>)")
	cm.RegisterCommand("DEBUG_WRITE", cm.cmdDebugWrite, "Write to MCU memory/register (CHIP=<name> ADDR=<address> VAL=<value>)")
	cm.RegisterCommand("DEBUG_STATS", cm.cmdDebugStats, "Report internal statistics")
	cm.RegisterCommand("DEBUG_TIMING", cm.cmdDebugTiming, "Report timing statistics")
}

// RegisterCommand registers a command handler with help text.
func (cm *CommandManager) RegisterCommand(name string, handler CommandHandler, help string) {
	cm.mu.Lock()
	defer cm.mu.Unlock()

	name = strings.ToUpper(name)
	cm.commands[name] = handler
	cm.commandHelp[name] = help
	cm.commandOrder = append(cm.commandOrder, name)
	sort.Strings(cm.commandOrder)
}

// UnregisterCommand removes a command handler.
func (cm *CommandManager) UnregisterCommand(name string) {
	cm.mu.Lock()
	defer cm.mu.Unlock()

	name = strings.ToUpper(name)
	delete(cm.commands, name)
	delete(cm.commandHelp, name)

	// Remove from order
	for i, n := range cm.commandOrder {
		if n == name {
			cm.commandOrder = append(cm.commandOrder[:i], cm.commandOrder[i+1:]...)
			break
		}
	}
}

// HandleCommand executes a command if registered.
func (cm *CommandManager) HandleCommand(name string, args map[string]string) (string, error, bool) {
	cm.mu.RLock()
	handler, ok := cm.commands[strings.ToUpper(name)]
	cm.mu.RUnlock()

	if !ok {
		return "", nil, false
	}

	result, err := handler(args)
	return result, err, true
}

// SetReady sets the printer ready state.
func (cm *CommandManager) SetReady(ready bool, msg string) {
	cm.mu.Lock()
	defer cm.mu.Unlock()

	cm.isReady = ready
	cm.stateMsg = msg
}

// SetPrinterName sets the printer name for M115.
func (cm *CommandManager) SetPrinterName(name string) {
	cm.mu.Lock()
	defer cm.mu.Unlock()

	cm.printerName = name
}

// cmdStatus handles the STATUS command.
func (cm *CommandManager) cmdStatus(args map[string]string) (string, error) {
	cm.mu.RLock()
	defer cm.mu.RUnlock()

	if cm.isReady {
		return "Klipper state: Ready", nil
	}

	return fmt.Sprintf("%s\nKlipper state: Not ready", cm.stateMsg), nil
}

// cmdHelp handles the HELP command.
func (cm *CommandManager) cmdHelp(args map[string]string) (string, error) {
	cm.mu.RLock()
	defer cm.mu.RUnlock()

	var lines []string
	if !cm.isReady {
		lines = append(lines, "Printer is not ready - not all commands available.")
	}
	lines = append(lines, "Available extended commands:")

	for _, name := range cm.commandOrder {
		help := cm.commandHelp[name]
		lines = append(lines, fmt.Sprintf("  %-20s: %s", name, help))
	}

	return strings.Join(lines, "\n"), nil
}

// cmdM115 handles the M115 command (firmware info).
func (cm *CommandManager) cmdM115(args map[string]string) (string, error) {
	cm.mu.RLock()
	printerName := cm.printerName
	cm.mu.RUnlock()

	uptime := time.Since(cm.startTime)

	info := []string{
		fmt.Sprintf("FIRMWARE_NAME:%s", printerName),
		"FIRMWARE_VERSION:Go-1.0",
		"PROTOCOL_VERSION:1.0",
		fmt.Sprintf("MACHINE_TYPE:%s", printerName),
		"EXTRUDER_COUNT:1",
		fmt.Sprintf("UPTIME:%.0f", uptime.Seconds()),
	}

	return strings.Join(info, " "), nil
}

// cmdGetUptime handles the GET_UPTIME command.
func (cm *CommandManager) cmdGetUptime(args map[string]string) (string, error) {
	uptime := time.Since(cm.startTime)

	hours := int(uptime.Hours())
	mins := int(uptime.Minutes()) % 60
	secs := int(uptime.Seconds()) % 60

	return fmt.Sprintf("Host uptime: %dh %dm %ds (%.0f seconds total)",
		hours, mins, secs, uptime.Seconds()), nil
}

// cmdDebugRead handles the DEBUG_READ command.
// Usage: DEBUG_READ CHIP=<name> ADDR=<address> [SIZE=<bytes>]
func (cm *CommandManager) cmdDebugRead(args map[string]string) (string, error) {
	chip := args["CHIP"]
	if chip == "" {
		chip = "mcu" // default to main MCU
	}

	addrStr := args["ADDR"]
	if addrStr == "" {
		return "", fmt.Errorf("DEBUG_READ requires ADDR parameter")
	}

	// Parse address (support hex with 0x prefix)
	addr, err := parseIntAuto(addrStr)
	if err != nil {
		return "", fmt.Errorf("invalid ADDR value: %s", addrStr)
	}

	// Parse optional size
	size := 4 // default to 4 bytes
	if sizeStr := args["SIZE"]; sizeStr != "" {
		size, err = parseIntAuto(sizeStr)
		if err != nil {
			return "", fmt.Errorf("invalid SIZE value: %s", sizeStr)
		}
		if size < 1 || size > 256 {
			return "", fmt.Errorf("SIZE must be between 1 and 256")
		}
	}

	cm.logger.WithFields(log.Fields{
		"chip": chip,
		"addr": fmt.Sprintf("0x%X", addr),
		"size": size,
	}).Debug("DEBUG_READ requested")

	// In file output mode, we can't actually read MCU memory
	// Return a placeholder response
	if cm.rt == nil || cm.rt.sq == nil {
		return fmt.Sprintf("DEBUG_READ: Chip=%s Addr=0x%08X Size=%d (file output mode - no actual read)",
			chip, addr, size), nil
	}

	// For live mode, this would send a debug_read command to the MCU
	// and wait for the response. For now, return a simulated response.
	return fmt.Sprintf("DEBUG_READ: Chip=%s Addr=0x%08X Size=%d (live mode not implemented)",
		chip, addr, size), nil
}

// cmdDebugWrite handles the DEBUG_WRITE command.
// Usage: DEBUG_WRITE CHIP=<name> ADDR=<address> VAL=<value>
func (cm *CommandManager) cmdDebugWrite(args map[string]string) (string, error) {
	chip := args["CHIP"]
	if chip == "" {
		chip = "mcu" // default to main MCU
	}

	addrStr := args["ADDR"]
	if addrStr == "" {
		return "", fmt.Errorf("DEBUG_WRITE requires ADDR parameter")
	}

	valStr := args["VAL"]
	if valStr == "" {
		return "", fmt.Errorf("DEBUG_WRITE requires VAL parameter")
	}

	// Parse address
	addr, err := parseIntAuto(addrStr)
	if err != nil {
		return "", fmt.Errorf("invalid ADDR value: %s", addrStr)
	}

	// Parse value
	val, err := parseIntAuto(valStr)
	if err != nil {
		return "", fmt.Errorf("invalid VAL value: %s", valStr)
	}

	cm.logger.WithFields(log.Fields{
		"chip": chip,
		"addr": fmt.Sprintf("0x%X", addr),
		"val":  fmt.Sprintf("0x%X", val),
	}).Debug("DEBUG_WRITE requested")

	// In file output mode, we can't actually write MCU memory
	if cm.rt == nil || cm.rt.sq == nil {
		return fmt.Sprintf("DEBUG_WRITE: Chip=%s Addr=0x%08X Val=0x%08X (file output mode - no actual write)",
			chip, addr, val), nil
	}

	// For live mode, this would send a debug_write command to the MCU
	return fmt.Sprintf("DEBUG_WRITE: Chip=%s Addr=0x%08X Val=0x%08X (live mode not implemented)",
		chip, addr, val), nil
}

// cmdDebugStats handles the DEBUG_STATS command.
func (cm *CommandManager) cmdDebugStats(args map[string]string) (string, error) {
	var m goruntime.MemStats
	goruntime.ReadMemStats(&m)

	lines := []string{
		"========== Debug Statistics ==========",
		fmt.Sprintf("Go routines:     %d", goruntime.NumGoroutine()),
		fmt.Sprintf("Memory alloc:    %.2f MB", float64(m.Alloc)/1024/1024),
		fmt.Sprintf("Total alloc:     %.2f MB", float64(m.TotalAlloc)/1024/1024),
		fmt.Sprintf("Sys memory:      %.2f MB", float64(m.Sys)/1024/1024),
		fmt.Sprintf("GC cycles:       %d", m.NumGC),
		fmt.Sprintf("Heap objects:    %d", m.HeapObjects),
		fmt.Sprintf("Host uptime:     %.0f seconds", time.Since(cm.startTime).Seconds()),
	}

	// Add runtime info if available
	if cm.rt != nil {
		if cm.rt.toolhead != nil && len(cm.rt.toolhead.commandedPos) >= 3 {
			lines = append(lines, fmt.Sprintf("Toolhead pos:    (%.3f, %.3f, %.3f)",
				cm.rt.toolhead.commandedPos[0], cm.rt.toolhead.commandedPos[1], cm.rt.toolhead.commandedPos[2]))
		}
		lines = append(lines, fmt.Sprintf("MCU freq:        %.0f Hz", cm.rt.mcuFreq))
	}

	cm.logger.Debug("DEBUG_STATS reported")

	return strings.Join(lines, "\n"), nil
}

// cmdDebugTiming handles the DEBUG_TIMING command.
func (cm *CommandManager) cmdDebugTiming(args map[string]string) (string, error) {
	lines := []string{
		"========== Timing Statistics ==========",
	}

	if cm.rt != nil {
		// Motion stats
		if cm.rt.toolhead != nil {
			lines = append(lines,
				fmt.Sprintf("Toolhead print time: %.6f", cm.rt.toolhead.printTime),
			)
		}

		// MCU frequency
		lines = append(lines,
			fmt.Sprintf("MCU frequency:       %.0f Hz", cm.rt.mcuFreq),
		)

		// Motion queue stats
		if cm.rt.motion != nil {
			lines = append(lines,
				fmt.Sprintf("Motion queue:        active"),
			)
		}
	}

	cm.logger.Debug("DEBUG_TIMING reported")

	return strings.Join(lines, "\n"), nil
}

// GetStatus returns the command manager status.
func (cm *CommandManager) GetStatus() map[string]any {
	cm.mu.RLock()
	defer cm.mu.RUnlock()

	return map[string]any{
		"is_ready":     cm.isReady,
		"state_msg":    cm.stateMsg,
		"uptime":       time.Since(cm.startTime).Seconds(),
		"num_commands": len(cm.commands),
	}
}

// ListCommands returns a list of all registered commands with their help text.
func (cm *CommandManager) ListCommands() map[string]string {
	cm.mu.RLock()
	defer cm.mu.RUnlock()

	result := make(map[string]string, len(cm.commandHelp))
	for k, v := range cm.commandHelp {
		result[k] = v
	}
	return result
}

// parseIntAuto parses an integer from string, supporting both decimal and hex (0x) formats.
func parseIntAuto(s string) (int, error) {
	s = strings.TrimSpace(s)
	if strings.HasPrefix(strings.ToLower(s), "0x") {
		val, err := strconv.ParseInt(s[2:], 16, 64)
		return int(val), err
	}
	val, err := strconv.ParseInt(s, 10, 64)
	return int(val), err
}

// RegisterExternalCommands registers additional commands from other modules.
// This allows other modules to add their commands to the HELP output.
func (cm *CommandManager) RegisterExternalCommands(commands map[string]string) {
	cm.mu.Lock()
	defer cm.mu.Unlock()

	for name, help := range commands {
		name = strings.ToUpper(name)
		if _, exists := cm.commandHelp[name]; !exists {
			cm.commandHelp[name] = help
			cm.commandOrder = append(cm.commandOrder, name)
		}
	}
	sort.Strings(cm.commandOrder)
}
