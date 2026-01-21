// Restart helper - MCU restart functionality
//
// This module implements MCU restart functionality, porting Python's
// klippy/mcu.py MCURestartHelper class. It supports multiple restart
// methods depending on the hardware platform.
//
// Copyright (C) 2019-2021 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
	"time"
)

// RestartMethod defines how to restart the MCU.
type RestartMethod string

const (
	// RestartArduino uses DTR signal toggle (Arduino bootloader method).
	RestartArduino RestartMethod = "arduino"

	// RestartCheetah uses RTS signal handling (specific to Cheetah boards).
	RestartCheetah RestartMethod = "cheetah"

	// RestartCommand uses reset or config_reset MCU commands.
	RestartCommand RestartMethod = "command"

	// RestartRPIUSB toggles USB power via hub_ctrl (Raspberry Pi specific).
	RestartRPIUSB RestartMethod = "rpi_usb"
)

// RestartHelperConfig configures the restart helper.
type RestartHelperConfig struct {
	// Method specifies the restart method to use.
	Method RestartMethod

	// IsMCUBridge indicates this is a bridge MCU (shouldn't restart normally).
	IsMCUBridge bool
}

// DefaultRestartHelperConfig returns the default restart configuration.
func DefaultRestartHelperConfig() RestartHelperConfig {
	return RestartHelperConfig{
		Method:      RestartArduino,
		IsMCUBridge: false,
	}
}

// RestartHelper manages MCU restart operations.
// This is the Go equivalent of Python's MCURestartHelper.
type RestartHelper struct {
	mu sync.RWMutex

	// Configuration
	config  RestartHelperConfig
	mcuName string

	// MCU connection
	mcuConn *MCUConnection

	// Commands (discovered during MCU identify)
	hasResetCmd       bool
	hasConfigResetCmd bool
	hasEmergencyStop  bool

	// Shutdown coordinator for triggering restart
	shutdownCoord *ShutdownCoordinator

	// Tracing
	trace bool
}

// NewRestartHelper creates a new restart helper.
func NewRestartHelper(mcuName string, mcuConn *MCUConnection, config RestartHelperConfig) *RestartHelper {
	return &RestartHelper{
		config:  config,
		mcuName: mcuName,
		mcuConn: mcuConn,
	}
}

// SetShutdownCoordinator sets the shutdown coordinator.
func (rh *RestartHelper) SetShutdownCoordinator(sc *ShutdownCoordinator) {
	rh.mu.Lock()
	defer rh.mu.Unlock()
	rh.shutdownCoord = sc
}

// SetTrace enables or disables trace logging.
func (rh *RestartHelper) SetTrace(enabled bool) {
	rh.mu.Lock()
	defer rh.mu.Unlock()
	rh.trace = enabled
}

// DiscoverCommands checks which restart commands are available.
// This should be called after MCU identify to determine available commands.
func (rh *RestartHelper) DiscoverCommands() {
	rh.mu.Lock()
	defer rh.mu.Unlock()

	if rh.mcuConn == nil {
		return
	}

	dict := rh.mcuConn.Dictionary()
	if dict == nil {
		return
	}

	// Check for available commands
	for cmd := range dict.Commands {
		switch extractCommandName(cmd) {
		case "reset":
			rh.hasResetCmd = true
		case "config_reset":
			rh.hasConfigResetCmd = true
		case "emergency_stop":
			rh.hasEmergencyStop = true
		}
	}

	rh.tracef("RestartHelper: Discovered commands for %s - reset=%v config_reset=%v emergency_stop=%v\n",
		rh.mcuName, rh.hasResetCmd, rh.hasConfigResetCmd, rh.hasEmergencyStop)
}

// FirmwareRestart performs a firmware restart.
// This restarts the MCU firmware using the configured method.
func (rh *RestartHelper) FirmwareRestart() error {
	rh.mu.RLock()
	isBridge := rh.config.IsMCUBridge
	method := rh.config.Method
	rh.mu.RUnlock()

	if isBridge {
		log.Printf("RestartHelper: Skipping firmware restart for bridge MCU %s", rh.mcuName)
		return nil
	}

	log.Printf("RestartHelper: Firmware restart for %s using method %s", rh.mcuName, method)

	switch method {
	case RestartRPIUSB:
		return rh.restartRPIUSB()
	case RestartCommand:
		return rh.restartViaCommand()
	case RestartCheetah:
		return rh.restartCheetah()
	default:
		return rh.restartArduino()
	}
}

// CheckRestartOnCRCMismatch checks if restart is needed due to CRC mismatch.
// This is called when the config CRC doesn't match the previous value.
func (rh *RestartHelper) CheckRestartOnCRCMismatch() error {
	rh.tracef("RestartHelper: CRC mismatch detected for %s\n", rh.mcuName)

	// Request firmware restart
	return rh.requestRestart("CRC mismatch")
}

// CheckRestartOnSendConfig checks if restart is needed before sending config.
// For rpi_usb method, we need a full reset before configuration.
func (rh *RestartHelper) CheckRestartOnSendConfig() error {
	rh.mu.RLock()
	method := rh.config.Method
	rh.mu.RUnlock()

	if method == RestartRPIUSB {
		rh.tracef("RestartHelper: RPi USB method requires full reset before config\n")
		return rh.requestRestart("full reset before config")
	}

	return nil
}

// CheckRestartOnAttach checks if restart is needed when device is attached.
// For rpi_usb method, checks if the serial port exists.
func (rh *RestartHelper) CheckRestartOnAttach(serialPort string) error {
	rh.mu.RLock()
	method := rh.config.Method
	rh.mu.RUnlock()

	if method == RestartRPIUSB {
		// In a real implementation, we would check if the serial port exists
		// and trigger USB power cycle if it doesn't
		rh.tracef("RestartHelper: Checking RPi USB attach for %s\n", rh.mcuName)
	}

	return nil
}

// requestRestart triggers a restart via the shutdown coordinator.
func (rh *RestartHelper) requestRestart(reason string) error {
	rh.mu.RLock()
	sc := rh.shutdownCoord
	rh.mu.RUnlock()

	if sc != nil {
		sc.PrepareRestart()
	}

	log.Printf("RestartHelper: Requesting restart for %s: %s", rh.mcuName, reason)

	// Perform the actual firmware restart
	if err := rh.FirmwareRestart(); err != nil {
		return fmt.Errorf("restart failed: %w", err)
	}

	return nil
}

// restartViaCommand restarts using MCU reset commands.
func (rh *RestartHelper) restartViaCommand() error {
	rh.mu.RLock()
	hasReset := rh.hasResetCmd
	hasConfigReset := rh.hasConfigResetCmd
	rh.mu.RUnlock()

	if !hasReset && !hasConfigReset {
		log.Printf("RestartHelper: No reset command available for %s", rh.mcuName)
		return fmt.Errorf("no reset command available")
	}

	if !hasReset {
		// Use config_reset - need to force local shutdown first
		if rh.mcuConn != nil {
			rh.mcuConn.ForceLocalShutdown()
			time.Sleep(15 * time.Millisecond)

			rh.tracef("RestartHelper: Sending config_reset to %s\n", rh.mcuName)
			if err := rh.mcuConn.SendCommand("config_reset", nil); err != nil {
				return fmt.Errorf("config_reset failed: %w", err)
			}
		}
	} else {
		// Use reset command directly
		rh.tracef("RestartHelper: Sending reset to %s\n", rh.mcuName)
		if rh.mcuConn != nil {
			if err := rh.mcuConn.SendCommand("reset", nil); err != nil {
				return fmt.Errorf("reset failed: %w", err)
			}
		}
	}

	time.Sleep(15 * time.Millisecond)

	// Disconnect after reset
	if rh.mcuConn != nil {
		rh.mcuConn.Disconnect()
	}

	return nil
}

// restartArduino restarts using DTR signal toggle (Arduino bootloader method).
func (rh *RestartHelper) restartArduino() error {
	rh.tracef("RestartHelper: Arduino reset for %s (DTR toggle)\n", rh.mcuName)

	if rh.mcuConn == nil {
		return fmt.Errorf("no MCU connection")
	}

	// Disconnect the MCU connection
	// The serial port's DTR toggle on close/open triggers Arduino reset
	if err := rh.mcuConn.Disconnect(); err != nil {
		return fmt.Errorf("disconnect failed: %w", err)
	}

	// Wait for bootloader
	time.Sleep(100 * time.Millisecond)

	return nil
}

// restartCheetah restarts using RTS signal handling for Cheetah boards.
func (rh *RestartHelper) restartCheetah() error {
	rh.tracef("RestartHelper: Cheetah reset for %s (RTS handling)\n", rh.mcuName)

	if rh.mcuConn == nil {
		return fmt.Errorf("no MCU connection")
	}

	// Cheetah requires special RTS handling
	// The exact sequence is board-specific
	if err := rh.mcuConn.Disconnect(); err != nil {
		return fmt.Errorf("disconnect failed: %w", err)
	}

	// Wait for board reset
	time.Sleep(500 * time.Millisecond)

	return nil
}

// restartRPIUSB restarts by toggling USB power via hub_ctrl.
func (rh *RestartHelper) restartRPIUSB() error {
	rh.tracef("RestartHelper: RPi USB power cycle for %s\n", rh.mcuName)

	if rh.mcuConn == nil {
		return fmt.Errorf("no MCU connection")
	}

	// Disconnect first
	if err := rh.mcuConn.Disconnect(); err != nil {
		rh.tracef("RestartHelper: Disconnect error (ignored): %v\n", err)
	}

	// In Python, this calls chelper.run_hub_ctrl(0) and run_hub_ctrl(1)
	// to toggle USB power. For now, we just wait.
	log.Printf("RestartHelper: USB power toggle not implemented - waiting 2s")

	// Wait for USB power cycle
	time.Sleep(2 * time.Second)

	return nil
}

// GetStatus returns the restart helper status.
func (rh *RestartHelper) GetStatus() map[string]interface{} {
	rh.mu.RLock()
	defer rh.mu.RUnlock()

	return map[string]interface{}{
		"mcu_name":           rh.mcuName,
		"method":             string(rh.config.Method),
		"is_mcu_bridge":      rh.config.IsMCUBridge,
		"has_reset_cmd":      rh.hasResetCmd,
		"has_config_reset":   rh.hasConfigResetCmd,
		"has_emergency_stop": rh.hasEmergencyStop,
	}
}

// tracef writes trace output if tracing is enabled.
func (rh *RestartHelper) tracef(format string, args ...interface{}) {
	if rh.trace {
		log.Printf(format, args...)
	}
}

// GCodeRestartHandler handles RESTART and FIRMWARE_RESTART G-code commands.
type GCodeRestartHandler struct {
	mu sync.RWMutex

	// Shutdown coordinator
	shutdownCoord *ShutdownCoordinator

	// Restart helpers by MCU name
	restartHelpers map[string]*RestartHelper
}

// NewGCodeRestartHandler creates a new G-code restart handler.
func NewGCodeRestartHandler(sc *ShutdownCoordinator) *GCodeRestartHandler {
	return &GCodeRestartHandler{
		shutdownCoord:  sc,
		restartHelpers: make(map[string]*RestartHelper),
	}
}

// AddRestartHelper adds a restart helper for an MCU.
func (gh *GCodeRestartHandler) AddRestartHelper(mcuName string, rh *RestartHelper) {
	gh.mu.Lock()
	defer gh.mu.Unlock()
	gh.restartHelpers[mcuName] = rh
}

// CmdRestart handles the RESTART G-code command.
// This performs a soft restart (host software restart).
func (gh *GCodeRestartHandler) CmdRestart(args map[string]interface{}) error {
	log.Printf("GCodeRestartHandler: RESTART command received")

	gh.mu.RLock()
	sc := gh.shutdownCoord
	gh.mu.RUnlock()

	if sc != nil {
		sc.PrepareRestart()
	}

	return nil
}

// CmdFirmwareRestart handles the FIRMWARE_RESTART G-code command.
// This performs a firmware restart (MCU reset + host restart).
func (gh *GCodeRestartHandler) CmdFirmwareRestart(args map[string]interface{}) error {
	log.Printf("GCodeRestartHandler: FIRMWARE_RESTART command received")

	gh.mu.RLock()
	sc := gh.shutdownCoord
	helpers := make(map[string]*RestartHelper, len(gh.restartHelpers))
	for k, v := range gh.restartHelpers {
		helpers[k] = v
	}
	gh.mu.RUnlock()

	// Restart all MCUs
	var firstErr error
	for name, helper := range helpers {
		if err := helper.FirmwareRestart(); err != nil {
			log.Printf("GCodeRestartHandler: Failed to restart MCU %s: %v", name, err)
			if firstErr == nil {
				firstErr = err
			}
		}
	}

	// Prepare for host restart
	if sc != nil {
		sc.PrepareRestart()
	}

	return firstErr
}
