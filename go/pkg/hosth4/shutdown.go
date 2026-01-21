// Package hosth4 provides shutdown coordination for the Go host.
// This implements the equivalent of Python's invoke_shutdown mechanism
// from klippy/klippy.py, managing graceful shutdown across all MCUs and components.
package hosth4

import (
	"fmt"
	"log"
	"sync"
	"time"

	"klipper-go-migration/pkg/reactor"
)

// ShutdownState represents the printer's operational state.
type ShutdownState string

const (
	// StateStartup indicates the printer is initializing.
	StateStartup ShutdownState = "startup"
	// StateReady indicates the printer is ready for operation.
	StateReady ShutdownState = "ready"
	// StateShutdown indicates the printer has been shut down.
	StateShutdown ShutdownState = "shutdown"
	// StateRestarting indicates the printer is restarting.
	StateRestarting ShutdownState = "restarting"
	// StateError indicates a fatal error occurred.
	StateError ShutdownState = "error"
)

// ShutdownDetails contains information about a shutdown event.
type ShutdownDetails struct {
	Reason        string                 // Human-readable reason
	MCUName       string                 // MCU that triggered the shutdown (if applicable)
	EventType     string                 // "shutdown", "is_shutdown", "timeout", etc.
	ShutdownClock uint64                 // MCU clock at shutdown time
	Extra         map[string]interface{} // Additional diagnostic data
}

// ShutdownHandler is called when a shutdown is triggered.
type ShutdownHandler func()

// AnalyzeShutdownHandler is called after shutdown to analyze the cause.
type AnalyzeShutdownHandler func(msg string, details *ShutdownDetails)

// ShutdownCoordinator manages the printer's shutdown lifecycle.
// It coordinates shutdown across all MCUs and registered components,
// ensuring a safe and orderly shutdown sequence.
type ShutdownCoordinator struct {
	mu sync.RWMutex

	// Current state
	state        ShutdownState
	shutdownMsg  string
	details      *ShutdownDetails
	shutdownTime time.Time

	// Event handlers (called in registration order)
	onShutdown        []shutdownHandlerEntry
	onAnalyzeShutdown []analyzeHandlerEntry

	// Dependencies
	mcuManager *MCUManager
	reactor    *reactor.Reactor

	// Logging
	trace bool
}

type shutdownHandlerEntry struct {
	name    string
	handler ShutdownHandler
}

type analyzeHandlerEntry struct {
	name    string
	handler AnalyzeShutdownHandler
}

// NewShutdownCoordinator creates a new shutdown coordinator.
func NewShutdownCoordinator(r *reactor.Reactor) *ShutdownCoordinator {
	return &ShutdownCoordinator{
		state:             StateStartup,
		onShutdown:        make([]shutdownHandlerEntry, 0),
		onAnalyzeShutdown: make([]analyzeHandlerEntry, 0),
		reactor:           r,
	}
}

// SetMCUManager sets the MCU manager for shutdown coordination.
func (sc *ShutdownCoordinator) SetMCUManager(m *MCUManager) {
	sc.mu.Lock()
	defer sc.mu.Unlock()
	sc.mcuManager = m
}

// SetTrace enables or disables trace logging.
func (sc *ShutdownCoordinator) SetTrace(enabled bool) {
	sc.mu.Lock()
	defer sc.mu.Unlock()
	sc.trace = enabled
}

// SetReady transitions the coordinator to ready state.
func (sc *ShutdownCoordinator) SetReady() {
	sc.mu.Lock()
	defer sc.mu.Unlock()
	if sc.state == StateStartup {
		sc.state = StateReady
		sc.tracef("ShutdownCoordinator: Transitioned to ready state\n")
	}
}

// RegisterShutdownHandler registers a handler to be called during shutdown.
// Handlers are called in registration order. The name is used for logging.
func (sc *ShutdownCoordinator) RegisterShutdownHandler(name string, handler ShutdownHandler) {
	sc.mu.Lock()
	defer sc.mu.Unlock()
	sc.onShutdown = append(sc.onShutdown, shutdownHandlerEntry{
		name:    name,
		handler: handler,
	})
	sc.tracef("ShutdownCoordinator: Registered shutdown handler: %s\n", name)
}

// RegisterAnalyzeHandler registers a handler to analyze shutdown causes.
// Called after all shutdown handlers have completed.
func (sc *ShutdownCoordinator) RegisterAnalyzeHandler(name string, handler AnalyzeShutdownHandler) {
	sc.mu.Lock()
	defer sc.mu.Unlock()
	sc.onAnalyzeShutdown = append(sc.onAnalyzeShutdown, analyzeHandlerEntry{
		name:    name,
		handler: handler,
	})
	sc.tracef("ShutdownCoordinator: Registered analyze handler: %s\n", name)
}

// InvokeShutdown triggers a coordinated shutdown.
// This is the main entry point for shutdown, equivalent to Python's invoke_shutdown().
// It is safe to call multiple times - subsequent calls are ignored.
func (sc *ShutdownCoordinator) InvokeShutdown(msg string, details *ShutdownDetails) {
	sc.mu.Lock()

	// Prevent duplicate shutdowns
	if sc.state == StateShutdown || sc.state == StateRestarting {
		sc.tracef("ShutdownCoordinator: Already in %s state, ignoring shutdown: %s\n", sc.state, msg)
		sc.mu.Unlock()
		return
	}

	log.Printf("ShutdownCoordinator: Transition to shutdown state: %s", msg)

	sc.state = StateShutdown
	sc.shutdownMsg = msg
	sc.details = details
	sc.shutdownTime = time.Now()

	if details == nil {
		sc.details = &ShutdownDetails{
			Reason: msg,
		}
	}

	// Copy handlers to call outside lock
	shutdownHandlers := make([]shutdownHandlerEntry, len(sc.onShutdown))
	copy(shutdownHandlers, sc.onShutdown)

	analyzeHandlers := make([]analyzeHandlerEntry, len(sc.onAnalyzeShutdown))
	copy(analyzeHandlers, sc.onAnalyzeShutdown)

	mcuMgr := sc.mcuManager
	sc.mu.Unlock()

	// Phase 1: Send emergency stop to all MCUs first
	if mcuMgr != nil {
		sc.sendEmergencyStopToAll(mcuMgr)
	}

	// Phase 2: Execute all shutdown handlers
	for _, entry := range shutdownHandlers {
		sc.tracef("ShutdownCoordinator: Calling shutdown handler: %s\n", entry.name)
		func() {
			defer func() {
				if r := recover(); r != nil {
					log.Printf("ShutdownCoordinator: Panic in shutdown handler %s: %v", entry.name, r)
				}
			}()
			entry.handler()
		}()
	}

	// Phase 3: Execute analyze handlers for diagnostics
	for _, entry := range analyzeHandlers {
		sc.tracef("ShutdownCoordinator: Calling analyze handler: %s\n", entry.name)
		func() {
			defer func() {
				if r := recover(); r != nil {
					log.Printf("ShutdownCoordinator: Panic in analyze handler %s: %v", entry.name, r)
				}
			}()
			entry.handler(msg, sc.details)
		}()
	}

	sc.tracef("ShutdownCoordinator: Shutdown sequence complete\n")
}

// InvokeAsyncShutdown triggers a shutdown from another goroutine.
// The actual shutdown is scheduled via the reactor to ensure thread safety.
func (sc *ShutdownCoordinator) InvokeAsyncShutdown(msg string, details *ShutdownDetails) {
	if sc.reactor == nil {
		// No reactor, call directly
		sc.InvokeShutdown(msg, details)
		return
	}

	sc.reactor.RegisterAsyncCallback(func(eventtime float64) interface{} {
		sc.InvokeShutdown(msg, details)
		return nil
	}, reactor.NOW)
}

// sendEmergencyStopToAll sends emergency_stop to all connected MCUs.
func (sc *ShutdownCoordinator) sendEmergencyStopToAll(mcuMgr *MCUManager) {
	mcus := mcuMgr.GetAllMCUs()
	for name, mcu := range mcus {
		if mcu.IsConnected() {
			sc.tracef("ShutdownCoordinator: Sending emergency_stop to MCU %s\n", name)
			if err := mcu.SendEmergencyStop(); err != nil {
				log.Printf("ShutdownCoordinator: Failed to send emergency_stop to %s: %v", name, err)
			}
		}
	}
}

// IsShutdown returns true if the printer is in shutdown state.
func (sc *ShutdownCoordinator) IsShutdown() bool {
	sc.mu.RLock()
	defer sc.mu.RUnlock()
	return sc.state == StateShutdown || sc.state == StateError
}

// IsReady returns true if the printer is ready for operation.
func (sc *ShutdownCoordinator) IsReady() bool {
	sc.mu.RLock()
	defer sc.mu.RUnlock()
	return sc.state == StateReady
}

// State returns the current state.
func (sc *ShutdownCoordinator) State() ShutdownState {
	sc.mu.RLock()
	defer sc.mu.RUnlock()
	return sc.state
}

// ShutdownMessage returns the shutdown message (empty if not shutdown).
func (sc *ShutdownCoordinator) ShutdownMessage() string {
	sc.mu.RLock()
	defer sc.mu.RUnlock()
	return sc.shutdownMsg
}

// ShutdownDetails returns the shutdown details (nil if not shutdown).
func (sc *ShutdownCoordinator) ShutdownDetails() *ShutdownDetails {
	sc.mu.RLock()
	defer sc.mu.RUnlock()
	return sc.details
}

// GetStatus returns the current status for reporting.
func (sc *ShutdownCoordinator) GetStatus() map[string]interface{} {
	sc.mu.RLock()
	defer sc.mu.RUnlock()

	status := map[string]interface{}{
		"state": string(sc.state),
	}

	if sc.state == StateShutdown || sc.state == StateError {
		status["message"] = sc.shutdownMsg
		if !sc.shutdownTime.IsZero() {
			status["shutdown_time"] = sc.shutdownTime.Unix()
		}
		if sc.details != nil {
			if sc.details.MCUName != "" {
				status["mcu"] = sc.details.MCUName
			}
			if sc.details.EventType != "" {
				status["event_type"] = sc.details.EventType
			}
		}
	}

	return status
}

// PrepareRestart transitions to restarting state.
// Returns error if restart is not allowed in current state.
func (sc *ShutdownCoordinator) PrepareRestart() error {
	sc.mu.Lock()
	defer sc.mu.Unlock()

	// Can restart from ready, shutdown, or error states
	if sc.state == StateStartup {
		return fmt.Errorf("cannot restart during startup")
	}
	if sc.state == StateRestarting {
		return fmt.Errorf("restart already in progress")
	}

	sc.state = StateRestarting
	sc.tracef("ShutdownCoordinator: Preparing for restart\n")
	return nil
}

// Reset resets the coordinator to startup state.
// This should only be called during a full restart sequence.
func (sc *ShutdownCoordinator) Reset() {
	sc.mu.Lock()
	defer sc.mu.Unlock()

	sc.state = StateStartup
	sc.shutdownMsg = ""
	sc.details = nil
	sc.shutdownTime = time.Time{}
	sc.tracef("ShutdownCoordinator: Reset to startup state\n")
}

// tracef writes trace output if tracing is enabled.
func (sc *ShutdownCoordinator) tracef(format string, args ...interface{}) {
	if sc.trace {
		log.Printf(format, args...)
	}
}

// Common shutdown reasons (matching Python's error messages)
const (
	ShutdownReasonM112          = "Shutdown due to M112 command"
	ShutdownReasonMCUTimeout    = "Lost communication with MCU '%s'"
	ShutdownReasonMCUShutdown   = "MCU '%s' shutdown: %s"
	ShutdownReasonMCURestart    = "MCU '%s' spontaneous restart"
	ShutdownReasonInternalError = "Internal error: %s"
	ShutdownReasonConfigError   = "Configuration error: %s"
)

// FormatShutdownReason formats a shutdown reason with arguments.
func FormatShutdownReason(format string, args ...interface{}) string {
	return fmt.Sprintf(format, args...)
}

// HandleMCUShutdown handles an MCU shutdown message.
// This is called from MCU message handlers when a shutdown/is_shutdown message is received.
func (sc *ShutdownCoordinator) HandleMCUShutdown(mcuName string, params map[string]interface{}) {
	reason := "unknown"
	if staticID, ok := params["static_string_id"].(int); ok {
		reason = fmt.Sprintf("static_string_id=%d", staticID)
	}

	var shutdownClock uint64
	if clock, ok := params["clock"].(int); ok {
		shutdownClock = uint64(clock)
	}

	eventType := "shutdown"
	if name, ok := params["#name"].(string); ok {
		eventType = name
	}

	details := &ShutdownDetails{
		Reason:        reason,
		MCUName:       mcuName,
		EventType:     eventType,
		ShutdownClock: shutdownClock,
		Extra:         params,
	}

	msg := FormatShutdownReason(ShutdownReasonMCUShutdown, mcuName, reason)
	sc.InvokeAsyncShutdown(msg, details)
}

// HandleMCUStarting handles an MCU spontaneous restart message.
// This indicates the MCU rebooted unexpectedly.
func (sc *ShutdownCoordinator) HandleMCUStarting(mcuName string, params map[string]interface{}) {
	details := &ShutdownDetails{
		Reason:    "spontaneous restart",
		MCUName:   mcuName,
		EventType: "starting",
		Extra:     params,
	}

	msg := FormatShutdownReason(ShutdownReasonMCURestart, mcuName)
	sc.InvokeAsyncShutdown(msg, details)
}

// HandleMCUTimeout handles an MCU communication timeout.
// This is called when the heartbeat monitor detects a timeout.
func (sc *ShutdownCoordinator) HandleMCUTimeout(mcuName string, eventtime float64) {
	details := &ShutdownDetails{
		Reason:    "communication timeout",
		MCUName:   mcuName,
		EventType: "timeout",
		Extra: map[string]interface{}{
			"eventtime": eventtime,
		},
	}

	msg := FormatShutdownReason(ShutdownReasonMCUTimeout, mcuName)
	sc.InvokeAsyncShutdown(msg, details)
}

// HandleM112 handles the M112 emergency stop G-code command.
func (sc *ShutdownCoordinator) HandleM112() {
	details := &ShutdownDetails{
		Reason:    "M112 emergency stop command",
		EventType: "m112",
	}

	sc.InvokeShutdown(ShutdownReasonM112, details)
}

// HandleInternalError handles an internal error that requires shutdown.
func (sc *ShutdownCoordinator) HandleInternalError(err error) {
	details := &ShutdownDetails{
		Reason:    err.Error(),
		EventType: "internal_error",
	}

	msg := FormatShutdownReason(ShutdownReasonInternalError, err.Error())
	sc.InvokeShutdown(msg, details)
}

// HandleConfigError handles a configuration error that prevents operation.
func (sc *ShutdownCoordinator) HandleConfigError(err error) {
	sc.mu.Lock()
	sc.state = StateError
	sc.shutdownMsg = FormatShutdownReason(ShutdownReasonConfigError, err.Error())
	sc.details = &ShutdownDetails{
		Reason:    err.Error(),
		EventType: "config_error",
	}
	sc.mu.Unlock()
}
