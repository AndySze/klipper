// MCU diagnostics - error analysis and reporting
//
// This module provides diagnostic analysis for MCU errors and shutdown events.
// It ports the functionality from klippy/extras/error_mcu.py to help users
// understand and resolve MCU-related issues.
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
	"time"
)

// Common MCU error messages and their explanations
var mcuErrorExplanations = map[string]string{
	"Timer too close": `
This error indicates that the host computer was not able to schedule
timer events in time. This is often caused by:
- Host CPU overload (check top/htop)
- Other processes consuming CPU time
- Disk I/O delays
- Memory pressure or swapping

Possible solutions:
- Close other applications
- Use a faster computer
- Check for SD card issues (on Raspberry Pi)
- Reduce stepper acceleration/velocity`,

	"Missed scheduling of next": `
This error indicates missed communication with the MCU. Common causes:
- USB/serial communication issues
- Cable problems or electrical interference
- MCU firmware crash
- Host software delays

Possible solutions:
- Check USB/serial cables
- Try a different USB port
- Reduce communication baud rate
- Check for EMI from stepper motors or heaters`,

	"ADC out of range": `
This error indicates that a temperature reading exceeded safe limits.
The printer shut down to prevent damage. Common causes:
- Thermistor disconnected or damaged
- Heater malfunction
- Incorrect thermistor type in config

Check the heater and thermistor connections, and verify your
temperature sensor configuration matches the hardware.`,

	"Rescheduled timer in the past": `
This error indicates the host couldn't keep up with step timing.
The printer shut down to prevent movement errors. Common causes:
- Stepper speed/acceleration too high
- Host CPU overload
- USB communication delays

Try reducing max_velocity and max_accel in printer.cfg.`,

	"Stepper too far in past": `
This error is similar to "Rescheduled timer in the past" and indicates
step timing issues. The host fell too far behind in generating steps.

Try reducing stepper speeds and acceleration, or use a faster host.`,

	"Command request": `
This error indicates an emergency stop was requested, either by:
- M112 command
- Internal error condition
- Safety check failure

Check the logs for more details about what triggered the shutdown.`,
}

// DiagnosticReport contains the analysis of a shutdown event.
type DiagnosticReport struct {
	// Shutdown information
	ShutdownReason string
	ShutdownTime   time.Time
	MCUName        string
	EventType      string

	// Analysis
	ErrorCategory   string
	Explanation     string
	Suggestions     []string
	RelatedSettings []string

	// Version information
	HostVersion string
	MCUVersion  string
	MCUFreq     float64

	// Protocol analysis
	ProtocolMismatch bool
	VersionMismatch  bool
}

// MCUDiagnostics provides error analysis for MCU events.
type MCUDiagnostics struct {
	mu sync.RWMutex

	// MCU reference
	mcuConn *MCUConnection
	mcuName string

	// Version tracking
	hostVersion string
	mcuVersion  string

	// Statistics
	shutdownCount   int
	lastShutdown    time.Time
	shutdownReasons map[string]int

	// Tracing
	trace bool
}

// NewMCUDiagnostics creates a new MCU diagnostics instance.
func NewMCUDiagnostics(mcuName string, mcuConn *MCUConnection) *MCUDiagnostics {
	return &MCUDiagnostics{
		mcuName:         mcuName,
		mcuConn:         mcuConn,
		shutdownReasons: make(map[string]int),
	}
}

// SetHostVersion sets the host software version.
func (md *MCUDiagnostics) SetHostVersion(version string) {
	md.mu.Lock()
	defer md.mu.Unlock()
	md.hostVersion = version
}

// SetMCUVersion sets the MCU firmware version.
func (md *MCUDiagnostics) SetMCUVersion(version string) {
	md.mu.Lock()
	defer md.mu.Unlock()
	md.mcuVersion = version
}

// SetTrace enables or disables trace logging.
func (md *MCUDiagnostics) SetTrace(enabled bool) {
	md.mu.Lock()
	defer md.mu.Unlock()
	md.trace = enabled
}

// AnalyzeShutdown analyzes a shutdown event and returns a diagnostic report.
func (md *MCUDiagnostics) AnalyzeShutdown(msg string, details *ShutdownDetails) *DiagnosticReport {
	md.mu.Lock()
	md.shutdownCount++
	md.lastShutdown = time.Now()
	if details != nil && details.Reason != "" {
		md.shutdownReasons[details.Reason]++
	}
	hostVer := md.hostVersion
	mcuVer := md.mcuVersion
	md.mu.Unlock()

	report := &DiagnosticReport{
		ShutdownReason: msg,
		ShutdownTime:   time.Now(),
		MCUName:        md.mcuName,
		HostVersion:    hostVer,
		MCUVersion:     mcuVer,
	}

	if details != nil {
		report.EventType = details.EventType
	}

	// Get MCU frequency if available
	if md.mcuConn != nil {
		report.MCUFreq = md.mcuConn.MCUFreq()
	}

	// Analyze the error message
	md.analyzeErrorMessage(msg, report)

	// Check for version mismatch
	if hostVer != "" && mcuVer != "" && hostVer != mcuVer {
		report.VersionMismatch = true
		report.Suggestions = append(report.Suggestions,
			"Host and MCU versions differ - consider reflashing MCU firmware")
	}

	// Log the analysis
	md.logReport(report)

	return report
}

// analyzeErrorMessage categorizes the error and provides explanation.
func (md *MCUDiagnostics) analyzeErrorMessage(msg string, report *DiagnosticReport) {
	// Check against known error patterns
	for pattern, explanation := range mcuErrorExplanations {
		if strings.Contains(msg, pattern) {
			report.ErrorCategory = pattern
			report.Explanation = explanation

			// Add specific suggestions based on error type
			switch pattern {
			case "Timer too close", "Rescheduled timer in the past", "Stepper too far in past":
				report.Suggestions = append(report.Suggestions,
					"Check host CPU usage",
					"Reduce max_velocity and max_accel",
					"Close other applications")
				report.RelatedSettings = append(report.RelatedSettings,
					"max_velocity", "max_accel", "max_z_velocity", "max_z_accel")

			case "Missed scheduling of next":
				report.Suggestions = append(report.Suggestions,
					"Check USB/serial cable",
					"Try different USB port",
					"Check for electrical interference")

			case "ADC out of range":
				report.Suggestions = append(report.Suggestions,
					"Check thermistor connections",
					"Verify sensor_type in config",
					"Check heater wiring")
				report.RelatedSettings = append(report.RelatedSettings,
					"sensor_type", "sensor_pin", "heater_pin")
			}

			return
		}
	}

	// Check for protocol/communication errors
	if strings.Contains(msg, "protocol") || strings.Contains(msg, "Protocol") {
		report.ErrorCategory = "Protocol Error"
		report.ProtocolMismatch = true
		report.Explanation = `
A protocol error indicates a communication mismatch between the host
and MCU. This often occurs when:
- MCU firmware version doesn't match host software
- Corrupted communication
- Wrong serial port settings`
		report.Suggestions = append(report.Suggestions,
			"Reflash MCU firmware",
			"Check serial port configuration")
	}

	// Check for timeout errors
	if strings.Contains(msg, "timeout") || strings.Contains(msg, "Timeout") ||
		strings.Contains(msg, "Lost communication") {
		report.ErrorCategory = "Communication Timeout"
		report.Explanation = `
Communication with the MCU was lost. This can occur due to:
- USB disconnect
- MCU crash or reset
- Serial port issues
- Power supply problems`
		report.Suggestions = append(report.Suggestions,
			"Check USB connection",
			"Verify MCU power supply",
			"Check for MCU hardware issues")
	}

	// Default if no pattern matched
	if report.ErrorCategory == "" {
		report.ErrorCategory = "Unknown Error"
		report.Explanation = "Unable to determine the specific cause of this error. " +
			"Check the full error message and logs for more details."
	}
}

// logReport logs the diagnostic report.
func (md *MCUDiagnostics) logReport(report *DiagnosticReport) {
	log.Printf("MCU Diagnostics for %s:", md.mcuName)
	log.Printf("  Shutdown reason: %s", report.ShutdownReason)
	log.Printf("  Error category: %s", report.ErrorCategory)

	if report.VersionMismatch {
		log.Printf("  WARNING: Version mismatch - Host: %s, MCU: %s",
			report.HostVersion, report.MCUVersion)
	}

	if len(report.Suggestions) > 0 {
		log.Printf("  Suggestions:")
		for _, s := range report.Suggestions {
			log.Printf("    - %s", s)
		}
	}
}

// DumpDebug returns a debug dump of MCU communication state.
func (md *MCUDiagnostics) DumpDebug() string {
	md.mu.RLock()
	defer md.mu.RUnlock()

	var b strings.Builder

	b.WriteString(fmt.Sprintf("MCU Diagnostics Debug Dump for %s\n", md.mcuName))
	b.WriteString(strings.Repeat("=", 50) + "\n")

	// Version info
	b.WriteString(fmt.Sprintf("Host version: %s\n", md.hostVersion))
	b.WriteString(fmt.Sprintf("MCU version: %s\n", md.mcuVersion))

	// Connection state
	if md.mcuConn != nil {
		b.WriteString(fmt.Sprintf("Connected: %v\n", md.mcuConn.IsConnected()))
		b.WriteString(fmt.Sprintf("Shutdown: %v\n", md.mcuConn.IsShutdown()))
		b.WriteString(fmt.Sprintf("MCU frequency: %.0f Hz\n", md.mcuConn.MCUFreq()))
	}

	// Shutdown statistics
	b.WriteString(fmt.Sprintf("\nShutdown count: %d\n", md.shutdownCount))
	if !md.lastShutdown.IsZero() {
		b.WriteString(fmt.Sprintf("Last shutdown: %v\n", md.lastShutdown))
	}

	if len(md.shutdownReasons) > 0 {
		b.WriteString("\nShutdown reasons:\n")
		for reason, count := range md.shutdownReasons {
			b.WriteString(fmt.Sprintf("  %s: %d\n", reason, count))
		}
	}

	return b.String()
}

// GetStatus returns the diagnostics status.
func (md *MCUDiagnostics) GetStatus() map[string]interface{} {
	md.mu.RLock()
	defer md.mu.RUnlock()

	status := map[string]interface{}{
		"mcu_name":        md.mcuName,
		"host_version":    md.hostVersion,
		"mcu_version":     md.mcuVersion,
		"shutdown_count":  md.shutdownCount,
		"version_mismatch": md.hostVersion != "" && md.mcuVersion != "" && md.hostVersion != md.mcuVersion,
	}

	if !md.lastShutdown.IsZero() {
		status["last_shutdown"] = md.lastShutdown.Unix()
	}

	return status
}

// GetShutdownContext returns context information for shutdown analysis.
func (md *MCUDiagnostics) GetShutdownContext() map[string]interface{} {
	md.mu.RLock()
	defer md.mu.RUnlock()

	context := map[string]interface{}{
		"mcu_name":        md.mcuName,
		"shutdown_count":  md.shutdownCount,
		"shutdown_reasons": md.shutdownReasons,
	}

	if md.mcuConn != nil {
		context["mcu_freq"] = md.mcuConn.MCUFreq()
		context["connected"] = md.mcuConn.IsConnected()
	}

	return context
}

// DiagnosticsManager manages diagnostics for multiple MCUs.
type DiagnosticsManager struct {
	mu sync.RWMutex

	diagnostics map[string]*MCUDiagnostics
	hostVersion string
}

// NewDiagnosticsManager creates a new diagnostics manager.
func NewDiagnosticsManager(hostVersion string) *DiagnosticsManager {
	return &DiagnosticsManager{
		diagnostics: make(map[string]*MCUDiagnostics),
		hostVersion: hostVersion,
	}
}

// AddMCU adds diagnostics for an MCU.
func (dm *DiagnosticsManager) AddMCU(mcuName string, mcuConn *MCUConnection) {
	dm.mu.Lock()
	defer dm.mu.Unlock()

	diag := NewMCUDiagnostics(mcuName, mcuConn)
	diag.SetHostVersion(dm.hostVersion)
	dm.diagnostics[mcuName] = diag
}

// GetDiagnostics returns diagnostics for an MCU.
func (dm *DiagnosticsManager) GetDiagnostics(mcuName string) *MCUDiagnostics {
	dm.mu.RLock()
	defer dm.mu.RUnlock()
	return dm.diagnostics[mcuName]
}

// AnalyzeShutdown analyzes a shutdown event.
func (dm *DiagnosticsManager) AnalyzeShutdown(msg string, details *ShutdownDetails) *DiagnosticReport {
	mcuName := ""
	if details != nil {
		mcuName = details.MCUName
	}

	dm.mu.RLock()
	diag := dm.diagnostics[mcuName]
	dm.mu.RUnlock()

	if diag != nil {
		return diag.AnalyzeShutdown(msg, details)
	}

	// No specific MCU diagnostics, create generic report
	return &DiagnosticReport{
		ShutdownReason: msg,
		ShutdownTime:   time.Now(),
		MCUName:        mcuName,
		HostVersion:    dm.hostVersion,
	}
}

// DumpAll returns debug dumps for all MCUs.
func (dm *DiagnosticsManager) DumpAll() string {
	dm.mu.RLock()
	defer dm.mu.RUnlock()

	var b strings.Builder
	for _, diag := range dm.diagnostics {
		b.WriteString(diag.DumpDebug())
		b.WriteString("\n")
	}
	return b.String()
}

// GetStatus returns status for all MCUs.
func (dm *DiagnosticsManager) GetStatus() map[string]interface{} {
	dm.mu.RLock()
	defer dm.mu.RUnlock()

	status := make(map[string]interface{})
	for name, diag := range dm.diagnostics {
		status[name] = diag.GetStatus()
	}
	return status
}
