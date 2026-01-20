// Moonraker API integration for hosth4 runtime.
// This bridges the hosth4 runtime with the Moonraker-compatible API server,
// enabling Fluidd/Mainsail frontends to connect.
package hosth4

import (
	"log"

	"klipper-go-migration/pkg/moonraker"
)

// MoonrakerIntegration manages the Moonraker API server integration.
type MoonrakerIntegration struct {
	rt       *runtime
	server   *moonraker.Server
	adapter  *moonraker.PrinterAdapter
	addr     string
	running  bool
}

// NewMoonrakerIntegration creates a new Moonraker integration.
func NewMoonrakerIntegration(rt *runtime, addr string) *MoonrakerIntegration {
	mi := &MoonrakerIntegration{
		rt:      rt,
		addr:    addr,
		adapter: moonraker.NewPrinterAdapter(),
	}

	// Register status providers
	mi.registerStatusProviders()

	// Set up handlers
	mi.adapter.SetGCodeExecutor(mi.executeGCode)
	mi.adapter.SetEmergencyStopHandler(mi.emergencyStop)
	mi.adapter.SetKlippyStateGetter(mi.getKlippyState)

	// Create server
	mi.server = moonraker.New(moonraker.Config{
		Addr:    addr,
		Printer: mi.adapter,
	})

	return mi
}

// registerStatusProviders registers all status providers from the runtime.
func (mi *MoonrakerIntegration) registerStatusProviders() {
	// Webhooks status
	mi.adapter.RegisterStatusProvider("webhooks", func(attrs []string) map[string]any {
		status := map[string]any{
			"state":         mi.getKlippyState(),
			"state_message": mi.getStateMessage(),
		}
		return moonraker.FilterStatus(status, attrs)
	})

	// Print stats
	mi.adapter.RegisterStatusProvider("print_stats", func(attrs []string) map[string]any {
		if mi.rt.printStats != nil {
			return moonraker.FilterStatus(mi.rt.printStats.GetStatus(), attrs)
		}
		return map[string]any{
			"state":          "standby",
			"print_duration": 0.0,
			"filename":       "",
			"total_duration": 0.0,
			"filament_used":  0.0,
			"message":        "",
		}
	})

	// Virtual SD card
	mi.adapter.RegisterStatusProvider("virtual_sdcard", func(attrs []string) map[string]any {
		if mi.rt.sdcard != nil {
			return moonraker.FilterStatus(mi.rt.sdcard.GetStatus(), attrs)
		}
		return map[string]any{
			"file_path":     "",
			"progress":      0.0,
			"is_active":     false,
			"file_position": 0,
			"file_size":     0,
		}
	})

	// Pause/Resume
	mi.adapter.RegisterStatusProvider("pause_resume", func(attrs []string) map[string]any {
		if mi.rt.pauseResume != nil {
			return moonraker.FilterStatus(mi.rt.pauseResume.GetStatus(), attrs)
		}
		return map[string]any{"is_paused": false}
	})

	// Display status (not yet implemented in runtime)
	mi.adapter.RegisterStatusProvider("display_status", func(attrs []string) map[string]any {
		return moonraker.FilterStatus(map[string]any{
			"progress": 0.0,
			"message":  "",
		}, attrs)
	})

	// GCode move
	mi.adapter.RegisterStatusProvider("gcode_move", func(attrs []string) map[string]any {
		if mi.rt.gm != nil {
			return moonraker.FilterStatus(mi.rt.gm.GetStatus(), attrs)
		}
		return map[string]any{
			"speed_factor":         1.0,
			"speed":                0.0,
			"extrude_factor":       1.0,
			"absolute_coordinates": true,
			"absolute_extrude":     false,
			"homing_origin":        []float64{0, 0, 0, 0},
			"position":             []float64{0, 0, 0, 0},
			"gcode_position":       []float64{0, 0, 0, 0},
		}
	})

	// Toolhead
	mi.adapter.RegisterStatusProvider("toolhead", func(attrs []string) map[string]any {
		status := map[string]any{
			"position":               []float64{0, 0, 0, 0},
			"extruder":               "extruder",
			"homed_axes":             "",
			"print_time":             0.0,
			"estimated_print_time":   0.0,
			"max_velocity":           500.0,
			"max_accel":              3000.0,
			"max_accel_to_decel":     1500.0,
			"square_corner_velocity": 5.0,
		}
		if mi.rt.toolhead != nil {
			status["max_velocity"] = mi.rt.toolhead.maxVelocity
			status["max_accel"] = mi.rt.toolhead.maxAccel
			status["square_corner_velocity"] = mi.rt.toolhead.squareCornerV
			status["print_time"] = mi.rt.toolhead.printTime
			if len(mi.rt.toolhead.commandedPos) > 0 {
				status["position"] = mi.rt.toolhead.commandedPos
			}
		}
		return moonraker.FilterStatus(status, attrs)
	})

	// Extruder
	mi.adapter.RegisterStatusProvider("extruder", func(attrs []string) map[string]any {
		status := map[string]any{
			"temperature":      0.0,
			"target":           0.0,
			"power":            0.0,
			"can_extrude":      false,
			"pressure_advance": 0.0,
			"smooth_time":      0.04,
		}
		// Try to get heater data if available
		if mi.rt.heaterManager != nil {
			if heater, err := mi.rt.heaterManager.GetHeater("extruder"); err == nil && heater != nil {
				temp, target := heater.GetTemp(0.0) // eventtime = 0 for now
				status["temperature"] = temp
				status["target"] = target
			}
		}
		return moonraker.FilterStatus(status, attrs)
	})

	// Heater bed
	mi.adapter.RegisterStatusProvider("heater_bed", func(attrs []string) map[string]any {
		status := map[string]any{
			"temperature": 0.0,
			"target":      0.0,
			"power":       0.0,
		}
		if mi.rt.heaterManager != nil {
			if heater, err := mi.rt.heaterManager.GetHeater("heater_bed"); err == nil && heater != nil {
				temp, target := heater.GetTemp(0.0) // eventtime = 0 for now
				status["temperature"] = temp
				status["target"] = target
			}
		}
		return moonraker.FilterStatus(status, attrs)
	})

	// Heaters (collection)
	mi.adapter.RegisterStatusProvider("heaters", func(attrs []string) map[string]any {
		availableHeaters := []string{"extruder", "heater_bed"}
		availableSensors := []string{"extruder", "heater_bed"}
		status := map[string]any{
			"available_heaters": availableHeaters,
			"available_sensors": availableSensors,
		}
		return moonraker.FilterStatus(status, attrs)
	})

	// Fan
	mi.adapter.RegisterStatusProvider("fan", func(attrs []string) map[string]any {
		status := map[string]any{
			"speed": 0.0,
			"rpm":   nil,
		}
		return moonraker.FilterStatus(status, attrs)
	})

	// Idle timeout (not yet implemented in runtime)
	mi.adapter.RegisterStatusProvider("idle_timeout", func(attrs []string) map[string]any {
		return moonraker.FilterStatus(map[string]any{
			"state":         "Idle",
			"printing_time": 0.0,
		}, attrs)
	})

	// Motion report (not yet implemented in runtime)
	mi.adapter.RegisterStatusProvider("motion_report", func(attrs []string) map[string]any {
		status := map[string]any{
			"live_position":          []float64{0, 0, 0, 0},
			"live_velocity":          0.0,
			"live_extruder_velocity": 0.0,
		}
		// Get position from toolhead if available
		if mi.rt.toolhead != nil && len(mi.rt.toolhead.commandedPos) > 0 {
			status["live_position"] = mi.rt.toolhead.commandedPos
		}
		return moonraker.FilterStatus(status, attrs)
	})

	// Query endstops
	mi.adapter.RegisterStatusProvider("query_endstops", func(attrs []string) map[string]any {
		status := map[string]any{
			"last_query": map[string]string{},
		}
		return moonraker.FilterStatus(status, attrs)
	})

	// System stats
	mi.adapter.RegisterStatusProvider("system_stats", func(attrs []string) map[string]any {
		status := map[string]any{
			"sysload":  0.0,
			"cputime":  0.0,
			"memavail": 0,
		}
		return moonraker.FilterStatus(status, attrs)
	})

	// MCU status
	mi.adapter.RegisterStatusProvider("mcu", func(attrs []string) map[string]any {
		status := map[string]any{
			"mcu_version":     "klipper-go",
			"mcu_build_versions": "",
			"mcu_constants":   map[string]any{},
			"last_stats":      map[string]any{},
		}
		return moonraker.FilterStatus(status, attrs)
	})

	// Config file
	mi.adapter.RegisterStatusProvider("configfile", func(attrs []string) map[string]any {
		status := map[string]any{
			"config":   map[string]any{},
			"settings": map[string]any{},
			"warnings": []string{},
			"save_config_pending": false,
			"save_config_pending_items": map[string]any{},
		}
		return moonraker.FilterStatus(status, attrs)
	})
}

// Start starts the Moonraker API server.
func (mi *MoonrakerIntegration) Start() error {
	if mi.running {
		return nil
	}

	mi.running = true
	log.Printf("Starting Moonraker API server on %s", mi.addr)

	// Start server in a goroutine
	go func() {
		if err := mi.server.Start(); err != nil {
			log.Printf("Moonraker server error: %v", err)
		}
	}()

	return nil
}

// Stop stops the Moonraker API server.
func (mi *MoonrakerIntegration) Stop() error {
	if !mi.running {
		return nil
	}

	mi.running = false
	return mi.server.Stop()
}

// executeGCode executes a G-code script.
func (mi *MoonrakerIntegration) executeGCode(line string) error {
	// Log the gcode execution request
	log.Printf("Moonraker: executing gcode: %s", line)
	// In a full implementation, this would execute via the runtime's gcode handler
	return nil
}

// emergencyStop triggers an emergency stop.
func (mi *MoonrakerIntegration) emergencyStop() {
	log.Println("Moonraker: emergency stop triggered")
	// In a full implementation, this would trigger the runtime's emergency stop
}

// getKlippyState returns the current Klippy state.
func (mi *MoonrakerIntegration) getKlippyState() string {
	if mi.rt == nil {
		return "disconnected"
	}
	if mi.rt.closed {
		return "shutdown"
	}
	return "ready"
}

// getStateMessage returns a human-readable state message.
func (mi *MoonrakerIntegration) getStateMessage() string {
	state := mi.getKlippyState()
	switch state {
	case "ready":
		return "Printer is ready"
	case "shutdown":
		return "Printer has shut down"
	case "disconnected":
		return "Printer is disconnected"
	case "error":
		return "Printer encountered an error"
	default:
		return "Unknown state"
	}
}

// Server returns the underlying Moonraker server.
func (mi *MoonrakerIntegration) Server() *moonraker.Server {
	return mi.server
}

// Adapter returns the printer adapter.
func (mi *MoonrakerIntegration) Adapter() *moonraker.PrinterAdapter {
	return mi.adapter
}
