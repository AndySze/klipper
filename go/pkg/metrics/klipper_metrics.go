// Klipper-specific metrics definitions
//
// Defines all metrics for the Klipper Go host including:
// - Motion/kinematics metrics
// - Temperature metrics
// - MCU communication metrics
// - Print statistics
// - System metrics
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package metrics

import (
	goruntime "runtime"
	"sync"
	"time"
)

// KlipperMetrics holds all Klipper-specific metrics
type KlipperMetrics struct {
	// Motion metrics
	ToolheadPosition   *Gauge
	StepsExecuted      *Counter
	MovePlanningTime   *Histogram
	LookaheadDepth     *Gauge
	MaxAcceleration    *Gauge
	MaxVelocity        *Gauge

	// Temperature metrics
	SensorTemperature  *Gauge
	HeaterTarget       *Gauge
	HeaterPWM          *Gauge
	HeaterOnTime       *Counter
	TemperatureError   *Gauge

	// Fan metrics
	FanPWM     *Gauge
	FanRPM     *Gauge
	FanOnTime  *Counter

	// Endstop metrics
	EndstopState     *Gauge
	HomingAttempts   *Counter
	HomingTime       *Histogram

	// MCU communication metrics
	MCUConnected         *Gauge
	MCUHeartbeatPending  *Gauge
	MCUTimeouts          *Counter
	MCULatency           *Histogram
	MCUMessagesSent      *Counter
	MCUMessagesReceived  *Counter
	MCUFrequency         *Gauge
	MCUBufferUsage       *Gauge

	// Print statistics
	PrintState           *Gauge
	PrintDuration        *Gauge
	PrintFilamentUsed    *Gauge
	PrintsCompleted      *Counter
	PrintsFailed         *Counter

	// System metrics
	HostUptime           *Counter
	GoGoroutines         *Gauge
	GoMemoryHeap         *Gauge
	GoMemoryAlloc        *Gauge
	GoGCCycles           *Counter
	GCodeCommandsTotal   *Counter
	GCodeExecutionTime   *Histogram

	// Error metrics
	ErrorsTotal      *Counter
	WarningsTotal    *Counter
	ShutdownEvents   *Counter

	// Internal
	startTime time.Time
	registry  *Registry
	mu        sync.RWMutex
}

// NewKlipperMetrics creates and registers all Klipper metrics
func NewKlipperMetrics() *KlipperMetrics {
	km := &KlipperMetrics{
		startTime: time.Now(),
		registry:  NewRegistry(),
	}

	// Motion metrics
	km.ToolheadPosition = NewGauge("klipper_toolhead_position_mm",
		"Current toolhead position in millimeters")
	km.StepsExecuted = NewCounter("klipper_steps_executed_total",
		"Total steps executed per stepper")
	km.MovePlanningTime = NewHistogram("klipper_move_planning_seconds",
		"Time spent planning moves", DefaultBuckets())
	km.LookaheadDepth = NewGauge("klipper_lookahead_queue_depth",
		"Number of moves in the lookahead queue")
	km.MaxAcceleration = NewGauge("klipper_max_acceleration_mm_s2",
		"Maximum acceleration setting")
	km.MaxVelocity = NewGauge("klipper_max_velocity_mm_s",
		"Maximum velocity setting")

	// Temperature metrics
	km.SensorTemperature = NewGauge("klipper_sensor_temperature_celsius",
		"Current temperature reading from sensor")
	km.HeaterTarget = NewGauge("klipper_heater_target_celsius",
		"Target temperature for heater")
	km.HeaterPWM = NewGauge("klipper_heater_pwm",
		"Current PWM value for heater (0-1)")
	km.HeaterOnTime = NewCounter("klipper_heater_on_time_seconds_total",
		"Total time heater has been active")
	km.TemperatureError = NewGauge("klipper_temperature_error_celsius",
		"Difference between target and current temperature")

	// Fan metrics
	km.FanPWM = NewGauge("klipper_fan_pwm",
		"Current PWM value for fan (0-1)")
	km.FanRPM = NewGauge("klipper_fan_rpm",
		"Current fan speed in RPM")
	km.FanOnTime = NewCounter("klipper_fan_on_time_seconds_total",
		"Total time fan has been running")

	// Endstop metrics
	km.EndstopState = NewGauge("klipper_endstop_triggered",
		"Endstop trigger state (1=triggered, 0=open)")
	km.HomingAttempts = NewCounter("klipper_homing_attempts_total",
		"Total homing attempts per axis")
	km.HomingTime = NewHistogram("klipper_homing_time_seconds",
		"Time to complete homing", []float64{0.5, 1, 2, 5, 10, 30})

	// MCU communication metrics
	km.MCUConnected = NewGauge("klipper_mcu_connected",
		"MCU connection state (1=connected, 0=disconnected)")
	km.MCUHeartbeatPending = NewGauge("klipper_mcu_heartbeat_pending",
		"Number of pending heartbeat queries")
	km.MCUTimeouts = NewCounter("klipper_mcu_timeouts_total",
		"Total MCU communication timeouts")
	km.MCULatency = NewHistogram("klipper_mcu_latency_seconds",
		"MCU command round-trip latency", []float64{0.001, 0.005, 0.01, 0.025, 0.05, 0.1})
	km.MCUMessagesSent = NewCounter("klipper_mcu_messages_sent_total",
		"Total messages sent to MCU")
	km.MCUMessagesReceived = NewCounter("klipper_mcu_messages_received_total",
		"Total messages received from MCU")
	km.MCUFrequency = NewGauge("klipper_mcu_frequency_hz",
		"MCU clock frequency")
	km.MCUBufferUsage = NewGauge("klipper_mcu_buffer_usage",
		"MCU command buffer usage (0-1)")

	// Print statistics
	km.PrintState = NewGauge("klipper_print_state",
		"Current print state (0=standby, 1=printing, 2=paused, 3=complete, 4=cancelled, 5=error)")
	km.PrintDuration = NewGauge("klipper_print_duration_seconds",
		"Current print duration")
	km.PrintFilamentUsed = NewGauge("klipper_print_filament_used_mm",
		"Filament used in current print")
	km.PrintsCompleted = NewCounter("klipper_prints_completed_total",
		"Total number of completed prints")
	km.PrintsFailed = NewCounter("klipper_prints_failed_total",
		"Total number of failed prints")

	// System metrics
	km.HostUptime = NewCounter("klipper_host_uptime_seconds_total",
		"Total host uptime in seconds")
	km.GoGoroutines = NewGauge("klipper_go_goroutines",
		"Number of active goroutines")
	km.GoMemoryHeap = NewGauge("klipper_go_memory_heap_bytes",
		"Go heap memory in use")
	km.GoMemoryAlloc = NewGauge("klipper_go_memory_alloc_bytes",
		"Go total memory allocated")
	km.GoGCCycles = NewCounter("klipper_go_gc_cycles_total",
		"Total Go garbage collection cycles")
	km.GCodeCommandsTotal = NewCounter("klipper_gcode_commands_total",
		"Total G-code commands processed")
	km.GCodeExecutionTime = NewHistogram("klipper_gcode_execution_seconds",
		"G-code command execution time", DefaultBuckets())

	// Error metrics
	km.ErrorsTotal = NewCounter("klipper_errors_total",
		"Total errors by type")
	km.WarningsTotal = NewCounter("klipper_warnings_total",
		"Total warnings by type")
	km.ShutdownEvents = NewCounter("klipper_shutdown_events_total",
		"Total MCU shutdown events")

	// Register all metrics
	km.registerAll()

	return km
}

// registerAll registers all metrics with the internal registry
func (km *KlipperMetrics) registerAll() {
	metrics := []Metric{
		km.ToolheadPosition, km.StepsExecuted, km.MovePlanningTime,
		km.LookaheadDepth, km.MaxAcceleration, km.MaxVelocity,
		km.SensorTemperature, km.HeaterTarget, km.HeaterPWM,
		km.HeaterOnTime, km.TemperatureError,
		km.FanPWM, km.FanRPM, km.FanOnTime,
		km.EndstopState, km.HomingAttempts, km.HomingTime,
		km.MCUConnected, km.MCUHeartbeatPending, km.MCUTimeouts,
		km.MCULatency, km.MCUMessagesSent, km.MCUMessagesReceived,
		km.MCUFrequency, km.MCUBufferUsage,
		km.PrintState, km.PrintDuration, km.PrintFilamentUsed,
		km.PrintsCompleted, km.PrintsFailed,
		km.HostUptime, km.GoGoroutines, km.GoMemoryHeap, km.GoMemoryAlloc,
		km.GoGCCycles, km.GCodeCommandsTotal, km.GCodeExecutionTime,
		km.ErrorsTotal, km.WarningsTotal, km.ShutdownEvents,
	}
	for _, m := range metrics {
		km.registry.MustRegister(m)
	}
}

// UpdateSystemMetrics updates Go runtime metrics
func (km *KlipperMetrics) UpdateSystemMetrics() {
	var m goruntime.MemStats
	goruntime.ReadMemStats(&m)

	km.GoGoroutines.Set(nil, float64(goruntime.NumGoroutine()))
	km.GoMemoryHeap.Set(nil, float64(m.HeapAlloc))
	km.GoMemoryAlloc.Set(nil, float64(m.Alloc))
	km.GoGCCycles.Add(nil, uint64(m.NumGC)-km.GoGCCycles.Get(nil))
	km.HostUptime.Add(nil, uint64(time.Since(km.startTime).Seconds()))
}

// SetToolheadPosition updates toolhead position
func (km *KlipperMetrics) SetToolheadPosition(x, y, z, e float64) {
	km.ToolheadPosition.Set(Labels{"axis": "x"}, x)
	km.ToolheadPosition.Set(Labels{"axis": "y"}, y)
	km.ToolheadPosition.Set(Labels{"axis": "z"}, z)
	km.ToolheadPosition.Set(Labels{"axis": "e"}, e)
}

// SetSensorTemperature updates a sensor temperature
func (km *KlipperMetrics) SetSensorTemperature(name string, temp float64) {
	km.SensorTemperature.Set(Labels{"sensor": name}, temp)
}

// SetHeaterStatus updates heater metrics
func (km *KlipperMetrics) SetHeaterStatus(name string, current, target, pwm float64) {
	km.SensorTemperature.Set(Labels{"sensor": name}, current)
	km.HeaterTarget.Set(Labels{"heater": name}, target)
	km.HeaterPWM.Set(Labels{"heater": name}, pwm)
	km.TemperatureError.Set(Labels{"heater": name}, target-current)
}

// SetFanStatus updates fan metrics
func (km *KlipperMetrics) SetFanStatus(name string, pwm float64, rpm float64) {
	km.FanPWM.Set(Labels{"fan": name}, pwm)
	if rpm >= 0 {
		km.FanRPM.Set(Labels{"fan": name}, rpm)
	}
}

// SetMCUStatus updates MCU metrics
func (km *KlipperMetrics) SetMCUStatus(name string, connected bool, heartbeatPending int, freq float64) {
	connectedVal := float64(0)
	if connected {
		connectedVal = 1
	}
	km.MCUConnected.Set(Labels{"mcu": name}, connectedVal)
	km.MCUHeartbeatPending.Set(Labels{"mcu": name}, float64(heartbeatPending))
	km.MCUFrequency.Set(Labels{"mcu": name}, freq)
}

// RecordMCULatency records MCU command latency
func (km *KlipperMetrics) RecordMCULatency(name string, latency time.Duration) {
	km.MCULatency.Observe(Labels{"mcu": name}, latency.Seconds())
}

// IncrementMCUMessages increments MCU message counters
func (km *KlipperMetrics) IncrementMCUMessages(name string, sent, received uint64) {
	if sent > 0 {
		km.MCUMessagesSent.Add(Labels{"mcu": name}, sent)
	}
	if received > 0 {
		km.MCUMessagesReceived.Add(Labels{"mcu": name}, received)
	}
}

// RecordGCodeCommand records a G-code command execution
func (km *KlipperMetrics) RecordGCodeCommand(cmdType string, duration time.Duration) {
	km.GCodeCommandsTotal.Inc(Labels{"type": cmdType})
	km.GCodeExecutionTime.Observe(Labels{"type": cmdType}, duration.Seconds())
}

// RecordError records an error
func (km *KlipperMetrics) RecordError(errorType string) {
	km.ErrorsTotal.Inc(Labels{"type": errorType})
}

// RecordWarning records a warning
func (km *KlipperMetrics) RecordWarning(warningType string) {
	km.WarningsTotal.Inc(Labels{"type": warningType})
}

// RecordShutdown records a shutdown event
func (km *KlipperMetrics) RecordShutdown(reason string) {
	km.ShutdownEvents.Inc(Labels{"reason": reason})
}

// SetPrintState updates print state
// States: 0=standby, 1=printing, 2=paused, 3=complete, 4=cancelled, 5=error
func (km *KlipperMetrics) SetPrintState(state int, duration float64, filamentUsed float64) {
	km.PrintState.Set(nil, float64(state))
	km.PrintDuration.Set(nil, duration)
	km.PrintFilamentUsed.Set(nil, filamentUsed)
}

// Gather returns all metrics in Prometheus text format
func (km *KlipperMetrics) Gather() string {
	km.UpdateSystemMetrics()
	return km.registry.Gather()
}

// Registry returns the internal registry
func (km *KlipperMetrics) Registry() *Registry {
	return km.registry
}

// PrintStateConst defines print state constants
const (
	PrintStateStandby   = 0
	PrintStatePrinting  = 1
	PrintStatePaused    = 2
	PrintStateComplete  = 3
	PrintStateCancelled = 4
	PrintStateError     = 5
)

// Global metrics instance
var globalMetrics *KlipperMetrics
var globalMetricsOnce sync.Once

// GlobalMetrics returns the global Klipper metrics instance
func GlobalMetrics() *KlipperMetrics {
	globalMetricsOnce.Do(func() {
		globalMetrics = NewKlipperMetrics()
	})
	return globalMetrics
}
