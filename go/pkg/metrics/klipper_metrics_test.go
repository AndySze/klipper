// Unit tests for Klipper-specific metrics
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package metrics

import (
	"strings"
	"testing"
	"time"
)

// TestNewKlipperMetrics tests metrics initialization
func TestNewKlipperMetrics(t *testing.T) {
	km := NewKlipperMetrics()

	// Check all metrics are initialized
	if km.ToolheadPosition == nil {
		t.Error("ToolheadPosition should be initialized")
	}
	if km.StepsExecuted == nil {
		t.Error("StepsExecuted should be initialized")
	}
	if km.MovePlanningTime == nil {
		t.Error("MovePlanningTime should be initialized")
	}
	if km.SensorTemperature == nil {
		t.Error("SensorTemperature should be initialized")
	}
	if km.MCUConnected == nil {
		t.Error("MCUConnected should be initialized")
	}
	if km.PrintState == nil {
		t.Error("PrintState should be initialized")
	}
	if km.ErrorsTotal == nil {
		t.Error("ErrorsTotal should be initialized")
	}

	// Check registry has metrics
	if km.Registry() == nil {
		t.Error("Registry should be initialized")
	}
}

// TestSetToolheadPosition tests position updates
func TestSetToolheadPosition(t *testing.T) {
	km := NewKlipperMetrics()

	km.SetToolheadPosition(100.5, 200.0, 10.25, 50.0)

	if v := km.ToolheadPosition.Get(Labels{"axis": "x"}); v != 100.5 {
		t.Errorf("expected x=100.5, got %f", v)
	}
	if v := km.ToolheadPosition.Get(Labels{"axis": "y"}); v != 200.0 {
		t.Errorf("expected y=200.0, got %f", v)
	}
	if v := km.ToolheadPosition.Get(Labels{"axis": "z"}); v != 10.25 {
		t.Errorf("expected z=10.25, got %f", v)
	}
	if v := km.ToolheadPosition.Get(Labels{"axis": "e"}); v != 50.0 {
		t.Errorf("expected e=50.0, got %f", v)
	}
}

// TestSetSensorTemperature tests temperature updates
func TestSetSensorTemperature(t *testing.T) {
	km := NewKlipperMetrics()

	km.SetSensorTemperature("extruder", 200.5)
	km.SetSensorTemperature("heater_bed", 60.0)

	if v := km.SensorTemperature.Get(Labels{"sensor": "extruder"}); v != 200.5 {
		t.Errorf("expected extruder temp 200.5, got %f", v)
	}
	if v := km.SensorTemperature.Get(Labels{"sensor": "heater_bed"}); v != 60.0 {
		t.Errorf("expected bed temp 60.0, got %f", v)
	}
}

// TestSetHeaterStatus tests heater status updates
func TestSetHeaterStatus(t *testing.T) {
	km := NewKlipperMetrics()

	km.SetHeaterStatus("extruder", 195.0, 200.0, 0.75)

	// Check all heater metrics
	if v := km.SensorTemperature.Get(Labels{"sensor": "extruder"}); v != 195.0 {
		t.Errorf("expected current temp 195.0, got %f", v)
	}
	if v := km.HeaterTarget.Get(Labels{"heater": "extruder"}); v != 200.0 {
		t.Errorf("expected target 200.0, got %f", v)
	}
	if v := km.HeaterPWM.Get(Labels{"heater": "extruder"}); v != 0.75 {
		t.Errorf("expected PWM 0.75, got %f", v)
	}
	if v := km.TemperatureError.Get(Labels{"heater": "extruder"}); v != 5.0 {
		t.Errorf("expected error 5.0, got %f", v)
	}
}

// TestSetFanStatus tests fan status updates
func TestSetFanStatus(t *testing.T) {
	km := NewKlipperMetrics()

	km.SetFanStatus("fan0", 0.5, 1200)

	if v := km.FanPWM.Get(Labels{"fan": "fan0"}); v != 0.5 {
		t.Errorf("expected PWM 0.5, got %f", v)
	}
	if v := km.FanRPM.Get(Labels{"fan": "fan0"}); v != 1200 {
		t.Errorf("expected RPM 1200, got %f", v)
	}

	// Test with negative RPM (should not be recorded)
	km.SetFanStatus("fan1", 0.3, -1)
	if v := km.FanRPM.Get(Labels{"fan": "fan1"}); v != 0 {
		t.Errorf("expected RPM 0 for unset, got %f", v)
	}
}

// TestSetMCUStatus tests MCU status updates
func TestSetMCUStatus(t *testing.T) {
	km := NewKlipperMetrics()

	km.SetMCUStatus("mcu", true, 2, 72000000)

	if v := km.MCUConnected.Get(Labels{"mcu": "mcu"}); v != 1 {
		t.Errorf("expected connected=1, got %f", v)
	}
	if v := km.MCUHeartbeatPending.Get(Labels{"mcu": "mcu"}); v != 2 {
		t.Errorf("expected pending=2, got %f", v)
	}
	if v := km.MCUFrequency.Get(Labels{"mcu": "mcu"}); v != 72000000 {
		t.Errorf("expected freq=72000000, got %f", v)
	}

	// Test disconnected
	km.SetMCUStatus("mcu", false, 5, 0)
	if v := km.MCUConnected.Get(Labels{"mcu": "mcu"}); v != 0 {
		t.Errorf("expected connected=0, got %f", v)
	}
}

// TestRecordMCULatency tests MCU latency recording
func TestRecordMCULatency(t *testing.T) {
	km := NewKlipperMetrics()

	km.RecordMCULatency("mcu", 5*time.Millisecond)
	km.RecordMCULatency("mcu", 10*time.Millisecond)
	km.RecordMCULatency("mcu", 3*time.Millisecond)

	snap := km.MCULatency.GetSnapshot(Labels{"mcu": "mcu"})

	if snap.Count != 3 {
		t.Errorf("expected count 3, got %d", snap.Count)
	}
	// Sum should be 0.018 seconds
	if snap.Sum < 0.017 || snap.Sum > 0.019 {
		t.Errorf("expected sum ~0.018, got %f", snap.Sum)
	}
}

// TestIncrementMCUMessages tests MCU message counters
func TestIncrementMCUMessages(t *testing.T) {
	km := NewKlipperMetrics()

	km.IncrementMCUMessages("mcu", 100, 50)
	km.IncrementMCUMessages("mcu", 50, 25)

	if v := km.MCUMessagesSent.Get(Labels{"mcu": "mcu"}); v != 150 {
		t.Errorf("expected sent=150, got %d", v)
	}
	if v := km.MCUMessagesReceived.Get(Labels{"mcu": "mcu"}); v != 75 {
		t.Errorf("expected received=75, got %d", v)
	}

	// Test with zero values (should not increment)
	km.IncrementMCUMessages("mcu", 0, 0)
	if v := km.MCUMessagesSent.Get(Labels{"mcu": "mcu"}); v != 150 {
		t.Errorf("expected sent=150 (unchanged), got %d", v)
	}
}

// TestRecordGCodeCommand tests G-code command recording
func TestRecordGCodeCommand(t *testing.T) {
	km := NewKlipperMetrics()

	km.RecordGCodeCommand("G1", 10*time.Millisecond)
	km.RecordGCodeCommand("G1", 15*time.Millisecond)
	km.RecordGCodeCommand("G28", 5*time.Second)

	if v := km.GCodeCommandsTotal.Get(Labels{"type": "G1"}); v != 2 {
		t.Errorf("expected G1 count=2, got %d", v)
	}
	if v := km.GCodeCommandsTotal.Get(Labels{"type": "G28"}); v != 1 {
		t.Errorf("expected G28 count=1, got %d", v)
	}
}

// TestRecordError tests error recording
func TestRecordError(t *testing.T) {
	km := NewKlipperMetrics()

	km.RecordError("timeout")
	km.RecordError("timeout")
	km.RecordError("protocol")

	if v := km.ErrorsTotal.Get(Labels{"type": "timeout"}); v != 2 {
		t.Errorf("expected timeout errors=2, got %d", v)
	}
	if v := km.ErrorsTotal.Get(Labels{"type": "protocol"}); v != 1 {
		t.Errorf("expected protocol errors=1, got %d", v)
	}
}

// TestRecordWarning tests warning recording
func TestRecordWarning(t *testing.T) {
	km := NewKlipperMetrics()

	km.RecordWarning("temp_overshoot")
	km.RecordWarning("temp_overshoot")

	if v := km.WarningsTotal.Get(Labels{"type": "temp_overshoot"}); v != 2 {
		t.Errorf("expected warnings=2, got %d", v)
	}
}

// TestRecordShutdown tests shutdown recording
func TestRecordShutdown(t *testing.T) {
	km := NewKlipperMetrics()

	km.RecordShutdown("mcu_timeout")

	if v := km.ShutdownEvents.Get(Labels{"reason": "mcu_timeout"}); v != 1 {
		t.Errorf("expected shutdowns=1, got %d", v)
	}
}

// TestSetPrintState tests print state updates
func TestSetPrintState(t *testing.T) {
	km := NewKlipperMetrics()

	// Test printing state
	km.SetPrintState(PrintStatePrinting, 3600.0, 500.0)

	if v := km.PrintState.Get(nil); v != float64(PrintStatePrinting) {
		t.Errorf("expected state=1, got %f", v)
	}
	if v := km.PrintDuration.Get(nil); v != 3600.0 {
		t.Errorf("expected duration=3600, got %f", v)
	}
	if v := km.PrintFilamentUsed.Get(nil); v != 500.0 {
		t.Errorf("expected filament=500, got %f", v)
	}
}

// TestPrintStateConstants tests print state constant values
func TestPrintStateConstants(t *testing.T) {
	tests := []struct {
		name     string
		constant int
		expected int
	}{
		{"standby", PrintStateStandby, 0},
		{"printing", PrintStatePrinting, 1},
		{"paused", PrintStatePaused, 2},
		{"complete", PrintStateComplete, 3},
		{"cancelled", PrintStateCancelled, 4},
		{"error", PrintStateError, 5},
	}

	for _, tt := range tests {
		if tt.constant != tt.expected {
			t.Errorf("%s: expected %d, got %d", tt.name, tt.expected, tt.constant)
		}
	}
}

// TestUpdateSystemMetrics tests system metrics update
func TestUpdateSystemMetrics(t *testing.T) {
	km := NewKlipperMetrics()

	// Update system metrics
	km.UpdateSystemMetrics()

	// Check goroutine count is positive
	if v := km.GoGoroutines.Get(nil); v <= 0 {
		t.Errorf("expected goroutines > 0, got %f", v)
	}

	// Check memory is being tracked
	if v := km.GoMemoryHeap.Get(nil); v <= 0 {
		t.Errorf("expected heap memory > 0, got %f", v)
	}
}

// TestGather tests full metrics gathering
func TestGather(t *testing.T) {
	km := NewKlipperMetrics()

	// Set some test values
	km.SetToolheadPosition(100, 200, 10, 50)
	km.SetHeaterStatus("extruder", 200, 200, 0.5)
	km.SetMCUStatus("mcu", true, 0, 72000000)

	output := km.Gather()

	// Check output contains expected metrics
	expectedMetrics := []string{
		"klipper_toolhead_position_mm",
		"klipper_sensor_temperature_celsius",
		"klipper_heater_target_celsius",
		"klipper_mcu_connected",
		"klipper_go_goroutines",
	}

	for _, metric := range expectedMetrics {
		if !strings.Contains(output, metric) {
			t.Errorf("output should contain %s", metric)
		}
	}

	// Check HELP and TYPE lines
	if !strings.Contains(output, "# HELP") {
		t.Error("output should contain HELP lines")
	}
	if !strings.Contains(output, "# TYPE") {
		t.Error("output should contain TYPE lines")
	}
}

// TestGlobalMetrics tests global metrics singleton
func TestGlobalMetrics(t *testing.T) {
	km1 := GlobalMetrics()
	km2 := GlobalMetrics()

	// Should be same instance
	if km1 != km2 {
		t.Error("GlobalMetrics should return same instance")
	}

	// Should be initialized
	if km1 == nil {
		t.Error("GlobalMetrics should not be nil")
	}
}

// TestMultipleMCUs tests metrics for multiple MCUs
func TestMultipleMCUs(t *testing.T) {
	km := NewKlipperMetrics()

	// Set status for multiple MCUs
	km.SetMCUStatus("mcu", true, 0, 72000000)
	km.SetMCUStatus("mcu_extruder", true, 1, 48000000)

	// Check each MCU has its own metrics
	if v := km.MCUFrequency.Get(Labels{"mcu": "mcu"}); v != 72000000 {
		t.Errorf("expected mcu freq=72000000, got %f", v)
	}
	if v := km.MCUFrequency.Get(Labels{"mcu": "mcu_extruder"}); v != 48000000 {
		t.Errorf("expected mcu_extruder freq=48000000, got %f", v)
	}
}

// TestMultipleHeaters tests metrics for multiple heaters
func TestMultipleHeaters(t *testing.T) {
	km := NewKlipperMetrics()

	km.SetHeaterStatus("extruder", 200, 210, 0.8)
	km.SetHeaterStatus("heater_bed", 60, 60, 0.2)
	km.SetHeaterStatus("extruder1", 195, 200, 0.5)

	// Check each heater
	if v := km.HeaterTarget.Get(Labels{"heater": "extruder"}); v != 210 {
		t.Errorf("expected extruder target=210, got %f", v)
	}
	if v := km.HeaterTarget.Get(Labels{"heater": "heater_bed"}); v != 60 {
		t.Errorf("expected bed target=60, got %f", v)
	}
	if v := km.HeaterTarget.Get(Labels{"heater": "extruder1"}); v != 200 {
		t.Errorf("expected extruder1 target=200, got %f", v)
	}
}

// BenchmarkSetToolheadPosition benchmarks position updates
func BenchmarkSetToolheadPosition(b *testing.B) {
	km := NewKlipperMetrics()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		km.SetToolheadPosition(float64(i), float64(i), float64(i), float64(i))
	}
}

// BenchmarkSetHeaterStatus benchmarks heater updates
func BenchmarkSetHeaterStatus(b *testing.B) {
	km := NewKlipperMetrics()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		km.SetHeaterStatus("extruder", float64(i), 200.0, 0.5)
	}
}

// BenchmarkGather benchmarks full metrics gathering
func BenchmarkGather(b *testing.B) {
	km := NewKlipperMetrics()

	// Set some test values
	km.SetToolheadPosition(100, 200, 10, 50)
	km.SetHeaterStatus("extruder", 200, 200, 0.5)
	km.SetMCUStatus("mcu", true, 0, 72000000)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = km.Gather()
	}
}
