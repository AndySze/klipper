// Diagnostics commands tests
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"strings"
	"testing"
)

func TestNewCommandManager(t *testing.T) {
	cm := newCommandManager(nil)
	if cm == nil {
		t.Fatal("expected command manager, got nil")
	}

	// Check builtin commands registered
	commands := cm.ListCommands()
	builtins := []string{"STATUS", "HELP", "M115", "GET_UPTIME", "DEBUG_READ", "DEBUG_WRITE", "DEBUG_STATS", "DEBUG_TIMING"}
	for _, name := range builtins {
		if _, ok := commands[name]; !ok {
			t.Errorf("expected builtin command %s to be registered", name)
		}
	}
}

func TestCommandManagerStatus(t *testing.T) {
	cm := newCommandManager(nil)

	// Default ready state
	result, err, handled := cm.HandleCommand("STATUS", nil)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !handled {
		t.Fatal("expected STATUS to be handled")
	}
	if !strings.Contains(result, "Ready") {
		t.Errorf("expected 'Ready' in result, got: %s", result)
	}

	// Not ready state
	cm.SetReady(false, "MCU error")
	result, err, _ = cm.HandleCommand("STATUS", nil)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !strings.Contains(result, "Not ready") {
		t.Errorf("expected 'Not ready' in result, got: %s", result)
	}
	if !strings.Contains(result, "MCU error") {
		t.Errorf("expected 'MCU error' in result, got: %s", result)
	}
}

func TestCommandManagerHelp(t *testing.T) {
	cm := newCommandManager(nil)

	result, err, handled := cm.HandleCommand("HELP", nil)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !handled {
		t.Fatal("expected HELP to be handled")
	}
	if !strings.Contains(result, "Available extended commands") {
		t.Errorf("expected 'Available extended commands' in result, got: %s", result)
	}
	if !strings.Contains(result, "STATUS") {
		t.Errorf("expected STATUS command in help, got: %s", result)
	}
}

func TestCommandManagerM115(t *testing.T) {
	cm := newCommandManager(nil)

	result, err, handled := cm.HandleCommand("M115", nil)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !handled {
		t.Fatal("expected M115 to be handled")
	}
	if !strings.Contains(result, "FIRMWARE_NAME") {
		t.Errorf("expected FIRMWARE_NAME in result, got: %s", result)
	}
	if !strings.Contains(result, "FIRMWARE_VERSION:Go-1.0") {
		t.Errorf("expected FIRMWARE_VERSION:Go-1.0 in result, got: %s", result)
	}
}

func TestCommandManagerGetUptime(t *testing.T) {
	cm := newCommandManager(nil)

	result, err, handled := cm.HandleCommand("GET_UPTIME", nil)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !handled {
		t.Fatal("expected GET_UPTIME to be handled")
	}
	if !strings.Contains(result, "Host uptime") {
		t.Errorf("expected 'Host uptime' in result, got: %s", result)
	}
}

func TestCommandManagerDebugRead(t *testing.T) {
	cm := newCommandManager(nil)

	// Missing ADDR
	_, err, _ := cm.HandleCommand("DEBUG_READ", map[string]string{})
	if err == nil {
		t.Error("expected error for missing ADDR")
	}

	// Valid decimal address
	result, err, handled := cm.HandleCommand("DEBUG_READ", map[string]string{
		"ADDR": "1024",
	})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !handled {
		t.Fatal("expected DEBUG_READ to be handled")
	}
	if !strings.Contains(result, "0x00000400") {
		t.Errorf("expected address 0x00000400 in result, got: %s", result)
	}

	// Valid hex address
	result, err, _ = cm.HandleCommand("DEBUG_READ", map[string]string{
		"ADDR": "0xFF",
		"CHIP": "mcu2",
	})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !strings.Contains(result, "0x000000FF") {
		t.Errorf("expected address 0x000000FF in result, got: %s", result)
	}
	if !strings.Contains(result, "mcu2") {
		t.Errorf("expected chip 'mcu2' in result, got: %s", result)
	}

	// Invalid address
	_, err, _ = cm.HandleCommand("DEBUG_READ", map[string]string{
		"ADDR": "invalid",
	})
	if err == nil {
		t.Error("expected error for invalid address")
	}
}

func TestCommandManagerDebugWrite(t *testing.T) {
	cm := newCommandManager(nil)

	// Missing ADDR
	_, err, _ := cm.HandleCommand("DEBUG_WRITE", map[string]string{
		"VAL": "123",
	})
	if err == nil {
		t.Error("expected error for missing ADDR")
	}

	// Missing VAL
	_, err, _ = cm.HandleCommand("DEBUG_WRITE", map[string]string{
		"ADDR": "0x1000",
	})
	if err == nil {
		t.Error("expected error for missing VAL")
	}

	// Valid write
	result, err, handled := cm.HandleCommand("DEBUG_WRITE", map[string]string{
		"ADDR": "0x1000",
		"VAL":  "0xDEADBEEF",
	})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !handled {
		t.Fatal("expected DEBUG_WRITE to be handled")
	}
	if !strings.Contains(result, "0x00001000") {
		t.Errorf("expected address 0x00001000 in result, got: %s", result)
	}
	if !strings.Contains(result, "0xDEADBEEF") {
		t.Errorf("expected value 0xDEADBEEF in result, got: %s", result)
	}
}

func TestCommandManagerDebugStats(t *testing.T) {
	cm := newCommandManager(nil)

	result, err, handled := cm.HandleCommand("DEBUG_STATS", nil)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !handled {
		t.Fatal("expected DEBUG_STATS to be handled")
	}
	if !strings.Contains(result, "Debug Statistics") {
		t.Errorf("expected 'Debug Statistics' in result, got: %s", result)
	}
	if !strings.Contains(result, "Go routines") {
		t.Errorf("expected 'Go routines' in result, got: %s", result)
	}
	if !strings.Contains(result, "Memory alloc") {
		t.Errorf("expected 'Memory alloc' in result, got: %s", result)
	}
}

func TestCommandManagerDebugTiming(t *testing.T) {
	cm := newCommandManager(nil)

	result, err, handled := cm.HandleCommand("DEBUG_TIMING", nil)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !handled {
		t.Fatal("expected DEBUG_TIMING to be handled")
	}
	if !strings.Contains(result, "Timing Statistics") {
		t.Errorf("expected 'Timing Statistics' in result, got: %s", result)
	}
}

func TestCommandManagerRegisterCommand(t *testing.T) {
	cm := newCommandManager(nil)

	// Register custom command
	called := false
	cm.RegisterCommand("TEST_CMD", func(args map[string]string) (string, error) {
		called = true
		return "test result", nil
	}, "Test command description")

	result, err, handled := cm.HandleCommand("TEST_CMD", nil)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !handled {
		t.Fatal("expected TEST_CMD to be handled")
	}
	if !called {
		t.Error("expected custom handler to be called")
	}
	if result != "test result" {
		t.Errorf("expected 'test result', got: %s", result)
	}

	// Verify in HELP
	helpResult, _, _ := cm.HandleCommand("HELP", nil)
	if !strings.Contains(helpResult, "TEST_CMD") {
		t.Errorf("expected TEST_CMD in help, got: %s", helpResult)
	}
	if !strings.Contains(helpResult, "Test command description") {
		t.Errorf("expected description in help, got: %s", helpResult)
	}
}

func TestCommandManagerUnregisterCommand(t *testing.T) {
	cm := newCommandManager(nil)

	// Register then unregister
	cm.RegisterCommand("TEMP_CMD", func(args map[string]string) (string, error) {
		return "temp", nil
	}, "Temporary command")

	_, _, handled := cm.HandleCommand("TEMP_CMD", nil)
	if !handled {
		t.Fatal("expected TEMP_CMD to be handled before unregister")
	}

	cm.UnregisterCommand("TEMP_CMD")

	_, _, handled = cm.HandleCommand("TEMP_CMD", nil)
	if handled {
		t.Error("expected TEMP_CMD to not be handled after unregister")
	}
}

func TestCommandManagerUnknownCommand(t *testing.T) {
	cm := newCommandManager(nil)

	_, _, handled := cm.HandleCommand("UNKNOWN_CMD", nil)
	if handled {
		t.Error("expected unknown command to not be handled")
	}
}

func TestCommandManagerCaseInsensitive(t *testing.T) {
	cm := newCommandManager(nil)

	// Commands should be case-insensitive
	_, _, handled1 := cm.HandleCommand("status", nil)
	_, _, handled2 := cm.HandleCommand("STATUS", nil)
	_, _, handled3 := cm.HandleCommand("Status", nil)

	if !handled1 || !handled2 || !handled3 {
		t.Error("expected command handling to be case-insensitive")
	}
}

func TestCommandManagerGetStatus(t *testing.T) {
	cm := newCommandManager(nil)

	status := cm.GetStatus()
	if status == nil {
		t.Fatal("expected status map, got nil")
	}

	if _, ok := status["is_ready"]; !ok {
		t.Error("expected 'is_ready' in status")
	}
	if _, ok := status["uptime"]; !ok {
		t.Error("expected 'uptime' in status")
	}
	if _, ok := status["num_commands"]; !ok {
		t.Error("expected 'num_commands' in status")
	}
}

func TestCommandManagerSetPrinterName(t *testing.T) {
	cm := newCommandManager(nil)

	cm.SetPrinterName("My Printer")

	result, _, _ := cm.HandleCommand("M115", nil)
	if !strings.Contains(result, "My Printer") {
		t.Errorf("expected 'My Printer' in M115 result, got: %s", result)
	}
}

func TestParseIntAuto(t *testing.T) {
	tests := []struct {
		input    string
		expected int
		hasError bool
	}{
		{"123", 123, false},
		{"0x7B", 123, false},
		{"0X7B", 123, false},
		{"0xff", 255, false},
		{"0xFF", 255, false},
		{"0xDEADBEEF", 0xDEADBEEF, false},
		{"  456  ", 456, false},
		{"invalid", 0, true},
		{"0xinvalid", 0, true},
	}

	for _, tt := range tests {
		result, err := parseIntAuto(tt.input)
		if tt.hasError {
			if err == nil {
				t.Errorf("parseIntAuto(%q): expected error, got nil", tt.input)
			}
		} else {
			if err != nil {
				t.Errorf("parseIntAuto(%q): unexpected error: %v", tt.input, err)
			}
			if result != tt.expected {
				t.Errorf("parseIntAuto(%q): expected %d, got %d", tt.input, tt.expected, result)
			}
		}
	}
}

func TestRegisterExternalCommands(t *testing.T) {
	cm := newCommandManager(nil)

	// Register external commands (help only, no handlers)
	cm.RegisterExternalCommands(map[string]string{
		"EXT_CMD1": "External command 1",
		"EXT_CMD2": "External command 2",
	})

	// Verify in help
	helpResult, _, _ := cm.HandleCommand("HELP", nil)
	if !strings.Contains(helpResult, "EXT_CMD1") {
		t.Errorf("expected EXT_CMD1 in help, got: %s", helpResult)
	}
	if !strings.Contains(helpResult, "EXT_CMD2") {
		t.Errorf("expected EXT_CMD2 in help, got: %s", helpResult)
	}

	// But they shouldn't be executable
	_, _, handled := cm.HandleCommand("EXT_CMD1", nil)
	if handled {
		t.Error("expected EXT_CMD1 to not be handled (no handler registered)")
	}
}
