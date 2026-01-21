package hosth4

import (
	"strings"
	"testing"
)

func TestTMC2208DriverCreation(t *testing.T) {
	cfg := DefaultTMC2208Config()
	cfg.Name = "stepper_x"
	cfg.RunCurrent = 0.8

	d, err := newTMC2208Driver(nil, cfg)
	if err != nil {
		t.Fatalf("failed to create driver: %v", err)
	}

	if d.GetName() != "stepper_x" {
		t.Errorf("expected name stepper_x, got %s", d.GetName())
	}

	status := d.GetStatus()
	if status["run_current"] != 0.8 {
		t.Errorf("expected run_current 0.8, got %v", status["run_current"])
	}
	if status["microsteps"] != 16 {
		t.Errorf("expected microsteps 16, got %v", status["microsteps"])
	}
}

func TestTMC2208InvalidConfig(t *testing.T) {
	cfg := DefaultTMC2208Config()
	cfg.RunCurrent = 0 // Invalid

	_, err := newTMC2208Driver(nil, cfg)
	if err == nil {
		t.Error("expected error for zero run_current")
	}

	cfg.RunCurrent = 0.8
	cfg.Microsteps = 15 // Invalid
	_, err = newTMC2208Driver(nil, cfg)
	if err == nil {
		t.Error("expected error for invalid microsteps")
	}
}

func TestTMC2208SetCurrent(t *testing.T) {
	cfg := DefaultTMC2208Config()
	cfg.Name = "stepper_x"
	cfg.RunCurrent = 0.8

	d, _ := newTMC2208Driver(nil, cfg)

	err := d.SetCurrent(1.0, 0.5)
	if err != nil {
		t.Fatalf("SetCurrent failed: %v", err)
	}

	status := d.GetStatus()
	if status["run_current"] != 1.0 {
		t.Errorf("expected run_current 1.0, got %v", status["run_current"])
	}
	if status["hold_current"] != 0.5 {
		t.Errorf("expected hold_current 0.5, got %v", status["hold_current"])
	}
}

func TestTMC2208SetMicrosteps(t *testing.T) {
	cfg := DefaultTMC2208Config()
	cfg.Name = "stepper_x"
	cfg.RunCurrent = 0.8

	d, _ := newTMC2208Driver(nil, cfg)

	err := d.SetMicrosteps(32)
	if err != nil {
		t.Fatalf("SetMicrosteps failed: %v", err)
	}

	status := d.GetStatus()
	if status["microsteps"] != 32 {
		t.Errorf("expected microsteps 32, got %v", status["microsteps"])
	}

	// Invalid value
	err = d.SetMicrosteps(15)
	if err == nil {
		t.Error("expected error for invalid microsteps")
	}
}

func TestTMC2209DriverCreation(t *testing.T) {
	cfg := DefaultTMC2209Config()
	cfg.TMC2208Config.Name = "stepper_x"
	cfg.TMC2208Config.RunCurrent = 1.2
	cfg.StallGuardThreshold = 100

	d, err := newTMC2209Driver(nil, cfg)
	if err != nil {
		t.Fatalf("failed to create driver: %v", err)
	}

	if d.GetName() != "stepper_x" {
		t.Errorf("expected name stepper_x, got %s", d.GetName())
	}

	status := d.GetStatus()
	if status["run_current"] != 1.2 {
		t.Errorf("expected run_current 1.2, got %v", status["run_current"])
	}
	if status["stallguard_threshold"].(uint8) != 100 {
		t.Errorf("expected stallguard_threshold 100, got %v", status["stallguard_threshold"])
	}
}

func TestTMC2209StallGuard(t *testing.T) {
	cfg := DefaultTMC2209Config()
	cfg.TMC2208Config.Name = "stepper_x"
	cfg.TMC2208Config.RunCurrent = 1.0

	d, _ := newTMC2209Driver(nil, cfg)

	d.SetStallGuardThreshold(150)
	if d.GetStallGuardThreshold() != 150 {
		t.Errorf("expected threshold 150, got %d", d.GetStallGuardThreshold())
	}

	d.SetTCoolThrs(1000)
	if d.GetTCoolThrs() != 1000 {
		t.Errorf("expected tcoolthrs 1000, got %d", d.GetTCoolThrs())
	}
}

func TestTMCDriverManager(t *testing.T) {
	m := newTMCDriverManager(nil)

	// Create and register drivers
	cfg1 := DefaultTMC2208Config()
	cfg1.Name = "stepper_x"
	cfg1.RunCurrent = 0.8
	d1, _ := newTMC2208Driver(nil, cfg1)
	m.RegisterDriver("stepper_x", d1)

	cfg2 := DefaultTMC2209Config()
	cfg2.TMC2208Config.Name = "stepper_y"
	cfg2.TMC2208Config.RunCurrent = 1.0
	d2, _ := newTMC2209Driver(nil, cfg2)
	m.RegisterDriver("stepper_y", d2)

	// Get driver
	driver, ok := m.GetDriver("stepper_x")
	if !ok {
		t.Error("stepper_x not found")
	}
	if driver.GetName() != "stepper_x" {
		t.Errorf("wrong driver name: %s", driver.GetName())
	}

	// Get all drivers
	all := m.GetAllDrivers()
	if len(all) != 2 {
		t.Errorf("expected 2 drivers, got %d", len(all))
	}
}

func TestCmdSetTMCCurrent(t *testing.T) {
	m := newTMCDriverManager(nil)

	cfg := DefaultTMC2208Config()
	cfg.Name = "stepper_x"
	cfg.RunCurrent = 0.8
	cfg.HoldCurrent = 0.4
	d, _ := newTMC2208Driver(nil, cfg)
	m.RegisterDriver("stepper_x", d)

	// Query current
	result, err := m.cmdSetTMCCurrent(map[string]string{"STEPPER": "stepper_x"})
	if err != nil {
		t.Fatalf("cmdSetTMCCurrent failed: %v", err)
	}
	if !strings.Contains(result, "0.80A") {
		t.Errorf("expected run current in result: %s", result)
	}

	// Set new current
	result, err = m.cmdSetTMCCurrent(map[string]string{
		"STEPPER": "stepper_x",
		"CURRENT": "1.0",
	})
	if err != nil {
		t.Fatalf("cmdSetTMCCurrent failed: %v", err)
	}
	if !strings.Contains(result, "1.00A") {
		t.Errorf("expected new current in result: %s", result)
	}

	// Missing stepper
	_, err = m.cmdSetTMCCurrent(map[string]string{})
	if err == nil {
		t.Error("expected error for missing STEPPER")
	}

	// Unknown stepper
	_, err = m.cmdSetTMCCurrent(map[string]string{"STEPPER": "unknown"})
	if err == nil {
		t.Error("expected error for unknown stepper")
	}
}

func TestCmdDumpTMC(t *testing.T) {
	m := newTMCDriverManager(nil)

	cfg := DefaultTMC2209Config()
	cfg.TMC2208Config.Name = "stepper_x"
	cfg.TMC2208Config.RunCurrent = 0.8
	d, _ := newTMC2209Driver(nil, cfg)
	m.RegisterDriver("stepper_x", d)

	result, err := m.cmdDumpTMC(map[string]string{"STEPPER": "stepper_x"})
	if err != nil {
		t.Fatalf("cmdDumpTMC failed: %v", err)
	}

	if !strings.Contains(result, "stepper_x") {
		t.Error("expected stepper name in dump")
	}
	if !strings.Contains(result, "run_current") {
		t.Error("expected run_current in dump")
	}
}

func TestCmdInitTMC(t *testing.T) {
	m := newTMCDriverManager(nil)

	cfg := DefaultTMC2208Config()
	cfg.Name = "stepper_x"
	cfg.RunCurrent = 0.8
	d, _ := newTMC2208Driver(nil, cfg)
	m.RegisterDriver("stepper_x", d)

	result, err := m.cmdInitTMC(map[string]string{"STEPPER": "stepper_x"})
	if err != nil {
		t.Fatalf("cmdInitTMC failed: %v", err)
	}

	if !strings.Contains(result, "initialized") {
		t.Error("expected 'initialized' in result")
	}
}

func TestCmdSetTMCField(t *testing.T) {
	m := newTMCDriverManager(nil)

	cfg := DefaultTMC2208Config()
	cfg.Name = "stepper_x"
	cfg.RunCurrent = 0.8
	d, _ := newTMC2208Driver(nil, cfg)
	m.RegisterDriver("stepper_x", d)

	result, err := m.cmdSetTMCField(map[string]string{
		"STEPPER": "stepper_x",
		"FIELD":   "toff",
		"VALUE":   "4",
	})
	if err != nil {
		t.Fatalf("cmdSetTMCField failed: %v", err)
	}

	if !strings.Contains(result, "toff") {
		t.Error("expected field name in result")
	}
}

func TestMicrostepsConversion(t *testing.T) {
	tests := []struct {
		microsteps int
		mres       uint32
	}{
		{256, 0},
		{128, 1},
		{64, 2},
		{32, 3},
		{16, 4},
		{8, 5},
		{4, 6},
		{2, 7},
		{1, 8},
	}

	for _, tc := range tests {
		mres, err := MicrostepsToMRES(tc.microsteps)
		if err != nil {
			t.Errorf("MicrostepsToMRES(%d) failed: %v", tc.microsteps, err)
			continue
		}
		if mres != tc.mres {
			t.Errorf("MicrostepsToMRES(%d) = %d, want %d", tc.microsteps, mres, tc.mres)
		}

		ms := MRESToMicrosteps(tc.mres)
		if ms != tc.microsteps {
			t.Errorf("MRESToMicrosteps(%d) = %d, want %d", tc.mres, ms, tc.microsteps)
		}
	}

	// Invalid microsteps
	_, err := MicrostepsToMRES(15)
	if err == nil {
		t.Error("expected error for invalid microsteps")
	}
}

func TestTMCFieldHelper(t *testing.T) {
	fields := map[string]map[string]uint32{
		"CHOPCONF": {
			"toff":  0x0F,
			"hstrt": 0x70,
		},
	}

	fh := newTMCFieldHelper(fields, nil)

	fh.SetField("toff", 3)
	fh.SetField("hstrt", 5)

	if fh.GetField("toff") != 3 {
		t.Errorf("expected toff=3, got %d", fh.GetField("toff"))
	}

	regVal := fh.GetRegisterValue("CHOPCONF")
	// toff=3 in bits 0-3, hstrt=5 in bits 4-6
	// 3 | (5 << 4) = 3 | 80 = 83
	expected := uint32(3) | (uint32(5) << 4)
	if regVal != expected {
		t.Errorf("expected register value %d, got %d", expected, regVal)
	}
}

func TestTMCCurrentHelper(t *testing.T) {
	ch := NewTMCCurrentHelper(1.0, 0.5, 0.110)

	if ch.GetRunCurrent() != 1.0 {
		t.Errorf("expected run_current 1.0, got %f", ch.GetRunCurrent())
	}
	if ch.GetHoldCurrent() != 0.5 {
		t.Errorf("expected hold_current 0.5, got %f", ch.GetHoldCurrent())
	}

	// Default hold current to run current
	ch2 := NewTMCCurrentHelper(1.0, 0, 0.110)
	if ch2.GetHoldCurrent() != 1.0 {
		t.Errorf("expected hold_current to default to run_current")
	}
}
