package config

import (
	"testing"
)

func TestLoadString(t *testing.T) {
	data := `
[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 3000

[stepper_x]
step_pin: PA5
dir_pin: !PA4
enable_pin: PA3
microsteps: 16
rotation_distance: 40
position_max: 200
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	// Test HasSection
	if !cfg.HasSection("printer") {
		t.Error("expected [printer] section to exist")
	}
	if !cfg.HasSection("stepper_x") {
		t.Error("expected [stepper_x] section to exist")
	}
	if cfg.HasSection("nonexistent") {
		t.Error("expected [nonexistent] section to not exist")
	}

	// Test GetSection
	printer, err := cfg.GetSection("printer")
	if err != nil {
		t.Fatalf("GetSection(printer) failed: %v", err)
	}
	if printer.GetName() != "printer" {
		t.Errorf("expected name 'printer', got '%s'", printer.GetName())
	}

	// Test Get
	kin, err := printer.Get("kinematics")
	if err != nil {
		t.Fatalf("Get(kinematics) failed: %v", err)
	}
	if kin != "cartesian" {
		t.Errorf("expected 'cartesian', got '%s'", kin)
	}

	// Test GetInt
	maxVel, err := printer.GetInt("max_velocity")
	if err != nil {
		t.Fatalf("GetInt(max_velocity) failed: %v", err)
	}
	if maxVel != 300 {
		t.Errorf("expected 300, got %d", maxVel)
	}

	// Test GetFloat
	maxAccel, err := printer.GetFloat("max_accel")
	if err != nil {
		t.Fatalf("GetFloat(max_accel) failed: %v", err)
	}
	if maxAccel != 3000.0 {
		t.Errorf("expected 3000.0, got %f", maxAccel)
	}
}

func TestSectionGet(t *testing.T) {
	data := `
[test]
string_val: hello
int_val: 42
float_val: 3.14
bool_true: true
bool_false: no
bool_one: 1
list_val: a, b, c
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	sec, _ := cfg.GetSection("test")

	// Test Get with fallback
	val, _ := sec.Get("missing", "default")
	if val != "default" {
		t.Errorf("expected 'default', got '%s'", val)
	}

	// Test GetInt
	i, _ := sec.GetInt("int_val")
	if i != 42 {
		t.Errorf("expected 42, got %d", i)
	}

	// Test GetInt with fallback
	i, _ = sec.GetInt("missing", 99)
	if i != 99 {
		t.Errorf("expected 99, got %d", i)
	}

	// Test GetFloat
	f, _ := sec.GetFloat("float_val")
	if f != 3.14 {
		t.Errorf("expected 3.14, got %f", f)
	}

	// Test GetBool
	b, _ := sec.GetBool("bool_true")
	if !b {
		t.Error("expected true")
	}

	b, _ = sec.GetBool("bool_false")
	if b {
		t.Error("expected false")
	}

	b, _ = sec.GetBool("bool_one")
	if !b {
		t.Error("expected true for '1'")
	}

	// Test GetList
	list, _ := sec.GetList("list_val", ",")
	if len(list) != 3 {
		t.Errorf("expected 3 items, got %d", len(list))
	}
	if list[0] != "a" || list[1] != "b" || list[2] != "c" {
		t.Errorf("unexpected list values: %v", list)
	}
}

func TestAccessTracking(t *testing.T) {
	data := `
[test]
used1: value1
used2: value2
unused1: value3
unused2: value4
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	sec, _ := cfg.GetSection("test")

	// Access some options
	sec.Get("used1")
	sec.Get("used2")

	// Check accessed options
	accessed := sec.GetAccessedOptions()
	if len(accessed) != 2 {
		t.Errorf("expected 2 accessed options, got %d", len(accessed))
	}

	// Check unused options
	unused := sec.GetUnusedOptions()
	if len(unused) != 2 {
		t.Errorf("expected 2 unused options, got %d", len(unused))
	}
}

func TestSectionTracking(t *testing.T) {
	data := `
[used_section]
key: value

[unused_section]
key: value
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	// Access one section
	cfg.GetSection("used_section")

	// Check accessed sections
	accessed := cfg.GetAccessedSections()
	if len(accessed) != 1 {
		t.Errorf("expected 1 accessed section, got %d", len(accessed))
	}

	// Check unused sections
	unused := cfg.GetUnusedSections()
	if len(unused) != 1 {
		t.Errorf("expected 1 unused section, got %d", len(unused))
	}
	if unused[0] != "unused_section" {
		t.Errorf("expected 'unused_section', got '%s'", unused[0])
	}
}

func TestGetPrefixSections(t *testing.T) {
	data := `
[stepper_x]
key: x

[stepper_y]
key: y

[stepper_z]
key: z

[printer]
key: printer
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	steppers := cfg.GetPrefixSections("stepper_")
	if len(steppers) != 3 {
		t.Errorf("expected 3 stepper sections, got %d", len(steppers))
	}
}

func TestParsePin(t *testing.T) {
	tests := []struct {
		desc     string
		opts     PinOptions
		wantName string
		wantChip string
		wantInv  bool
		wantPull int
		wantErr  bool
	}{
		{
			desc:     "PA5",
			opts:     PinOptions{},
			wantName: "PA5",
			wantChip: "mcu",
		},
		{
			desc:     "!PA5",
			opts:     PinOptions{CanInvert: true},
			wantName: "PA5",
			wantChip: "mcu",
			wantInv:  true,
		},
		{
			desc:     "^PA5",
			opts:     PinOptions{CanPullup: true},
			wantName: "PA5",
			wantChip: "mcu",
			wantPull: 1,
		},
		{
			desc:     "~PA5",
			opts:     PinOptions{CanPullup: true},
			wantName: "PA5",
			wantChip: "mcu",
			wantPull: -1,
		},
		{
			desc:     "^!PA5",
			opts:     PinOptions{CanInvert: true, CanPullup: true},
			wantName: "PA5",
			wantChip: "mcu",
			wantInv:  true,
			wantPull: 1,
		},
		{
			desc:     "mcu:PA5",
			opts:     PinOptions{},
			wantName: "PA5",
			wantChip: "mcu",
		},
		{
			desc:     "probe:z_virtual_endstop",
			opts:     PinOptions{},
			wantName: "z_virtual_endstop",
			wantChip: "probe",
		},
		{
			desc:    "",
			opts:    PinOptions{},
			wantErr: true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.desc, func(t *testing.T) {
			pin, err := ParsePin(tt.desc, tt.opts)
			if tt.wantErr {
				if err == nil {
					t.Error("expected error")
				}
				return
			}
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if pin.Name != tt.wantName {
				t.Errorf("name: got %q, want %q", pin.Name, tt.wantName)
			}
			if pin.Chip != tt.wantChip {
				t.Errorf("chip: got %q, want %q", pin.Chip, tt.wantChip)
			}
			if pin.Invert != tt.wantInv {
				t.Errorf("invert: got %v, want %v", pin.Invert, tt.wantInv)
			}
			if pin.Pullup != tt.wantPull {
				t.Errorf("pullup: got %v, want %v", pin.Pullup, tt.wantPull)
			}
		})
	}
}

func TestGetChoice(t *testing.T) {
	data := `
[test]
mode: fast
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	sec, _ := cfg.GetSection("test")

	// Valid choice
	mode, err := sec.GetChoice("mode", []string{"slow", "fast", "turbo"})
	if err != nil {
		t.Fatalf("GetChoice failed: %v", err)
	}
	if mode != "fast" {
		t.Errorf("expected 'fast', got '%s'", mode)
	}

	// Invalid choice
	_, err = sec.GetChoice("mode", []string{"slow", "turbo"})
	if err == nil {
		t.Error("expected error for invalid choice")
	}
}

func TestBoundsChecking(t *testing.T) {
	data := `
[test]
value: 50
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	sec, _ := cfg.GetSection("test")

	// Within bounds
	min := 0.0
	max := 100.0
	v, err := sec.GetFloatWithBounds("value", FloatBounds{MinVal: &min, MaxVal: &max})
	if err != nil {
		t.Fatalf("GetFloatWithBounds failed: %v", err)
	}
	if v != 50.0 {
		t.Errorf("expected 50.0, got %f", v)
	}

	// Below minimum
	min = 60.0
	_, err = sec.GetFloatWithBounds("value", FloatBounds{MinVal: &min})
	if err == nil {
		t.Error("expected error for value below minimum")
	}

	// Above maximum
	max = 40.0
	_, err = sec.GetFloatWithBounds("value", FloatBounds{MaxVal: &max})
	if err == nil {
		t.Error("expected error for value above maximum")
	}

	// Must be above
	above := 50.0
	_, err = sec.GetFloatWithBounds("value", FloatBounds{Above: &above})
	if err == nil {
		t.Error("expected error for value not above threshold")
	}
}

func TestMissingOptionError(t *testing.T) {
	data := `
[test]
exists: value
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	sec, _ := cfg.GetSection("test")

	// Missing required option
	_, err = sec.Get("missing")
	if err == nil {
		t.Error("expected error for missing option")
	}

	configErr, ok := err.(*ConfigError)
	if !ok {
		t.Errorf("expected *ConfigError, got %T", err)
	}
	if configErr.Section != "test" {
		t.Errorf("expected section 'test', got '%s'", configErr.Section)
	}
	if configErr.Option != "missing" {
		t.Errorf("expected option 'missing', got '%s'", configErr.Option)
	}
}

func TestConfigMerge(t *testing.T) {
	base := `
[printer]
kinematics: cartesian
max_velocity: 300

[stepper_x]
position_max: 200
`

	override := `
[printer]
max_velocity: 500

[stepper_y]
position_max: 200
`

	baseCfg, _ := LoadString(base)
	overrideCfg, _ := LoadString(override)

	baseCfg.Merge(overrideCfg)

	// Check merged value
	printer, _ := baseCfg.GetSection("printer")
	v, _ := printer.GetInt("max_velocity")
	if v != 500 {
		t.Errorf("expected 500 after merge, got %d", v)
	}

	// Check original value preserved
	kin, _ := printer.Get("kinematics")
	if kin != "cartesian" {
		t.Errorf("expected 'cartesian', got '%s'", kin)
	}

	// Check new section added
	if !baseCfg.HasSection("stepper_y") {
		t.Error("expected [stepper_y] section after merge")
	}
}
