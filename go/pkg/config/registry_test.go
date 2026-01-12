package config

import (
	"testing"
)

// testModule is a simple module for testing.
type testModule struct {
	name string
}

func (m *testModule) GetName() string {
	return m.name
}

func TestRegistryExactMatch(t *testing.T) {
	r := NewRegistry()

	// Register exact match
	r.Register("printer", func(sec *Section) (Module, error) {
		return &testModule{name: sec.GetName()}, nil
	})

	// Test factory lookup
	factory := r.GetFactory("printer")
	if factory == nil {
		t.Fatal("expected factory for 'printer'")
	}

	// Test non-match
	factory = r.GetFactory("extruder")
	if factory != nil {
		t.Fatal("expected no factory for 'extruder'")
	}
}

func TestRegistryPrefixMatch(t *testing.T) {
	r := NewRegistry()

	// Register prefix match
	r.RegisterPrefix("stepper", func(sec *Section) (Module, error) {
		return &testModule{name: sec.GetName()}, nil
	})

	// Test matches
	tests := []struct {
		name    string
		matches bool
	}{
		{"stepper_x", true},
		{"stepper_y", true},
		{"stepper_z", true},
		{"stepper", true}, // Full prefix match also works
		{"extruder", false},
	}

	for _, tt := range tests {
		factory := r.GetFactory(tt.name)
		if tt.matches && factory == nil {
			t.Errorf("expected factory for %q", tt.name)
		}
		if !tt.matches && factory != nil {
			t.Errorf("expected no factory for %q", tt.name)
		}
	}
}

func TestRegistryWithPrefixMatch(t *testing.T) {
	r := NewRegistry()

	// Register full prefix match (Python-style named sections)
	r.RegisterWithPrefix("heater_generic ", func(sec *Section) (Module, error) {
		return &testModule{name: sec.GetName()}, nil
	})

	// Test matches
	tests := []struct {
		name    string
		matches bool
	}{
		{"heater_generic chamber", true},
		{"heater_generic bed_outer", true},
		{"heater_generic", false}, // No space and name
		{"heater_bed", false},
	}

	for _, tt := range tests {
		factory := r.GetFactory(tt.name)
		if tt.matches && factory == nil {
			t.Errorf("expected factory for %q", tt.name)
		}
		if !tt.matches && factory != nil {
			t.Errorf("expected no factory for %q", tt.name)
		}
	}
}

func TestRegistryLoadModules(t *testing.T) {
	data := `
[printer]
kinematics: cartesian

[stepper_x]
step_pin: PA0

[stepper_y]
step_pin: PA1

[fan]
pin: PA2
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	r := NewRegistry()

	// Register factories
	r.Register("printer", func(sec *Section) (Module, error) {
		return &testModule{name: sec.GetName()}, nil
	})
	r.RegisterPrefix("stepper", func(sec *Section) (Module, error) {
		return &testModule{name: sec.GetName()}, nil
	})
	r.Register("fan", func(sec *Section) (Module, error) {
		return &testModule{name: sec.GetName()}, nil
	})

	// Load modules
	modules, err := r.LoadModules(cfg)
	if err != nil {
		t.Fatalf("LoadModules failed: %v", err)
	}

	// Verify all modules loaded
	expected := []string{"printer", "stepper_x", "stepper_y", "fan"}
	for _, name := range expected {
		if _, ok := modules[name]; !ok {
			t.Errorf("expected module %q to be loaded", name)
		}
	}

	if len(modules) != len(expected) {
		t.Errorf("expected %d modules, got %d", len(expected), len(modules))
	}
}

func TestRegistryGetModule(t *testing.T) {
	data := `
[printer]
kinematics: cartesian
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	r := NewRegistry()
	r.Register("printer", func(sec *Section) (Module, error) {
		return &testModule{name: "printer"}, nil
	})

	// Load modules
	_, err = r.LoadModules(cfg)
	if err != nil {
		t.Fatalf("LoadModules failed: %v", err)
	}

	// Get loaded module
	m := r.GetModule("printer")
	if m == nil {
		t.Fatal("expected to get printer module")
	}
	if m.GetName() != "printer" {
		t.Errorf("expected name 'printer', got %q", m.GetName())
	}

	// Get non-existent module
	m = r.GetModule("nonexistent")
	if m != nil {
		t.Error("expected nil for nonexistent module")
	}
}

func TestRegistryClear(t *testing.T) {
	data := `
[printer]
kinematics: cartesian
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	r := NewRegistry()
	r.Register("printer", func(sec *Section) (Module, error) {
		return &testModule{name: "printer"}, nil
	})

	// Load modules
	_, err = r.LoadModules(cfg)
	if err != nil {
		t.Fatalf("LoadModules failed: %v", err)
	}

	// Verify module loaded
	if r.GetModule("printer") == nil {
		t.Fatal("expected printer module to be loaded")
	}

	// Clear
	r.Clear()

	// Verify module cleared
	if r.GetModule("printer") != nil {
		t.Error("expected printer module to be cleared")
	}
}

func TestRegistryExactTakesPrecedence(t *testing.T) {
	r := NewRegistry()

	exactCalled := false
	prefixCalled := false

	// Register both exact and prefix for "stepper"
	r.Register("stepper_x", func(sec *Section) (Module, error) {
		exactCalled = true
		return &testModule{name: "exact"}, nil
	})
	r.RegisterPrefix("stepper", func(sec *Section) (Module, error) {
		prefixCalled = true
		return &testModule{name: "prefix"}, nil
	})

	data := `
[stepper_x]
step_pin: PA0

[stepper_y]
step_pin: PA1
`

	cfg, err := LoadString(data)
	if err != nil {
		t.Fatalf("LoadString failed: %v", err)
	}

	modules, err := r.LoadModules(cfg)
	if err != nil {
		t.Fatalf("LoadModules failed: %v", err)
	}

	// stepper_x should use exact match
	if m, ok := modules["stepper_x"]; ok {
		if m.GetName() != "exact" {
			t.Error("stepper_x should use exact match factory")
		}
	}

	// stepper_y should use prefix match
	if m, ok := modules["stepper_y"]; ok {
		if m.GetName() != "prefix" {
			t.Error("stepper_y should use prefix match factory")
		}
	}

	if !exactCalled {
		t.Error("exact factory should have been called")
	}
	if !prefixCalled {
		t.Error("prefix factory should have been called")
	}
}
