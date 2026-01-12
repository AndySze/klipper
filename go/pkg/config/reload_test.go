package config

import (
	"errors"
	"testing"
)

// reloadableModule is a test module that supports hot reload.
type reloadableModule struct {
	name       string
	value      string
	canReload  bool
	reloadErr  error
	reloadCall int
}

func (m *reloadableModule) GetName() string {
	return m.name
}

func (m *reloadableModule) CanReload() bool {
	return m.canReload
}

func (m *reloadableModule) Reload(newConfig *Section) error {
	m.reloadCall++
	if m.reloadErr != nil {
		return m.reloadErr
	}
	val, _ := newConfig.Get("value", "")
	m.value = val
	return nil
}

func TestReloadManagerDetectChanges(t *testing.T) {
	oldData := `
[printer]
kinematics: cartesian

[stepper_x]
step_pin: PA0
`

	newData := `
[printer]
kinematics: corexy

[stepper_y]
step_pin: PA1
`

	oldCfg, _ := LoadString(oldData)
	newCfg, _ := LoadString(newData)

	reg := NewRegistry()
	rm := NewReloadManager(reg, oldCfg, "")

	changed := rm.DetectChanges(newCfg)

	// Should detect: printer (modified), stepper_x (deleted), stepper_y (new)
	if len(changed) != 3 {
		t.Errorf("expected 3 changes, got %d: %v", len(changed), changed)
	}

	// Verify specific changes
	hasChange := func(name string) bool {
		for _, c := range changed {
			if c == name {
				return true
			}
		}
		return false
	}

	if !hasChange("printer") {
		t.Error("expected printer to be changed")
	}
	if !hasChange("stepper_x") {
		t.Error("expected stepper_x to be changed (deleted)")
	}
	if !hasChange("stepper_y") {
		t.Error("expected stepper_y to be changed (new)")
	}
}

func TestReloadManagerNoChanges(t *testing.T) {
	data := `
[printer]
kinematics: cartesian
`

	cfg1, _ := LoadString(data)
	cfg2, _ := LoadString(data)

	reg := NewRegistry()
	rm := NewReloadManager(reg, cfg1, "")

	changed := rm.DetectChanges(cfg2)

	if len(changed) != 0 {
		t.Errorf("expected no changes, got %v", changed)
	}
}

func TestReloadManagerReloadModule(t *testing.T) {
	oldData := `
[test_module]
value: old_value
`

	newData := `
[test_module]
value: new_value
`

	oldCfg, _ := LoadString(oldData)
	newCfg, _ := LoadString(newData)

	reg := NewRegistry()

	// Create and register a reloadable module
	module := &reloadableModule{
		name:      "test_module",
		value:     "old_value",
		canReload: true,
	}

	reg.Register("test_module", func(sec *Section) (Module, error) {
		return module, nil
	})

	// Load module
	reg.LoadModules(oldCfg)

	rm := NewReloadManager(reg, oldCfg, "")

	// Reload with new config
	results, err := rm.ReloadWithConfig(newCfg)
	if err != nil {
		t.Fatalf("ReloadWithConfig failed: %v", err)
	}

	// Check results
	if len(results) != 1 {
		t.Fatalf("expected 1 result, got %d", len(results))
	}

	result := results[0]
	if !result.Success {
		t.Errorf("expected success, got error: %v", result.Error)
	}
	if !result.WasReloaded {
		t.Error("expected WasReloaded to be true")
	}
	if module.reloadCall != 1 {
		t.Errorf("expected 1 reload call, got %d", module.reloadCall)
	}
	if module.value != "new_value" {
		t.Errorf("expected 'new_value', got %q", module.value)
	}
}

func TestReloadManagerNonReloadable(t *testing.T) {
	oldData := `
[test_module]
value: old_value
`

	newData := `
[test_module]
value: new_value
`

	oldCfg, _ := LoadString(oldData)
	newCfg, _ := LoadString(newData)

	reg := NewRegistry()

	// Create a non-reloadable module (using testModule which doesn't implement Reloadable)
	reg.Register("test_module", func(sec *Section) (Module, error) {
		return &testModule{name: "test_module"}, nil
	})

	reg.LoadModules(oldCfg)

	rm := NewReloadManager(reg, oldCfg, "")

	// Check for non-reloadable changes
	changed := rm.DetectChanges(newCfg)
	nonReloadable := rm.HasNonReloadableChanges(changed)

	if len(nonReloadable) != 1 || nonReloadable[0] != "test_module" {
		t.Errorf("expected ['test_module'], got %v", nonReloadable)
	}
}

func TestReloadManagerReloadError(t *testing.T) {
	oldData := `
[test_module]
value: old_value
`

	newData := `
[test_module]
value: new_value
`

	oldCfg, _ := LoadString(oldData)
	newCfg, _ := LoadString(newData)

	reg := NewRegistry()

	// Create a module that fails on reload
	module := &reloadableModule{
		name:      "test_module",
		value:     "old_value",
		canReload: true,
		reloadErr: errors.New("reload failed"),
	}

	reg.Register("test_module", func(sec *Section) (Module, error) {
		return module, nil
	})

	reg.LoadModules(oldCfg)

	rm := NewReloadManager(reg, oldCfg, "")

	results, err := rm.ReloadWithConfig(newCfg)
	if err != nil {
		t.Fatalf("ReloadWithConfig failed: %v", err)
	}

	if len(results) != 1 {
		t.Fatalf("expected 1 result, got %d", len(results))
	}

	result := results[0]
	if result.Success {
		t.Error("expected failure")
	}
	if result.Error == nil {
		t.Error("expected error to be set")
	}
	// Module value should remain unchanged
	if module.value != "old_value" {
		t.Errorf("expected 'old_value' unchanged, got %q", module.value)
	}
}

func TestReloadManagerCallbacks(t *testing.T) {
	oldData := `
[test_module]
value: old_value
`

	newData := `
[test_module]
value: new_value
`

	oldCfg, _ := LoadString(oldData)
	newCfg, _ := LoadString(newData)

	reg := NewRegistry()

	module := &reloadableModule{
		name:      "test_module",
		canReload: true,
	}

	reg.Register("test_module", func(sec *Section) (Module, error) {
		return module, nil
	})

	reg.LoadModules(oldCfg)

	rm := NewReloadManager(reg, oldCfg, "")

	startCalled := false
	completeCalled := false
	var completedResults []ReloadResult

	rm.SetCallbacks(
		func() { startCalled = true },
		func(results []ReloadResult) {
			completeCalled = true
			completedResults = results
		},
	)

	rm.ReloadWithConfig(newCfg)

	if !startCalled {
		t.Error("expected onReloadStart to be called")
	}
	if !completeCalled {
		t.Error("expected onReloadComplete to be called")
	}
	if len(completedResults) != 1 {
		t.Errorf("expected 1 result in callback, got %d", len(completedResults))
	}
}

func TestReloadManagerNewModule(t *testing.T) {
	oldData := `
[printer]
kinematics: cartesian
`

	newData := `
[printer]
kinematics: cartesian

[new_module]
value: test
`

	oldCfg, _ := LoadString(oldData)
	newCfg, _ := LoadString(newData)

	reg := NewRegistry()

	var createdModule *testModule

	reg.Register("printer", func(sec *Section) (Module, error) {
		return &testModule{name: "printer"}, nil
	})

	reg.Register("new_module", func(sec *Section) (Module, error) {
		createdModule = &testModule{name: "new_module"}
		return createdModule, nil
	})

	reg.LoadModules(oldCfg)

	rm := NewReloadManager(reg, oldCfg, "")

	results, err := rm.ReloadWithConfig(newCfg)
	if err != nil {
		t.Fatalf("ReloadWithConfig failed: %v", err)
	}

	// Should have result for new_module
	var newModuleResult *ReloadResult
	for i := range results {
		if results[i].Section == "new_module" {
			newModuleResult = &results[i]
			break
		}
	}

	if newModuleResult == nil {
		t.Fatal("expected result for new_module")
	}

	if !newModuleResult.Success {
		t.Errorf("expected success for new module, got error: %v", newModuleResult.Error)
	}

	if createdModule == nil {
		t.Error("expected new module to be created")
	}
}
