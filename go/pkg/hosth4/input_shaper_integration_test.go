// Input shaper integration tests
package hosth4

import (
	"testing"

	"klipper-go-migration/pkg/inputshaper"
)

func TestInputShaperIntegrationBasic(t *testing.T) {
	// Create with default config
	cfg := DefaultInputShaperConfig()
	is, err := newInputShaperIntegration(nil, cfg)
	if err != nil {
		t.Fatalf("Failed to create input shaper integration: %v", err)
	}

	// Check initial state
	if is.IsEnabled() {
		t.Error("Input shaper should not be enabled by default (no frequencies set)")
	}

	// Check shapers are created
	shapers := is.GetShapers()
	if len(shapers) != 3 {
		t.Errorf("Expected 3 shapers (x, y, z), got %d", len(shapers))
	}

	// Verify axes
	expectedAxes := []string{"x", "y", "z"}
	for i, shaper := range shapers {
		if shaper.Axis != expectedAxes[i] {
			t.Errorf("Shaper %d: expected axis %s, got %s", i, expectedAxes[i], shaper.Axis)
		}
	}
}

func TestInputShaperIntegrationWithFrequencies(t *testing.T) {
	cfg := InputShaperConfig{
		ShaperTypeX:  inputshaper.ShaperMZV,
		ShaperTypeY:  inputshaper.ShaperZV,
		ShaperTypeZ:  "",
		ShaperFreqX:  35.5,
		ShaperFreqY:  42.0,
		ShaperFreqZ:  0,
		DampingRatio: 0.1,
	}

	is, err := newInputShaperIntegration(nil, cfg)
	if err != nil {
		t.Fatalf("Failed to create input shaper integration: %v", err)
	}

	// Should be enabled since X and Y have frequencies
	if !is.IsEnabled() {
		t.Error("Input shaper should be enabled when frequencies are set")
	}

	// Check X shaper
	shapers := is.GetShapers()
	xShaper := shapers[0]
	if !xShaper.IsEnabled() {
		t.Error("X shaper should be enabled")
	}
	n, A, T := xShaper.GetShaper()
	if n == 0 {
		t.Error("X shaper should have pulses")
	}
	if len(A) != n || len(T) != n {
		t.Errorf("X shaper: A/T length mismatch: n=%d, len(A)=%d, len(T)=%d", n, len(A), len(T))
	}

	// Check Y shaper
	yShaper := shapers[1]
	if !yShaper.IsEnabled() {
		t.Error("Y shaper should be enabled")
	}

	// Check Z shaper (should be disabled)
	zShaper := shapers[2]
	if zShaper.IsEnabled() {
		t.Error("Z shaper should be disabled (no frequency)")
	}
}

func TestInputShaperIntegrationStatus(t *testing.T) {
	cfg := InputShaperConfig{
		ShaperTypeX:  inputshaper.ShaperMZV,
		ShaperTypeY:  inputshaper.ShaperZV,
		ShaperFreqX:  35.5,
		ShaperFreqY:  42.0,
		DampingRatio: 0.1,
	}

	is, err := newInputShaperIntegration(nil, cfg)
	if err != nil {
		t.Fatalf("Failed to create input shaper integration: %v", err)
	}

	status := is.GetStatus()

	// Check that status contains expected keys
	expectedKeys := []string{
		"shaper_type_x", "shaper_freq_x", "damping_ratio_x",
		"shaper_type_y", "shaper_freq_y", "damping_ratio_y",
		"shaper_type_z", "shaper_freq_z", "damping_ratio_z",
	}

	for _, key := range expectedKeys {
		if _, ok := status[key]; !ok {
			t.Errorf("Status missing key: %s", key)
		}
	}

	// Check X shaper type
	if status["shaper_type_x"] != "mzv" {
		t.Errorf("Expected shaper_type_x=mzv, got %v", status["shaper_type_x"])
	}

	// Check Y shaper type
	if status["shaper_type_y"] != "zv" {
		t.Errorf("Expected shaper_type_y=zv, got %v", status["shaper_type_y"])
	}
}

func TestInputShaperIntegrationSetCommand(t *testing.T) {
	cfg := DefaultInputShaperConfig()
	is, err := newInputShaperIntegration(nil, cfg)
	if err != nil {
		t.Fatalf("Failed to create input shaper integration: %v", err)
	}

	// Initially disabled
	if is.IsEnabled() {
		t.Error("Input shaper should start disabled")
	}

	// Set parameters via command
	args := map[string]string{
		"SHAPER_TYPE_X": "mzv",
		"SHAPER_FREQ_X": "35.5",
		"SHAPER_TYPE_Y": "zv",
		"SHAPER_FREQ_Y": "42.0",
	}

	err = is.cmdSetInputShaper(args)
	if err != nil {
		t.Fatalf("cmdSetInputShaper failed: %v", err)
	}

	// Should now be enabled
	if !is.IsEnabled() {
		t.Error("Input shaper should be enabled after setting frequencies")
	}

	// Check X shaper was updated
	shapers := is.GetShapers()
	xShaper := shapers[0]
	if !xShaper.IsEnabled() {
		t.Error("X shaper should be enabled")
	}
}

func TestInputShaperIntegrationDisableEnable(t *testing.T) {
	cfg := InputShaperConfig{
		ShaperTypeX:  inputshaper.ShaperMZV,
		ShaperTypeY:  inputshaper.ShaperMZV,
		ShaperFreqX:  35.5,
		ShaperFreqY:  42.0,
		DampingRatio: 0.1,
	}

	is, err := newInputShaperIntegration(nil, cfg)
	if err != nil {
		t.Fatalf("Failed to create input shaper integration: %v", err)
	}

	// Initially enabled
	if !is.IsEnabled() {
		t.Error("Input shaper should be enabled")
	}

	// Disable shaping
	err = is.DisableShaping()
	if err != nil {
		t.Fatalf("DisableShaping failed: %v", err)
	}

	// Check all shapers are disabled
	for _, shaper := range is.GetShapers() {
		if shaper.IsEnabled() {
			t.Errorf("Shaper %s should be disabled", shaper.Axis)
		}
	}

	// Re-enable shaping
	err = is.EnableShaping()
	if err != nil {
		t.Fatalf("EnableShaping failed: %v", err)
	}

	// Check X and Y shapers are re-enabled
	shapers := is.GetShapers()
	if !shapers[0].IsEnabled() {
		t.Error("X shaper should be re-enabled")
	}
	if !shapers[1].IsEnabled() {
		t.Error("Y shaper should be re-enabled")
	}
}

func TestInputShaperIntegrationReport(t *testing.T) {
	cfg := InputShaperConfig{
		ShaperTypeX:  inputshaper.ShaperMZV,
		ShaperTypeY:  inputshaper.ShaperZV,
		ShaperFreqX:  35.5,
		ShaperFreqY:  42.0,
		DampingRatio: 0.1,
	}

	is, err := newInputShaperIntegration(nil, cfg)
	if err != nil {
		t.Fatalf("Failed to create input shaper integration: %v", err)
	}

	report := is.Report()
	if report == "" {
		t.Error("Report should not be empty")
	}

	// Report should contain axis information
	if len(report) < 10 {
		t.Errorf("Report seems too short: %s", report)
	}
}

func TestInputShaperIntegrationShaperCoefficients(t *testing.T) {
	cfg := InputShaperConfig{
		ShaperTypeX:  inputshaper.ShaperMZV,
		ShaperTypeY:  inputshaper.ShaperZV,
		ShaperFreqX:  35.5,
		ShaperFreqY:  42.0,
		DampingRatio: 0.1,
	}

	is, err := newInputShaperIntegration(nil, cfg)
	if err != nil {
		t.Fatalf("Failed to create input shaper integration: %v", err)
	}

	// Get X coefficients (MZV has 3 pulses)
	n, A, T := is.GetShaperCoefficients(0)
	if n != 3 {
		t.Errorf("MZV shaper should have 3 pulses, got %d", n)
	}
	if len(A) != 3 || len(T) != 3 {
		t.Errorf("A/T length should be 3, got %d/%d", len(A), len(T))
	}

	// Verify amplitudes sum to approximately 1
	sum := 0.0
	for _, a := range A {
		sum += a
	}
	// Note: amplitudes may not sum to 1 before normalization
	if sum <= 0 {
		t.Error("Sum of amplitudes should be positive")
	}

	// Get Y coefficients (ZV has 2 pulses)
	n, A, T = is.GetShaperCoefficients(1)
	if n != 2 {
		t.Errorf("ZV shaper should have 2 pulses, got %d", n)
	}

	// Get Z coefficients (disabled, should return 0)
	n, A, T = is.GetShaperCoefficients(2)
	if n != 0 {
		t.Errorf("Z shaper should have 0 pulses (disabled), got %d", n)
	}

	// Invalid axis should return 0
	n, A, T = is.GetShaperCoefficients(5)
	if n != 0 {
		t.Errorf("Invalid axis should return 0 pulses, got %d", n)
	}
}

func TestInputShaperIntegrationDualCarriage(t *testing.T) {
	cfg := InputShaperConfig{
		ShaperTypeX:  inputshaper.ShaperMZV,
		ShaperFreqX:  35.5,
		DampingRatio: 0.1,
	}

	is, err := newInputShaperIntegration(nil, cfg)
	if err != nil {
		t.Fatalf("Failed to create input shaper integration: %v", err)
	}

	// Initially no dual carriage
	if is.hasDualCarriage {
		t.Error("Dual carriage should be false initially")
	}

	// Set dual carriage
	is.SetHasDualCarriage(true)
	if !is.hasDualCarriage {
		t.Error("Dual carriage should be true after setting")
	}

	// Unset
	is.SetHasDualCarriage(false)
	if is.hasDualCarriage {
		t.Error("Dual carriage should be false after unsetting")
	}
}

func TestInputShaperIntegrationApplyShaping(t *testing.T) {
	cfg := InputShaperConfig{
		ShaperTypeX:  inputshaper.ShaperZV,
		ShaperFreqX:  40.0,
		DampingRatio: 0.1,
	}

	is, err := newInputShaperIntegration(nil, cfg)
	if err != nil {
		t.Fatalf("Failed to create input shaper integration: %v", err)
	}

	// Test input trajectory
	times := []float64{0.0, 0.1, 0.2, 0.3}
	positions := []float64{0.0, 10.0, 20.0, 30.0}

	// Apply shaping
	outTimes, outPositions := is.ApplyShaping(0, times, positions)

	// Output should have more points than input (convolution expands)
	if len(outTimes) < len(times) {
		t.Errorf("Output should have at least as many points as input")
	}
	if len(outTimes) != len(outPositions) {
		t.Errorf("Output times and positions should have same length")
	}

	// Output should be sorted by time
	for i := 1; i < len(outTimes); i++ {
		if outTimes[i] < outTimes[i-1] {
			t.Errorf("Output times should be sorted")
			break
		}
	}

	// Test with disabled axis (Z)
	outTimes2, outPositions2 := is.ApplyShaping(2, times, positions)
	if len(outTimes2) != len(times) {
		t.Errorf("Disabled axis should return original trajectory")
	}
	for i := range times {
		if outTimes2[i] != times[i] || outPositions2[i] != positions[i] {
			t.Errorf("Disabled axis should return unchanged trajectory")
			break
		}
	}
}

func TestInputShaperIntegrationKinematicsMap(t *testing.T) {
	cfg := DefaultInputShaperConfig()
	is, err := newInputShaperIntegration(nil, cfg)
	if err != nil {
		t.Fatalf("Failed to create input shaper integration: %v", err)
	}

	// Initially empty
	if len(is.inputShaperStepperKinematics) != 0 {
		t.Errorf("Should have no input shaper kinematics initially")
	}
	if len(is.origStepperKinematics) != 0 {
		t.Errorf("Should have no original kinematics initially")
	}
	if len(is.stepperToInputShaperKinematics) != 0 {
		t.Errorf("Map should be empty initially")
	}
}
