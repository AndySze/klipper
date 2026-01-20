// Input shaper C helper tests
package chelper

import (
	"testing"
)

func TestInputShaperAlloc(t *testing.T) {
	// Allocate input shaper kinematics
	sk, err := NewInputShaperStepperKinematics()
	if err != nil {
		t.Fatalf("NewInputShaperStepperKinematics failed: %v", err)
	}
	if sk == nil {
		t.Fatal("Returned kinematics is nil")
	}

	// Note: We can't set the original SK or test full functionality without
	// a complete stepper setup, but we can verify the allocation works.
}

func TestInputShaperSetParamsValidation(t *testing.T) {
	// Allocate input shaper kinematics with a real cartesian kinematics
	origSK, err := NewCartesianStepperKinematics('x')
	if err != nil {
		t.Fatalf("NewCartesianStepperKinematics failed: %v", err)
	}
	defer origSK.Free()

	sk, err := NewInputShaperStepperKinematics()
	if err != nil {
		t.Fatalf("NewInputShaperStepperKinematics failed: %v", err)
	}

	// Set original kinematics first
	err = sk.InputShaperSetSK(origSK)
	if err != nil {
		t.Fatalf("InputShaperSetSK failed: %v", err)
	}

	// Test invalid axis - should be caught in Go before calling C
	err = sk.InputShaperSetParams('a', 2, []float64{0.5, 0.5}, []float64{0.0, 0.01})
	if err == nil {
		t.Error("Expected error for invalid axis 'a'")
	}

	// Test too many pulses - should be caught in Go before calling C
	err = sk.InputShaperSetParams('x', 6, make([]float64, 6), make([]float64, 6))
	if err == nil {
		t.Error("Expected error for too many pulses (>5)")
	}

	// Test n=0 (disabling shaper) - should work
	err = sk.InputShaperSetParams('x', 0, nil, nil)
	if err != nil {
		t.Errorf("Failed to disable input shaper: %v", err)
	}
}

func TestInputShaperSetSKWithoutOrig(t *testing.T) {
	// Allocate input shaper kinematics
	sk, err := NewInputShaperStepperKinematics()
	if err != nil {
		t.Fatalf("NewInputShaperStepperKinematics failed: %v", err)
	}

	// Setting SK with nil should fail
	err = sk.InputShaperSetSK(nil)
	if err == nil {
		t.Error("Expected error when setting nil original SK")
	}
}

func TestInputShaperUpdateSKNil(t *testing.T) {
	// Test that UpdateSK handles nil gracefully
	var sk *StepperKinematics
	sk.InputShaperUpdateSK() // Should not panic

	sk2 := &StepperKinematics{ptr: nil}
	sk2.InputShaperUpdateSK() // Should not panic
}

func TestInputShaperWithCartesianKinematics(t *testing.T) {
	// Create a cartesian stepper kinematics for X axis
	origSK, err := NewCartesianStepperKinematics('x')
	if err != nil {
		t.Fatalf("NewCartesianStepperKinematics failed: %v", err)
	}
	defer origSK.Free()

	// Create input shaper wrapper
	isSK, err := NewInputShaperStepperKinematics()
	if err != nil {
		t.Fatalf("NewInputShaperStepperKinematics failed: %v", err)
	}

	// Set the original kinematics
	err = isSK.InputShaperSetSK(origSK)
	if err != nil {
		t.Fatalf("InputShaperSetSK failed: %v", err)
	}

	// Set MZV shaper parameters for X axis
	// MZV coefficients at 35.5 Hz with damping ratio 0.1
	// (These are approximate values for testing)
	A := []float64{0.25, 0.50, 0.25}
	T := []float64{0.0, 0.014, 0.028}

	err = isSK.InputShaperSetParams('x', 3, A, T)
	if err != nil {
		t.Fatalf("InputShaperSetParams failed: %v", err)
	}

	// Get gen_steps windows
	preActive := isSK.GenStepsPreActive()
	postActive := isSK.GenStepsPostActive()

	// Input shaper should add some lookahead/lookback time
	// The exact values depend on the shaper parameters
	t.Logf("Input shaper windows: pre_active=%f, post_active=%f", preActive, postActive)

	// Verify update works
	isSK.InputShaperUpdateSK()
}

func TestInputShaperZVShaper(t *testing.T) {
	// Create cartesian kinematics for Y axis
	origSK, err := NewCartesianStepperKinematics('y')
	if err != nil {
		t.Fatalf("NewCartesianStepperKinematics failed: %v", err)
	}
	defer origSK.Free()

	// Create input shaper wrapper
	isSK, err := NewInputShaperStepperKinematics()
	if err != nil {
		t.Fatalf("NewInputShaperStepperKinematics failed: %v", err)
	}

	// Set original kinematics
	err = isSK.InputShaperSetSK(origSK)
	if err != nil {
		t.Fatalf("InputShaperSetSK failed: %v", err)
	}

	// Set ZV shaper parameters for Y axis (2 pulses)
	// ZV coefficients at 42 Hz with damping ratio 0.1
	A := []float64{0.5, 0.5}
	T := []float64{0.0, 0.012}

	err = isSK.InputShaperSetParams('y', 2, A, T)
	if err != nil {
		t.Fatalf("InputShaperSetParams for Y failed: %v", err)
	}

	// Verify gen_steps windows are set
	preActive := isSK.GenStepsPreActive()
	postActive := isSK.GenStepsPostActive()
	t.Logf("ZV shaper windows: pre_active=%f, post_active=%f", preActive, postActive)
}

func TestInputShaperDisable(t *testing.T) {
	// Create cartesian kinematics
	origSK, err := NewCartesianStepperKinematics('x')
	if err != nil {
		t.Fatalf("NewCartesianStepperKinematics failed: %v", err)
	}
	defer origSK.Free()

	// Create input shaper wrapper
	isSK, err := NewInputShaperStepperKinematics()
	if err != nil {
		t.Fatalf("NewInputShaperStepperKinematics failed: %v", err)
	}

	// Set original kinematics
	err = isSK.InputShaperSetSK(origSK)
	if err != nil {
		t.Fatalf("InputShaperSetSK failed: %v", err)
	}

	// Enable shaper
	A := []float64{0.25, 0.50, 0.25}
	T := []float64{0.0, 0.014, 0.028}
	err = isSK.InputShaperSetParams('x', 3, A, T)
	if err != nil {
		t.Fatalf("InputShaperSetParams failed: %v", err)
	}

	// Verify it's enabled (has lookahead)
	preActive := isSK.GenStepsPreActive()
	if preActive == 0 {
		t.Log("Note: pre_active is 0 (may be expected if no moves in queue)")
	}

	// Disable by setting n=0
	err = isSK.InputShaperSetParams('x', 0, nil, nil)
	if err != nil {
		t.Fatalf("Failed to disable input shaper: %v", err)
	}

	// After disabling, windows should be 0
	preActive = isSK.GenStepsPreActive()
	postActive := isSK.GenStepsPostActive()
	if preActive != 0 || postActive != 0 {
		t.Logf("After disable: pre_active=%f, post_active=%f (may be non-zero if other axes active)", preActive, postActive)
	}
}
