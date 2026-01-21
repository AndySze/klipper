// Shaper Calibrate tests
//
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"math"
	"testing"
)

// TestShaperCalibrateCreate tests shaper calibrator creation.
func TestShaperCalibrateCreate(t *testing.T) {
	sc := newShaperCalibrate(nil)
	if sc == nil {
		t.Fatal("newShaperCalibrate returned nil")
	}

	if sc.scv != defaultSCV {
		t.Errorf("Expected scv=%f, got %f", defaultSCV, sc.scv)
	}
}

// TestShaperCalibrateSCV tests setting square corner velocity.
func TestShaperCalibrateSCV(t *testing.T) {
	sc := newShaperCalibrate(nil)
	sc.SetSCV(10.0)

	if sc.scv != 10.0 {
		t.Errorf("Expected scv=10.0, got %f", sc.scv)
	}
}

// TestCalcSmoothing tests smoothing calculation.
func TestCalcSmoothing(t *testing.T) {
	sc := newShaperCalibrate(nil)

	// Test with a simple ZV-like shaper
	A := []float64{0.5, 0.5}
	T := []float64{0.0, 0.01}

	smoothing := sc.calcSmoothing(A, T)

	// Smoothing should be positive and reasonable
	if smoothing <= 0 {
		t.Errorf("Expected positive smoothing, got %f", smoothing)
	}

	// For this shaper, smoothing should be related to 0.01
	if smoothing > 0.02 {
		t.Errorf("Smoothing too large: %f", smoothing)
	}
}

// TestCalcMaxAccel tests maximum acceleration calculation.
func TestCalcMaxAccel(t *testing.T) {
	sc := newShaperCalibrate(nil)
	sc.SetSCV(5.0)

	A := []float64{0.5, 0.5}
	T := []float64{0.0, 0.01}

	maxAccel := sc.calcMaxAccel(A, T, 0.01)

	// max_accel = scv / (0.5 * smoothing) = 5 / (0.5 * 0.01) = 1000
	expected := 5.0 / (0.5 * 0.01)
	if math.Abs(maxAccel-expected) > 0.1 {
		t.Errorf("Expected maxAccel=%f, got %f", expected, maxAccel)
	}
}

// TestCalcMaxAccelZeroSmoothing tests max accel with zero smoothing.
func TestCalcMaxAccelZeroSmoothing(t *testing.T) {
	sc := newShaperCalibrate(nil)

	maxAccel := sc.calcMaxAccel(nil, nil, 0.0)

	if maxAccel != 10000.0 {
		t.Errorf("Expected maxAccel=10000.0, got %f", maxAccel)
	}
}

// TestFitShaperBasic tests basic shaper fitting.
func TestFitShaperBasic(t *testing.T) {
	sc := newShaperCalibrate(nil)

	// Generate synthetic PSD data with a peak at 40 Hz
	freqBins := make([]float64, 100)
	psd := make([]float64, 100)
	for i := range freqBins {
		freqBins[i] = float64(i) * 2.0 // 0-198 Hz
		// Gaussian peak at 40 Hz
		df := freqBins[i] - 40.0
		psd[i] = math.Exp(-df * df / 100.0)
	}

	// Fit MZV shaper
	shaper := GetShaperByName("mzv")
	if shaper == nil {
		t.Fatal("MZV shaper not found")
	}

	result := sc.fitShaper(freqBins, psd, "mzv", shaper, 0)
	if result == nil {
		t.Fatal("fitShaper returned nil")
	}

	// Result should have reasonable frequency
	if result.Freq < 20 || result.Freq > 80 {
		t.Errorf("Unexpected shaper frequency: %f", result.Freq)
	}

	// Score should be positive
	if result.Score <= 0 {
		t.Errorf("Score should be positive, got %f", result.Score)
	}
}

// TestEstimateShaper tests complete shaper estimation.
func TestEstimateShaper(t *testing.T) {
	sc := newShaperCalibrate(nil)

	// Generate calibration data with peak at 50 Hz
	freqBins := make([]float64, 100)
	psdSum := make([]float64, 100)
	for i := range freqBins {
		freqBins[i] = float64(i) * 2.0
		df := freqBins[i] - 50.0
		psdSum[i] = math.Exp(-df * df / 200.0)
	}

	calibData := NewCalibrationData("test", freqBins, psdSum, psdSum, nil, nil)

	result, err := sc.EstimateShaper(calibData, "sum", 0)
	if err != nil {
		t.Fatalf("EstimateShaper failed: %v", err)
	}

	if result == nil {
		t.Fatal("EstimateShaper returned nil result")
	}

	// Should pick a reasonable shaper
	validShapers := map[string]bool{"zv": true, "mzv": true, "ei": true, "2hump_ei": true, "3hump_ei": true}
	if !validShapers[result.Name] {
		t.Errorf("Unexpected shaper name: %s", result.Name)
	}

	t.Logf("Best shaper: %s @ %.1f Hz, score=%.4f, maxAccel=%.0f", result.Name, result.Freq, result.Score, result.MaxAccel)
}

// TestEstimateShapersMaxSmoothing tests with max smoothing constraint.
func TestEstimateShapersMaxSmoothing(t *testing.T) {
	sc := newShaperCalibrate(nil)

	freqBins := make([]float64, 100)
	psdSum := make([]float64, 100)
	for i := range freqBins {
		freqBins[i] = float64(i) * 2.0
		df := freqBins[i] - 50.0
		psdSum[i] = math.Exp(-df * df / 200.0)
	}

	calibData := NewCalibrationData("test", freqBins, psdSum, psdSum, nil, nil)

	// Very tight max smoothing constraint
	result, err := sc.EstimateShaper(calibData, "sum", 0.001)
	if err != nil {
		// Should still find something or return error
		t.Logf("Max smoothing constraint too tight: %v", err)
	} else if result != nil {
		if result.Smoothing > 0.001 {
			t.Errorf("Smoothing %f exceeds max 0.001", result.Smoothing)
		}
	}
}

// TestGetAutotuneResult tests the autotune helper.
func TestGetAutotuneResult(t *testing.T) {
	sc := newShaperCalibrate(nil)

	freqBins := make([]float64, 100)
	psdSum := make([]float64, 100)
	for i := range freqBins {
		freqBins[i] = float64(i) * 2.0
		df := freqBins[i] - 60.0
		psdSum[i] = math.Exp(-df * df / 150.0)
	}

	calibData := NewCalibrationData("test", freqBins, psdSum, psdSum, nil, nil)

	name, freq, maxAccel, err := sc.GetAutotuneResult(calibData, "sum")
	if err != nil {
		t.Fatalf("GetAutotuneResult failed: %v", err)
	}

	if name == "" {
		t.Error("Shaper name should not be empty")
	}

	if freq <= 0 {
		t.Errorf("Frequency should be positive, got %f", freq)
	}

	if maxAccel <= 0 {
		t.Errorf("Max accel should be positive, got %f", maxAccel)
	}

	t.Logf("Autotune result: %s @ %.1f Hz, maxAccel=%.0f", name, freq, maxAccel)
}

// TestCompareResults tests result comparison.
func TestCompareResults(t *testing.T) {
	sc := newShaperCalibrate(nil)

	results := []*CalibrationResult{
		{Name: "mzv", Score: 0.5},
		{Name: "ei", Score: 0.3},
		{Name: "zv", Score: 0.8},
	}

	best := sc.CompareResults(results)
	if best == nil {
		t.Fatal("CompareResults returned nil")
	}

	if best.Name != "ei" {
		t.Errorf("Expected best shaper 'ei', got '%s'", best.Name)
	}
}

// TestCompareResultsEmpty tests empty result comparison.
func TestCompareResultsEmpty(t *testing.T) {
	sc := newShaperCalibrate(nil)

	best := sc.CompareResults([]*CalibrationResult{})
	if best != nil {
		t.Error("Expected nil for empty results")
	}
}

// TestCalibrationDataNormalize tests PSD normalization.
func TestCalibrationDataNormalize(t *testing.T) {
	// Use frequencies above the low-frequency filter threshold (2*5Hz = 10Hz)
	freqBins := []float64{20.0, 40.0, 80.0, 100.0}
	psdSum := []float64{1.0, 1.0, 1.0, 1.0}

	calibData := NewCalibrationData("test", freqBins, psdSum, nil, nil, nil)
	calibData.NormalizeToFrequencies()

	// After normalization, higher frequencies should have smaller PSD
	// (since we divide by frequency)
	// PSD[0] = 1/20 = 0.05, PSD[3] = 1/100 = 0.01
	if calibData.PsdSum[0] <= calibData.PsdSum[3] {
		t.Errorf("Expected lower frequency to have higher normalized PSD: PSD[0]=%f, PSD[3]=%f",
			calibData.PsdSum[0], calibData.PsdSum[3])
	}
}

// TestCalibrationDataGetPSD tests axis-specific PSD retrieval.
func TestCalibrationDataGetPSD(t *testing.T) {
	psdX := []float64{1, 2, 3}
	psdY := []float64{4, 5, 6}
	psdZ := []float64{7, 8, 9}
	psdSum := []float64{10, 11, 12}

	calibData := NewCalibrationData("test", nil, psdSum, psdX, psdY, psdZ)

	if psd := calibData.GetPSD("x"); psd[0] != 1 {
		t.Error("GetPSD('x') returned wrong data")
	}

	if psd := calibData.GetPSD("y"); psd[0] != 4 {
		t.Error("GetPSD('y') returned wrong data")
	}

	if psd := calibData.GetPSD("z"); psd[0] != 7 {
		t.Error("GetPSD('z') returned wrong data")
	}

	if psd := calibData.GetPSD("sum"); psd[0] != 10 {
		t.Error("GetPSD('sum') returned wrong data")
	}
}

// TestCalibrationDataAddData tests adding data sets.
func TestCalibrationDataAddData(t *testing.T) {
	cd1 := NewCalibrationData("test1", nil, nil, nil, nil, nil)
	cd2 := NewCalibrationData("test2", nil, nil, nil, nil, nil)

	cd1.AddData(cd2)

	datasets := cd1.GetDatasets()
	if len(datasets) != 2 {
		t.Errorf("Expected 2 datasets, got %d", len(datasets))
	}
}

// BenchmarkFitShaper benchmarks shaper fitting.
func BenchmarkFitShaper(b *testing.B) {
	sc := newShaperCalibrate(nil)

	freqBins := make([]float64, 200)
	psd := make([]float64, 200)
	for i := range freqBins {
		freqBins[i] = float64(i)
		df := freqBins[i] - 50.0
		psd[i] = math.Exp(-df * df / 200.0)
	}

	shaper := GetShaperByName("mzv")

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		sc.fitShaper(freqBins, psd, "mzv", shaper, 0)
	}
}

// BenchmarkEstimateShaper benchmarks complete estimation.
func BenchmarkEstimateShaper(b *testing.B) {
	sc := newShaperCalibrate(nil)

	freqBins := make([]float64, 200)
	psdSum := make([]float64, 200)
	for i := range freqBins {
		freqBins[i] = float64(i)
		df := freqBins[i] - 50.0
		psdSum[i] = math.Exp(-df * df / 200.0)
	}

	calibData := NewCalibrationData("test", freqBins, psdSum, psdSum, nil, nil)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		sc.EstimateShaper(calibData, "sum", 0)
	}
}
