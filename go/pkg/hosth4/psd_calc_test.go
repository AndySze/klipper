// PSD calculation tests
//
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"math"
	"testing"
)

// TestKaiserWindow verifies the Kaiser window generation.
func TestKaiserWindow(t *testing.T) {
	// Test with beta=6, n=10 (same as Python numpy.kaiser(10, 6))
	window := kaiserWindow(10, 6.0)

	if len(window) != 10 {
		t.Errorf("Expected window length 10, got %d", len(window))
	}

	// Kaiser window should be symmetric
	for i := 0; i < 5; i++ {
		if math.Abs(window[i]-window[9-i]) > 1e-10 {
			t.Errorf("Window not symmetric at index %d: %f != %f", i, window[i], window[9-i])
		}
	}

	// Window should peak in the center
	if window[4] <= window[0] || window[5] <= window[0] {
		t.Error("Window should peak in the center")
	}

	// Window values should be in [0, 1]
	for i, v := range window {
		if v < 0 || v > 1 {
			t.Errorf("Window value out of range at index %d: %f", i, v)
		}
	}
}

// TestBesselI0 verifies the Bessel I0 function.
func TestBesselI0(t *testing.T) {
	// Test cases from known values
	tests := []struct {
		x        float64
		expected float64
		tol      float64
	}{
		{0.0, 1.0, 1e-10},
		{1.0, 1.2660658, 1e-6},
		{2.0, 2.2795853, 1e-6},
		{5.0, 27.239872, 1e-4},
		{10.0, 2815.7167, 1e-2}, // Large argument
	}

	for _, tt := range tests {
		result := besselI0(tt.x)
		if math.Abs(result-tt.expected) > tt.tol {
			t.Errorf("besselI0(%f) = %f, expected %f (tol %f)", tt.x, result, tt.expected, tt.tol)
		}
	}
}

// TestPSDCalculator tests the PSD calculation with a simple sine wave.
func TestPSDCalculator(t *testing.T) {
	fs := 1000.0 // 1000 Hz sampling rate
	nfft := 256
	duration := 1.0 // 1 second
	freq := 50.0    // 50 Hz sine wave

	// Generate a sine wave
	n := int(fs * duration)
	signal := make([]float64, n)
	for i := 0; i < n; i++ {
		t := float64(i) / fs
		signal[i] = math.Sin(2 * math.Pi * freq * t)
	}

	calc := NewPSDCalculator(nfft)
	freqs, psd := calc.PSD(signal, fs)

	if freqs == nil || psd == nil {
		t.Fatal("PSD calculation returned nil")
	}

	// Find the peak frequency
	peakIdx := 0
	peakVal := 0.0
	for i, p := range psd {
		if p > peakVal {
			peakVal = p
			peakIdx = i
		}
	}

	peakFreq := freqs[peakIdx]

	// Peak should be near our input frequency
	if math.Abs(peakFreq-freq) > 5.0 {
		t.Errorf("Peak frequency %f Hz not near expected %f Hz", peakFreq, freq)
	}
}

// TestSplitIntoWindows tests window splitting.
func TestSplitIntoWindows(t *testing.T) {
	data := make([]float64, 100)
	for i := range data {
		data[i] = float64(i)
	}

	// 50% overlap with nfft=32
	windows := splitIntoWindows(data, 32, 16)

	// Expected number of windows: (100 - 16) / 16 = 5
	if len(windows) < 4 || len(windows) > 6 {
		t.Errorf("Expected ~5 windows, got %d", len(windows))
	}

	// Each window should have correct length
	for i, w := range windows {
		if len(w) != 32 {
			t.Errorf("Window %d has length %d, expected 32", i, len(w))
		}
	}
}

// TestCalcFreqResponse tests shaper frequency response calculation.
func TestCalcFreqResponse(t *testing.T) {
	// Simple ZV shaper at 50 Hz
	A := []float64{1.0, 1.0}
	T := []float64{0.0, 0.01} // 10ms delay

	freqs := []float64{0, 10, 20, 50, 100}
	response := CalcFreqResponse(freqs, A, T)

	if response == nil {
		t.Fatal("CalcFreqResponse returned nil")
	}

	if len(response) != len(freqs) {
		t.Errorf("Response length %d doesn't match freqs length %d", len(response), len(freqs))
	}

	// At DC (0 Hz), response should be 1.0
	dcMag := math.Sqrt(real(response[0])*real(response[0]) + imag(response[0])*imag(response[0]))
	if math.Abs(dcMag-1.0) > 1e-10 {
		t.Errorf("DC response magnitude %f, expected 1.0", dcMag)
	}
}

// TestCalcShaperPower tests power response calculation.
func TestCalcShaperPower(t *testing.T) {
	// Simple impulse response
	A := []float64{1.0}
	T := []float64{0.0}

	freqs := []float64{0, 50, 100}
	power := CalcShaperPower(freqs, A, T)

	if power == nil {
		t.Fatal("CalcShaperPower returned nil")
	}

	// For a single impulse, power should be 1.0 at all frequencies
	for i, p := range power {
		if math.Abs(p-1.0) > 1e-10 {
			t.Errorf("Power at freq[%d] = %f, expected 1.0", i, p)
		}
	}
}

// TestEstimateResidualVibration tests vibration estimation.
func TestEstimateResidualVibration(t *testing.T) {
	// Generate a simple PSD with a peak at 50 Hz
	freqs := make([]float64, 100)
	psd := make([]float64, 100)
	for i := range freqs {
		freqs[i] = float64(i) * 2.0 // 0-198 Hz in 2 Hz steps
		// Peak at 50 Hz
		df := freqs[i] - 50.0
		psd[i] = math.Exp(-df * df / 200.0)
	}

	// ZV shaper tuned to 50 Hz should reduce vibration significantly
	A, T := zvShaper(50.0, 0.1)
	freqRange := [2]float64{5.0, 200.0}

	vibr := EstimateResidualVibration(freqs, psd, A, T, freqRange)

	// Residual vibration should be less than 1.0 (input level)
	if vibr >= 1.0 {
		t.Errorf("Residual vibration %f should be < 1.0", vibr)
	}
}

// TestAccelDataProcessor tests accelerometer data processing.
func TestAccelDataProcessor(t *testing.T) {
	fs := 3200.0 // Typical Klipper accelerometer rate
	duration := 0.5
	n := int(fs * duration)

	// Generate test data: sine waves at different frequencies
	times := make([]float64, n)
	accelX := make([]float64, n)
	accelY := make([]float64, n)
	accelZ := make([]float64, n)

	for i := 0; i < n; i++ {
		times[i] = float64(i) / fs
		accelX[i] = math.Sin(2*math.Pi*30*times[i]) + 0.1*math.Sin(2*math.Pi*60*times[i])
		accelY[i] = math.Sin(2*math.Pi*40*times[i]) + 0.1*math.Sin(2*math.Pi*80*times[i])
		accelZ[i] = 0.5 * math.Sin(2*math.Pi*50*times[i])
	}

	processor := NewAccelDataProcessor(fs)
	calibData := processor.ProcessAccelData("test", times, accelX, accelY, accelZ)

	if calibData == nil {
		t.Fatal("ProcessAccelData returned nil")
	}

	if calibData.Name != "test" {
		t.Errorf("Expected name 'test', got '%s'", calibData.Name)
	}

	if len(calibData.FreqBins) == 0 {
		t.Error("FreqBins is empty")
	}

	if len(calibData.PsdX) != len(calibData.FreqBins) {
		t.Error("PsdX length doesn't match FreqBins")
	}
}

// TestNextPowerOf2 tests power of 2 rounding.
func TestNextPowerOf2(t *testing.T) {
	tests := []struct {
		input    int
		expected int
	}{
		{1, 1},
		{2, 2},
		{3, 4},
		{5, 8},
		{100, 128},
		{256, 256},
		{257, 512},
	}

	for _, tt := range tests {
		result := nextPowerOf2(tt.input)
		if result != tt.expected {
			t.Errorf("nextPowerOf2(%d) = %d, expected %d", tt.input, result, tt.expected)
		}
	}
}

// Helper: simple ZV shaper for testing
func zvShaper(freq, dampingRatio float64) ([]float64, []float64) {
	omega := 2 * math.Pi * freq
	dampedOmega := omega * math.Sqrt(1-dampingRatio*dampingRatio)
	K := math.Exp(-dampingRatio * omega / dampedOmega)

	A := []float64{1.0, K}
	sumA := 1.0 + K

	// Normalize
	for i := range A {
		A[i] /= sumA
	}

	T := []float64{0.0, 0.5 / freq}
	return A, T
}

// BenchmarkPSDCalculation benchmarks PSD calculation.
func BenchmarkPSDCalculation(b *testing.B) {
	fs := 3200.0
	n := 3200 // 1 second of data
	signal := make([]float64, n)
	for i := 0; i < n; i++ {
		t := float64(i) / fs
		signal[i] = math.Sin(2*math.Pi*50*t) + 0.5*math.Sin(2*math.Pi*100*t)
	}

	calc := NewPSDCalculator(256)

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		calc.PSD(signal, fs)
	}
}

// BenchmarkKaiserWindow benchmarks Kaiser window generation.
func BenchmarkKaiserWindow(b *testing.B) {
	for i := 0; i < b.N; i++ {
		kaiserWindow(256, 6.0)
	}
}
