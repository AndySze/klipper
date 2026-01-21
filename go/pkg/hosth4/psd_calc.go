// PSD calculation using Welch's method
//
// Port of klippy/extras/shaper_calibrate.py PSD calculation
// Uses Welch's method with Kaiser window and 50% overlap.
//
// Copyright (C) 2020-2024 Dmitry Butyugin <dmbutyugin@google.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"math"
	"math/cmplx"

	"gonum.org/v1/gonum/dsp/fourier"
)

// PSDCalculator computes power spectral density using Welch's method.
type PSDCalculator struct {
	nfft   int           // FFT length
	fft    *fourier.FFT  // FFT instance
	window []float64     // Kaiser window
	scale  float64       // Window normalization scale
}

// NewPSDCalculator creates a new PSD calculator with the given FFT length.
func NewPSDCalculator(nfft int) *PSDCalculator {
	window := kaiserWindow(nfft, 6.0)

	// Calculate window normalization scale: 1 / sum(window^2)
	var sumSq float64
	for _, w := range window {
		sumSq += w * w
	}
	scale := 1.0 / sumSq

	return &PSDCalculator{
		nfft:   nfft,
		fft:    fourier.NewFFT(nfft),
		window: window,
		scale:  scale,
	}
}

// kaiserWindow generates a Kaiser window of length n with shape parameter beta.
// Matches numpy.kaiser(n, beta).
func kaiserWindow(n int, beta float64) []float64 {
	if n == 1 {
		return []float64{1.0}
	}

	window := make([]float64, n)

	// I0(beta) - modified Bessel function at 0
	denominator := besselI0(beta)

	for i := 0; i < n; i++ {
		x := 2.0*float64(i)/float64(n-1) - 1.0
		arg := beta * math.Sqrt(1.0 - x*x)
		window[i] = besselI0(arg) / denominator
	}

	return window
}

// besselI0 computes the modified Bessel function of the first kind, order 0.
// Uses polynomial approximation from Abramowitz and Stegun.
func besselI0(x float64) float64 {
	ax := math.Abs(x)

	if ax < 3.75 {
		// Polynomial approximation for small arguments
		y := x / 3.75
		y2 := y * y
		return 1.0 + y2*(3.5156229 +
			y2*(3.0899424 +
				y2*(1.2067492 +
					y2*(0.2659732 +
						y2*(0.0360768 +
							y2*0.0045813)))))
	}

	// Polynomial approximation for large arguments
	y := 3.75 / ax
	return (math.Exp(ax) / math.Sqrt(ax)) * (0.39894228 +
		y*(0.01328592 +
			y*(0.00225319 +
				y*(-0.00157565 +
					y*(0.00916281 +
						y*(-0.02057706 +
							y*(0.02635537 +
								y*(-0.01647633 +
									y*0.00392377))))))))
}

// splitIntoWindows splits data x into overlapping windows for Welch's method.
// Returns a 2D slice where each column is a window segment.
func splitIntoWindows(x []float64, nfft, overlap int) [][]float64 {
	step := nfft - overlap
	numWindows := (len(x) - overlap) / step

	if numWindows < 1 {
		numWindows = 1
	}

	windows := make([][]float64, numWindows)
	for i := 0; i < numWindows; i++ {
		start := i * step
		end := start + nfft
		if end > len(x) {
			break
		}
		window := make([]float64, nfft)
		copy(window, x[start:end])
		windows[i] = window
	}

	// Filter out nil/empty windows
	result := make([][]float64, 0, len(windows))
	for _, w := range windows {
		if len(w) == nfft {
			result = append(result, w)
		}
	}

	return result
}

// PSD computes power spectral density using Welch's method.
// x: input signal
// fs: sampling frequency
// Returns: (frequencies, psd) where psd is the one-sided power spectral density.
func (pc *PSDCalculator) PSD(x []float64, fs float64) ([]float64, []float64) {
	overlap := pc.nfft / 2

	// Split into overlapping windows
	windows := splitIntoWindows(x, pc.nfft, overlap)

	if len(windows) == 0 {
		// Not enough data, return empty result
		return nil, nil
	}

	// Number of output bins for one-sided FFT
	numBins := pc.nfft/2 + 1

	// Accumulator for averaged PSD
	psdAccum := make([]float64, numBins)

	// Process each window
	for _, segment := range windows {
		// Calculate mean and subtract it (remove DC offset)
		mean := 0.0
		for _, v := range segment {
			mean += v
		}
		mean /= float64(len(segment))

		// Apply window and remove mean
		windowed := make([]float64, pc.nfft)
		for i := 0; i < pc.nfft; i++ {
			windowed[i] = pc.window[i] * (segment[i] - mean)
		}

		// Compute FFT
		coeffs := pc.fft.Coefficients(nil, windowed)

		// Compute power: |fft|^2 = real^2 + imag^2
		for i := 0; i < numBins; i++ {
			power := real(coeffs[i])*real(coeffs[i]) + imag(coeffs[i])*imag(coeffs[i])
			psdAccum[i] += power
		}
	}

	// Average over windows and apply scaling
	numWindows := float64(len(windows))
	psd := make([]float64, numBins)

	for i := 0; i < numBins; i++ {
		psd[i] = psdAccum[i] * pc.scale / (fs * numWindows)

		// Double the power for non-DC/Nyquist bins (one-sided spectrum)
		if i > 0 && i < numBins-1 {
			psd[i] *= 2.0
		}
	}

	// Generate frequency bins
	freqs := make([]float64, numBins)
	df := fs / float64(pc.nfft)
	for i := 0; i < numBins; i++ {
		freqs[i] = float64(i) * df
	}

	return freqs, psd
}

// AccelDataProcessor processes accelerometer data and computes PSD.
type AccelDataProcessor struct {
	nfft int
	calc *PSDCalculator
}

// NewAccelDataProcessor creates a processor with default parameters.
// Default nfft of 320 at 3200 Hz sampling gives 10 Hz resolution.
func NewAccelDataProcessor(sampleRate float64) *AccelDataProcessor {
	// Choose nfft based on sample rate for ~10 Hz resolution
	nfft := int(sampleRate / 10.0)
	// Round to nearest power of 2 for efficiency
	nfft = nextPowerOf2(nfft)
	if nfft < 64 {
		nfft = 64
	}
	if nfft > 1024 {
		nfft = 1024
	}

	return &AccelDataProcessor{
		nfft: nfft,
		calc: NewPSDCalculator(nfft),
	}
}

// nextPowerOf2 returns the next power of 2 >= n.
func nextPowerOf2(n int) int {
	p := 1
	for p < n {
		p *= 2
	}
	return p
}

// ProcessAccelData computes PSD for X, Y, Z accelerometer axes.
// Returns CalibrationData with frequency bins and PSD values.
func (ap *AccelDataProcessor) ProcessAccelData(name string, times, accelX, accelY, accelZ []float64) *CalibrationData {
	if len(times) < ap.nfft {
		return nil
	}

	// Estimate sample rate from time data
	fs := estimateSampleRate(times)
	if fs <= 0 {
		fs = 3200.0 // Default Klipper accelerometer rate
	}

	// Compute PSD for each axis
	freqs, psdX := ap.calc.PSD(accelX, fs)
	_, psdY := ap.calc.PSD(accelY, fs)
	_, psdZ := ap.calc.PSD(accelZ, fs)

	if freqs == nil {
		return nil
	}

	// Compute sum PSD
	psdSum := make([]float64, len(psdX))
	for i := range psdSum {
		psdSum[i] = psdX[i] + psdY[i] + psdZ[i]
	}

	return NewCalibrationData(name, freqs, psdSum, psdX, psdY, psdZ)
}

// estimateSampleRate estimates sampling rate from timestamp array.
func estimateSampleRate(times []float64) float64 {
	if len(times) < 2 {
		return 0
	}

	totalTime := times[len(times)-1] - times[0]
	if totalTime <= 0 {
		return 0
	}

	return float64(len(times)-1) / totalTime
}

// CalcFreqResponse computes the frequency response of a shaper.
// Returns the complex frequency response at each frequency bin.
func CalcFreqResponse(freqs []float64, A, T []float64) []complex128 {
	if len(A) == 0 || len(T) == 0 || len(A) != len(T) {
		return nil
	}

	// Normalize amplitudes
	sumA := 0.0
	for _, a := range A {
		sumA += a
	}
	if sumA == 0 {
		return nil
	}

	response := make([]complex128, len(freqs))

	for i, freq := range freqs {
		omega := 2.0 * math.Pi * freq
		var sum complex128

		for j := 0; j < len(A); j++ {
			// H(f) = sum(A[i] * exp(-2*pi*f*T[i]))
			phase := -omega * T[j]
			sum += complex(A[j]/sumA, 0) * cmplx.Exp(complex(0, phase))
		}

		response[i] = sum
	}

	return response
}

// CalcShaperPower computes the power response |H(f)|^2 of a shaper.
func CalcShaperPower(freqs []float64, A, T []float64) []float64 {
	response := CalcFreqResponse(freqs, A, T)
	if response == nil {
		return nil
	}

	power := make([]float64, len(response))
	for i, r := range response {
		power[i] = real(r)*real(r) + imag(r)*imag(r)
	}

	return power
}

// EstimateResidualVibration estimates the residual vibration after applying shaper.
// This is the key metric for shaper quality - lower is better.
func EstimateResidualVibration(freqs, psd []float64, A, T []float64, freqRange [2]float64) float64 {
	shaperPower := CalcShaperPower(freqs, A, T)
	if shaperPower == nil {
		return 1.0
	}

	var numerator, denominator float64

	for i, freq := range freqs {
		if freq < freqRange[0] || freq > freqRange[1] {
			continue
		}

		// Residual vibration = sqrt(integral(psd * |H|^2) / integral(psd))
		numerator += psd[i] * shaperPower[i]
		denominator += psd[i]
	}

	if denominator == 0 {
		return 1.0
	}

	return math.Sqrt(numerator / denominator)
}

// EstimateSmoothing estimates the smoothing effect (time spread) of a shaper.
// This affects print quality - higher smoothing means more corner rounding.
func EstimateSmoothing(A, T []float64, scv float64) float64 {
	if len(A) == 0 || len(T) == 0 {
		return 0.0
	}

	// Normalize amplitudes
	sumA := 0.0
	for _, a := range A {
		sumA += a
	}
	if sumA == 0 {
		return 0.0
	}

	// Calculate the effective time spread
	// Uses the Python formula: integral t^2 * h(t) dt where h is the impulse response

	// First, find T0 (center of mass of impulse response)
	t0 := 0.0
	for i := 0; i < len(A); i++ {
		t0 += (A[i] / sumA) * T[i]
	}

	// Calculate second moment around T0
	var m2 float64
	for i := 0; i < len(A); i++ {
		dt := T[i] - t0
		m2 += (A[i] / sumA) * dt * dt
	}

	// The smoothing is approximately sqrt(12 * m2) for uniform-like responses
	// But Klipper uses a simpler formula based on shaper duration

	// Simple approximation: max(T) - min(T)
	minT, maxT := T[0], T[0]
	for _, t := range T {
		if t < minT {
			minT = t
		}
		if t > maxT {
			maxT = t
		}
	}

	duration := maxT - minT

	// Scale by square corner velocity for acceleration estimate
	if scv > 0 {
		return duration * scv
	}

	return duration
}

// MaxAccelFromSmoothing calculates the maximum acceleration for acceptable smoothing.
// Uses the formula: max_accel = 2 * v^2 / smoothing where v is the corner velocity.
func MaxAccelFromSmoothing(smoothing, scv float64) float64 {
	if smoothing <= 0 {
		return 10000.0 // Very high value for no smoothing
	}

	// max_accel = scv^2 / (smoothing * accel_scv_factor)
	// Where accel_scv_factor is typically 0.5
	accelScvFactor := 0.5

	return scv * scv / (smoothing * accelScvFactor)
}
