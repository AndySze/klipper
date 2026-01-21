// Shaper Calibrate - port of klippy/extras/shaper_calibrate.py
//
// Automatic calibration of input shapers
//
// Copyright (C) 2020-2024 Dmitry Butyugin <dmbutyugin@google.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"math"
	"sort"
	"sync"
)

const (
	calibrateMinFreq       = 5.0
	calibrateMaxFreq       = 200.0
	calibrateWindowTSec    = 0.5
	calibrateMaxShaperFreq = 150.0

	// Default square corner velocity for smoothing calculation
	defaultSCV = 5.0

	// Frequency resolution for shaper fitting (Hz)
	freqStep = 1.0

	// Scoring formula coefficients: score = smoothing * (vibrs^1.5 + vibrs*0.2 + 0.01)
	vibrPower  = 1.5
	vibrCoef   = 0.2
	vibrOffset = 0.01
)

// Test damping ratios for calibration.
var testDampingRatios = []float64{0.075, 0.1, 0.15}

// Autotune shaper types.
var autotuneShapers = []string{"zv", "mzv", "ei", "2hump_ei", "3hump_ei"}

// CalibrationData holds frequency response data.
type CalibrationData struct {
	Name     string
	FreqBins []float64
	PsdSum   []float64
	PsdX     []float64
	PsdY     []float64
	PsdZ     []float64
	DataSets []*CalibrationData
	mu       sync.Mutex
}

// NewCalibrationData creates new calibration data.
func NewCalibrationData(name string, freqBins, psdSum, psdX, psdY, psdZ []float64) *CalibrationData {
	return &CalibrationData{
		Name:     name,
		FreqBins: freqBins,
		PsdSum:   psdSum,
		PsdX:     psdX,
		PsdY:     psdY,
		PsdZ:     psdZ,
		DataSets: make([]*CalibrationData, 0),
	}
}

// AddData adds another calibration data set.
func (cd *CalibrationData) AddData(other *CalibrationData) {
	cd.mu.Lock()
	defer cd.mu.Unlock()
	cd.DataSets = append(cd.DataSets, other.GetDatasets()...)
}

// NormalizeToFrequencies normalizes PSD data to frequencies.
func (cd *CalibrationData) NormalizeToFrequencies() {
	cd.mu.Lock()
	defer cd.mu.Unlock()

	psds := [][]float64{cd.PsdSum, cd.PsdX, cd.PsdY, cd.PsdZ}
	for _, psd := range psds {
		if psd == nil {
			continue
		}
		for i := range psd {
			// Avoid division by zero
			psd[i] /= (cd.FreqBins[i] + 0.1)
			// Remove low-frequency noise
			if cd.FreqBins[i] < 2.0*calibrateMinFreq {
				factor := math.Exp(-(2.0*calibrateMinFreq/(cd.FreqBins[i]+0.1))*(2.0*calibrateMinFreq/(cd.FreqBins[i]+0.1)) + 1.0)
				psd[i] *= factor
			}
		}
	}

	for _, other := range cd.DataSets {
		other.NormalizeToFrequencies()
	}
}

// GetPSD returns PSD data for a specific axis.
func (cd *CalibrationData) GetPSD(axis string) []float64 {
	cd.mu.Lock()
	defer cd.mu.Unlock()

	switch axis {
	case "x":
		return cd.PsdX
	case "y":
		return cd.PsdY
	case "z":
		return cd.PsdZ
	default:
		return cd.PsdSum
	}
}

// GetDatasets returns all data sets including self.
func (cd *CalibrationData) GetDatasets() []*CalibrationData {
	cd.mu.Lock()
	defer cd.mu.Unlock()

	result := []*CalibrationData{cd}
	result = append(result, cd.DataSets...)
	return result
}

// CalibrationResult holds the result of shaper calibration.
type CalibrationResult struct {
	Name      string
	Freq      float64
	FreqBins  []float64
	Vals      []float64
	Vibrs     []float64
	Smoothing float64
	Score     float64
	MaxAccel  float64
}

// ShaperCalibrate provides input shaper calibration.
type ShaperCalibrate struct {
	rt  *runtime
	scv float64 // Square corner velocity
	mu  sync.Mutex
}

// newShaperCalibrate creates a new shaper calibrator.
func newShaperCalibrate(rt *runtime) *ShaperCalibrate {
	sc := &ShaperCalibrate{
		rt:  rt,
		scv: defaultSCV,
	}

	log.Printf("shaper_calibrate: initialized")
	return sc
}

// SetSCV sets the square corner velocity used for smoothing calculations.
func (sc *ShaperCalibrate) SetSCV(scv float64) {
	sc.mu.Lock()
	defer sc.mu.Unlock()
	sc.scv = scv
}

// EstimateShaper estimates the best shaper for the given calibration data.
func (sc *ShaperCalibrate) EstimateShaper(calibData *CalibrationData, axis string, maxSmoothing float64) (*CalibrationResult, error) {
	sc.mu.Lock()
	defer sc.mu.Unlock()

	if calibData == nil {
		return nil, fmt.Errorf("no calibration data provided")
	}

	calibData.NormalizeToFrequencies()
	psd := calibData.GetPSD(axis)
	if psd == nil {
		return nil, fmt.Errorf("no PSD data for axis %s", axis)
	}

	var bestResult *CalibrationResult

	for _, shaperName := range autotuneShapers {
		shaper := GetShaperByName(shaperName)
		if shaper == nil {
			continue
		}

		result := sc.fitShaper(calibData.FreqBins, psd, shaperName, shaper, maxSmoothing)
		if result != nil && (bestResult == nil || result.Score < bestResult.Score) {
			bestResult = result
		}
	}

	if bestResult == nil {
		return nil, fmt.Errorf("failed to find suitable shaper")
	}

	return bestResult, nil
}

// fitShaper fits a shaper to the given PSD data, matching Python's fit_shaper().
// It tests all frequency/damping combinations and returns the one with best score.
func (sc *ShaperCalibrate) fitShaper(freqBins, psd []float64, shaperName string, shaper *InputShaperCfg, maxSmoothing float64) *CalibrationResult {
	freqRange := [2]float64{calibrateMinFreq, calibrateMaxFreq}

	// Build arrays for all frequencies we'll evaluate
	var testFreqs []float64
	for freq := shaper.MinFreq; freq <= calibrateMaxShaperFreq; freq += freqStep {
		testFreqs = append(testFreqs, freq)
	}

	if len(testFreqs) == 0 {
		return nil
	}

	// For each damping ratio, calculate vibrations at all test frequencies
	var allVibrs [][]float64
	for _, dampingRatio := range testDampingRatios {
		if dampingRatio > shaper.MaxDampingRatio {
			continue
		}

		vibrs := make([]float64, len(testFreqs))
		for i, freq := range testFreqs {
			A, T := shaper.InitFunc(freq, dampingRatio)
			vibrs[i] = sc.calcResidualVibration(freqBins, psd, A, T, freqRange)
		}
		allVibrs = append(allVibrs, vibrs)
	}

	if len(allVibrs) == 0 {
		return nil
	}

	// Find the "worst case" vibration across all damping ratios
	// This ensures robustness to unknown actual damping ratio
	worstVibrs := make([]float64, len(testFreqs))
	for i := range testFreqs {
		maxVibr := 0.0
		for _, vibrs := range allVibrs {
			if vibrs[i] > maxVibr {
				maxVibr = vibrs[i]
			}
		}
		worstVibrs[i] = maxVibr
	}

	// Calculate scores for each frequency
	// score = smoothing * (vibrs^1.5 + vibrs*0.2 + 0.01)
	var bestResult *CalibrationResult
	bestScore := math.Inf(1)

	for i, freq := range testFreqs {
		// Get shaper at default damping for smoothing calculation
		A, T := shaper.InitFunc(freq, testDampingRatios[0])
		smoothing := sc.calcSmoothing(A, T)

		// Skip if exceeds max smoothing constraint
		if maxSmoothing > 0 && smoothing > maxSmoothing {
			continue
		}

		vibr := worstVibrs[i]

		// Klipper scoring formula
		score := smoothing * (math.Pow(vibr, vibrPower) + vibr*vibrCoef + vibrOffset)

		if score < bestScore {
			bestScore = score
			maxAccel := sc.calcMaxAccel(A, T, smoothing)

			bestResult = &CalibrationResult{
				Name:      shaperName,
				Freq:      freq,
				FreqBins:  freqBins,
				Vals:      worstVibrs,
				Vibrs:     []float64{vibr},
				Smoothing: smoothing,
				Score:     score,
				MaxAccel:  maxAccel,
			}
		}
	}

	return bestResult
}

// calcResidualVibration calculates residual vibration using the proper formula.
// This matches Python's estimate_shaper() function.
func (sc *ShaperCalibrate) calcResidualVibration(freqBins, psd []float64, A, T []float64, freqRange [2]float64) float64 {
	return EstimateResidualVibration(freqBins, psd, A, T, freqRange)
}

// calcSmoothing calculates the smoothing effect of a shaper.
// This matches Python's get_shaper_smoothing() function.
func (sc *ShaperCalibrate) calcSmoothing(A, T []float64) float64 {
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

	// Calculate the center of mass (T0)
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

	// Smoothing = sqrt(12) * sqrt(m2) for uniform-like distributions
	// This gives the "equivalent uniform width" of the impulse response
	return math.Sqrt(12.0 * m2)
}

// calcMaxAccel calculates the maximum recommended acceleration.
// Based on the smoothing and square corner velocity.
func (sc *ShaperCalibrate) calcMaxAccel(A, T []float64, smoothing float64) float64 {
	if smoothing <= 0 {
		return 10000.0
	}

	// Klipper formula: max_accel = scv / (0.5 * smoothing)
	// This ensures corner velocities don't exceed scv
	accelFactor := 0.5
	return sc.scv / (accelFactor * smoothing)
}

// GetAutotuneResult performs auto-tuning and returns the recommended shaper.
func (sc *ShaperCalibrate) GetAutotuneResult(calibData *CalibrationData, axis string) (string, float64, float64, error) {
	result, err := sc.EstimateShaper(calibData, axis, 0)
	if err != nil {
		return "", 0, 0, err
	}

	return result.Name, result.Freq, result.MaxAccel, nil
}

// CompareResults compares multiple calibration results.
func (sc *ShaperCalibrate) CompareResults(results []*CalibrationResult) *CalibrationResult {
	if len(results) == 0 {
		return nil
	}

	// Sort by score
	sort.Slice(results, func(i, j int) bool {
		return results[i].Score < results[j].Score
	})

	return results[0]
}
