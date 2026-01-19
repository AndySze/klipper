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
	calibrateMinFreq      = 5.0
	calibrateMaxFreq      = 200.0
	calibrateWindowTSec   = 0.5
	calibrateMaxShaperFreq = 150.0
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
	rt *runtime
	mu sync.Mutex
}

// newShaperCalibrate creates a new shaper calibrator.
func newShaperCalibrate(rt *runtime) *ShaperCalibrate {
	sc := &ShaperCalibrate{
		rt: rt,
	}

	log.Printf("shaper_calibrate: initialized")
	return sc
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

// fitShaper fits a shaper to the given PSD data.
func (sc *ShaperCalibrate) fitShaper(freqBins, psd []float64, shaperName string, shaper *InputShaperCfg, maxSmoothing float64) *CalibrationResult {
	// Try different frequencies and damping ratios
	var bestResult *CalibrationResult

	for freq := shaper.MinFreq; freq <= calibrateMaxShaperFreq; freq += 1.0 {
		for _, dampingRatio := range testDampingRatios {
			if dampingRatio > shaper.MaxDampingRatio {
				continue
			}

			// Get shaper coefficients
			A, T := shaper.InitFunc(freq, dampingRatio)

			// Calculate vibration and smoothing
			vibr := sc.estimateResidualVibration(freqBins, psd, A, T)
			smoothing := sc.estimateSmoothing(A, T)

			if maxSmoothing > 0 && smoothing > maxSmoothing {
				continue
			}

			// Calculate score (lower is better)
			score := vibr * math.Pow(smoothing+0.1, 0.65)

			if bestResult == nil || score < bestResult.Score {
				bestResult = &CalibrationResult{
					Name:      shaperName,
					Freq:      freq,
					FreqBins:  freqBins,
					Vibrs:     []float64{vibr},
					Smoothing: smoothing,
					Score:     score,
					MaxAccel:  sc.estimateMaxAccel(smoothing),
				}
			}
		}
	}

	return bestResult
}

// estimateResidualVibration estimates residual vibration with the shaper.
func (sc *ShaperCalibrate) estimateResidualVibration(freqBins, psd []float64, A, T []float64) float64 {
	if len(A) == 0 {
		return 1.0
	}

	// Simplified calculation - in reality this involves complex frequency response
	totalVibr := 0.0
	totalPsd := 0.0

	for i, freq := range freqBins {
		if freq < calibrateMinFreq || freq > calibrateMaxFreq {
			continue
		}

		// Calculate shaper response at this frequency
		response := sc.shaperResponse(freq, A, T)
		totalVibr += psd[i] * response * response
		totalPsd += psd[i]
	}

	if totalPsd == 0 {
		return 1.0
	}

	return math.Sqrt(totalVibr / totalPsd)
}

// shaperResponse calculates the frequency response of a shaper.
func (sc *ShaperCalibrate) shaperResponse(freq float64, A, T []float64) float64 {
	if len(A) == 0 {
		return 1.0
	}

	// Normalize amplitudes
	sumA := 0.0
	for _, a := range A {
		sumA += a
	}

	// Calculate complex frequency response
	omega := 2.0 * math.Pi * freq
	realPart := 0.0
	imagPart := 0.0

	for i, a := range A {
		phase := omega * T[i]
		realPart += (a / sumA) * math.Cos(phase)
		imagPart += (a / sumA) * math.Sin(phase)
	}

	return math.Sqrt(realPart*realPart + imagPart*imagPart)
}

// estimateSmoothing estimates the smoothing effect of a shaper.
func (sc *ShaperCalibrate) estimateSmoothing(A, T []float64) float64 {
	if len(T) == 0 {
		return 0.0
	}

	// Smoothing is approximately the shaper duration
	maxT := 0.0
	for _, t := range T {
		if t > maxT {
			maxT = t
		}
	}

	return maxT
}

// estimateMaxAccel estimates the maximum acceleration for a given smoothing.
func (sc *ShaperCalibrate) estimateMaxAccel(smoothing float64) float64 {
	if smoothing <= 0 {
		return 10000.0
	}

	// Rough estimate: max_accel ≈ max_velocity / smoothing
	// Using a typical max_velocity of 500 mm/s
	return 500.0 / smoothing
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
