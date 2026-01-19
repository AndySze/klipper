// Shaper Definitions - port of klippy/extras/shaper_defs.py
//
// Definitions of the supported input shapers
//
// Copyright (C) 2020-2021 Dmitry Butyugin <dmbutyugin@google.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"math"
)

const (
	// ShaperVibrationReduction is the default vibration reduction factor.
	ShaperVibrationReduction = 20.0
	// DefaultDampingRatio is the default damping ratio.
	DefaultDampingRatio = 0.1
)

// ShaperInitFunc is a function that initializes a shaper.
type ShaperInitFunc func(shaperFreq, dampingRatio float64) ([]float64, []float64)

// InputShaperCfg holds configuration for an input shaper type.
type InputShaperCfg struct {
	Name            string
	InitFunc        ShaperInitFunc
	MinFreq         float64
	MaxDampingRatio float64
}

// GetNoneShaper returns an empty shaper (no shaping).
func GetNoneShaper() ([]float64, []float64) {
	return []float64{}, []float64{}
}

// GetZVShaper returns the ZV (Zero Vibration) shaper coefficients.
func GetZVShaper(shaperFreq, dampingRatio float64) ([]float64, []float64) {
	df := math.Sqrt(1.0 - dampingRatio*dampingRatio)
	K := math.Exp(-dampingRatio * math.Pi / df)
	tD := 1.0 / (shaperFreq * df)
	A := []float64{1.0, K}
	T := []float64{0.0, 0.5 * tD}
	return A, T
}

// GetZVDShaper returns the ZVD (Zero Vibration and Derivative) shaper coefficients.
func GetZVDShaper(shaperFreq, dampingRatio float64) ([]float64, []float64) {
	df := math.Sqrt(1.0 - dampingRatio*dampingRatio)
	K := math.Exp(-dampingRatio * math.Pi / df)
	tD := 1.0 / (shaperFreq * df)
	A := []float64{1.0, 2.0 * K, K * K}
	T := []float64{0.0, 0.5 * tD, tD}
	return A, T
}

// GetMZVShaper returns the MZV (Modified Zero Vibration) shaper coefficients.
func GetMZVShaper(shaperFreq, dampingRatio float64) ([]float64, []float64) {
	df := math.Sqrt(1.0 - dampingRatio*dampingRatio)
	K := math.Exp(-0.75 * dampingRatio * math.Pi / df)
	tD := 1.0 / (shaperFreq * df)

	a1 := 1.0 - 1.0/math.Sqrt(2.0)
	a2 := (math.Sqrt(2.0) - 1.0) * K
	a3 := a1 * K * K

	A := []float64{a1, a2, a3}
	T := []float64{0.0, 0.375 * tD, 0.75 * tD}
	return A, T
}

// GetEIShaper returns the EI (Extra Insensitive) shaper coefficients.
func GetEIShaper(shaperFreq, dampingRatio float64) ([]float64, []float64) {
	vTol := 1.0 / ShaperVibrationReduction // vibration tolerance
	df := math.Sqrt(1.0 - dampingRatio*dampingRatio)
	tD := 1.0 / (shaperFreq * df)
	dr := dampingRatio

	a1 := (0.24968 + 0.24961*vTol) + ((0.80008+1.23328*vTol)+
		(0.49599+3.17316*vTol)*dr)*dr
	a3 := (0.25149 + 0.21474*vTol) + ((-0.83249+1.41498*vTol)+
		(0.85181-4.90094*vTol)*dr)*dr
	a2 := 1.0 - a1 - a3

	t2 := 0.4999 + (((0.46159+8.57843*vTol)*vTol)+
		(((4.26169-108.644*vTol)*vTol)+
			((1.75601+336.989*vTol)*vTol)*dr)*dr)*dr

	A := []float64{a1, a2, a3}
	T := []float64{0.0, t2 * tD, tD}
	return A, T
}

// getShaperFromExpansionCoeffs calculates shaper from expansion coefficients.
func getShaperFromExpansionCoeffs(shaperFreq, dampingRatio float64, t, a [][]float64) ([]float64, []float64) {
	tau := 1.0 / shaperFreq
	n := len(a)
	k := len(a[0])

	T := make([]float64, n)
	A := make([]float64, n)

	for i := 0; i < n; i++ {
		u := t[i][k-1]
		v := a[i][k-1]
		for j := 0; j < k-1; j++ {
			u = u*dampingRatio + t[i][k-j-2]
			v = v*dampingRatio + a[i][k-j-2]
		}
		T[i] = u * tau
		A[i] = v
	}
	return A, T
}

// Get2HumpEIShaper returns the 2-hump EI shaper coefficients.
func Get2HumpEIShaper(shaperFreq, dampingRatio float64) ([]float64, []float64) {
	t := [][]float64{
		{0., 0., 0., 0.},
		{0.49890, 0.16270, -0.54262, 6.16180},
		{0.99748, 0.18382, -1.58270, 8.17120},
		{1.49920, -0.09297, -0.28338, 1.85710},
	}
	a := [][]float64{
		{0.16054, 0.76699, 2.26560, -1.22750},
		{0.33911, 0.45081, -2.58080, 1.73650},
		{0.34089, -0.61533, -0.68765, 0.42261},
		{0.15997, -0.60246, 1.00280, -0.93145},
	}
	return getShaperFromExpansionCoeffs(shaperFreq, dampingRatio, t, a)
}

// Get3HumpEIShaper returns the 3-hump EI shaper coefficients.
func Get3HumpEIShaper(shaperFreq, dampingRatio float64) ([]float64, []float64) {
	t := [][]float64{
		{0., 0., 0., 0.},
		{0.49974, 0.23834, 0.44559, 12.4720},
		{0.99849, 0.29808, -2.36460, 23.3990},
		{1.49870, 0.10306, -2.01390, 17.0320},
		{1.99960, -0.28231, 0.61536, 5.40450},
	}
	a := [][]float64{
		{0.11275, 0.76632, 3.29160, -1.44380},
		{0.23698, 0.61164, -2.57850, 4.85220},
		{0.30008, -0.19062, -2.14560, 0.13744},
		{0.23775, -0.73297, 0.46885, -2.08650},
		{0.11244, -0.45439, 0.96382, -1.46000},
	}
	return getShaperFromExpansionCoeffs(shaperFreq, dampingRatio, t, a)
}

// InputShapers is the list of available input shapers.
// min_freq for each shaper is chosen to have projected max_accel ~= 1500
var InputShapers = []InputShaperCfg{
	{Name: "zv", InitFunc: GetZVShaper, MinFreq: 21., MaxDampingRatio: 0.99},
	{Name: "mzv", InitFunc: GetMZVShaper, MinFreq: 23., MaxDampingRatio: 0.99},
	{Name: "zvd", InitFunc: GetZVDShaper, MinFreq: 29., MaxDampingRatio: 0.99},
	{Name: "ei", InitFunc: GetEIShaper, MinFreq: 29., MaxDampingRatio: 0.4},
	{Name: "2hump_ei", InitFunc: Get2HumpEIShaper, MinFreq: 39., MaxDampingRatio: 0.3},
	{Name: "3hump_ei", InitFunc: Get3HumpEIShaper, MinFreq: 48., MaxDampingRatio: 0.2},
}

// GetShaperByName returns a shaper configuration by name.
func GetShaperByName(name string) *InputShaperCfg {
	for i := range InputShapers {
		if InputShapers[i].Name == name {
			return &InputShapers[i]
		}
	}
	return nil
}

// GetShaperNames returns the names of all available shapers.
func GetShaperNames() []string {
	names := make([]string, len(InputShapers))
	for i, s := range InputShapers {
		names[i] = s.Name
	}
	return names
}
