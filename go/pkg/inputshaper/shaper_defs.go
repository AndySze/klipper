// Input shaper definitions - port of klippy/extras/shaper_defs.py
//
// Copyright (C) 2020-2021  Dmitry Butyugin <dmbutyugin@google.com>
// Copyright (C) 2025  Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package inputshaper

import "math"

const (
	ShaperVibrationReduction = 20.0
	DefaultDampingRatio      = 0.1
)

// ShaperType represents an input shaper algorithm type.
type ShaperType string

const (
	ShaperNone    ShaperType = "none"
	ShaperZV      ShaperType = "zv"
	ShaperMZV     ShaperType = "mzv"
	ShaperZVD     ShaperType = "zvd"
	ShaperEI      ShaperType = "ei"
	Shaper2HumpEI ShaperType = "2hump_ei"
	Shaper3HumpEI ShaperType = "3hump_ei"
)

// ShaperConfig defines the configuration for a shaper type.
type ShaperConfig struct {
	Name            ShaperType
	InitFunc        func(shaperFreq, dampingRatio float64) (A, T []float64)
	MinFreq         float64
	MaxDampingRatio float64
}

// InputShapers is the list of available input shapers.
var InputShapers = []ShaperConfig{
	{Name: ShaperZV, InitFunc: GetZVShaper, MinFreq: 21.0, MaxDampingRatio: 0.99},
	{Name: ShaperMZV, InitFunc: GetMZVShaper, MinFreq: 23.0, MaxDampingRatio: 0.99},
	{Name: ShaperZVD, InitFunc: GetZVDShaper, MinFreq: 29.0, MaxDampingRatio: 0.99},
	{Name: ShaperEI, InitFunc: GetEIShaper, MinFreq: 29.0, MaxDampingRatio: 0.4},
	{Name: Shaper2HumpEI, InitFunc: Get2HumpEIShaper, MinFreq: 39.0, MaxDampingRatio: 0.3},
	{Name: Shaper3HumpEI, InitFunc: Get3HumpEIShaper, MinFreq: 48.0, MaxDampingRatio: 0.2},
}

// GetShaperByName returns the shaper config for the given name.
func GetShaperByName(name ShaperType) *ShaperConfig {
	for i := range InputShapers {
		if InputShapers[i].Name == name {
			return &InputShapers[i]
		}
	}
	return nil
}

// GetNoneShaper returns empty shaper arrays (no shaping).
func GetNoneShaper() (A, T []float64) {
	return []float64{}, []float64{}
}

// GetZVShaper computes the ZV (Zero Vibration) shaper coefficients.
func GetZVShaper(shaperFreq, dampingRatio float64) (A, T []float64) {
	df := math.Sqrt(1.0 - dampingRatio*dampingRatio)
	K := math.Exp(-dampingRatio * math.Pi / df)
	t_d := 1.0 / (shaperFreq * df)
	A = []float64{1.0, K}
	T = []float64{0.0, 0.5 * t_d}
	return A, T
}

// GetZVDShaper computes the ZVD (Zero Vibration Derivative) shaper coefficients.
func GetZVDShaper(shaperFreq, dampingRatio float64) (A, T []float64) {
	df := math.Sqrt(1.0 - dampingRatio*dampingRatio)
	K := math.Exp(-dampingRatio * math.Pi / df)
	t_d := 1.0 / (shaperFreq * df)
	A = []float64{1.0, 2.0 * K, K * K}
	T = []float64{0.0, 0.5 * t_d, t_d}
	return A, T
}

// GetMZVShaper computes the MZV (Modified Zero Vibration) shaper coefficients.
func GetMZVShaper(shaperFreq, dampingRatio float64) (A, T []float64) {
	df := math.Sqrt(1.0 - dampingRatio*dampingRatio)
	K := math.Exp(-0.75 * dampingRatio * math.Pi / df)
	t_d := 1.0 / (shaperFreq * df)

	a1 := 1.0 - 1.0/math.Sqrt(2.0)
	a2 := (math.Sqrt(2.0) - 1.0) * K
	a3 := a1 * K * K

	A = []float64{a1, a2, a3}
	T = []float64{0.0, 0.375 * t_d, 0.75 * t_d}
	return A, T
}

// GetEIShaper computes the EI (Extra Insensitive) shaper coefficients.
func GetEIShaper(shaperFreq, dampingRatio float64) (A, T []float64) {
	v_tol := 1.0 / ShaperVibrationReduction
	df := math.Sqrt(1.0 - dampingRatio*dampingRatio)
	t_d := 1.0 / (shaperFreq * df)
	dr := dampingRatio

	a1 := (0.24968 + 0.24961*v_tol) + ((0.80008+1.23328*v_tol)+
		(0.49599+3.17316*v_tol)*dr)*dr
	a3 := (0.25149 + 0.21474*v_tol) + ((-0.83249+1.41498*v_tol)+
		(0.85181-4.90094*v_tol)*dr)*dr
	a2 := 1.0 - a1 - a3

	t2 := 0.4999 + (((0.46159+8.57843*v_tol)*v_tol)+
		(((4.26169-108.644*v_tol)*v_tol)+
			((1.75601+336.989*v_tol)*v_tol)*dr)*dr)*dr

	A = []float64{a1, a2, a3}
	T = []float64{0.0, t2 * t_d, t_d}
	return A, T
}

// getShaperFromExpansionCoeffs computes shaper from polynomial expansion coefficients.
func getShaperFromExpansionCoeffs(shaperFreq, dampingRatio float64, t, a [][]float64) (A, T []float64) {
	tau := 1.0 / shaperFreq
	n := len(a)
	k := len(a[0])
	A = make([]float64, n)
	T = make([]float64, n)

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

// Get2HumpEIShaper computes the 2-hump EI shaper coefficients.
func Get2HumpEIShaper(shaperFreq, dampingRatio float64) (A, T []float64) {
	t := [][]float64{
		{0.0, 0.0, 0.0, 0.0},
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

// Get3HumpEIShaper computes the 3-hump EI shaper coefficients.
func Get3HumpEIShaper(shaperFreq, dampingRatio float64) (A, T []float64) {
	t := [][]float64{
		{0.0, 0.0, 0.0, 0.0},
		{0.49974, 0.23834, 0.44559, 12.4720},
		{0.99849, 0.29808, -2.36460, 23.3990},
		{1.49870, 0.10306, -2.01390, 17.0320},
		{1.99960, -0.28231, 0.61536, 5.40450},
	}
	a := [][]float64{
		{0.11275, 0.76632, 3.29160, -1.44380}, // Note: Python has typo 3.29160 -1.44380
		{0.23698, 0.61164, -2.57850, 4.85220},
		{0.30008, -0.19062, -2.14560, 0.13744},
		{0.23775, -0.73297, 0.46885, -2.08650},
		{0.11244, -0.45439, 0.96382, -1.46000},
	}
	return getShaperFromExpansionCoeffs(shaperFreq, dampingRatio, t, a)
}
