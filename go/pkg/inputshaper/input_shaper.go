// Input shaper for minimizing motion vibrations - port of klippy/extras/input_shaper.py
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020-2025  Dmitry Butyugin <dmbutyugin@google.com>
// Copyright (C) 2025  Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package inputshaper

import (
	"fmt"
	"strings"
)

// InputShaperParams holds the configuration parameters for one axis shaper.
type InputShaperParams struct {
	Axis         string
	ShaperType   ShaperType
	DampingRatio float64
	ShaperFreq   float64
}

// NewInputShaperParams creates shaper parameters from config values.
func NewInputShaperParams(axis string, shaperType ShaperType, dampingRatio, shaperFreq float64) (*InputShaperParams, error) {
	if shaperType == "" {
		shaperType = ShaperMZV // default
	}

	cfg := GetShaperByName(shaperType)
	if cfg == nil {
		return nil, fmt.Errorf("unsupported shaper type: %s", shaperType)
	}

	if dampingRatio <= 0 {
		dampingRatio = DefaultDampingRatio
	}
	if dampingRatio > cfg.MaxDampingRatio {
		return nil, fmt.Errorf("damping ratio %.3f exceeds maximum %.3f for shaper %s",
			dampingRatio, cfg.MaxDampingRatio, shaperType)
	}

	return &InputShaperParams{
		Axis:         axis,
		ShaperType:   shaperType,
		DampingRatio: dampingRatio,
		ShaperFreq:   shaperFreq,
	}, nil
}

// GetShaper returns the shaper coefficients (n, A, T).
func (p *InputShaperParams) GetShaper() (n int, A, T []float64) {
	if p.ShaperFreq == 0 {
		A, T = GetNoneShaper()
		return len(A), A, T
	}

	cfg := GetShaperByName(p.ShaperType)
	if cfg == nil {
		A, T = GetNoneShaper()
		return len(A), A, T
	}

	A, T = cfg.InitFunc(p.ShaperFreq, p.DampingRatio)
	return len(A), A, T
}

// Update updates the shaper parameters from G-code command values.
func (p *InputShaperParams) Update(shaperType *ShaperType, dampingRatio, shaperFreq *float64) error {
	if shaperType != nil && *shaperType != "" {
		cfg := GetShaperByName(*shaperType)
		if cfg == nil {
			return fmt.Errorf("unsupported shaper type: %s", *shaperType)
		}
		p.ShaperType = *shaperType
	}

	if dampingRatio != nil {
		cfg := GetShaperByName(p.ShaperType)
		if cfg != nil && *dampingRatio > cfg.MaxDampingRatio {
			return fmt.Errorf("damping ratio %.3f too high for shaper %s on axis %s",
				*dampingRatio, p.ShaperType, strings.ToUpper(p.Axis))
		}
		p.DampingRatio = *dampingRatio
	}

	if shaperFreq != nil {
		p.ShaperFreq = *shaperFreq
	}

	return nil
}

// GetStatus returns status information about this shaper.
func (p *InputShaperParams) GetStatus() map[string]interface{} {
	return map[string]interface{}{
		"shaper_type":   string(p.ShaperType),
		"shaper_freq":   fmt.Sprintf("%.3f", p.ShaperFreq),
		"damping_ratio": fmt.Sprintf("%.6f", p.DampingRatio),
	}
}

// AxisInputShaper manages input shaping for a single axis.
type AxisInputShaper struct {
	Axis   string
	Params *InputShaperParams
	N      int
	A      []float64
	T      []float64
	saved  *struct {
		N int
		A []float64
		T []float64
	}
}

// NewAxisInputShaper creates an axis input shaper.
func NewAxisInputShaper(axis string, shaperType ShaperType, dampingRatio, shaperFreq float64) (*AxisInputShaper, error) {
	params, err := NewInputShaperParams(axis, shaperType, dampingRatio, shaperFreq)
	if err != nil {
		return nil, err
	}

	shaper := &AxisInputShaper{
		Axis:   axis,
		Params: params,
	}
	shaper.N, shaper.A, shaper.T = params.GetShaper()
	return shaper, nil
}

// GetName returns the shaper name.
func (s *AxisInputShaper) GetName() string {
	return "shaper_" + s.Axis
}

// GetShaper returns the current shaper coefficients.
func (s *AxisInputShaper) GetShaper() (n int, A, T []float64) {
	return s.N, s.A, s.T
}

// Update updates the shaper from command parameters.
func (s *AxisInputShaper) Update(shaperType *ShaperType, dampingRatio, shaperFreq *float64) error {
	if err := s.Params.Update(shaperType, dampingRatio, shaperFreq); err != nil {
		return err
	}
	s.N, s.A, s.T = s.Params.GetShaper()
	return nil
}

// IsEnabled returns true if shaping is active.
func (s *AxisInputShaper) IsEnabled() bool {
	return s.N > 0
}

// DisableShaping temporarily disables shaping.
func (s *AxisInputShaper) DisableShaping() {
	if s.saved == nil && s.N > 0 {
		s.saved = &struct {
			N int
			A []float64
			T []float64
		}{s.N, s.A, s.T}
	}
	A, T := GetNoneShaper()
	s.N, s.A, s.T = len(A), A, T
}

// EnableShaping re-enables previously saved shaping.
func (s *AxisInputShaper) EnableShaping() {
	if s.saved == nil {
		return
	}
	s.N, s.A, s.T = s.saved.N, s.saved.A, s.saved.T
	s.saved = nil
}

// GetStatus returns status information.
func (s *AxisInputShaper) GetStatus() map[string]interface{} {
	return s.Params.GetStatus()
}

// InputShaper manages input shaping for all axes.
type InputShaper struct {
	Shapers []*AxisInputShaper
}

// NewInputShaper creates an input shaper from config.
func NewInputShaper(xType, yType, zType ShaperType, xFreq, yFreq, zFreq, dampingRatio float64) (*InputShaper, error) {
	xShaper, err := NewAxisInputShaper("x", xType, dampingRatio, xFreq)
	if err != nil {
		return nil, fmt.Errorf("x axis: %w", err)
	}

	yShaper, err := NewAxisInputShaper("y", yType, dampingRatio, yFreq)
	if err != nil {
		return nil, fmt.Errorf("y axis: %w", err)
	}

	zShaper, err := NewAxisInputShaper("z", zType, dampingRatio, zFreq)
	if err != nil {
		return nil, fmt.Errorf("z axis: %w", err)
	}

	return &InputShaper{
		Shapers: []*AxisInputShaper{xShaper, yShaper, zShaper},
	}, nil
}

// GetShapers returns all axis shapers.
func (is *InputShaper) GetShapers() []*AxisInputShaper {
	return is.Shapers
}

// DisableShaping disables all shaping.
func (is *InputShaper) DisableShaping() {
	for _, shaper := range is.Shapers {
		shaper.DisableShaping()
	}
}

// EnableShaping re-enables all shaping.
func (is *InputShaper) EnableShaping() {
	for _, shaper := range is.Shapers {
		shaper.EnableShaping()
	}
}

// GetStatus returns status for all shapers.
func (is *InputShaper) GetStatus() map[string]interface{} {
	result := make(map[string]interface{})
	for _, shaper := range is.Shapers {
		for k, v := range shaper.GetStatus() {
			result[k+"_"+shaper.Axis] = v
		}
	}
	return result
}
