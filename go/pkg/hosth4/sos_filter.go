// SOS Filter - port of klippy/extras/sos_filter.py
//
// Second-Order Sections (SOS) digital filter implementation
//
// Copyright (C) 2020-2023 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"log"
	"sync"
)

// SOSSection represents a single second-order section (biquad).
type SOSSection struct {
	b0, b1, b2 float64 // Numerator coefficients
	a1, a2     float64 // Denominator coefficients (a0 is normalized to 1)
	z1, z2     float64 // State variables
}

// SOSFilter represents a cascade of second-order sections.
type SOSFilter struct {
	rt       *runtime
	name     string
	sections []SOSSection
	gain     float64
	mu       sync.Mutex
}

// SOSFilterConfig holds configuration for SOS filter.
type SOSFilterConfig struct {
	Name     string
	Sections [][]float64 // Each section: [b0, b1, b2, a0, a1, a2]
	Gain     float64
}

// DefaultSOSFilterConfig returns default configuration.
func DefaultSOSFilterConfig() SOSFilterConfig {
	return SOSFilterConfig{
		Gain: 1.0,
	}
}

// newSOSFilter creates a new SOS filter.
func newSOSFilter(rt *runtime, cfg SOSFilterConfig) *SOSFilter {
	sections := make([]SOSSection, len(cfg.Sections))

	for i, s := range cfg.Sections {
		if len(s) >= 6 {
			// Normalize by a0
			a0 := s[3]
			if a0 == 0 {
				a0 = 1
			}
			sections[i] = SOSSection{
				b0: s[0] / a0,
				b1: s[1] / a0,
				b2: s[2] / a0,
				a1: s[4] / a0,
				a2: s[5] / a0,
			}
		}
	}

	f := &SOSFilter{
		rt:       rt,
		name:     cfg.Name,
		sections: sections,
		gain:     cfg.Gain,
	}

	log.Printf("sos_filter: initialized '%s' with %d sections", cfg.Name, len(sections))
	return f
}

// GetName returns the filter name.
func (f *SOSFilter) GetName() string {
	return f.name
}

// Process processes a single sample through the filter.
func (f *SOSFilter) Process(input float64) float64 {
	f.mu.Lock()
	defer f.mu.Unlock()

	x := input * f.gain

	for i := range f.sections {
		s := &f.sections[i]

		// Direct Form II Transposed
		y := s.b0*x + s.z1
		s.z1 = s.b1*x - s.a1*y + s.z2
		s.z2 = s.b2*x - s.a2*y

		x = y
	}

	return x
}

// ProcessBatch processes multiple samples through the filter.
func (f *SOSFilter) ProcessBatch(inputs []float64) []float64 {
	outputs := make([]float64, len(inputs))
	for i, input := range inputs {
		outputs[i] = f.Process(input)
	}
	return outputs
}

// Reset resets the filter state.
func (f *SOSFilter) Reset() {
	f.mu.Lock()
	defer f.mu.Unlock()

	for i := range f.sections {
		f.sections[i].z1 = 0
		f.sections[i].z2 = 0
	}
}

// SetGain sets the input gain.
func (f *SOSFilter) SetGain(gain float64) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.gain = gain
}

// GetGain returns the input gain.
func (f *SOSFilter) GetGain() float64 {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.gain
}

// GetSectionCount returns the number of sections.
func (f *SOSFilter) GetSectionCount() int {
	return len(f.sections)
}

// GetStatus returns the filter status.
func (f *SOSFilter) GetStatus() map[string]any {
	f.mu.Lock()
	defer f.mu.Unlock()

	return map[string]any{
		"name":          f.name,
		"section_count": len(f.sections),
		"gain":          f.gain,
	}
}

// CreateButterworthLowpass creates Butterworth lowpass filter coefficients.
// This is a helper for common filter design.
func CreateButterworthLowpass(order int, cutoffFreq, sampleRate float64) [][]float64 {
	// Simplified 2nd order Butterworth for demonstration
	// In real implementation, would use proper filter design
	if order != 2 {
		log.Printf("sos_filter: only order 2 Butterworth implemented")
	}

	// Normalized frequency
	wc := cutoffFreq / sampleRate * 2

	// Simple 2nd order lowpass approximation
	sections := [][]float64{
		{wc * wc, 2 * wc * wc, wc * wc, 1, 2 * wc, wc * wc},
	}

	return sections
}

// CreateNotchFilter creates a notch filter at specified frequency.
func CreateNotchFilter(notchFreq, bandwidth, sampleRate float64) [][]float64 {
	// Simplified notch filter design
	w0 := notchFreq / sampleRate * 2
	bw := bandwidth / sampleRate * 2

	// Second-order notch
	sections := [][]float64{
		{1, -2 * w0, 1, 1, -2 * w0 * (1 - bw), 1 - bw*bw},
	}

	return sections
}
