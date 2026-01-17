// thermistor implements thermistor temperature calculation using Steinhart-Hart equation.
// This provides accurate temperature conversion from ADC readings for NTC thermistors.
package hosth4

import (
	"fmt"
	"math"
)

const (
	// KelvinToCelsius is the offset from Kelvin to Celsius
	KelvinToCelsius = -273.15
)

// Thermistor calculates temperature from ADC readings using Steinhart-Hart coefficients.
type Thermistor struct {
	pullup         float64 // Pullup resistor value (ohms)
	inlineResistor float64 // Inline resistor value (ohms)
	c1, c2, c3     float64 // Steinhart-Hart coefficients
}

// NewThermistor creates a new thermistor with the given pullup and inline resistors.
func NewThermistor(pullup, inlineResistor float64) *Thermistor {
	return &Thermistor{
		pullup:         pullup,
		inlineResistor: inlineResistor,
	}
}

// SetupCoefficients calculates Steinhart-Hart coefficients from three temperature/resistance pairs.
// Temperatures should be in Celsius, resistances in ohms.
func (t *Thermistor) SetupCoefficients(t1, r1, t2, r2, t3, r3 float64) error {
	// Convert temperatures to Kelvin
	invT1 := 1.0 / (t1 - KelvinToCelsius)
	invT2 := 1.0 / (t2 - KelvinToCelsius)
	invT3 := 1.0 / (t3 - KelvinToCelsius)

	// Natural log of resistances
	lnR1 := math.Log(r1)
	lnR2 := math.Log(r2)
	lnR3 := math.Log(r3)

	// Cubed values
	ln3R1 := lnR1 * lnR1 * lnR1
	ln3R2 := lnR2 * lnR2 * lnR2
	ln3R3 := lnR3 * lnR3 * lnR3

	// Differences
	invT12 := invT1 - invT2
	invT13 := invT1 - invT3
	lnR12 := lnR1 - lnR2
	lnR13 := lnR1 - lnR3
	ln3R12 := ln3R1 - ln3R2
	ln3R13 := ln3R1 - ln3R3

	// Calculate c3
	t.c3 = (invT12 - invT13*lnR12/lnR13) / (ln3R12 - ln3R13*lnR12/lnR13)

	// If c3 is invalid, fall back to beta calculation
	if t.c3 <= 0 {
		// Calculate beta and use simpler model
		beta := lnR13 / invT13
		return t.SetupCoefficientsBeta(t1, r1, beta)
	}

	// Calculate c2 and c1
	t.c2 = (invT12 - t.c3*ln3R12) / lnR12
	t.c1 = invT1 - t.c2*lnR1 - t.c3*ln3R1

	return nil
}

// SetupCoefficientsBeta calculates coefficients from a single temperature/resistance point and beta value.
func (t *Thermistor) SetupCoefficientsBeta(t1, r1, beta float64) error {
	invT1 := 1.0 / (t1 - KelvinToCelsius)
	lnR1 := math.Log(r1)

	t.c3 = 0
	t.c2 = 1.0 / beta
	t.c1 = invT1 - t.c2*lnR1

	return nil
}

// CalcTemp calculates temperature (Celsius) from ADC reading (0.0-1.0).
func (t *Thermistor) CalcTemp(adc float64) float64 {
	// Clamp ADC to valid range
	adc = math.Max(0.00001, math.Min(0.99999, adc))

	// Calculate resistance from voltage divider
	// ADC = R / (R + pullup) for high-side pullup
	// Solving: R = pullup * adc / (1.0 - adc)
	r := t.pullup * adc / (1.0 - adc)

	// Account for inline resistor
	r = r - t.inlineResistor
	if r <= 0 {
		return 300.0 // Return high temp for short circuit
	}

	// Calculate temperature using Steinhart-Hart equation
	lnR := math.Log(r)
	invT := t.c1 + t.c2*lnR + t.c3*lnR*lnR*lnR

	if invT <= 0 {
		return 300.0 // Return high temp for invalid calculation
	}

	return 1.0/invT + KelvinToCelsius
}

// CalcADC calculates ADC reading (0.0-1.0) from temperature (Celsius).
func (t *Thermistor) CalcADC(temp float64) float64 {
	if temp <= KelvinToCelsius {
		return 1.0
	}

	invT := 1.0 / (temp - KelvinToCelsius)

	var lnR float64
	if t.c3 != 0 {
		// Solve cubic equation using Cardano's formula
		y := (t.c1 - invT) / (2.0 * t.c3)
		x := math.Sqrt(math.Pow(t.c2/(3.0*t.c3), 3) + y*y)
		lnR = math.Cbrt(x-y) - math.Cbrt(x+y)
	} else {
		// Simple case when c3 = 0
		lnR = (invT - t.c1) / t.c2
	}

	r := math.Exp(lnR) + t.inlineResistor

	// Convert resistance to ADC
	return r / (t.pullup + r)
}

// ThermistorProfile defines a thermistor type with its parameters.
type ThermistorProfile struct {
	Name string
	T1   float64 // Temperature 1 (°C)
	R1   float64 // Resistance at T1 (ohms)
	T2   float64 // Temperature 2 (°C)
	R2   float64 // Resistance at T2 (ohms)
	T3   float64 // Temperature 3 (°C)
	R3   float64 // Resistance at T3 (ohms)
	Beta float64 // Beta value (optional, if set, use beta instead of Steinhart-Hart)
}

// Common thermistor profiles based on Klipper's built-in types
var ThermistorProfiles = map[string]ThermistorProfile{
	// EPCOS 100K B57560G104F thermistor
	"EPCOS 100K B57560G104F": {
		Name: "EPCOS 100K B57560G104F",
		T1:   25, R1: 100000,
		T2: 150, R2: 1641.9,
		T3: 250, R3: 226.15,
	},
	// Generic 100K NTC with beta=3950
	"NTC 100K beta 3950": {
		Name: "NTC 100K beta 3950",
		T1:   25, R1: 100000,
		Beta: 3950,
	},
	// ATC Semitec 104GT-2/104NT-4-R025H42G
	"ATC Semitec 104GT-2": {
		Name: "ATC Semitec 104GT-2",
		T1:   25, R1: 100000,
		T2:   160, R2: 1074,
		T3:   300, R3: 82.78,
	},
	// ATC Semitec 104NT-4-R025H42G (same as above but different name)
	"ATC Semitec 104NT-4-R025H42G": {
		Name: "ATC Semitec 104NT-4-R025H42G",
		T1:   25, R1: 100000,
		T2:   160, R2: 1074,
		T3:   300, R3: 82.78,
	},
	// Generic 3950 NTC thermistor
	"Generic 3950": {
		Name: "Generic 3950",
		T1:   25, R1: 100000,
		Beta: 3950,
	},
	// Honeywell 135-104LAG-J01
	"Honeywell 100K 135-104LAG-J01": {
		Name: "Honeywell 100K 135-104LAG-J01",
		T1:   25, R1: 100000,
		T2:   150, R2: 1768,
		T3:   310, R3: 130.2,
	},
	// SliceEngineering 450 (high temperature)
	"SliceEngineering 450": {
		Name: "SliceEngineering 450",
		T1:   25, R1: 500000,
		T2:   260, R2: 2240,
		T3:   460, R3: 107.2,
	},
	// TDK NTCG104LH104JT1 (AIO)
	"TDK NTCG104LH104JT1": {
		Name: "TDK NTCG104LH104JT1",
		T1:   25, R1: 100000,
		T2:   150, R2: 1449,
		T3:   250, R3: 195.5,
	},
	// PT1000 RTD sensor (linear approximation)
	"PT1000": {
		Name: "PT1000",
		T1:   0, R1: 1000,
		T2:   100, R2: 1385,
		T3:   200, R3: 1758,
	},
}

// NewThermistorFromProfile creates a thermistor from a named profile.
func NewThermistorFromProfile(profileName string, pullup, inlineResistor float64) (*Thermistor, error) {
	profile, ok := ThermistorProfiles[profileName]
	if !ok {
		// Try case-insensitive match
		for name, p := range ThermistorProfiles {
			if equalFoldSimple(name, profileName) {
				profile = p
				ok = true
				break
			}
		}
	}
	if !ok {
		return nil, fmt.Errorf("unknown thermistor profile: %s", profileName)
	}

	t := NewThermistor(pullup, inlineResistor)

	if profile.Beta > 0 {
		// Use beta calculation
		if err := t.SetupCoefficientsBeta(profile.T1, profile.R1, profile.Beta); err != nil {
			return nil, err
		}
	} else {
		// Use Steinhart-Hart calculation
		if err := t.SetupCoefficients(profile.T1, profile.R1, profile.T2, profile.R2, profile.T3, profile.R3); err != nil {
			return nil, err
		}
	}

	return t, nil
}

// equalFoldSimple is a simple case-insensitive string comparison.
func equalFoldSimple(a, b string) bool {
	if len(a) != len(b) {
		return false
	}
	for i := 0; i < len(a); i++ {
		ca, cb := a[i], b[i]
		if ca >= 'A' && ca <= 'Z' {
			ca += 'a' - 'A'
		}
		if cb >= 'A' && cb <= 'Z' {
			cb += 'a' - 'A'
		}
		if ca != cb {
			return false
		}
	}
	return true
}

// LinearInterpolator provides temperature lookup from resistance/voltage tables.
type LinearInterpolator struct {
	keys   []float64
	slopes []struct{ gain, offset float64 }
}

// NewLinearInterpolator creates a linear interpolator from (key, value) pairs.
func NewLinearInterpolator(samples [][2]float64) (*LinearInterpolator, error) {
	if len(samples) < 2 {
		return nil, fmt.Errorf("need at least two samples for interpolation")
	}

	// Sort samples by key
	sorted := make([][2]float64, len(samples))
	copy(sorted, samples)
	for i := 0; i < len(sorted)-1; i++ {
		for j := i + 1; j < len(sorted); j++ {
			if sorted[j][0] < sorted[i][0] {
				sorted[i], sorted[j] = sorted[j], sorted[i]
			}
		}
	}

	li := &LinearInterpolator{
		keys:   make([]float64, 0, len(sorted)),
		slopes: make([]struct{ gain, offset float64 }, 0, len(sorted)),
	}

	var lastKey, lastValue float64
	first := true

	for _, sample := range sorted {
		key, value := sample[0], sample[1]

		if first {
			lastKey, lastValue = key, value
			first = false
			continue
		}

		if key <= lastKey {
			continue // Skip duplicates
		}

		gain := (value - lastValue) / (key - lastKey)
		offset := lastValue - lastKey*gain

		// Check if this slope matches the previous one
		if len(li.slopes) > 0 {
			prev := li.slopes[len(li.slopes)-1]
			if prev.gain == gain && prev.offset == offset {
				continue
			}
		}

		li.keys = append(li.keys, key)
		li.slopes = append(li.slopes, struct{ gain, offset float64 }{gain, offset})

		lastKey, lastValue = key, value
	}

	if len(li.keys) == 0 {
		return nil, fmt.Errorf("need at least two distinct samples")
	}

	// Add sentinel for extrapolation
	li.keys = append(li.keys, math.MaxFloat64)
	li.slopes = append(li.slopes, li.slopes[len(li.slopes)-1])

	return li, nil
}

// Interpolate returns the interpolated value for the given key.
func (li *LinearInterpolator) Interpolate(key float64) float64 {
	// Binary search for position
	pos := 0
	for pos < len(li.keys)-1 && li.keys[pos] < key {
		pos++
	}

	slope := li.slopes[pos]
	return key*slope.gain + slope.offset
}

// ReverseInterpolate returns the key for the given value.
func (li *LinearInterpolator) ReverseInterpolate(value float64) float64 {
	// Find the segment containing this value
	for i := 0; i < len(li.slopes); i++ {
		slope := li.slopes[i]
		if slope.gain == 0 {
			continue
		}
		return (value - slope.offset) / slope.gain
	}
	return 0
}
