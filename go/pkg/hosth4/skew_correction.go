// Skew Correction - port of klippy/extras/skew_correction.py
//
// Printer Skew Correction
//
// This implementation is a port of Marlin's skew correction as
// implemented in planner.h, Copyright (C) Marlin Firmware
//
// Copyright (C) 2019 Eric Callahan <arksine.code@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"math"
	"sync"
)

// CalcSkewFactor calculates skew factor from measured diagonal lengths.
func CalcSkewFactor(ac, bd, ad float64) float64 {
	side := math.Sqrt(2*ac*ac+2*bd*bd-4*ad*ad) / 2.0
	return math.Tan(math.Pi/2 - math.Acos((ac*ac-side*side-ad*ad)/(2*side*ad)))
}

// PrinterSkew provides skew correction functionality.
type PrinterSkew struct {
	rt                 *runtime
	currentProfileName string
	xyFactor           float64
	xzFactor           float64
	yzFactor           float64
	skewProfiles       map[string]SkewProfile
	mu                 sync.RWMutex
}

// SkewProfile holds a set of skew correction factors.
type SkewProfile struct {
	XYSkew float64 `json:"xy_skew"`
	XZSkew float64 `json:"xz_skew"`
	YZSkew float64 `json:"yz_skew"`
}

// SkewCorrectionConfig holds configuration for skew correction.
type SkewCorrectionConfig struct {
	Profiles map[string]SkewProfile
}

// newPrinterSkew creates a new skew correction instance.
func newPrinterSkew(rt *runtime, cfg SkewCorrectionConfig) *PrinterSkew {
	ps := &PrinterSkew{
		rt:           rt,
		skewProfiles: make(map[string]SkewProfile),
	}

	// Copy profiles
	for name, profile := range cfg.Profiles {
		ps.skewProfiles[name] = profile
	}

	log.Printf("skew_correction: initialized with %d profiles", len(ps.skewProfiles))
	return ps
}

// CalcSkew applies skew correction to a position.
func (ps *PrinterSkew) CalcSkew(pos []float64) []float64 {
	ps.mu.RLock()
	defer ps.mu.RUnlock()

	if len(pos) < 3 {
		return pos
	}

	skewedX := pos[0] - pos[1]*ps.xyFactor - pos[2]*(ps.xzFactor-(ps.xyFactor*ps.yzFactor))
	skewedY := pos[1] - pos[2]*ps.yzFactor

	result := make([]float64, len(pos))
	result[0] = skewedX
	result[1] = skewedY
	copy(result[2:], pos[2:])

	return result
}

// CalcUnskew reverses skew correction to get original position.
func (ps *PrinterSkew) CalcUnskew(pos []float64) []float64 {
	ps.mu.RLock()
	defer ps.mu.RUnlock()

	if len(pos) < 3 {
		return pos
	}

	skewedX := pos[0] + pos[1]*ps.xyFactor + pos[2]*ps.xzFactor
	skewedY := pos[1] + pos[2]*ps.yzFactor

	result := make([]float64, len(pos))
	result[0] = skewedX
	result[1] = skewedY
	copy(result[2:], pos[2:])

	return result
}

// SetSkew sets the skew correction factors directly.
func (ps *PrinterSkew) SetSkew(xyFactor, xzFactor, yzFactor float64) {
	ps.mu.Lock()
	defer ps.mu.Unlock()

	ps.xyFactor = xyFactor
	ps.xzFactor = xzFactor
	ps.yzFactor = yzFactor

	log.Printf("skew_correction: set factors XY=%.6f XZ=%.6f YZ=%.6f",
		xyFactor, xzFactor, yzFactor)
}

// ClearSkew resets all skew correction factors to zero.
func (ps *PrinterSkew) ClearSkew() {
	ps.SetSkew(0, 0, 0)
}

// LoadProfile loads a skew profile by name.
func (ps *PrinterSkew) LoadProfile(name string) error {
	ps.mu.Lock()
	defer ps.mu.Unlock()

	profile, ok := ps.skewProfiles[name]
	if !ok {
		return fmt.Errorf("skew_correction: unknown profile '%s'", name)
	}

	ps.xyFactor = profile.XYSkew
	ps.xzFactor = profile.XZSkew
	ps.yzFactor = profile.YZSkew
	ps.currentProfileName = name

	log.Printf("skew_correction: loaded profile '%s'", name)
	return nil
}

// SaveProfile saves current skew factors to a profile.
func (ps *PrinterSkew) SaveProfile(name string) {
	ps.mu.Lock()
	defer ps.mu.Unlock()

	ps.skewProfiles[name] = SkewProfile{
		XYSkew: ps.xyFactor,
		XZSkew: ps.xzFactor,
		YZSkew: ps.yzFactor,
	}

	log.Printf("skew_correction: saved profile '%s'", name)
}

// RemoveProfile removes a skew profile.
func (ps *PrinterSkew) RemoveProfile(name string) bool {
	ps.mu.Lock()
	defer ps.mu.Unlock()

	if _, ok := ps.skewProfiles[name]; !ok {
		return false
	}

	delete(ps.skewProfiles, name)
	log.Printf("skew_correction: removed profile '%s'", name)
	return true
}

// GetCurrentSkew returns the current skew factors.
func (ps *PrinterSkew) GetCurrentSkew() (xy, xz, yz float64) {
	ps.mu.RLock()
	defer ps.mu.RUnlock()
	return ps.xyFactor, ps.xzFactor, ps.yzFactor
}

// GetProfileNames returns the names of all stored profiles.
func (ps *PrinterSkew) GetProfileNames() []string {
	ps.mu.RLock()
	defer ps.mu.RUnlock()

	names := make([]string, 0, len(ps.skewProfiles))
	for name := range ps.skewProfiles {
		names = append(names, name)
	}
	return names
}

// GetStatus returns the skew correction status.
func (ps *PrinterSkew) GetStatus() map[string]any {
	ps.mu.RLock()
	defer ps.mu.RUnlock()

	return map[string]any{
		"current_profile_name": ps.currentProfileName,
		"xy_skew":              ps.xyFactor,
		"xz_skew":              ps.xzFactor,
		"yz_skew":              ps.yzFactor,
	}
}

// SetSkewFromMeasurements calculates and sets skew from measured lengths.
func (ps *PrinterSkew) SetSkewFromMeasurements(plane string, ac, bd, ad float64) error {
	factor := CalcSkewFactor(ac, bd, ad)

	ps.mu.Lock()
	defer ps.mu.Unlock()

	switch plane {
	case "XY", "xy":
		ps.xyFactor = factor
	case "XZ", "xz":
		ps.xzFactor = factor
	case "YZ", "yz":
		ps.yzFactor = factor
	default:
		return fmt.Errorf("skew_correction: unknown plane '%s'", plane)
	}

	log.Printf("skew_correction: set %s factor from measurements: %.6f radians (%.2f degrees)",
		plane, factor, factor*180/math.Pi)
	return nil
}
