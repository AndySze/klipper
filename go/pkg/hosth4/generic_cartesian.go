package hosth4

import (
	"fmt"
	"regexp"
	"strconv"
	"strings"

	"klipper-go-migration/pkg/chelper"
)

// Carriage represents a primary carriage (X, Y, or Z axis).
type Carriage struct {
	Name         string
	Axis         int // 0=X, 1=Y, 2=Z
	AxisName     string
	EndstopPin   pin
	PosEndstop   float64
	PosMin       float64
	PosMax       float64
	HomingSpeed  float64
	DualCarriage *DualCarriage // nil if no dual carriage on this axis
}

// DualCarriage represents an alternate carriage on the same axis.
type DualCarriage struct {
	Name           string
	PrimaryName    string
	Primary        *Carriage
	EndstopPin     pin
	PosEndstop     float64
	PosMin         float64
	PosMax         float64
	HomingSpeed    float64
	SafeDistance   float64
	Active         bool // whether this dual carriage is currently active
}

// ExtraCarriage represents an additional carriage linked to a primary.
type ExtraCarriage struct {
	Name        string
	PrimaryName string
	Primary     *Carriage
	EndstopPin  pin
}

// KinematicStepper represents a stepper with coefficient-based carriage mapping.
type KinematicStepper struct {
	Name       string
	Carriages  string     // Original carriages string
	Coeffs     [3]float64 // Coefficients for X, Y, Z axes
	StepPin    pin
	DirPin     pin
	EnablePin  pin
	Microsteps int
	RotDist    float64

	// Runtime fields
	sk        *chelper.StepperKinematics
	se        *chelper.SyncEmitter
	oid       int
	enableOID int
}

// GenericCartesianKinematics manages the generic cartesian kinematics state.
type GenericCartesianKinematics struct {
	Carriages      map[string]*Carriage
	DualCarriages  []*DualCarriage
	ExtraCarriages []*ExtraCarriage
	AllCarriages   map[string]interface{} // All carriages by name
	Steppers       []*KinematicStepper

	// Dual carriage state save/restore
	savedDCState []bool // Active state for each dual carriage
}

// parseCarriagesString parses a carriages expression like "carriage_x+carriage_y"
// or "1*carriage_y+0.5*carriage_z" and returns coefficients [3]float64.
func parseCarriagesString(carriagesStr string, allCarriages map[string]interface{}) ([3]float64, []string, error) {
	var coeffs [3]float64
	var refNames []string

	// Pattern to split on + or - (keeping the sign)
	pat := regexp.MustCompile(`[+-]`)
	nxt := 0

	for nxt < len(carriagesStr) {
		// Find next + or - after position nxt+1
		match := pat.FindStringIndex(carriagesStr[nxt+1:])
		var end int
		if match == nil {
			end = len(carriagesStr)
		} else {
			end = nxt + 1 + match[0]
		}

		term := strings.TrimSpace(carriagesStr[nxt:end])
		if term == "" {
			nxt = end
			continue
		}

		// Parse term: could be "carriage_x", "-carriage_y", "1.5*carriage_z"
		parts := strings.Split(term, "*")
		var coeff float64
		var carriageName string

		if len(parts) == 2 {
			// Coefficient * carriage
			c, err := strconv.ParseFloat(strings.TrimSpace(parts[0]), 64)
			if err != nil {
				return coeffs, nil, fmt.Errorf("invalid coefficient in '%s': %w", term, err)
			}
			coeff = c
			carriageName = strings.TrimSpace(parts[1])
		} else if len(parts) == 1 {
			// Just carriage name, possibly with sign
			carriageName = strings.TrimSpace(parts[0])
			if strings.HasPrefix(carriageName, "-") {
				coeff = -1.0
				carriageName = strings.TrimPrefix(carriageName, "-")
			} else {
				coeff = 1.0
				carriageName = strings.TrimPrefix(carriageName, "+")
			}
		} else {
			return coeffs, nil, fmt.Errorf("invalid term '%s' in carriages string", term)
		}

		// Look up carriage
		c, ok := allCarriages[carriageName]
		if !ok {
			return coeffs, nil, fmt.Errorf("unknown carriage '%s' in '%s'", carriageName, carriagesStr)
		}

		// Get axis from carriage
		var axis int
		switch v := c.(type) {
		case *Carriage:
			axis = v.Axis
		case *DualCarriage:
			axis = v.Primary.Axis
		case *ExtraCarriage:
			axis = v.Primary.Axis
		default:
			return coeffs, nil, fmt.Errorf("unknown carriage type for '%s'", carriageName)
		}

		if coeffs[axis] != 0 {
			return coeffs, nil, fmt.Errorf("axis %s referenced multiple times in '%s'", "xyz"[axis:axis+1], carriagesStr)
		}
		coeffs[axis] = coeff
		refNames = append(refNames, carriageName)

		nxt = end
	}

	return coeffs, refNames, nil
}

// loadGenericCartesianKinematics loads the generic cartesian kinematics from config.
func loadGenericCartesianKinematics(cfg *configWrapper) (*GenericCartesianKinematics, error) {
	kin := &GenericCartesianKinematics{
		Carriages:    make(map[string]*Carriage),
		AllCarriages: make(map[string]interface{}),
	}

	// Load main carriages [carriage NAME]
	for _, section := range cfg.sectionsByPrefix("carriage ") {
		name := strings.TrimPrefix(section, "carriage ")
		sec, _ := cfg.section(section)

		axisStr := strings.TrimSpace(sec["axis"])
		if axisStr == "" {
			// Default to carriage name if it matches x/y/z
			if name == "x" || name == "y" || name == "z" {
				axisStr = name
			} else if strings.HasSuffix(name, "_x") {
				axisStr = "x"
			} else if strings.HasSuffix(name, "_y") {
				axisStr = "y"
			} else if strings.HasSuffix(name, "_z") {
				axisStr = "z"
			} else {
				return nil, fmt.Errorf("[%s] missing required 'axis' option", section)
			}
		}

		axis := -1
		switch axisStr {
		case "x":
			axis = 0
		case "y":
			axis = 1
		case "z":
			axis = 2
		default:
			return nil, fmt.Errorf("[%s] invalid axis '%s'", section, axisStr)
		}

		c := &Carriage{
			Name:     name,
			Axis:     axis,
			AxisName: axisStr,
		}

		if ep := sec["endstop_pin"]; ep != "" {
			p, err := parsePinDesc(ep, false, true)
			if err != nil {
				return nil, fmt.Errorf("[%s] invalid endstop_pin: %w", section, err)
			}
			c.EndstopPin = p
		}
		if v := sec["position_endstop"]; v != "" {
			f, _ := strconv.ParseFloat(v, 64)
			c.PosEndstop = f
		}
		if v := sec["position_min"]; v != "" {
			f, _ := strconv.ParseFloat(v, 64)
			c.PosMin = f
		}
		if v := sec["position_max"]; v != "" {
			f, _ := strconv.ParseFloat(v, 64)
			c.PosMax = f
		}
		if v := sec["homing_speed"]; v != "" {
			f, _ := strconv.ParseFloat(v, 64)
			c.HomingSpeed = f
		} else {
			c.HomingSpeed = 5.0
		}

		kin.Carriages[name] = c
		kin.AllCarriages[name] = c
	}

	// Load dual carriages [dual_carriage NAME]
	for _, section := range cfg.sectionsByPrefix("dual_carriage ") {
		name := strings.TrimPrefix(section, "dual_carriage ")
		sec, _ := cfg.section(section)

		primaryName := strings.TrimSpace(sec["primary_carriage"])
		primary, ok := kin.Carriages[primaryName]
		if !ok {
			return nil, fmt.Errorf("[%s] unknown primary_carriage '%s'", section, primaryName)
		}

		dc := &DualCarriage{
			Name:        name,
			PrimaryName: primaryName,
			Primary:     primary,
		}

		if ep := sec["endstop_pin"]; ep != "" {
			p, err := parsePinDesc(ep, false, true)
			if err != nil {
				return nil, fmt.Errorf("[%s] invalid endstop_pin: %w", section, err)
			}
			dc.EndstopPin = p
		}
		if v := sec["position_endstop"]; v != "" {
			f, _ := strconv.ParseFloat(v, 64)
			dc.PosEndstop = f
		}
		if v := sec["position_min"]; v != "" {
			f, _ := strconv.ParseFloat(v, 64)
			dc.PosMin = f
		}
		if v := sec["position_max"]; v != "" {
			f, _ := strconv.ParseFloat(v, 64)
			dc.PosMax = f
		}
		if v := sec["homing_speed"]; v != "" {
			f, _ := strconv.ParseFloat(v, 64)
			dc.HomingSpeed = f
		} else {
			dc.HomingSpeed = 5.0
		}
		if v := sec["safe_distance"]; v != "" {
			f, _ := strconv.ParseFloat(v, 64)
			dc.SafeDistance = f
		}

		primary.DualCarriage = dc
		kin.DualCarriages = append(kin.DualCarriages, dc)
		kin.AllCarriages[name] = dc
	}

	// Load extra carriages [extra_carriage NAME]
	for _, section := range cfg.sectionsByPrefix("extra_carriage ") {
		name := strings.TrimPrefix(section, "extra_carriage ")
		sec, _ := cfg.section(section)

		primaryName := strings.TrimSpace(sec["primary_carriage"])
		primary, ok := kin.Carriages[primaryName]
		if !ok {
			return nil, fmt.Errorf("[%s] unknown primary_carriage '%s'", section, primaryName)
		}

		ec := &ExtraCarriage{
			Name:        name,
			PrimaryName: primaryName,
			Primary:     primary,
		}

		if ep := sec["endstop_pin"]; ep != "" {
			p, err := parsePinDesc(ep, false, true)
			if err != nil {
				return nil, fmt.Errorf("[%s] invalid endstop_pin: %w", section, err)
			}
			ec.EndstopPin = p
		}

		kin.ExtraCarriages = append(kin.ExtraCarriages, ec)
		kin.AllCarriages[name] = ec
	}

	// Load steppers [stepper NAME]
	for _, section := range cfg.sectionsByPrefix("stepper ") {
		name := strings.TrimPrefix(section, "stepper ")
		sec, _ := cfg.section(section)

		carriagesStr := strings.TrimSpace(sec["carriages"])
		if carriagesStr == "" {
			return nil, fmt.Errorf("[%s] missing required 'carriages' option", section)
		}

		coeffs, _, err := parseCarriagesString(strings.ToLower(carriagesStr), kin.AllCarriages)
		if err != nil {
			return nil, fmt.Errorf("[%s] %w", section, err)
		}

		s := &KinematicStepper{
			Name:      name,
			Carriages: carriagesStr,
			Coeffs:    coeffs,
		}

		if v := sec["step_pin"]; v != "" {
			p, err := parsePinDesc(v, false, false)
			if err != nil {
				return nil, fmt.Errorf("[%s] invalid step_pin: %w", section, err)
			}
			s.StepPin = p
		}
		if v := sec["dir_pin"]; v != "" {
			p, err := parsePinDesc(v, true, false)
			if err != nil {
				return nil, fmt.Errorf("[%s] invalid dir_pin: %w", section, err)
			}
			s.DirPin = p
		}
		if v := sec["enable_pin"]; v != "" {
			p, err := parsePinDesc(v, true, false)
			if err != nil {
				return nil, fmt.Errorf("[%s] invalid enable_pin: %w", section, err)
			}
			s.EnablePin = p
		}
		if v := sec["microsteps"]; v != "" {
			i, _ := strconv.Atoi(v)
			s.Microsteps = i
		}
		if v := sec["rotation_distance"]; v != "" {
			f, _ := strconv.ParseFloat(v, 64)
			s.RotDist = f
		}

		kin.Steppers = append(kin.Steppers, s)
	}

	return kin, nil
}

// getActiveRange returns the position range for an axis considering dual carriage state.
func (kin *GenericCartesianKinematics) getActiveRange(axis int) (float64, float64) {
	for _, c := range kin.Carriages {
		if c.Axis == axis {
			if c.DualCarriage != nil && c.DualCarriage.Active {
				dc := c.DualCarriage
				return dc.PosMin, dc.PosMax
			}
			return c.PosMin, c.PosMax
		}
	}
	return 0, 0
}

// getCarriageForAxis returns the active carriage for the given axis.
func (kin *GenericCartesianKinematics) getCarriageForAxis(axis int) *Carriage {
	for _, c := range kin.Carriages {
		if c.Axis == axis {
			return c
		}
	}
	return nil
}

// setDualCarriageActive sets the active state for a dual carriage by name.
func (kin *GenericCartesianKinematics) setDualCarriageActive(name string) error {
	// Find the carriage
	c, ok := kin.AllCarriages[name]
	if !ok {
		return fmt.Errorf("unknown carriage '%s'", name)
	}

	var axis int
	switch v := c.(type) {
	case *Carriage:
		axis = v.Axis
		// Deactivate all dual carriages on this axis
		for _, dc := range kin.DualCarriages {
			if dc.Primary.Axis == axis {
				dc.Active = false
			}
		}
	case *DualCarriage:
		axis = v.Primary.Axis
		// Deactivate all dual carriages on this axis, then activate this one
		for _, dc := range kin.DualCarriages {
			if dc.Primary.Axis == axis {
				dc.Active = (dc == v)
			}
		}
	default:
		return fmt.Errorf("cannot activate carriage '%s'", name)
	}

	return nil
}

// saveDualCarriageState saves the current dual carriage activation state.
func (kin *GenericCartesianKinematics) saveDualCarriageState() {
	kin.savedDCState = make([]bool, len(kin.DualCarriages))
	for i, dc := range kin.DualCarriages {
		kin.savedDCState[i] = dc.Active
	}
}

// restoreDualCarriageState restores the saved dual carriage activation state.
func (kin *GenericCartesianKinematics) restoreDualCarriageState() {
	if kin.savedDCState == nil {
		return
	}
	for i, dc := range kin.DualCarriages {
		if i < len(kin.savedDCState) {
			dc.Active = kin.savedDCState[i]
		}
	}
}
