package config

import (
	"strconv"
	"strings"
	"sync"
)

// Section provides access to a config section with access tracking.
// It implements the same pattern as Python's ConfigWrapper.
type Section struct {
	name    string
	options map[string]string

	// Access tracking
	mu       sync.RWMutex
	accessed map[string]struct{}
}

// newSection creates a new Section.
func newSection(name string, options map[string]string) *Section {
	opts := make(map[string]string, len(options))
	for k, v := range options {
		opts[strings.ToLower(k)] = v
	}
	return &Section{
		name:     name,
		options:  opts,
		accessed: make(map[string]struct{}),
	}
}

// GetName returns the section name.
func (s *Section) GetName() string {
	return s.name
}

// markAccessed records that an option was accessed.
func (s *Section) markAccessed(option string) {
	s.mu.Lock()
	s.accessed[strings.ToLower(option)] = struct{}{}
	s.mu.Unlock()
}

// GetAccessedOptions returns a list of options that were accessed.
func (s *Section) GetAccessedOptions() []string {
	s.mu.RLock()
	defer s.mu.RUnlock()
	result := make([]string, 0, len(s.accessed))
	for opt := range s.accessed {
		result = append(result, opt)
	}
	return result
}

// GetUnusedOptions returns a list of options that were not accessed.
func (s *Section) GetUnusedOptions() []string {
	s.mu.RLock()
	defer s.mu.RUnlock()
	var result []string
	for opt := range s.options {
		if _, ok := s.accessed[opt]; !ok {
			result = append(result, opt)
		}
	}
	return result
}

// HasOption checks if an option exists in this section.
func (s *Section) HasOption(option string) bool {
	_, ok := s.options[strings.ToLower(option)]
	return ok
}

// Get returns a string option value.
// If default is provided and option doesn't exist, returns default.
// If no default and option doesn't exist, returns error.
func (s *Section) Get(option string, fallback ...string) (string, error) {
	key := strings.ToLower(option)
	if v, ok := s.options[key]; ok {
		s.markAccessed(option)
		return v, nil
	}
	if len(fallback) > 0 {
		s.markAccessed(option)
		return fallback[0], nil
	}
	return "", ErrMissingOption(s.name, option)
}

// GetInt returns an integer option value.
func (s *Section) GetInt(option string, fallback ...int) (int, error) {
	key := strings.ToLower(option)
	if v, ok := s.options[key]; ok {
		s.markAccessed(option)
		i, err := strconv.Atoi(strings.TrimSpace(v))
		if err != nil {
			return 0, ErrInvalidValue(s.name, option, v, "integer")
		}
		return i, nil
	}
	if len(fallback) > 0 {
		s.markAccessed(option)
		return fallback[0], nil
	}
	return 0, ErrMissingOption(s.name, option)
}

// GetIntWithBounds returns an integer option value with bounds checking.
func (s *Section) GetIntWithBounds(option string, minVal, maxVal *int, fallback ...int) (int, error) {
	v, err := s.GetInt(option, fallback...)
	if err != nil {
		return 0, err
	}
	if minVal != nil && v < *minVal {
		return 0, ErrOutOfRange(s.name, option, float64(v), "must have minimum of "+strconv.Itoa(*minVal))
	}
	if maxVal != nil && v > *maxVal {
		return 0, ErrOutOfRange(s.name, option, float64(v), "must have maximum of "+strconv.Itoa(*maxVal))
	}
	return v, nil
}

// GetFloat returns a float64 option value.
func (s *Section) GetFloat(option string, fallback ...float64) (float64, error) {
	key := strings.ToLower(option)
	if v, ok := s.options[key]; ok {
		s.markAccessed(option)
		f, err := strconv.ParseFloat(strings.TrimSpace(v), 64)
		if err != nil {
			return 0, ErrInvalidValue(s.name, option, v, "float")
		}
		return f, nil
	}
	if len(fallback) > 0 {
		s.markAccessed(option)
		return fallback[0], nil
	}
	return 0, ErrMissingOption(s.name, option)
}

// FloatBounds specifies bounds for GetFloatWithBounds.
type FloatBounds struct {
	MinVal *float64 // minimum value (>=)
	MaxVal *float64 // maximum value (<=)
	Above  *float64 // must be above this value (>)
	Below  *float64 // must be below this value (<)
}

// GetFloatWithBounds returns a float64 option value with bounds checking.
// Matches Python's getfloat(minval, maxval, above, below) pattern.
func (s *Section) GetFloatWithBounds(option string, bounds FloatBounds, fallback ...float64) (float64, error) {
	v, err := s.GetFloat(option, fallback...)
	if err != nil {
		return 0, err
	}
	if bounds.MinVal != nil && v < *bounds.MinVal {
		return 0, ErrOutOfRange(s.name, option, v, "must have minimum of "+strconv.FormatFloat(*bounds.MinVal, 'f', -1, 64))
	}
	if bounds.MaxVal != nil && v > *bounds.MaxVal {
		return 0, ErrOutOfRange(s.name, option, v, "must have maximum of "+strconv.FormatFloat(*bounds.MaxVal, 'f', -1, 64))
	}
	if bounds.Above != nil && v <= *bounds.Above {
		return 0, ErrOutOfRange(s.name, option, v, "must be above "+strconv.FormatFloat(*bounds.Above, 'f', -1, 64))
	}
	if bounds.Below != nil && v >= *bounds.Below {
		return 0, ErrOutOfRange(s.name, option, v, "must be below "+strconv.FormatFloat(*bounds.Below, 'f', -1, 64))
	}
	return v, nil
}

// GetBool returns a boolean option value.
// Accepts: 1, true, yes, on (true) and 0, false, no, off (false).
func (s *Section) GetBool(option string, fallback ...bool) (bool, error) {
	key := strings.ToLower(option)
	if v, ok := s.options[key]; ok {
		s.markAccessed(option)
		switch strings.ToLower(strings.TrimSpace(v)) {
		case "1", "true", "yes", "on":
			return true, nil
		case "0", "false", "no", "off":
			return false, nil
		default:
			return false, ErrInvalidValue(s.name, option, v, "boolean (true/false/yes/no/on/off/1/0)")
		}
	}
	if len(fallback) > 0 {
		s.markAccessed(option)
		return fallback[0], nil
	}
	return false, ErrMissingOption(s.name, option)
}

// GetChoice returns a string option that must be one of the valid choices.
func (s *Section) GetChoice(option string, choices []string, fallback ...string) (string, error) {
	v, err := s.Get(option, fallback...)
	if err != nil {
		return "", err
	}
	for _, c := range choices {
		if strings.EqualFold(v, c) {
			return c, nil
		}
	}
	return "", ErrInvalidChoice(s.name, option, v, choices)
}

// GetList returns a list of strings split by the given separator.
func (s *Section) GetList(option string, sep string, fallback ...[]string) ([]string, error) {
	key := strings.ToLower(option)
	if v, ok := s.options[key]; ok {
		s.markAccessed(option)
		v = strings.TrimSpace(v)
		if v == "" {
			return []string{}, nil
		}
		parts := strings.Split(v, sep)
		result := make([]string, 0, len(parts))
		for _, p := range parts {
			p = strings.TrimSpace(p)
			if p != "" {
				result = append(result, p)
			}
		}
		return result, nil
	}
	if len(fallback) > 0 {
		s.markAccessed(option)
		return fallback[0], nil
	}
	return nil, ErrMissingOption(s.name, option)
}

// GetFloatList returns a list of floats split by the given separator.
func (s *Section) GetFloatList(option string, sep string, fallback ...[]float64) ([]float64, error) {
	key := strings.ToLower(option)
	if v, ok := s.options[key]; ok {
		s.markAccessed(option)
		v = strings.TrimSpace(v)
		if v == "" {
			return []float64{}, nil
		}
		parts := strings.Split(v, sep)
		result := make([]float64, 0, len(parts))
		for _, p := range parts {
			p = strings.TrimSpace(p)
			if p == "" {
				continue
			}
			f, err := strconv.ParseFloat(p, 64)
			if err != nil {
				return nil, ErrInvalidValue(s.name, option, p, "float")
			}
			result = append(result, f)
		}
		return result, nil
	}
	if len(fallback) > 0 {
		s.markAccessed(option)
		return fallback[0], nil
	}
	return nil, ErrMissingOption(s.name, option)
}

// GetIntList returns a list of integers split by the given separator.
func (s *Section) GetIntList(option string, sep string, fallback ...[]int) ([]int, error) {
	key := strings.ToLower(option)
	if v, ok := s.options[key]; ok {
		s.markAccessed(option)
		v = strings.TrimSpace(v)
		if v == "" {
			return []int{}, nil
		}
		parts := strings.Split(v, sep)
		result := make([]int, 0, len(parts))
		for _, p := range parts {
			p = strings.TrimSpace(p)
			if p == "" {
				continue
			}
			i, err := strconv.Atoi(p)
			if err != nil {
				return nil, ErrInvalidValue(s.name, option, p, "integer")
			}
			result = append(result, i)
		}
		return result, nil
	}
	if len(fallback) > 0 {
		s.markAccessed(option)
		return fallback[0], nil
	}
	return nil, ErrMissingOption(s.name, option)
}

// GetPrefixOptions returns all option names that start with the given prefix.
func (s *Section) GetPrefixOptions(prefix string) []string {
	prefix = strings.ToLower(prefix)
	var result []string
	for opt := range s.options {
		if strings.HasPrefix(opt, prefix) {
			result = append(result, opt)
		}
	}
	return result
}

// RawOptions returns a copy of the raw options map.
// This is useful for iterating over all options.
func (s *Section) RawOptions() map[string]string {
	result := make(map[string]string, len(s.options))
	for k, v := range s.options {
		result[k] = v
	}
	return result
}
