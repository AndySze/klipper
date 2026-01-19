// Save Variables - port of klippy/extras/save_variables.py
//
// Save arbitrary variables so that values can be kept across restarts.
//
// Copyright (C) 2020 Dushyant Ahuja <dusht.ahuja@gmail.com>
// Copyright (C) 2016-2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"bufio"
	"fmt"
	"log"
	"os"
	"sort"
	"strconv"
	"strings"
	"sync"
)

// SaveVariables provides persistent variable storage.
type SaveVariables struct {
	rt          *runtime
	filename    string
	allVariables map[string]any
	mu          sync.RWMutex
}

// SaveVariablesConfig holds configuration for save variables.
type SaveVariablesConfig struct {
	Filename string
}

// newSaveVariables creates a new save variables handler.
func newSaveVariables(rt *runtime, cfg SaveVariablesConfig) (*SaveVariables, error) {
	if cfg.Filename == "" {
		return nil, fmt.Errorf("save_variables: filename is required")
	}

	// Expand ~ in path
	filename := cfg.Filename
	if strings.HasPrefix(filename, "~/") {
		home, err := os.UserHomeDir()
		if err == nil {
			filename = home + filename[1:]
		}
	}

	sv := &SaveVariables{
		rt:           rt,
		filename:     filename,
		allVariables: make(map[string]any),
	}

	// Create file if it doesn't exist
	if _, err := os.Stat(filename); os.IsNotExist(err) {
		f, err := os.Create(filename)
		if err != nil {
			return nil, fmt.Errorf("save_variables: unable to create file '%s': %v",
				filename, err)
		}
		f.Close()
	}

	// Load existing variables
	if err := sv.loadVariables(); err != nil {
		return nil, err
	}

	log.Printf("save_variables: initialized with file '%s', %d variables loaded",
		filename, len(sv.allVariables))
	return sv, nil
}

// loadVariables loads variables from the file.
func (sv *SaveVariables) loadVariables() error {
	sv.mu.Lock()
	defer sv.mu.Unlock()

	f, err := os.Open(sv.filename)
	if err != nil {
		if os.IsNotExist(err) {
			return nil
		}
		return fmt.Errorf("save_variables: unable to open file: %v", err)
	}
	defer f.Close()

	allVars := make(map[string]any)
	inVariables := false

	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())

		// Check for section header
		if line == "[Variables]" {
			inVariables = true
			continue
		}
		if strings.HasPrefix(line, "[") {
			inVariables = false
			continue
		}

		// Parse variable if in [Variables] section
		if inVariables && strings.Contains(line, " = ") {
			parts := strings.SplitN(line, " = ", 2)
			if len(parts) == 2 {
				name := strings.TrimSpace(parts[0])
				valueStr := strings.TrimSpace(parts[1])
				value := parseVariableValue(valueStr)
				allVars[name] = value
			}
		}
	}

	if err := scanner.Err(); err != nil {
		return fmt.Errorf("save_variables: error reading file: %v", err)
	}

	sv.allVariables = allVars
	return nil
}

// parseVariableValue parses a variable value from string representation.
func parseVariableValue(s string) any {
	// Try integer
	if i, err := strconv.ParseInt(s, 10, 64); err == nil {
		return i
	}

	// Try float
	if f, err := strconv.ParseFloat(s, 64); err == nil {
		return f
	}

	// Try boolean
	lower := strings.ToLower(s)
	if lower == "true" {
		return true
	}
	if lower == "false" {
		return false
	}

	// Try quoted string
	if len(s) >= 2 && s[0] == '\'' && s[len(s)-1] == '\'' {
		return s[1 : len(s)-1]
	}
	if len(s) >= 2 && s[0] == '"' && s[len(s)-1] == '"' {
		return s[1 : len(s)-1]
	}

	// Return as-is
	return s
}

// SaveVariable saves a variable to the file.
func (sv *SaveVariables) SaveVariable(name string, value any) error {
	// Validate name (must be lowercase)
	if strings.ToLower(name) != name {
		return fmt.Errorf("VARIABLE must not contain upper case")
	}

	sv.mu.Lock()
	defer sv.mu.Unlock()

	// Update variable
	newVars := make(map[string]any)
	for k, v := range sv.allVariables {
		newVars[k] = v
	}
	newVars[name] = value

	// Write file
	if err := sv.writeVariables(newVars); err != nil {
		return err
	}

	sv.allVariables = newVars
	log.Printf("save_variables: saved variable '%s'", name)
	return nil
}

// writeVariables writes all variables to the file.
func (sv *SaveVariables) writeVariables(vars map[string]any) error {
	f, err := os.Create(sv.filename)
	if err != nil {
		return fmt.Errorf("save_variables: unable to save variable: %v", err)
	}
	defer f.Close()

	// Write header
	fmt.Fprintln(f, "[Variables]")

	// Sort keys for consistent output
	keys := make([]string, 0, len(vars))
	for k := range vars {
		keys = append(keys, k)
	}
	sort.Strings(keys)

	// Write variables
	for _, name := range keys {
		value := vars[name]
		fmt.Fprintf(f, "%s = %s\n", name, formatVariableValue(value))
	}

	return nil
}

// formatVariableValue formats a value for file storage.
func formatVariableValue(v any) string {
	switch val := v.(type) {
	case string:
		return fmt.Sprintf("'%s'", val)
	case bool:
		if val {
			return "True"
		}
		return "False"
	case int, int32, int64:
		return fmt.Sprintf("%d", val)
	case float32, float64:
		return fmt.Sprintf("%v", val)
	default:
		return fmt.Sprintf("'%v'", val)
	}
}

// GetVariable returns a variable value.
func (sv *SaveVariables) GetVariable(name string) (any, bool) {
	sv.mu.RLock()
	defer sv.mu.RUnlock()
	val, ok := sv.allVariables[name]
	return val, ok
}

// GetAllVariables returns all variables.
func (sv *SaveVariables) GetAllVariables() map[string]any {
	sv.mu.RLock()
	defer sv.mu.RUnlock()

	result := make(map[string]any)
	for k, v := range sv.allVariables {
		result[k] = v
	}
	return result
}

// GetStatus returns the save variables status.
func (sv *SaveVariables) GetStatus() map[string]any {
	return map[string]any{
		"variables": sv.GetAllVariables(),
	}
}

// DeleteVariable deletes a variable.
func (sv *SaveVariables) DeleteVariable(name string) error {
	sv.mu.Lock()
	defer sv.mu.Unlock()

	if _, ok := sv.allVariables[name]; !ok {
		return fmt.Errorf("variable '%s' not found", name)
	}

	newVars := make(map[string]any)
	for k, v := range sv.allVariables {
		if k != name {
			newVars[k] = v
		}
	}

	if err := sv.writeVariables(newVars); err != nil {
		return err
	}

	sv.allVariables = newVars
	log.Printf("save_variables: deleted variable '%s'", name)
	return nil
}
