// GCode Macro - port of klippy/extras/gcode_macro.py
//
// Add ability to define custom g-code macros
//
// Copyright (C) 2018-2021 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"regexp"
	"strings"
	"sync"
)

// TemplateWrapper wraps a template for rendering.
type TemplateWrapper struct {
	name     string
	script   string
	template string // Pre-processed template
}

// newTemplateWrapper creates a new template wrapper.
func newTemplateWrapper(name, script string) *TemplateWrapper {
	return &TemplateWrapper{
		name:     name,
		script:   script,
		template: script,
	}
}

// Render renders the template with given context.
func (tw *TemplateWrapper) Render(context map[string]any) (string, error) {
	result := tw.template

	// Simple variable substitution for {variable} syntax
	// Note: Full Jinja2 implementation would require a template engine
	re := regexp.MustCompile(`\{(\w+(?:\.\w+)*)\}`)
	result = re.ReplaceAllStringFunc(result, func(match string) string {
		varName := match[1 : len(match)-1]
		if val, ok := resolveVariable(context, varName); ok {
			return fmt.Sprintf("%v", val)
		}
		return match
	})

	// Handle {params.XXX} substitutions
	paramsRe := regexp.MustCompile(`\{params\.(\w+)\}`)
	result = paramsRe.ReplaceAllStringFunc(result, func(match string) string {
		paramName := match[8 : len(match)-1] // Extract after "params."
		if params, ok := context["params"].(map[string]string); ok {
			if val, ok := params[paramName]; ok {
				return val
			}
		}
		return ""
	})

	return result, nil
}

// resolveVariable resolves a dotted variable path from context.
func resolveVariable(context map[string]any, path string) (any, bool) {
	parts := strings.Split(path, ".")
	var current any = context

	for _, part := range parts {
		switch v := current.(type) {
		case map[string]any:
			val, ok := v[part]
			if !ok {
				return nil, false
			}
			current = val
		case map[string]string:
			val, ok := v[part]
			if !ok {
				return nil, false
			}
			current = val
		default:
			return nil, false
		}
	}
	return current, true
}

// PrinterGCodeMacro provides the main gcode macro template tracking.
type PrinterGCodeMacro struct {
	rt     *runtime
	macros map[string]*GCodeMacro
	mu     sync.RWMutex
}

// newPrinterGCodeMacro creates a new gcode macro handler.
func newPrinterGCodeMacro(rt *runtime) *PrinterGCodeMacro {
	return &PrinterGCodeMacro{
		rt:     rt,
		macros: make(map[string]*GCodeMacro),
	}
}

// LoadTemplate loads a template from config.
func (pgm *PrinterGCodeMacro) LoadTemplate(name, script string) *TemplateWrapper {
	return newTemplateWrapper(name, script)
}

// RegisterMacro registers a new macro.
func (pgm *PrinterGCodeMacro) RegisterMacro(name string, macro *GCodeMacro) {
	pgm.mu.Lock()
	defer pgm.mu.Unlock()
	pgm.macros[strings.ToUpper(name)] = macro
}

// GetMacro returns a macro by name.
func (pgm *PrinterGCodeMacro) GetMacro(name string) *GCodeMacro {
	pgm.mu.RLock()
	defer pgm.mu.RUnlock()
	return pgm.macros[strings.ToUpper(name)]
}

// CreateTemplateContext creates a template context for rendering.
func (pgm *PrinterGCodeMacro) CreateTemplateContext(eventtime float64) map[string]any {
	return map[string]any{
		"printer": pgm.getStatusWrapper(eventtime),
	}
}

// getStatusWrapper returns a status wrapper for template access.
func (pgm *PrinterGCodeMacro) getStatusWrapper(eventtime float64) map[string]any {
	// This would provide access to printer object statuses
	// Simplified implementation
	return map[string]any{}
}

// GCodeMacro represents a single gcode macro.
type GCodeMacro struct {
	pgm            *PrinterGCodeMacro
	name           string
	alias          string
	template       *TemplateWrapper
	description    string
	renameExisting string
	variables      map[string]any
	inScript       bool
	mu             sync.Mutex
}

// GCodeMacroConfig holds configuration for a gcode macro.
type GCodeMacroConfig struct {
	Name           string
	GCode          string
	Description    string
	RenameExisting string
	Variables      map[string]any
}

// newGCodeMacro creates a new gcode macro.
func newGCodeMacro(pgm *PrinterGCodeMacro, cfg GCodeMacroConfig) (*GCodeMacro, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("macro name is required")
	}
	if cfg.GCode == "" {
		return nil, fmt.Errorf("gcode is required for macro '%s'", cfg.Name)
	}

	alias := strings.ToUpper(cfg.Name)

	gm := &GCodeMacro{
		pgm:            pgm,
		name:           cfg.Name,
		alias:          alias,
		template:       pgm.LoadTemplate(cfg.Name+":gcode", cfg.GCode),
		description:    cfg.Description,
		renameExisting: cfg.RenameExisting,
		variables:      make(map[string]any),
	}

	// Copy variables
	for k, v := range cfg.Variables {
		gm.variables[k] = v
	}

	if gm.description == "" {
		gm.description = "G-Code macro"
	}

	pgm.RegisterMacro(cfg.Name, gm)
	log.Printf("gcode_macro: registered macro '%s'", cfg.Name)

	return gm, nil
}

// Execute runs the macro with given parameters.
func (gm *GCodeMacro) Execute(params map[string]string) (string, error) {
	gm.mu.Lock()
	defer gm.mu.Unlock()

	if gm.inScript {
		return "", fmt.Errorf("macro %s called recursively", gm.alias)
	}

	// Build context
	context := gm.pgm.CreateTemplateContext(0)
	for k, v := range gm.variables {
		context[k] = v
	}
	context["params"] = params

	gm.inScript = true
	defer func() { gm.inScript = false }()

	return gm.template.Render(context)
}

// SetVariable sets a macro variable.
func (gm *GCodeMacro) SetVariable(name string, value any) error {
	gm.mu.Lock()
	defer gm.mu.Unlock()

	if _, ok := gm.variables[name]; !ok {
		return fmt.Errorf("unknown gcode_macro variable '%s'", name)
	}

	gm.variables[name] = value
	return nil
}

// GetStatus returns the macro status (variables).
func (gm *GCodeMacro) GetStatus() map[string]any {
	gm.mu.Lock()
	defer gm.mu.Unlock()

	result := make(map[string]any)
	for k, v := range gm.variables {
		result[k] = v
	}
	return result
}

// GetName returns the macro name.
func (gm *GCodeMacro) GetName() string {
	return gm.name
}

// GetAlias returns the macro alias (uppercase name).
func (gm *GCodeMacro) GetAlias() string {
	return gm.alias
}

// GetDescription returns the macro description.
func (gm *GCodeMacro) GetDescription() string {
	return gm.description
}
