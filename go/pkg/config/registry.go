package config

import (
	"fmt"
	"strings"
	"sync"
)

// Module represents a loaded configuration module.
type Module interface {
	// GetName returns the module name (usually the section name).
	GetName() string
}

// ModuleFactory creates a module instance from a config section.
// The factory receives the section and should return a configured module.
type ModuleFactory func(section *Section) (Module, error)

// Registry manages module factories and handles auto-loading of modules
// based on config sections. It matches Python's extras loading pattern.
type Registry struct {
	mu sync.RWMutex

	// exact matches section name exactly (e.g., "printer", "extruder")
	exact map[string]ModuleFactory

	// prefixes matches section names by prefix (e.g., "stepper" for "stepper_x")
	prefixes map[string]ModuleFactory

	// loaded tracks which modules have been loaded
	loaded map[string]Module
}

// NewRegistry creates a new module registry.
func NewRegistry() *Registry {
	return &Registry{
		exact:    make(map[string]ModuleFactory),
		prefixes: make(map[string]ModuleFactory),
		loaded:   make(map[string]Module),
	}
}

// Register adds a factory for an exact section name match.
// Example: Register("printer", printerFactory) matches [printer]
func (r *Registry) Register(name string, factory ModuleFactory) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.exact[name] = factory
}

// RegisterPrefix adds a factory for sections matching a prefix.
// Example: RegisterPrefix("stepper", stepperFactory) matches [stepper_x], [stepper_y]
// The prefix is matched against the part before the first underscore.
func (r *Registry) RegisterPrefix(prefix string, factory ModuleFactory) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.prefixes[prefix] = factory
}

// RegisterWithPrefix adds a factory for sections starting with a prefix.
// Example: RegisterWithPrefix("heater_generic ", heaterFactory) matches [heater_generic foo]
// This matches the full prefix including space for Python-style named sections.
func (r *Registry) RegisterWithPrefix(prefix string, factory ModuleFactory) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.prefixes[prefix] = factory
}

// GetFactory returns the factory for a section name, or nil if not found.
func (r *Registry) GetFactory(sectionName string) ModuleFactory {
	r.mu.RLock()
	defer r.mu.RUnlock()

	// Try exact match first
	if factory, ok := r.exact[sectionName]; ok {
		return factory
	}

	// Try prefix match (stepper_x -> stepper)
	if idx := strings.Index(sectionName, "_"); idx > 0 {
		prefix := sectionName[:idx]
		if factory, ok := r.prefixes[prefix]; ok {
			return factory
		}
	}

	// Try full prefix match (heater_generic foo -> heater_generic )
	for prefix, factory := range r.prefixes {
		if strings.HasPrefix(sectionName, prefix) {
			return factory
		}
	}

	return nil
}

// LoadModules loads all modules from the config using registered factories.
// Returns a map of section name -> loaded module.
func (r *Registry) LoadModules(cfg *Config) (map[string]Module, error) {
	r.mu.Lock()
	defer r.mu.Unlock()

	modules := make(map[string]Module)

	for _, section := range cfg.GetSections() {
		name := section.GetName()

		// Skip if already loaded
		if _, ok := r.loaded[name]; ok {
			modules[name] = r.loaded[name]
			continue
		}

		// Find factory
		factory := r.getFactoryLocked(name)
		if factory == nil {
			// No factory registered - section will be reported as unused
			continue
		}

		// Create module
		module, err := factory(section)
		if err != nil {
			return nil, fmt.Errorf("failed to load module [%s]: %w", name, err)
		}

		modules[name] = module
		r.loaded[name] = module
	}

	return modules, nil
}

// getFactoryLocked returns the factory for a section name (must hold lock).
func (r *Registry) getFactoryLocked(sectionName string) ModuleFactory {
	// Try exact match first
	if factory, ok := r.exact[sectionName]; ok {
		return factory
	}

	// Try prefix match (stepper_x -> stepper)
	if idx := strings.Index(sectionName, "_"); idx > 0 {
		prefix := sectionName[:idx]
		if factory, ok := r.prefixes[prefix]; ok {
			return factory
		}
	}

	// Try full prefix match
	for prefix, factory := range r.prefixes {
		if strings.HasPrefix(sectionName, prefix) {
			return factory
		}
	}

	return nil
}

// GetModule returns a loaded module by name, or nil if not found.
func (r *Registry) GetModule(name string) Module {
	r.mu.RLock()
	defer r.mu.RUnlock()
	return r.loaded[name]
}

// GetLoadedModules returns all loaded modules.
func (r *Registry) GetLoadedModules() map[string]Module {
	r.mu.RLock()
	defer r.mu.RUnlock()

	result := make(map[string]Module, len(r.loaded))
	for k, v := range r.loaded {
		result[k] = v
	}
	return result
}

// Clear removes all loaded modules (useful for testing or reload).
func (r *Registry) Clear() {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.loaded = make(map[string]Module)
}

// HasFactory checks if a factory is registered for the section name.
func (r *Registry) HasFactory(sectionName string) bool {
	return r.GetFactory(sectionName) != nil
}

// RegisteredNames returns all registered exact match names.
func (r *Registry) RegisteredNames() []string {
	r.mu.RLock()
	defer r.mu.RUnlock()

	names := make([]string, 0, len(r.exact))
	for name := range r.exact {
		names = append(names, name)
	}
	return names
}

// RegisteredPrefixes returns all registered prefix patterns.
func (r *Registry) RegisteredPrefixes() []string {
	r.mu.RLock()
	defer r.mu.RUnlock()

	prefixes := make([]string, 0, len(r.prefixes))
	for prefix := range r.prefixes {
		prefixes = append(prefixes, prefix)
	}
	return prefixes
}
