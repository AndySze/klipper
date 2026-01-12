package config

import (
	"fmt"
	"sync"
	"time"
)

// Reloadable interface for modules that support hot-reload.
// Modules implementing this interface can update their configuration
// at runtime without requiring a full restart.
type Reloadable interface {
	Module

	// CanReload returns true if the module supports hot reload.
	// Some modules may not support reload due to hardware constraints.
	CanReload() bool

	// Reload is called when the module's config section changes.
	// It should update the module's internal state to match the new config.
	// Returns an error if reload fails (module should remain in previous state).
	Reload(newConfig *Section) error
}

// ReloadResult represents the result of a reload operation for a single module.
type ReloadResult struct {
	Section     string
	Success     bool
	Error       error
	CanReload   bool
	WasReloaded bool
}

// ReloadManager handles hot-reloading of configuration.
type ReloadManager struct {
	mu sync.RWMutex

	// registry holds module factories
	registry *Registry

	// currentConfig is the current loaded configuration
	currentConfig *Config

	// configPath is the path to watch for changes
	configPath string

	// debounceTime is how long to wait after a change before reloading
	debounceTime time.Duration

	// lastReload tracks when we last reloaded
	lastReload time.Time

	// onReloadStart is called before reload begins
	onReloadStart func()

	// onReloadComplete is called after reload completes
	onReloadComplete func(results []ReloadResult)
}

// NewReloadManager creates a new reload manager.
func NewReloadManager(registry *Registry, cfg *Config, path string) *ReloadManager {
	return &ReloadManager{
		registry:      registry,
		currentConfig: cfg,
		configPath:    path,
		debounceTime:  100 * time.Millisecond,
	}
}

// SetDebounceTime sets how long to wait after detecting a change before reloading.
func (rm *ReloadManager) SetDebounceTime(d time.Duration) {
	rm.mu.Lock()
	defer rm.mu.Unlock()
	rm.debounceTime = d
}

// SetCallbacks sets callback functions for reload events.
func (rm *ReloadManager) SetCallbacks(onStart func(), onComplete func([]ReloadResult)) {
	rm.mu.Lock()
	defer rm.mu.Unlock()
	rm.onReloadStart = onStart
	rm.onReloadComplete = onComplete
}

// DetectChanges compares a new config with the current config and returns
// a list of sections that have changed.
func (rm *ReloadManager) DetectChanges(newConfig *Config) []string {
	rm.mu.RLock()
	defer rm.mu.RUnlock()

	var changed []string

	// Check for modified or new sections
	for _, newSec := range newConfig.GetSections() {
		name := newSec.GetName()
		oldSec := rm.currentConfig.GetSectionOptional(name)

		if oldSec == nil {
			// New section
			changed = append(changed, name)
			continue
		}

		// Compare options
		if !sectionsEqual(oldSec, newSec) {
			changed = append(changed, name)
		}
	}

	// Check for deleted sections
	for _, oldSec := range rm.currentConfig.GetSections() {
		name := oldSec.GetName()
		if !newConfig.HasSection(name) {
			changed = append(changed, name)
		}
	}

	return changed
}

// sectionsEqual checks if two sections have the same options.
func sectionsEqual(a, b *Section) bool {
	aOpts := a.RawOptions()
	bOpts := b.RawOptions()

	if len(aOpts) != len(bOpts) {
		return false
	}

	for k, v := range aOpts {
		if bOpts[k] != v {
			return false
		}
	}

	return true
}

// ReloadFromFile reloads configuration from the file and updates modules.
func (rm *ReloadManager) ReloadFromFile() ([]ReloadResult, error) {
	rm.mu.Lock()
	defer rm.mu.Unlock()

	// Check debounce
	if time.Since(rm.lastReload) < rm.debounceTime {
		return nil, nil
	}

	// Load new config
	newConfig, err := Load(rm.configPath)
	if err != nil {
		return nil, fmt.Errorf("failed to load config: %w", err)
	}

	return rm.reloadWithConfigLocked(newConfig)
}

// ReloadWithConfig reloads using a provided config (useful for testing).
func (rm *ReloadManager) ReloadWithConfig(newConfig *Config) ([]ReloadResult, error) {
	rm.mu.Lock()
	defer rm.mu.Unlock()
	return rm.reloadWithConfigLocked(newConfig)
}

// reloadWithConfigLocked performs the actual reload (must hold lock).
func (rm *ReloadManager) reloadWithConfigLocked(newConfig *Config) ([]ReloadResult, error) {
	// Call onReloadStart callback
	if rm.onReloadStart != nil {
		rm.onReloadStart()
	}

	var results []ReloadResult

	// Detect changes
	changed := rm.detectChangesLocked(newConfig)
	if len(changed) == 0 {
		return results, nil
	}

	// Process each changed section
	for _, sectionName := range changed {
		result := ReloadResult{Section: sectionName}

		// Get the module
		module := rm.registry.GetModule(sectionName)
		if module == nil {
			// Module not loaded - try to load it
			newSec := newConfig.GetSectionOptional(sectionName)
			if newSec != nil {
				factory := rm.registry.GetFactory(sectionName)
				if factory != nil {
					newModule, err := factory(newSec)
					if err != nil {
						result.Error = err
					} else {
						rm.registry.loaded[sectionName] = newModule
						result.Success = true
						result.WasReloaded = true
					}
				}
			}
			results = append(results, result)
			continue
		}

		// Check if module supports reload
		reloadable, ok := module.(Reloadable)
		if !ok {
			result.CanReload = false
			results = append(results, result)
			continue
		}

		result.CanReload = reloadable.CanReload()
		if !result.CanReload {
			results = append(results, result)
			continue
		}

		// Get new section config
		newSec := newConfig.GetSectionOptional(sectionName)
		if newSec == nil {
			// Section was deleted
			result.Error = fmt.Errorf("section deleted")
			results = append(results, result)
			continue
		}

		// Attempt reload
		err := reloadable.Reload(newSec)
		if err != nil {
			result.Error = err
		} else {
			result.Success = true
			result.WasReloaded = true
		}
		results = append(results, result)
	}

	// Update current config
	rm.currentConfig = newConfig
	rm.lastReload = time.Now()

	// Call onReloadComplete callback
	if rm.onReloadComplete != nil {
		rm.onReloadComplete(results)
	}

	return results, nil
}

// detectChangesLocked detects changes (must hold lock).
func (rm *ReloadManager) detectChangesLocked(newConfig *Config) []string {
	var changed []string

	// Check for modified or new sections
	for _, newSec := range newConfig.GetSections() {
		name := newSec.GetName()
		oldSec := rm.currentConfig.GetSectionOptional(name)

		if oldSec == nil {
			changed = append(changed, name)
			continue
		}

		if !sectionsEqual(oldSec, newSec) {
			changed = append(changed, name)
		}
	}

	// Check for deleted sections
	for _, oldSec := range rm.currentConfig.GetSections() {
		name := oldSec.GetName()
		if !newConfig.HasSection(name) {
			changed = append(changed, name)
		}
	}

	return changed
}

// GetCurrentConfig returns the current configuration.
func (rm *ReloadManager) GetCurrentConfig() *Config {
	rm.mu.RLock()
	defer rm.mu.RUnlock()
	return rm.currentConfig
}

// GetConfigPath returns the config file path.
func (rm *ReloadManager) GetConfigPath() string {
	return rm.configPath
}

// HasNonReloadableChanges checks if any changed sections cannot be hot-reloaded.
func (rm *ReloadManager) HasNonReloadableChanges(changed []string) []string {
	rm.mu.RLock()
	defer rm.mu.RUnlock()

	var nonReloadable []string

	for _, sectionName := range changed {
		module := rm.registry.GetModule(sectionName)
		if module == nil {
			// New module - can be loaded
			continue
		}

		reloadable, ok := module.(Reloadable)
		if !ok || !reloadable.CanReload() {
			nonReloadable = append(nonReloadable, sectionName)
		}
	}

	return nonReloadable
}
