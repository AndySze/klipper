package config

import (
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"
	"time"
)

// AutosaveConfig extends Config with runtime modification and save capabilities.
// It tracks changes made after loading and can save them back to a file.
type AutosaveConfig struct {
	*Config

	mu sync.RWMutex

	// originalPath is the path the config was loaded from
	originalPath string

	// modified tracks sections/options that have been changed
	modified map[string]map[string]string

	// deleted tracks sections that have been deleted
	deleted map[string]struct{}
}

// NewAutosaveConfig wraps a Config with autosave capabilities.
func NewAutosaveConfig(cfg *Config, path string) *AutosaveConfig {
	return &AutosaveConfig{
		Config:       cfg,
		originalPath: path,
		modified:     make(map[string]map[string]string),
		deleted:      make(map[string]struct{}),
	}
}

// LoadAutosave loads a config file with autosave capabilities.
func LoadAutosave(path string) (*AutosaveConfig, error) {
	cfg, err := Load(path)
	if err != nil {
		return nil, err
	}
	return NewAutosaveConfig(cfg, path), nil
}

// SetOption sets or updates an option value in a section.
// The change is tracked for later saving.
func (c *AutosaveConfig) SetOption(section, option, value string) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	// Ensure section exists in Config
	sec := c.Config.GetSectionOptional(section)
	if sec == nil {
		// Create new section
		c.Config.addSection(section, map[string]string{option: value})
	} else {
		// Update existing section
		sec.options[strings.ToLower(option)] = value
	}

	// Track modification
	if c.modified[section] == nil {
		c.modified[section] = make(map[string]string)
	}
	c.modified[section][option] = value

	// Remove from deleted if it was there
	delete(c.deleted, section)

	return nil
}

// DeleteSection marks a section for deletion.
func (c *AutosaveConfig) DeleteSection(section string) {
	c.mu.Lock()
	defer c.mu.Unlock()

	c.deleted[section] = struct{}{}
	delete(c.modified, section)
}

// GetModifiedSections returns a list of sections that have been modified.
func (c *AutosaveConfig) GetModifiedSections() []string {
	c.mu.RLock()
	defer c.mu.RUnlock()

	result := make([]string, 0, len(c.modified))
	for sec := range c.modified {
		result = append(result, sec)
	}
	sort.Strings(result)
	return result
}

// GetDeletedSections returns a list of sections marked for deletion.
func (c *AutosaveConfig) GetDeletedSections() []string {
	c.mu.RLock()
	defer c.mu.RUnlock()

	result := make([]string, 0, len(c.deleted))
	for sec := range c.deleted {
		result = append(result, sec)
	}
	sort.Strings(result)
	return result
}

// HasChanges returns true if there are unsaved changes.
func (c *AutosaveConfig) HasChanges() bool {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return len(c.modified) > 0 || len(c.deleted) > 0
}

// ClearChanges discards all tracked changes without saving.
func (c *AutosaveConfig) ClearChanges() {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.modified = make(map[string]map[string]string)
	c.deleted = make(map[string]struct{})
}

// SaveChanges writes the current configuration to the specified path.
// If path is empty, saves to the original path with a timestamp backup.
func (c *AutosaveConfig) SaveChanges(path string) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if path == "" {
		path = c.originalPath
	}

	// Create backup if saving to original path
	if path == c.originalPath && c.originalPath != "" {
		if err := c.createBackup(); err != nil {
			return fmt.Errorf("failed to create backup: %w", err)
		}
	}

	// Build config content
	content := c.buildConfigContent()

	// Write atomically using temp file
	dir := filepath.Dir(path)
	tmpFile, err := os.CreateTemp(dir, ".config-*.tmp")
	if err != nil {
		return fmt.Errorf("failed to create temp file: %w", err)
	}
	tmpPath := tmpFile.Name()

	_, err = tmpFile.WriteString(content)
	if err != nil {
		tmpFile.Close()
		os.Remove(tmpPath)
		return fmt.Errorf("failed to write config: %w", err)
	}

	if err := tmpFile.Close(); err != nil {
		os.Remove(tmpPath)
		return fmt.Errorf("failed to close temp file: %w", err)
	}

	if err := os.Rename(tmpPath, path); err != nil {
		os.Remove(tmpPath)
		return fmt.Errorf("failed to rename temp file: %w", err)
	}

	// Clear tracked changes after successful save
	c.modified = make(map[string]map[string]string)
	c.deleted = make(map[string]struct{})

	return nil
}

// createBackup creates a timestamped backup of the original file.
func (c *AutosaveConfig) createBackup() error {
	if c.originalPath == "" {
		return nil
	}

	// Check if original file exists
	if _, err := os.Stat(c.originalPath); os.IsNotExist(err) {
		return nil
	}

	// Generate backup filename: printer.cfg -> printer-20060102_150405.cfg
	ext := filepath.Ext(c.originalPath)
	base := strings.TrimSuffix(c.originalPath, ext)
	timestamp := time.Now().Format("20060102_150405")
	backupPath := fmt.Sprintf("%s-%s%s", base, timestamp, ext)

	// Copy original to backup
	data, err := os.ReadFile(c.originalPath)
	if err != nil {
		return fmt.Errorf("failed to read original file: %w", err)
	}

	if err := os.WriteFile(backupPath, data, 0644); err != nil {
		return fmt.Errorf("failed to write backup file: %w", err)
	}

	return nil
}

// buildConfigContent generates the INI-format config content.
func (c *AutosaveConfig) buildConfigContent() string {
	var sb strings.Builder

	// Get all section names and sort them
	sectionNames := c.Config.GetSectionNames()
	sort.Strings(sectionNames)

	for i, name := range sectionNames {
		// Skip deleted sections
		if _, deleted := c.deleted[name]; deleted {
			continue
		}

		if i > 0 {
			sb.WriteString("\n")
		}

		sb.WriteString("[")
		sb.WriteString(name)
		sb.WriteString("]\n")

		// Get section and its options
		sec := c.Config.GetSectionOptional(name)
		if sec == nil {
			continue
		}

		// Get and sort option names
		options := sec.RawOptions()
		optionNames := make([]string, 0, len(options))
		for opt := range options {
			optionNames = append(optionNames, opt)
		}
		sort.Strings(optionNames)

		for _, opt := range optionNames {
			value := options[opt]
			sb.WriteString(opt)
			sb.WriteString(": ")
			sb.WriteString(value)
			sb.WriteString("\n")
		}
	}

	return sb.String()
}

// GetOriginalPath returns the path the config was loaded from.
func (c *AutosaveConfig) GetOriginalPath() string {
	return c.originalPath
}

// ReloadFromDisk reloads the config from the original file.
// All unsaved changes are discarded.
func (c *AutosaveConfig) ReloadFromDisk() error {
	if c.originalPath == "" {
		return fmt.Errorf("no original path to reload from")
	}

	cfg, err := Load(c.originalPath)
	if err != nil {
		return err
	}

	c.mu.Lock()
	defer c.mu.Unlock()

	c.Config = cfg
	c.modified = make(map[string]map[string]string)
	c.deleted = make(map[string]struct{})

	return nil
}
