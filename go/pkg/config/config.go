package config

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"
)

// Config provides access to a configuration file with access tracking.
// It implements the same pattern as Python's PrinterConfig.
type Config struct {
	mu       sync.RWMutex
	sections map[string]*Section
	order    []string // Maintains section order

	// Access tracking for sections
	accessedSections map[string]struct{}
}

// New creates a new empty Config.
func New() *Config {
	return &Config{
		sections:         make(map[string]*Section),
		accessedSections: make(map[string]struct{}),
	}
}

// Load reads a configuration file and returns a Config.
// Supports [include path] directives for including other config files.
func Load(path string) (*Config, error) {
	c := New()
	visited := make(map[string]bool)
	if err := c.parseFile(path, visited); err != nil {
		return nil, err
	}
	return c, nil
}

// parseFile parses a config file and handles include directives.
func (c *Config) parseFile(path string, visited map[string]bool) error {
	abs, err := filepath.Abs(path)
	if err != nil {
		return fmt.Errorf("config: invalid path %s: %w", path, err)
	}

	// Check for recursive includes
	if visited[abs] {
		return fmt.Errorf("config: recursive include: %s", path)
	}
	visited[abs] = true
	defer func() { visited[abs] = false }()

	f, err := os.Open(abs)
	if err != nil {
		return fmt.Errorf("config: unable to open %s: %w", path, err)
	}
	defer f.Close()

	dir := filepath.Dir(abs)
	var currentSection string
	var currentOptions map[string]string

	scanner := bufio.NewScanner(f)
	lineNum := 0
	for scanner.Scan() {
		lineNum++
		line := strings.TrimSpace(scanner.Text())

		// Skip empty lines
		if line == "" {
			continue
		}

		// Handle Klipper SAVE_CONFIG format: lines starting with "#*#" are
		// auto-generated config that should be parsed as regular config.
		// Strip the "#*#" prefix and continue parsing.
		if strings.HasPrefix(line, "#*#") {
			line = strings.TrimSpace(line[3:])
			if line == "" {
				continue
			}
			// Fall through to normal parsing
		} else if idx := strings.IndexByte(line, '#'); idx >= 0 {
			// Strip regular comments
			line = strings.TrimSpace(line[:idx])
			if line == "" {
				continue
			}
		}

		// Section header
		if strings.HasPrefix(line, "[") && strings.HasSuffix(line, "]") {
			// Save previous section
			if currentSection != "" {
				c.addSection(currentSection, currentOptions)
			}

			header := strings.TrimSpace(line[1 : len(line)-1])
			if header == "" {
				return fmt.Errorf("config: empty section header at line %d in %s", lineNum, path)
			}

			// Handle include directive
			if strings.HasPrefix(header, "include ") {
				spec := strings.TrimSpace(header[8:])
				if spec == "" {
					return fmt.Errorf("config: empty include at line %d in %s", lineNum, path)
				}
				glob := filepath.Join(dir, spec)
				matches, err := filepath.Glob(glob)
				if err != nil {
					return fmt.Errorf("config: invalid include pattern %q: %w", spec, err)
				}
				sort.Strings(matches)
				if len(matches) == 0 && !hasGlobMeta(glob) {
					return fmt.Errorf("config: include file does not exist: %s", glob)
				}
				for _, m := range matches {
					if err := c.parseFile(m, visited); err != nil {
						return err
					}
				}
				currentSection = ""
				currentOptions = nil
				continue
			}

			currentSection = header
			currentOptions = make(map[string]string)
			continue
		}

		// Skip options before first section
		if currentSection == "" {
			continue
		}

		// Parse key: value or key = value
		kv := strings.SplitN(line, ":", 2)
		if len(kv) != 2 {
			kv = strings.SplitN(line, "=", 2)
		}
		if len(kv) != 2 {
			// Invalid line - skip it
			continue
		}

		key := strings.TrimSpace(kv[0])
		value := strings.TrimSpace(kv[1])
		if key == "" {
			continue
		}
		currentOptions[key] = value
	}

	// Save last section
	if currentSection != "" {
		c.addSection(currentSection, currentOptions)
	}

	if err := scanner.Err(); err != nil {
		return fmt.Errorf("config: error reading %s: %w", path, err)
	}

	return nil
}

// hasGlobMeta returns true if the path contains glob metacharacters.
func hasGlobMeta(path string) bool {
	return strings.ContainsAny(path, "*?[")
}

// LoadString parses a configuration from a string.
func LoadString(data string) (*Config, error) {
	c := New()
	var currentSection string
	var currentOptions map[string]string

	lines := strings.Split(data, "\n")
	lineNum := 0
	for _, rawLine := range lines {
		lineNum++
		line := strings.TrimSpace(rawLine)

		// Skip empty lines
		if line == "" {
			continue
		}

		// Handle Klipper SAVE_CONFIG format: lines starting with "#*#" are
		// auto-generated config that should be parsed as regular config.
		// Strip the "#*#" prefix and continue parsing.
		if strings.HasPrefix(line, "#*#") {
			line = strings.TrimSpace(line[3:])
			if line == "" {
				continue
			}
			// Fall through to normal parsing
		} else if idx := strings.IndexByte(line, '#'); idx >= 0 {
			// Strip regular comments
			line = strings.TrimSpace(line[:idx])
			if line == "" {
				continue
			}
		}

		// Section header
		if strings.HasPrefix(line, "[") && strings.HasSuffix(line, "]") {
			// Save previous section
			if currentSection != "" {
				c.addSection(currentSection, currentOptions)
			}
			currentSection = strings.TrimSpace(line[1 : len(line)-1])
			if currentSection == "" {
				return nil, fmt.Errorf("config: empty section header at line %d", lineNum)
			}
			currentOptions = make(map[string]string)
			continue
		}

		// Skip options before first section
		if currentSection == "" {
			continue
		}

		// Parse key: value or key = value
		kv := strings.SplitN(line, ":", 2)
		if len(kv) != 2 {
			kv = strings.SplitN(line, "=", 2)
		}
		if len(kv) != 2 {
			continue
		}

		key := strings.TrimSpace(kv[0])
		value := strings.TrimSpace(kv[1])
		if key == "" {
			continue
		}
		currentOptions[key] = value
	}

	// Save last section
	if currentSection != "" {
		c.addSection(currentSection, currentOptions)
	}

	return c, nil
}

// addSection adds a section to the config.
func (c *Config) addSection(name string, options map[string]string) {
	c.mu.Lock()
	defer c.mu.Unlock()

	// If section already exists, merge options
	if existing, ok := c.sections[name]; ok {
		for k, v := range options {
			existing.options[strings.ToLower(k)] = v
		}
		return
	}

	c.sections[name] = newSection(name, options)
	c.order = append(c.order, name)
}

// GetSection returns a Section by name, or error if not found.
func (c *Config) GetSection(name string) (*Section, error) {
	c.mu.RLock()
	defer c.mu.RUnlock()

	sec, ok := c.sections[name]
	if !ok {
		return nil, ErrMissingSection(name)
	}

	c.mu.RUnlock()
	c.mu.Lock()
	c.accessedSections[name] = struct{}{}
	c.mu.Unlock()
	c.mu.RLock()

	return sec, nil
}

// GetSectionOptional returns a Section if it exists, or nil if not.
func (c *Config) GetSectionOptional(name string) *Section {
	c.mu.Lock()
	defer c.mu.Unlock()

	sec, ok := c.sections[name]
	if ok {
		c.accessedSections[name] = struct{}{}
	}
	return sec
}

// HasSection checks if a section exists.
func (c *Config) HasSection(name string) bool {
	c.mu.RLock()
	defer c.mu.RUnlock()
	_, ok := c.sections[name]
	return ok
}

// GetSections returns all sections.
func (c *Config) GetSections() []*Section {
	c.mu.RLock()
	defer c.mu.RUnlock()

	result := make([]*Section, 0, len(c.sections))
	for _, name := range c.order {
		result = append(result, c.sections[name])
	}
	return result
}

// GetSectionNames returns all section names in order.
func (c *Config) GetSectionNames() []string {
	c.mu.RLock()
	defer c.mu.RUnlock()

	result := make([]string, len(c.order))
	copy(result, c.order)
	return result
}

// GetPrefixSections returns all sections that start with the given prefix.
func (c *Config) GetPrefixSections(prefix string) []*Section {
	c.mu.RLock()
	defer c.mu.RUnlock()

	var result []*Section
	for _, name := range c.order {
		if strings.HasPrefix(name, prefix) {
			result = append(result, c.sections[name])
		}
	}
	return result
}

// GetPrefixSectionNames returns all section names that start with the given prefix.
func (c *Config) GetPrefixSectionNames(prefix string) []string {
	c.mu.RLock()
	defer c.mu.RUnlock()

	var result []string
	for _, name := range c.order {
		if strings.HasPrefix(name, prefix) {
			result = append(result, name)
		}
	}
	return result
}

// GetAccessedSections returns a list of sections that were accessed.
func (c *Config) GetAccessedSections() []string {
	c.mu.RLock()
	defer c.mu.RUnlock()

	result := make([]string, 0, len(c.accessedSections))
	for name := range c.accessedSections {
		result = append(result, name)
	}
	sort.Strings(result)
	return result
}

// GetUnusedSections returns a list of sections that were not accessed.
func (c *Config) GetUnusedSections() []string {
	c.mu.RLock()
	defer c.mu.RUnlock()

	var result []string
	for name := range c.sections {
		if _, ok := c.accessedSections[name]; !ok {
			result = append(result, name)
		}
	}
	sort.Strings(result)
	return result
}

// CheckUnusedSections returns an error if there are unused sections.
func (c *Config) CheckUnusedSections() error {
	unused := c.GetUnusedSections()
	if len(unused) > 0 {
		return NewConfigError("", "", fmt.Sprintf("unused sections: %v", unused))
	}
	return nil
}

// CheckUnusedOptions returns an error if any section has unused options.
func (c *Config) CheckUnusedOptions() error {
	c.mu.RLock()
	defer c.mu.RUnlock()

	var errors []string
	for name, sec := range c.sections {
		unused := sec.GetUnusedOptions()
		if len(unused) > 0 {
			errors = append(errors, fmt.Sprintf("[%s]: unused options %v", name, unused))
		}
	}
	if len(errors) > 0 {
		sort.Strings(errors)
		return NewConfigError("", "", strings.Join(errors, "; "))
	}
	return nil
}

// Merge combines another Config into this one.
// Sections and options from other override this Config.
func (c *Config) Merge(other *Config) {
	c.mu.Lock()
	defer c.mu.Unlock()

	other.mu.RLock()
	defer other.mu.RUnlock()

	for _, name := range other.order {
		otherSec := other.sections[name]
		if existing, ok := c.sections[name]; ok {
			// Merge options into existing section
			for k, v := range otherSec.options {
				existing.options[k] = v
			}
		} else {
			// Add new section
			opts := make(map[string]string)
			for k, v := range otherSec.options {
				opts[k] = v
			}
			c.sections[name] = newSection(name, opts)
			c.order = append(c.order, name)
		}
	}
}
