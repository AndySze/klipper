package hosth4

import (
	"fmt"
	"sort"
	"strings"

	"klipper-go-migration/pkg/protocol"
)

// MCUInfo holds information about a single MCU in a multi-MCU setup.
type MCUInfo struct {
	Name       string              // MCU name (e.g., "mcu", "zboard", "auxboard")
	Serial     string              // Serial port path
	Dict       *protocol.Dictionary // Protocol dictionary for this MCU
	NextOID    int                 // Next available OID for this MCU
	IsPrimary  bool                // True if this is the primary timing MCU
}

// MCURegistry manages multiple MCUs in a printer configuration.
type MCURegistry struct {
	mcus    map[string]*MCUInfo // MCU name -> info
	primary string              // Name of the primary MCU (timing source)
	order   []string            // MCU names in discovery order
}

// NewMCURegistry creates a new empty MCU registry.
func NewMCURegistry() *MCURegistry {
	return &MCURegistry{
		mcus:  make(map[string]*MCUInfo),
		order: make([]string, 0),
	}
}

// Register adds an MCU to the registry.
// The first MCU registered becomes the primary timing source.
func (r *MCURegistry) Register(name, serial string, dict *protocol.Dictionary) error {
	if name == "" {
		return fmt.Errorf("MCU name cannot be empty")
	}
	if _, exists := r.mcus[name]; exists {
		return fmt.Errorf("MCU %q already registered", name)
	}

	isPrimary := len(r.mcus) == 0
	r.mcus[name] = &MCUInfo{
		Name:      name,
		Serial:    serial,
		Dict:      dict,
		NextOID:   0,
		IsPrimary: isPrimary,
	}
	r.order = append(r.order, name)
	if isPrimary {
		r.primary = name
	}
	return nil
}

// Get returns the MCU info for the given name, or nil if not found.
func (r *MCURegistry) Get(name string) *MCUInfo {
	return r.mcus[name]
}

// GetPrimary returns the primary MCU info.
func (r *MCURegistry) GetPrimary() *MCUInfo {
	return r.mcus[r.primary]
}

// AllocateOID allocates the next OID for the given MCU.
func (r *MCURegistry) AllocateOID(mcuName string) (int, error) {
	mcu := r.mcus[mcuName]
	if mcu == nil {
		return 0, fmt.Errorf("unknown MCU %q", mcuName)
	}
	oid := mcu.NextOID
	mcu.NextOID++
	return oid, nil
}

// GetOIDCount returns the total number of OIDs allocated for an MCU.
func (r *MCURegistry) GetOIDCount(mcuName string) int {
	mcu := r.mcus[mcuName]
	if mcu == nil {
		return 0
	}
	return mcu.NextOID
}

// Names returns the names of all registered MCUs in discovery order.
func (r *MCURegistry) Names() []string {
	result := make([]string, len(r.order))
	copy(result, r.order)
	return result
}

// Count returns the number of registered MCUs.
func (r *MCURegistry) Count() int {
	return len(r.mcus)
}

// ParseMCUSections extracts MCU configurations from a config wrapper.
// Returns a map of MCU name -> serial path.
// The main [mcu] section becomes "mcu", [mcu name] sections become "name".
func ParseMCUSections(cfg *configWrapper) map[string]string {
	result := make(map[string]string)

	// Check for main [mcu] section
	if sec, ok := cfg.section("mcu"); ok {
		serial := strings.TrimSpace(sec["serial"])
		if serial != "" {
			result["mcu"] = serial
		} else {
			// Even without serial, register the MCU (for simulation)
			result["mcu"] = ""
		}
	}

	// Check for additional [mcu name] sections
	for _, secName := range cfg.sectionsByPrefix("mcu ") {
		mcuName := strings.TrimPrefix(secName, "mcu ")
		mcuName = strings.TrimSpace(mcuName)
		if mcuName == "" {
			continue
		}
		if sec, ok := cfg.section(secName); ok {
			serial := strings.TrimSpace(sec["serial"])
			result[mcuName] = serial
		}
	}

	return result
}

// SortedMCUNames returns MCU names sorted with "mcu" first, then alphabetically.
func SortedMCUNames(names map[string]string) []string {
	result := make([]string, 0, len(names))
	for name := range names {
		result = append(result, name)
	}
	sort.Slice(result, func(i, j int) bool {
		// "mcu" always comes first
		if result[i] == "mcu" {
			return true
		}
		if result[j] == "mcu" {
			return false
		}
		return result[i] < result[j]
	})
	return result
}
