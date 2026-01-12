package hosth1

import (
	"fmt"
	"sort"
	"strings"

	"klipper-go-migration/pkg/protocol"
)

// MCUCompileResult holds the compilation result for a single MCU.
type MCUCompileResult struct {
	Name     string   // MCU name (e.g., "mcu", "zboard")
	Commands []string // Commands for this MCU
	OIDCount int      // Number of OIDs allocated
}

// MultiMCUCompileResult holds the compilation results for all MCUs.
type MultiMCUCompileResult struct {
	MCUs []*MCUCompileResult // Results per MCU (primary first)
}

// MCUBuilder builds commands for a single MCU.
type MCUBuilder struct {
	Name      string
	Dict      *protocol.Dictionary
	MCUFreq   float64
	ADCMax    float64
	NextOID   int
	Commands  []string
}

// NewMCUBuilder creates a new MCU builder.
func NewMCUBuilder(name string, dict *protocol.Dictionary) (*MCUBuilder, error) {
	clockFreq, err := dictConfigFloat(dict, "CLOCK_FREQ")
	if err != nil {
		return nil, err
	}
	adcMax, err := dictConfigFloat(dict, "ADC_MAX")
	if err != nil {
		return nil, err
	}
	return &MCUBuilder{
		Name:     name,
		Dict:     dict,
		MCUFreq:  clockFreq,
		ADCMax:   adcMax,
		NextOID:  0,
		Commands: make([]string, 0),
	}, nil
}

// AllocOID allocates the next OID and returns it.
func (b *MCUBuilder) AllocOID() int {
	oid := b.NextOID
	b.NextOID++
	return oid
}

// AddCommand adds a command to this MCU's command list.
func (b *MCUBuilder) AddCommand(cmd string) {
	b.Commands = append(b.Commands, cmd)
}

// Finalize adds the allocate_oids and finalize_config commands.
func (b *MCUBuilder) Finalize() {
	// Insert allocate_oids at the beginning
	allocCmd := fmt.Sprintf("allocate_oids count=%d", b.NextOID)
	b.Commands = append([]string{allocCmd}, b.Commands...)

	// Calculate CRC32 of all commands (matching Klipper's algorithm)
	crc := uint32(0)
	for _, cmd := range b.Commands {
		// Skip allocate_oids for CRC calculation (it's added dynamically)
		if strings.HasPrefix(cmd, "allocate_oids") {
			continue
		}
		crc = crc32Update(crc, []byte(cmd))
	}
	b.Commands = append(b.Commands, fmt.Sprintf("finalize_config crc=%d", crc))
}

// Result returns the compile result for this MCU.
func (b *MCUBuilder) Result() *MCUCompileResult {
	return &MCUCompileResult{
		Name:     b.Name,
		Commands: b.Commands,
		OIDCount: b.NextOID,
	}
}

// ParseMCUSectionsFromConfig extracts MCU configurations from a config.
// Returns a map of MCU name -> serial path.
func ParseMCUSectionsFromConfig(cfg *config) map[string]string {
	result := make(map[string]string)

	// Check for main [mcu] section
	if sec, ok := cfg.sections["mcu"]; ok {
		serial := strings.TrimSpace(sec["serial"])
		result["mcu"] = serial
	}

	// Check for additional [mcu name] sections
	for name := range cfg.sections {
		if strings.HasPrefix(name, "mcu ") {
			mcuName := strings.TrimSpace(strings.TrimPrefix(name, "mcu "))
			if mcuName == "" {
				continue
			}
			sec := cfg.sections[name]
			serial := strings.TrimSpace(sec["serial"])
			result[mcuName] = serial
		}
	}

	return result
}

// SortedMCUNames returns MCU names sorted with "mcu" first.
func SortedMCUNames(mcus map[string]string) []string {
	names := make([]string, 0, len(mcus))
	for name := range mcus {
		names = append(names, name)
	}
	sort.Slice(names, func(i, j int) bool {
		if names[i] == "mcu" {
			return true
		}
		if names[j] == "mcu" {
			return false
		}
		return names[i] < names[j]
	})
	return names
}

// GetPinMCU extracts the MCU name from a pin string.
// Returns "mcu" if no MCU prefix is present.
func GetPinMCU(pinDesc string) string {
	// Parse the pin to extract MCU name
	// Format: [^~!]?[mcu:]pin
	desc := strings.TrimSpace(pinDesc)

	// Remove pullup/invert prefixes
	for len(desc) > 0 && (desc[0] == '^' || desc[0] == '~' || desc[0] == '!') {
		desc = strings.TrimSpace(desc[1:])
	}

	// Check for MCU prefix
	if idx := strings.Index(desc, ":"); idx > 0 {
		return desc[:idx]
	}
	return "mcu"
}

// GetPinName extracts the pin name without MCU prefix.
func GetPinName(pinDesc string) string {
	desc := strings.TrimSpace(pinDesc)

	// Remove pullup/invert prefixes
	for len(desc) > 0 && (desc[0] == '^' || desc[0] == '~' || desc[0] == '!') {
		desc = strings.TrimSpace(desc[1:])
	}

	// Remove MCU prefix if present
	if idx := strings.Index(desc, ":"); idx > 0 {
		return desc[idx+1:]
	}
	return desc
}

// IsPinInverted checks if a pin has the invert prefix (!).
func IsPinInverted(pinDesc string) bool {
	desc := strings.TrimSpace(pinDesc)
	// Pullup comes before invert
	for len(desc) > 0 && (desc[0] == '^' || desc[0] == '~') {
		desc = strings.TrimSpace(desc[1:])
	}
	return len(desc) > 0 && desc[0] == '!'
}

// IsPinPullup checks if a pin has pullup (^) or pulldown (~) prefix.
func IsPinPullup(pinDesc string) int {
	desc := strings.TrimSpace(pinDesc)
	if len(desc) > 0 {
		if desc[0] == '^' {
			return 1
		}
		if desc[0] == '~' {
			return -1
		}
	}
	return 0
}

// crc32Update updates a CRC32 value with new data (IEEE polynomial).
func crc32Update(crc uint32, data []byte) uint32 {
	const poly = 0xedb88320
	for _, b := range data {
		crc ^= uint32(b)
		for i := 0; i < 8; i++ {
			if crc&1 != 0 {
				crc = (crc >> 1) ^ poly
			} else {
				crc >>= 1
			}
		}
	}
	return crc
}
