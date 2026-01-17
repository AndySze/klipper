// mcu_manager provides multi-MCU management and reactor integration.
// This is the high-level interface between the hosth4 runtime and MCU hardware.
package hosth4

import (
	"fmt"
	"io"
	"sync"
	"time"

	"klipper-go-migration/pkg/reactor"
)

// MCUManager manages connections to one or more MCUs.
type MCUManager struct {
	mu sync.RWMutex

	// Reactor for event-driven scheduling
	reactor *reactor.Reactor

	// MCU connections by name
	mcus map[string]*MCUConnection

	// Primary MCU (usually "mcu")
	primary *MCUConnection

	// Configuration
	trace io.Writer

	// Connection state
	connected bool
	shutdown  bool

	// Timing
	startTime     float64
	printTime     float64
	lastEventTime float64

	// Callbacks
	onShutdown []func(reason string)
}

// NewMCUManager creates a new MCU manager.
func NewMCUManager() *MCUManager {
	return &MCUManager{
		reactor: reactor.New(),
		mcus:    make(map[string]*MCUConnection),
	}
}

// SetTrace enables debug tracing.
func (m *MCUManager) SetTrace(w io.Writer) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.trace = w
}

// Reactor returns the reactor instance.
func (m *MCUManager) Reactor() *reactor.Reactor {
	return m.reactor
}

// AddMCU adds an MCU configuration to be connected.
func (m *MCUManager) AddMCU(name string, cfg MCUConnectionConfig) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if _, exists := m.mcus[name]; exists {
		return fmt.Errorf("mcu %s already registered", name)
	}

	// Apply trace if set
	if m.trace != nil {
		cfg.Trace = m.trace
	}

	conn := NewMCUConnection(name, cfg)
	conn.SetReactor(m.reactor)
	m.mcus[name] = conn

	// First MCU is the primary
	if m.primary == nil {
		m.primary = conn
	}

	return nil
}

// Connect establishes connections to all configured MCUs.
func (m *MCUManager) Connect() error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.connected {
		return nil
	}

	m.tracef("MCUManager: Connecting to %d MCUs\n", len(m.mcus))

	// Connect to each MCU
	for name, mcu := range m.mcus {
		m.tracef("MCUManager: Connecting to MCU %s\n", name)
		if err := mcu.Connect(); err != nil {
			// Disconnect any already-connected MCUs
			for n, mc := range m.mcus {
				if mc.IsConnected() {
					mc.Disconnect()
				}
				if n == name {
					break
				}
			}
			return fmt.Errorf("connect MCU %s: %w", name, err)
		}
	}

	m.connected = true
	m.startTime = m.reactor.Monotonic()

	m.tracef("MCUManager: All MCUs connected\n")
	return nil
}

// Disconnect closes all MCU connections.
func (m *MCUManager) Disconnect() error {
	m.mu.Lock()
	defer m.mu.Unlock()

	if !m.connected {
		return nil
	}

	m.tracef("MCUManager: Disconnecting all MCUs\n")

	var firstErr error
	for name, mcu := range m.mcus {
		if err := mcu.Disconnect(); err != nil && firstErr == nil {
			firstErr = fmt.Errorf("disconnect MCU %s: %w", name, err)
		}
	}

	m.connected = false
	return firstErr
}

// GetMCU returns the MCU connection by name.
func (m *MCUManager) GetMCU(name string) *MCUConnection {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.mcus[name]
}

// PrimaryMCU returns the primary MCU connection.
func (m *MCUManager) PrimaryMCU() *MCUConnection {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.primary
}

// IsConnected returns true if all MCUs are connected.
func (m *MCUManager) IsConnected() bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.connected
}

// Run starts the reactor event loop.
func (m *MCUManager) Run() {
	m.reactor.Run()
}

// End stops the reactor event loop.
func (m *MCUManager) End() {
	m.reactor.End()
	m.reactor.Wait()
}

// SendCommand sends a command to the primary MCU.
func (m *MCUManager) SendCommand(cmdName string, params map[string]interface{}) error {
	m.mu.RLock()
	primary := m.primary
	m.mu.RUnlock()

	if primary == nil {
		return ErrMCUNotConnected
	}
	return primary.SendCommand(cmdName, params)
}

// SendCommandToMCU sends a command to a specific MCU.
func (m *MCUManager) SendCommandToMCU(mcuName, cmdName string, params map[string]interface{}) error {
	m.mu.RLock()
	mcu := m.mcus[mcuName]
	m.mu.RUnlock()

	if mcu == nil {
		return fmt.Errorf("unknown MCU: %s", mcuName)
	}
	return mcu.SendCommand(cmdName, params)
}

// SendCommandWithResponse sends a command to the primary MCU and waits for response.
func (m *MCUManager) SendCommandWithResponse(cmdName string, params map[string]interface{},
	responseName string, timeout time.Duration) (map[string]interface{}, error) {

	m.mu.RLock()
	primary := m.primary
	m.mu.RUnlock()

	if primary == nil {
		return nil, ErrMCUNotConnected
	}
	return primary.SendCommandWithResponse(cmdName, params, responseName, timeout)
}

// RegisterHandler registers a message handler on the primary MCU.
func (m *MCUManager) RegisterHandler(name string, handler MCUMessageHandler) {
	m.mu.RLock()
	primary := m.primary
	m.mu.RUnlock()

	if primary != nil {
		primary.RegisterHandler(name, handler)
	}
}

// RegisterOIDHandler registers an OID-specific handler on the primary MCU.
func (m *MCUManager) RegisterOIDHandler(name string, oid int, handler MCUMessageHandler) {
	m.mu.RLock()
	primary := m.primary
	m.mu.RUnlock()

	if primary != nil {
		primary.RegisterOIDHandler(name, oid, handler)
	}
}

// RegisterHandlerOnMCU registers a message handler on a specific MCU.
func (m *MCUManager) RegisterHandlerOnMCU(mcuName, msgName string, handler MCUMessageHandler) error {
	m.mu.RLock()
	mcu := m.mcus[mcuName]
	m.mu.RUnlock()

	if mcu == nil {
		return fmt.Errorf("unknown MCU: %s", mcuName)
	}
	mcu.RegisterHandler(msgName, handler)
	return nil
}

// RegisterOIDHandlerOnMCU registers an OID-specific handler on a specific MCU.
func (m *MCUManager) RegisterOIDHandlerOnMCU(mcuName, msgName string, oid int, handler MCUMessageHandler) error {
	m.mu.RLock()
	mcu := m.mcus[mcuName]
	m.mu.RUnlock()

	if mcu == nil {
		return fmt.Errorf("unknown MCU: %s", mcuName)
	}
	mcu.RegisterOIDHandler(msgName, oid, handler)
	return nil
}

// OnShutdown registers a callback for MCU shutdown events.
func (m *MCUManager) OnShutdown(callback func(reason string)) {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.onShutdown = append(m.onShutdown, callback)
}

// PrintTime returns the current print time based on reactor monotonic clock.
func (m *MCUManager) PrintTime() float64 {
	m.mu.RLock()
	startTime := m.startTime
	m.mu.RUnlock()

	return m.reactor.Monotonic() - startTime
}

// WaitReady waits until all MCUs are ready for commands.
func (m *MCUManager) WaitReady(timeout time.Duration) error {
	deadline := time.Now().Add(timeout)

	m.mu.RLock()
	mcus := make([]*MCUConnection, 0, len(m.mcus))
	for _, mcu := range m.mcus {
		mcus = append(mcus, mcu)
	}
	m.mu.RUnlock()

	for _, mcu := range mcus {
		for time.Now().Before(deadline) {
			if mcu.IsConnected() {
				break
			}
			time.Sleep(10 * time.Millisecond)
		}
		if !mcu.IsConnected() {
			return fmt.Errorf("MCU %s not ready", mcu.name)
		}
	}

	return nil
}

// ConfigureMCU sends configuration commands to an MCU.
// This is typically called during the connect phase.
func (m *MCUManager) ConfigureMCU(mcuName string, configCmds []ConfigCommand) error {
	m.mu.RLock()
	mcu := m.mcus[mcuName]
	m.mu.RUnlock()

	if mcu == nil {
		return fmt.Errorf("unknown MCU: %s", mcuName)
	}

	for _, cmd := range configCmds {
		if err := mcu.SendCommand(cmd.Name, cmd.Params); err != nil {
			return fmt.Errorf("config command %s: %w", cmd.Name, err)
		}
	}

	return nil
}

// ConfigCommand represents a configuration command.
type ConfigCommand struct {
	Name   string
	Params map[string]interface{}
}

// AllocateOID allocates an OID from the primary MCU.
func (m *MCUManager) AllocateOID() (int, error) {
	// In real implementation, this would request an OID from the MCU
	// For now, we'll use a simple counter approach
	m.mu.Lock()
	defer m.mu.Unlock()

	// This would typically be tracked per-MCU
	// For now, return a simple incrementing value
	return 0, fmt.Errorf("OID allocation not yet implemented")
}

// GetStatus returns status information about all MCUs.
func (m *MCUManager) GetStatus() map[string]interface{} {
	m.mu.RLock()
	defer m.mu.RUnlock()

	status := make(map[string]interface{})
	status["connected"] = m.connected

	mcuStatus := make(map[string]interface{})
	for name, mcu := range m.mcus {
		mcuStatus[name] = map[string]interface{}{
			"connected": mcu.IsConnected(),
			"freq":      mcu.MCUFreq(),
		}
	}
	status["mcus"] = mcuStatus

	return status
}

// tracef writes trace output if tracing is enabled.
func (m *MCUManager) tracef(format string, args ...interface{}) {
	m.mu.RLock()
	trace := m.trace
	m.mu.RUnlock()

	if trace != nil {
		fmt.Fprintf(trace, format, args...)
	}
}

// MCUStatusInfo contains runtime status information about an MCU connection.
type MCUStatusInfo struct {
	Name      string
	Connected bool
	Freq      float64
	Version   string
}

// ListMCUs returns status information about all configured MCUs.
func (m *MCUManager) ListMCUs() []MCUStatusInfo {
	m.mu.RLock()
	defer m.mu.RUnlock()

	var infos []MCUStatusInfo
	for name, conn := range m.mcus {
		infos = append(infos, MCUStatusInfo{
			Name:      name,
			Connected: conn.IsConnected(),
			Freq:      conn.MCUFreq(),
		})
	}
	return infos
}
