// mcu_connection provides real-time MCU communication for the hosth4 runtime.
// This integrates serial port communication, reactor event loop, and MCU message dispatch
// to enable real hardware operation.
package hosth4

import (
	"context"
	"errors"
	"fmt"
	"io"
	"strings"
	"sync"
	"time"

	"klipper-go-migration/pkg/chelper"
	mcupkg "klipper-go-migration/pkg/mcu"
	"klipper-go-migration/pkg/protocol"
	"klipper-go-migration/pkg/reactor"
	"klipper-go-migration/pkg/serial"
)

// Common errors for MCU connection
var (
	ErrMCUNotConnected   = errors.New("mcu: not connected")
	ErrMCUTimeout        = errors.New("mcu: communication timeout")
	ErrMCUProtocolError  = errors.New("mcu: protocol error")
	ErrMCUShutdown       = errors.New("mcu: shutdown requested")
	ErrMCUClockNotSynced = errors.New("mcu: clock not synchronized")
)

// MCUConnectionConfig holds configuration for connecting to an MCU.
type MCUConnectionConfig struct {
	// Serial port configuration
	Device   string // Device path (e.g., /dev/ttyUSB0)
	BaudRate int    // Default: 250000

	// Connection timeouts
	ConnectTimeout   time.Duration // Default: 60s
	HandshakeTimeout time.Duration // Default: 5s
	ResponseTimeout  time.Duration // Default: 2s

	// Clock synchronization
	ClockSyncInterval time.Duration // Default: 100ms
	ClockSyncSamples  int           // Default: 8

	// ResetOnConnect toggles DTR to reset the MCU before handshake (default: true)
	// This is needed if the MCU may be in a previously-configured state
	ResetOnConnect bool

	// UseSerialQueue enables the chelper serialqueue for MCU communication.
	// This provides better retransmission handling and timing accuracy.
	// When false (default), uses the pure Go serial I/O implementation.
	UseSerialQueue bool

	// Debugging
	Trace io.Writer
}

// DefaultMCUConnectionConfig returns default configuration.
func DefaultMCUConnectionConfig() MCUConnectionConfig {
	return MCUConnectionConfig{
		BaudRate:          250000,
		ConnectTimeout:    60 * time.Second,
		HandshakeTimeout:  15 * time.Second, // Allow time for reset, drain, and identify
		ResponseTimeout:   2 * time.Second, // Match testSerial timeout
		ClockSyncInterval: 100 * time.Millisecond,
		ClockSyncSamples:  8,
		ResetOnConnect:    false, // Disable reset - may cause issues with some serial adapters
	}
}

// MCUConnection manages real-time communication with a single MCU.
type MCUConnection struct {
	mu sync.RWMutex

	name   string
	config MCUConnectionConfig

	// Serial connection
	port *serial.Port

	// Protocol dictionary (from handshake)
	dict *protocol.Dictionary

	// Message reader for background reception (used when UseSerialQueue=false)
	reader *mcupkg.Reader

	// SerialQueue adapter (used when UseSerialQueue=true)
	sqAdapter *chelper.SerialQueueAdapter

	// Reactor for timing and scheduling
	reactor *reactor.Reactor

	// Clock synchronization state
	mcuFreq       float64 // MCU frequency (Hz)
	clockConv     float64 // Host time to MCU clock conversion factor
	clockOffset   float64 // Offset between host and MCU clocks
	lastSyncTime  float64 // Last successful sync time (host time)
	clockSyncMu   sync.RWMutex
	clockSynced   bool
	clockSamples  []clockSample
	clockSyncStop chan struct{}

	// Message handlers
	handlers    map[string]MCUMessageHandler
	handlersMu  sync.RWMutex
	oidHandlers map[int]map[string]MCUMessageHandler // oid -> name -> handler

	// Connection state
	connected      bool
	shutdown       bool
	shutdownReason string
	ctx            context.Context
	cancel         context.CancelFunc
	wg             sync.WaitGroup

	// Command queue for outgoing messages
	cmdQueue   chan mcuCmd
	cmdSeq     uint8
	cmdSeqLock sync.Mutex

	// Pending responses - indexed by response name
	pendingMu       sync.Mutex
	pending         map[uint8]chan *mcupkg.Message // legacy: by seq
	pendingByName   map[string]pendingResp         // by response name
}

// pendingResp tracks a pending response request
type pendingResp struct {
	oid      *int                   // Expected OID (nil for any)
	respChan chan *mcupkg.Message
}

// clockSample records a single clock synchronization measurement.
type clockSample struct {
	hostTime  float64
	mcuClock  uint64
	roundTrip float64
}

// mcuCmd represents a command to send to the MCU.
type mcuCmd struct {
	data     []byte
	response string        // Expected response name (empty for no response)
	oid      *int          // Expected OID (nil for any)
	timeout  time.Duration
	respChan chan *mcupkg.Message
}

// MCUMessageHandler is called when a message is received from the MCU.
type MCUMessageHandler func(params map[string]interface{}, receiveTime time.Time)

// NewMCUConnection creates a new MCU connection.
func NewMCUConnection(name string, cfg MCUConnectionConfig) *MCUConnection {
	ctx, cancel := context.WithCancel(context.Background())
	return &MCUConnection{
		name:          name,
		config:        cfg,
		handlers:      make(map[string]MCUMessageHandler),
		oidHandlers:   make(map[int]map[string]MCUMessageHandler),
		ctx:           ctx,
		cancel:        cancel,
		cmdQueue:      make(chan mcuCmd, 100),
		pending:       make(map[uint8]chan *mcupkg.Message),
		pendingByName: make(map[string]pendingResp),
	}
}

// Connect establishes connection with the MCU.
// This performs:
// 1. Serial port connection
// 2. MCU identification handshake
// 3. Dictionary retrieval
// 4. Clock synchronization initialization
func (mc *MCUConnection) Connect() error {
	mc.mu.Lock()
	if mc.connected {
		mc.mu.Unlock()
		return nil // Already connected
	}

	mc.tracef("MCU %s: Connecting to %s at %d baud\n", mc.name, mc.config.Device, mc.config.BaudRate)

	// Open serial port
	serialCfg := serial.Config{
		Device:         mc.config.Device,
		BaudRate:       mc.config.BaudRate,
		ConnectTimeout: mc.config.ConnectTimeout,
		ReadTimeout:    mc.config.ResponseTimeout,
		RTSOnConnect:   true,
		DTROnConnect:   true,
	}

	port, err := serial.Open(serialCfg)
	if err != nil {
		return fmt.Errorf("mcu %s: open serial: %w", mc.name, err)
	}
	mc.port = port

	mc.tracef("MCU %s: Serial port opened, performing handshake\n", mc.name)

	// Perform MCU identification handshake
	handshakeCfg := mcupkg.DefaultHandshakeConfig()
	handshakeCfg.Timeout = mc.config.HandshakeTimeout
	handshakeCfg.Trace = mc.config.Trace
	handshakeCfg.ResetBeforeIdentify = mc.config.ResetOnConnect
	identifyData, err := mcupkg.Handshake(port, handshakeCfg)
	if err != nil {
		port.Close()
		return fmt.Errorf("mcu %s: identify: %w", mc.name, err)
	}
	mc.dict = identifyData.Dictionary

	// Extract MCU frequency from config
	if freq, ok := mc.dict.Config["CLOCK_FREQ"].(float64); ok {
		mc.mcuFreq = freq
	} else if freq, ok := mc.dict.Config["CLOCK_FREQ"].(int); ok {
		mc.mcuFreq = float64(freq)
	} else {
		mc.mcuFreq = 16000000 // Default to 16 MHz
	}

	mc.tracef("MCU %s: Identified - version: %s, freq: %.0f Hz\n",
		mc.name, identifyData.Version, mc.mcuFreq)

	// Initialize clock conversion factor (MCU clocks per second)
	mc.clockConv = mc.mcuFreq

	// Initialize communication layer based on configuration
	if mc.config.UseSerialQueue {
		// Use chelper serialqueue for communication
		mc.tracef("MCU %s: Using chelper serialqueue for communication\n", mc.name)

		sq, err := chelper.NewSerialQueue(port.Fd(), 'u', 0, mc.name)
		if err != nil {
			port.Close()
			mc.mu.Unlock()
			return fmt.Errorf("mcu %s: create serialqueue: %w", mc.name, err)
		}

		// Configure serialqueue
		sq.SetWireFrequency(float64(mc.config.BaudRate) * 10) // bits per second
		sq.SetReceiveWindow(25) // Default receive window

		adapter, err := chelper.NewSerialQueueAdapter(sq, mc.dict)
		if err != nil {
			sq.Free()
			port.Close()
			mc.mu.Unlock()
			return fmt.Errorf("mcu %s: create adapter: %w", mc.name, err)
		}

		// Register handlers on adapter
		mc.sqAdapter = adapter
		mc.registerSerialQueueHandlers()

		// Start adapter receive loop
		if err := adapter.Start(); err != nil {
			adapter.Close()
			port.Close()
			mc.mu.Unlock()
			return fmt.Errorf("mcu %s: start adapter: %w", mc.name, err)
		}
	} else {
		// Use traditional mcupkg.Reader
		mc.reader = mcupkg.NewReader(port, mc.dict)
		mc.reader.SetDefaultHandler(mc.handleUnknownMessage)

		// Register standard message handlers
		mc.registerStandardHandlers()

		// Start reader background goroutine
		mc.reader.Start()
	}

	// Start command sender goroutine
	mc.wg.Add(1)
	go mc.commandSenderLoop()

	// Initialize clock synchronization
	mc.clockSyncStop = make(chan struct{})
	mc.wg.Add(1)
	go mc.clockSyncLoop()

	// Release lock before waiting for clock sync to avoid deadlock
	// (encodeCommand needs RLock)
	mc.mu.Unlock()

	// Wait for initial clock sync
	if err := mc.waitForClockSync(mc.config.HandshakeTimeout); err != nil {
		mc.Disconnect() // Use public method since we don't hold lock
		return fmt.Errorf("mcu %s: clock sync: %w", mc.name, err)
	}

	mc.mu.Lock()
	mc.connected = true
	mc.mu.Unlock()
	mc.tracef("MCU %s: Connected successfully\n", mc.name)

	return nil
}

// Disconnect closes the MCU connection.
func (mc *MCUConnection) Disconnect() error {
	mc.mu.Lock()
	defer mc.mu.Unlock()
	return mc.disconnectLocked()
}

func (mc *MCUConnection) disconnectLocked() error {
	if !mc.connected && mc.port == nil {
		return nil
	}

	mc.tracef("MCU %s: Disconnecting\n", mc.name)

	// Stop clock sync
	if mc.clockSyncStop != nil {
		close(mc.clockSyncStop)
	}

	// Cancel context
	mc.cancel()

	// Stop reader or SerialQueueAdapter
	if mc.reader != nil {
		mc.reader.Stop()
	}
	if mc.sqAdapter != nil {
		mc.sqAdapter.Close()
		mc.sqAdapter = nil
	}

	// Wait for goroutines
	mc.mu.Unlock()
	mc.wg.Wait()
	mc.mu.Lock()

	// Close serial port
	var err error
	if mc.port != nil {
		err = mc.port.Close()
		mc.port = nil
	}

	mc.connected = false
	mc.clockSynced = false

	mc.tracef("MCU %s: Disconnected\n", mc.name)
	return err
}

// IsConnected returns true if the MCU is connected.
func (mc *MCUConnection) IsConnected() bool {
	mc.mu.RLock()
	defer mc.mu.RUnlock()
	return mc.connected
}

// Dictionary returns the MCU's protocol dictionary.
func (mc *MCUConnection) Dictionary() *protocol.Dictionary {
	mc.mu.RLock()
	defer mc.mu.RUnlock()
	return mc.dict
}

// MCUFreq returns the MCU's clock frequency in Hz.
func (mc *MCUConnection) MCUFreq() float64 {
	mc.mu.RLock()
	defer mc.mu.RUnlock()
	return mc.mcuFreq
}

// RegisterHandler registers a handler for a specific message type.
func (mc *MCUConnection) RegisterHandler(name string, handler MCUMessageHandler) {
	mc.handlersMu.Lock()
	defer mc.handlersMu.Unlock()
	mc.handlers[name] = handler
}

// RegisterOIDHandler registers a handler for a specific message type and OID.
func (mc *MCUConnection) RegisterOIDHandler(name string, oid int, handler MCUMessageHandler) {
	mc.handlersMu.Lock()
	defer mc.handlersMu.Unlock()

	if mc.oidHandlers[oid] == nil {
		mc.oidHandlers[oid] = make(map[string]MCUMessageHandler)
	}
	mc.oidHandlers[oid][name] = handler
}

// UnregisterHandler removes a handler for a message type.
func (mc *MCUConnection) UnregisterHandler(name string) {
	mc.handlersMu.Lock()
	defer mc.handlersMu.Unlock()
	delete(mc.handlers, name)
}

// SendCommand sends a command to the MCU without waiting for a response.
func (mc *MCUConnection) SendCommand(cmdName string, params map[string]interface{}) error {
	data, err := mc.encodeCommand(cmdName, params)
	if err != nil {
		return err
	}

	cmd := mcuCmd{
		data: data,
	}

	select {
	case mc.cmdQueue <- cmd:
		return nil
	case <-mc.ctx.Done():
		return ErrMCUNotConnected
	}
}

// SendCommandWithResponse sends a command and waits for a specific response.
func (mc *MCUConnection) SendCommandWithResponse(cmdName string, params map[string]interface{},
	responseName string, timeout time.Duration) (map[string]interface{}, error) {
	return mc.SendCommandWithResponseOID(cmdName, params, responseName, nil, timeout)
}

// SendCommandWithResponseOID sends a command and waits for a response with specific OID.
func (mc *MCUConnection) SendCommandWithResponseOID(cmdName string, params map[string]interface{},
	responseName string, oid *int, timeout time.Duration) (map[string]interface{}, error) {

	data, err := mc.encodeCommand(cmdName, params)
	if err != nil {
		return nil, err
	}

	respChan := make(chan *mcupkg.Message, 1)

	cmd := mcuCmd{
		data:     data,
		response: responseName,
		oid:      oid,
		timeout:  timeout,
		respChan: respChan,
	}

	select {
	case mc.cmdQueue <- cmd:
	case <-mc.ctx.Done():
		return nil, ErrMCUNotConnected
	}

	// Wait for response
	select {
	case msg := <-respChan:
		if msg == nil {
			return nil, ErrMCUTimeout
		}
		return msg.Params, nil
	case <-time.After(timeout):
		return nil, ErrMCUTimeout
	case <-mc.ctx.Done():
		return nil, ErrMCUNotConnected
	}
}

// PrintTime converts host time to MCU clock ticks.
func (mc *MCUConnection) PrintTime(hostTime float64) uint64 {
	mc.clockSyncMu.RLock()
	defer mc.clockSyncMu.RUnlock()

	if !mc.clockSynced {
		return 0
	}

	// Convert host time to MCU clock
	return uint64((hostTime + mc.clockOffset) * mc.clockConv)
}

// EstimatedPrintTime converts current reactor time to estimated print time.
func (mc *MCUConnection) EstimatedPrintTime() float64 {
	mc.clockSyncMu.RLock()
	defer mc.clockSyncMu.RUnlock()

	if !mc.clockSynced {
		return 0
	}

	if mc.reactor != nil {
		return mc.reactor.Monotonic() + mc.clockOffset
	}
	return mc.clockOffset
}

// SetReactor sets the reactor for timing and scheduling.
func (mc *MCUConnection) SetReactor(r *reactor.Reactor) {
	mc.mu.Lock()
	defer mc.mu.Unlock()
	mc.reactor = r
}

// commandSenderLoop sends commands from the queue to the MCU.
func (mc *MCUConnection) commandSenderLoop() {
	defer mc.wg.Done()

	for {
		select {
		case cmd := <-mc.cmdQueue:
			mc.sendCommandDirect(cmd)
		case <-mc.ctx.Done():
			return
		}
	}
}

func (mc *MCUConnection) sendCommandDirect(cmd mcuCmd) {
	if cmd.data == nil {
		return
	}

	// If expecting a response, register pending by name
	if cmd.response != "" && cmd.respChan != nil {
		mc.pendingMu.Lock()
		mc.pendingByName[cmd.response] = pendingResp{
			oid:      cmd.oid,
			respChan: cmd.respChan,
		}
		mc.pendingMu.Unlock()

		// Set up timeout cleanup
		go func() {
			time.Sleep(cmd.timeout)
			mc.pendingMu.Lock()
			delete(mc.pendingByName, cmd.response)
			mc.pendingMu.Unlock()
		}()
	}

	mc.mu.RLock()
	useSerialQueue := mc.config.UseSerialQueue
	sqAdapter := mc.sqAdapter
	port := mc.port
	mc.mu.RUnlock()

	if useSerialQueue && sqAdapter != nil {
		// Send via SerialQueueAdapter (handles msgblock encoding internally)
		if err := sqAdapter.SendImmediate(cmd.data); err != nil {
			mc.tracef("MCU %s: send error: %v\n", mc.name, err)
		}
	} else if port != nil {
		// Traditional send via serial port
		mc.cmdSeqLock.Lock()
		seq := mc.cmdSeq
		mc.cmdSeq = (mc.cmdSeq + 1) & protocol.MESSAGE_SEQ_MASK
		mc.cmdSeqLock.Unlock()

		// Register pending by seq for legacy mode
		if cmd.response != "" && cmd.respChan != nil {
			mc.pendingMu.Lock()
			mc.pending[seq] = cmd.respChan
			mc.pendingMu.Unlock()
		}

		msg := protocol.EncodeMsgblock(int(seq), cmd.data)
		port.Write(msg)
	}
}

// encodeCommand encodes a command with parameters.
func (mc *MCUConnection) encodeCommand(cmdName string, params map[string]interface{}) ([]byte, error) {
	mc.tracef("MCU %s: encodeCommand %s params=%v\n", mc.name, cmdName, params)

	mc.mu.RLock()
	dict := mc.dict
	mc.mu.RUnlock()

	if dict == nil {
		mc.tracef("MCU %s: encodeCommand - no dictionary\n", mc.name)
		return nil, ErrMCUNotConnected
	}

	// Find command format and ID
	var cmdFormat string
	var cmdID int
	for format, id := range dict.Commands {
		name := extractCommandName(format)
		if name == cmdName {
			cmdFormat = format
			cmdID = id
			break
		}
	}

	if cmdFormat == "" {
		mc.tracef("MCU %s: encodeCommand - unknown command %s\n", mc.name, cmdName)
		return nil, fmt.Errorf("mcu: unknown command %s", cmdName)
	}

	mc.tracef("MCU %s: encodeCommand - found format=%s id=%d\n", mc.name, cmdFormat, cmdID)

	// Build command line string
	cmdLine := buildCommandLine(cmdName, cmdFormat, params)
	mc.tracef("MCU %s: encodeCommand - cmdLine=%s\n", mc.name, cmdLine)

	// Use protocol's EncodeCommand which takes command formats map
	// We need to create a temporary formats map with just this command
	mc.tracef("MCU %s: encodeCommand - building message formats\n", mc.name)
	formats, err := buildMessageFormats(dict)
	if err != nil {
		mc.tracef("MCU %s: encodeCommand - buildMessageFormats error: %v\n", mc.name, err)
		return nil, err
	}
	mc.tracef("MCU %s: encodeCommand - encoding with %d formats\n", mc.name, len(formats))

	_ = cmdID // Used in message encoding
	result, err := protocol.EncodeCommand(formats, cmdLine)
	if err != nil {
		mc.tracef("MCU %s: encodeCommand - EncodeCommand error: %v\n", mc.name, err)
		return nil, err
	}
	mc.tracef("MCU %s: encodeCommand - result: %x\n", mc.name, result)
	return result, nil
}

// buildCommandLine builds a command line string from name and params.
func buildCommandLine(cmdName, cmdFormat string, params map[string]interface{}) string {
	if params == nil || len(params) == 0 {
		return cmdName
	}

	var parts []string
	parts = append(parts, cmdName)

	// Parse parameter names from format
	words := strings.Fields(cmdFormat)
	for _, word := range words[1:] {
		eqIdx := strings.Index(word, "=")
		if eqIdx == -1 {
			continue
		}
		paramName := word[:eqIdx]
		if val, ok := params[paramName]; ok {
			parts = append(parts, fmt.Sprintf("%s=%v", paramName, val))
		}
	}

	return strings.Join(parts, " ")
}

// buildMessageFormats creates MessageFormat map from dictionary.
func buildMessageFormats(dict *protocol.Dictionary) (map[string]*protocol.MessageFormat, error) {
	return dict.BuildCommandFormats()
}

// extractCommandName extracts the command name from a format string.
func extractCommandName(format string) string {
	for i, c := range format {
		if c == ' ' {
			return format[:i]
		}
	}
	return format
}

// registerStandardHandlers sets up handlers for standard MCU messages.
func (mc *MCUConnection) registerStandardHandlers() {
	// Shutdown response
	mc.reader.RegisterHandler("shutdown", nil, func(msg *mcupkg.Message) {
		mc.handleShutdown(msg)
	})

	// Clock response for synchronization
	mc.reader.RegisterHandler("clock", nil, func(msg *mcupkg.Message) {
		mc.handleClockResponse(msg)
	})

	// Status message
	mc.reader.RegisterHandler("status", nil, func(msg *mcupkg.Message) {
		mc.handleStatus(msg)
	})

	// Config CRC response
	mc.reader.RegisterHandler("config", nil, func(msg *mcupkg.Message) {
		mc.dispatchMessage(msg)
	})

	// Thermocouple/temperature responses
	mc.reader.RegisterHandler("thermocouple_result", nil, func(msg *mcupkg.Message) {
		mc.dispatchMessage(msg)
	})

	// ADC responses
	mc.reader.RegisterHandler("analog_in_state", nil, func(msg *mcupkg.Message) {
		mc.dispatchMessage(msg)
	})

	// Endstop responses
	mc.reader.RegisterHandler("endstop_state", nil, func(msg *mcupkg.Message) {
		mc.dispatchMessage(msg)
	})
}

// registerSerialQueueHandlers sets up handlers on the SerialQueueAdapter.
func (mc *MCUConnection) registerSerialQueueHandlers() {
	// Set default handler for unknown messages
	mc.sqAdapter.SetDefaultHandler(func(name string, params map[string]interface{}, sentTime, receiveTime float64) {
		mc.tracef("MCU %s: Unknown message %s: %v\n", mc.name, name, params)
		mc.dispatchSQMessage(name, params, receiveTime)
	})

	// Shutdown response
	mc.sqAdapter.RegisterHandler("shutdown", func(name string, params map[string]interface{}, sentTime, receiveTime float64) {
		mc.handleShutdownSQ(params)
	})

	// Clock response for synchronization
	mc.sqAdapter.RegisterHandler("clock", func(name string, params map[string]interface{}, sentTime, receiveTime float64) {
		mc.handleClockResponseSQ(params, receiveTime)
	})

	// Status message
	mc.sqAdapter.RegisterHandler("status", func(name string, params map[string]interface{}, sentTime, receiveTime float64) {
		mc.tracef("MCU %s: Status: %v\n", mc.name, params)
	})

	// Config CRC response
	mc.sqAdapter.RegisterHandler("config", func(name string, params map[string]interface{}, sentTime, receiveTime float64) {
		mc.dispatchSQMessage(name, params, receiveTime)
	})

	// Thermocouple/temperature responses
	mc.sqAdapter.RegisterHandler("thermocouple_result", func(name string, params map[string]interface{}, sentTime, receiveTime float64) {
		mc.dispatchSQMessage(name, params, receiveTime)
	})

	// ADC responses
	mc.sqAdapter.RegisterHandler("analog_in_state", func(name string, params map[string]interface{}, sentTime, receiveTime float64) {
		mc.dispatchSQMessage(name, params, receiveTime)
	})

	// Endstop responses
	mc.sqAdapter.RegisterHandler("endstop_state", func(name string, params map[string]interface{}, sentTime, receiveTime float64) {
		mc.dispatchSQMessage(name, params, receiveTime)
	})
}

// dispatchSQMessage dispatches a message received via SerialQueueAdapter.
func (mc *MCUConnection) dispatchSQMessage(name string, params map[string]interface{}, receiveTime float64) {
	// First, check for pending response waiters
	mc.pendingMu.Lock()
	if pending, ok := mc.pendingByName[name]; ok {
		// Check OID match if specified
		var oidMatch bool
		if pending.oid == nil {
			oidMatch = true
		} else if oid, ok := params["oid"].(int); ok && *pending.oid == oid {
			oidMatch = true
		}
		if oidMatch {
			delete(mc.pendingByName, name)
			mc.pendingMu.Unlock()

			// Create mcupkg.Message for compatibility
			msg := &mcupkg.Message{
				Name:        name,
				Params:      params,
				ReceiveTime: time.Unix(0, int64(receiveTime*1e9)),
			}
			if oid, ok := params["oid"].(int); ok {
				msg.OID = &oid
			}

			select {
			case pending.respChan <- msg:
			default:
			}
			return
		}
	}
	mc.pendingMu.Unlock()

	// Then check handlers
	mc.handlersMu.RLock()
	defer mc.handlersMu.RUnlock()

	recvTime := time.Unix(0, int64(receiveTime*1e9))

	// Check for OID-specific handler first
	if oid, ok := params["oid"].(int); ok {
		if oidHandlers, ok := mc.oidHandlers[oid]; ok {
			if handler, ok := oidHandlers[name]; ok {
				handler(params, recvTime)
				return
			}
		}
	}

	// Check for global handler
	if handler, ok := mc.handlers[name]; ok {
		handler(params, recvTime)
	}
}

// handleShutdownSQ handles MCU shutdown notification via SerialQueueAdapter.
func (mc *MCUConnection) handleShutdownSQ(params map[string]interface{}) {
	mc.mu.Lock()
	defer mc.mu.Unlock()

	mc.shutdown = true
	if reason, ok := params["static_string_id"].(int); ok {
		mc.shutdownReason = fmt.Sprintf("static_string_id=%d", reason)
	}

	mc.tracef("MCU %s: Shutdown received: %s\n", mc.name, mc.shutdownReason)
}

// handleClockResponseSQ handles clock synchronization response via SerialQueueAdapter.
func (mc *MCUConnection) handleClockResponseSQ(params map[string]interface{}, receiveTime float64) {
	clock, ok := params["clock"].(int)
	if !ok {
		return
	}

	mc.clockSyncMu.Lock()
	defer mc.clockSyncMu.Unlock()

	// Record sample
	sample := clockSample{
		hostTime: receiveTime,
		mcuClock: uint64(clock),
	}

	mc.clockSamples = append(mc.clockSamples, sample)
	if len(mc.clockSamples) > mc.config.ClockSyncSamples {
		mc.clockSamples = mc.clockSamples[1:]
	}

	// Update clock offset if we have enough samples
	if len(mc.clockSamples) >= 3 {
		mc.updateClockOffset()
		mc.clockSynced = true
	}
}

// handleUnknownMessage handles messages without specific handlers.
func (mc *MCUConnection) handleUnknownMessage(msg *mcupkg.Message) {
	mc.tracef("MCU %s: Unknown message %s: %v\n", mc.name, msg.Name, msg.Params)
	mc.dispatchMessage(msg)
}

// dispatchMessage dispatches a message to registered handlers.
func (mc *MCUConnection) dispatchMessage(msg *mcupkg.Message) {
	// First, check for pending response waiters
	mc.pendingMu.Lock()
	if pending, ok := mc.pendingByName[msg.Name]; ok {
		// Check OID match if specified
		oidMatch := pending.oid == nil || (msg.OID != nil && *pending.oid == *msg.OID)
		if oidMatch {
			delete(mc.pendingByName, msg.Name)
			mc.pendingMu.Unlock()

			// Send response to waiting channel (non-blocking)
			select {
			case pending.respChan <- msg:
			default:
			}
			return
		}
	}
	mc.pendingMu.Unlock()

	// Then check handlers
	mc.handlersMu.RLock()
	defer mc.handlersMu.RUnlock()

	// Check for OID-specific handler first
	if msg.OID != nil {
		if oidHandlers, ok := mc.oidHandlers[*msg.OID]; ok {
			if handler, ok := oidHandlers[msg.Name]; ok {
				handler(msg.Params, msg.ReceiveTime)
				return
			}
		}
	}

	// Check for global handler
	if handler, ok := mc.handlers[msg.Name]; ok {
		handler(msg.Params, msg.ReceiveTime)
	}
}

// handleShutdown handles MCU shutdown notification.
func (mc *MCUConnection) handleShutdown(msg *mcupkg.Message) {
	mc.mu.Lock()
	defer mc.mu.Unlock()

	mc.shutdown = true
	if reason, ok := msg.Params["static_string_id"].(int); ok {
		mc.shutdownReason = fmt.Sprintf("static_string_id=%d", reason)
	}

	mc.tracef("MCU %s: Shutdown received: %s\n", mc.name, mc.shutdownReason)
}

// handleClockResponse handles clock synchronization response.
func (mc *MCUConnection) handleClockResponse(msg *mcupkg.Message) {
	receiveTime := time.Since(time.Now()).Seconds() // Should use reactor time
	if mc.reactor != nil {
		receiveTime = mc.reactor.Monotonic()
	}

	clock, ok := msg.Params["clock"].(int)
	if !ok {
		return
	}

	mc.clockSyncMu.Lock()
	defer mc.clockSyncMu.Unlock()

	// Record sample
	sample := clockSample{
		hostTime: receiveTime,
		mcuClock: uint64(clock),
	}

	mc.clockSamples = append(mc.clockSamples, sample)
	if len(mc.clockSamples) > mc.config.ClockSyncSamples {
		mc.clockSamples = mc.clockSamples[1:]
	}

	// Update clock offset if we have enough samples
	if len(mc.clockSamples) >= 3 {
		mc.updateClockOffset()
		mc.clockSynced = true
	}
}

// updateClockOffset calculates clock offset from samples.
func (mc *MCUConnection) updateClockOffset() {
	if len(mc.clockSamples) < 2 {
		return
	}

	// Use simple linear regression to estimate clock relationship
	// This is a simplified version - real implementation would use
	// Klipper's more sophisticated clock tracking algorithm

	var sumX, sumY, sumXY, sumXX float64
	n := float64(len(mc.clockSamples))

	for _, s := range mc.clockSamples {
		x := s.hostTime
		y := float64(s.mcuClock) / mc.mcuFreq
		sumX += x
		sumY += y
		sumXY += x * y
		sumXX += x * x
	}

	// Calculate slope and intercept
	slope := (n*sumXY - sumX*sumY) / (n*sumXX - sumX*sumX)
	intercept := (sumY - slope*sumX) / n

	// Clock offset is the difference between host time and MCU time
	mc.clockOffset = intercept
	mc.lastSyncTime = mc.clockSamples[len(mc.clockSamples)-1].hostTime

	mc.tracef("MCU %s: Clock sync - offset=%.6f slope=%.6f\n",
		mc.name, mc.clockOffset, slope)
}

// handleStatus handles MCU status message.
func (mc *MCUConnection) handleStatus(msg *mcupkg.Message) {
	mc.tracef("MCU %s: Status: %v\n", mc.name, msg.Params)
}

// clockSyncLoop periodically synchronizes the clock with the MCU.
func (mc *MCUConnection) clockSyncLoop() {
	defer mc.wg.Done()

	// Initial sync requests - send several quickly
	for i := 0; i < mc.config.ClockSyncSamples; i++ {
		mc.sendClockRequest()
		time.Sleep(10 * time.Millisecond)
	}

	ticker := time.NewTicker(mc.config.ClockSyncInterval)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			mc.sendClockRequest()
		case <-mc.clockSyncStop:
			return
		case <-mc.ctx.Done():
			return
		}
	}
}

// sendClockRequest sends a get_clock command to the MCU.
func (mc *MCUConnection) sendClockRequest() {
	mc.SendCommand("get_clock", nil)
}

// waitForClockSync waits until clock is synchronized.
func (mc *MCUConnection) waitForClockSync(timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		mc.clockSyncMu.RLock()
		synced := mc.clockSynced
		mc.clockSyncMu.RUnlock()

		if synced {
			return nil
		}
		time.Sleep(10 * time.Millisecond)
	}
	return ErrMCUClockNotSynced
}

// tracef writes trace output if tracing is enabled.
func (mc *MCUConnection) tracef(format string, args ...interface{}) {
	if mc.config.Trace != nil {
		fmt.Fprintf(mc.config.Trace, format, args...)
	}
}
