// Package chelper provides CGO bindings to Klipper's C helper library.
//
// SerialQueueAdapter provides a Go-friendly interface to serialqueue,
// bridging the C-based message transport to Go's handler dispatch system.
package chelper

import (
	"errors"
	"fmt"
	"sync"
	"sync/atomic"

	"klipper-go-migration/pkg/protocol"
)

// Common errors for SerialQueueAdapter
var (
	ErrAdapterClosed     = errors.New("serialqueue adapter: closed")
	ErrAdapterNotStarted = errors.New("serialqueue adapter: not started")
)

// MessageHandler is called when a message is received from the MCU.
// The handler receives the decoded message name, parameters, and timing info.
type MessageHandler func(name string, params map[string]interface{}, sentTime, receiveTime float64)

// SerialQueueAdapter wraps chelper's SerialQueue to provide Go-friendly
// message send/receive with automatic decoding and handler dispatch.
type SerialQueueAdapter struct {
	mu sync.RWMutex

	sq   *SerialQueue
	cq   *CommandQueue
	dict *protocol.Dictionary

	// Message formats for decoding
	responseFormats map[int]*protocol.MessageFormat

	// Handler dispatch
	handlers      map[string]MessageHandler
	defaultHandler MessageHandler
	handlersMu    sync.RWMutex

	// Receive loop control
	running   atomic.Bool
	stopChan  chan struct{}
	stoppedCh chan struct{}

	// Stats
	messagesReceived uint64
	messagesSent     uint64
	decodeErrors     uint64
}

// NewSerialQueueAdapter creates a new adapter wrapping an existing SerialQueue.
// The dictionary is used for decoding received messages.
func NewSerialQueueAdapter(sq *SerialQueue, dict *protocol.Dictionary) (*SerialQueueAdapter, error) {
	if sq == nil {
		return nil, errors.New("serialqueue is nil")
	}
	if dict == nil {
		return nil, errors.New("dictionary is nil")
	}

	// Build response formats for decoding
	responseFormats, err := dict.BuildResponseFormats()
	if err != nil {
		return nil, fmt.Errorf("build response formats: %w", err)
	}

	cq := NewCommandQueue()
	if cq == nil {
		return nil, errors.New("failed to allocate command queue")
	}

	return &SerialQueueAdapter{
		sq:              sq,
		cq:              cq,
		dict:            dict,
		responseFormats: responseFormats,
		handlers:        make(map[string]MessageHandler),
		stopChan:        make(chan struct{}),
		stoppedCh:       make(chan struct{}),
	}, nil
}

// RegisterHandler registers a handler for a specific message type.
func (a *SerialQueueAdapter) RegisterHandler(name string, handler MessageHandler) {
	a.handlersMu.Lock()
	defer a.handlersMu.Unlock()
	a.handlers[name] = handler
}

// UnregisterHandler removes a handler for a message type.
func (a *SerialQueueAdapter) UnregisterHandler(name string) {
	a.handlersMu.Lock()
	defer a.handlersMu.Unlock()
	delete(a.handlers, name)
}

// SetDefaultHandler sets the handler for messages without specific handlers.
func (a *SerialQueueAdapter) SetDefaultHandler(handler MessageHandler) {
	a.handlersMu.Lock()
	defer a.handlersMu.Unlock()
	a.defaultHandler = handler
}

// Start begins the message receive loop in a background goroutine.
func (a *SerialQueueAdapter) Start() error {
	if a.running.Load() {
		return nil // Already running
	}

	a.mu.Lock()
	a.stopChan = make(chan struct{})
	a.stoppedCh = make(chan struct{})
	a.mu.Unlock()

	a.running.Store(true)
	go a.receiveLoop()
	return nil
}

// Stop signals the receive loop to stop and waits for it to finish.
func (a *SerialQueueAdapter) Stop() {
	if !a.running.Load() {
		return
	}

	a.mu.RLock()
	stopChan := a.stopChan
	stoppedCh := a.stoppedCh
	a.mu.RUnlock()

	// Signal serialqueue to exit (unblocks Pull)
	a.sq.Exit()

	// Close stop channel to signal loop
	close(stopChan)

	// Wait for loop to finish
	<-stoppedCh

	a.running.Store(false)
}

// Send sends a pre-encoded command to the MCU.
// minClock and reqClock are MCU clock values for timing.
func (a *SerialQueueAdapter) Send(data []byte, minClock, reqClock uint64) error {
	if !a.running.Load() {
		return ErrAdapterNotStarted
	}

	err := a.sq.Send(a.cq, data, minClock, reqClock)
	if err != nil {
		return err
	}

	atomic.AddUint64(&a.messagesSent, 1)
	return nil
}

// SendImmediate sends a command with no clock constraints.
func (a *SerialQueueAdapter) SendImmediate(data []byte) error {
	return a.Send(data, 0, 0)
}

// SetClockEst updates the clock estimation parameters.
func (a *SerialQueueAdapter) SetClockEst(estFreq, convTime float64, convClock, lastClock uint64) {
	a.sq.SetClockEst(estFreq, convTime, convClock, lastClock)
}

// GetClockEst retrieves the current clock estimation.
func (a *SerialQueueAdapter) GetClockEst() (ClockEstimate, error) {
	return a.sq.GetClockEst()
}

// SetReceiveWindow sets the receive window for flow control.
func (a *SerialQueueAdapter) SetReceiveWindow(window int) {
	a.sq.SetReceiveWindow(window)
}

// SetWireFrequency sets the baud rate for timing calculations.
func (a *SerialQueueAdapter) SetWireFrequency(freq float64) {
	a.sq.SetWireFrequency(freq)
}

// GetStats returns statistics about the adapter.
func (a *SerialQueueAdapter) GetStats() (AdapterStats, error) {
	sqStats, err := a.sq.GetStats()
	if err != nil {
		return AdapterStats{}, err
	}

	return AdapterStats{
		MessagesReceived: atomic.LoadUint64(&a.messagesReceived),
		MessagesSent:     atomic.LoadUint64(&a.messagesSent),
		DecodeErrors:     atomic.LoadUint64(&a.decodeErrors),
		SerialStats:      sqStats,
	}, nil
}

// AdapterStats holds statistics for the adapter.
type AdapterStats struct {
	MessagesReceived uint64
	MessagesSent     uint64
	DecodeErrors     uint64
	SerialStats      SerialStats
}

// Close stops the adapter and frees resources.
func (a *SerialQueueAdapter) Close() {
	a.Stop()

	a.mu.Lock()
	defer a.mu.Unlock()

	if a.cq != nil {
		a.cq.Free()
		a.cq = nil
	}
	// Note: We don't free sq here as it may be shared/owned elsewhere
}

// receiveLoop pulls messages from serialqueue and dispatches them.
func (a *SerialQueueAdapter) receiveLoop() {
	defer close(a.stoppedCh)

	for {
		select {
		case <-a.stopChan:
			return
		default:
		}

		// Pull next message (blocks until available or Exit called)
		pqm, err := a.sq.Pull()
		if err != nil {
			// Check if we should stop
			select {
			case <-a.stopChan:
				return
			default:
				continue
			}
		}

		// Empty message indicates exit was called
		if pqm.Len == 0 {
			select {
			case <-a.stopChan:
				return
			default:
				continue
			}
		}

		atomic.AddUint64(&a.messagesReceived, 1)

		// Decode and dispatch the message
		a.decodeAndDispatch(pqm)
	}
}

// decodeAndDispatch decodes a raw message and calls the appropriate handler.
func (a *SerialQueueAdapter) decodeAndDispatch(pqm PullQueueMessage) {
	// Extract message data (skip msgblock header if present)
	data := pqm.Msg[:pqm.Len]

	// The data from serialqueue_pull is already unwrapped from msgblock
	// It contains: [cmd_id] [params...]
	if len(data) < 1 {
		atomic.AddUint64(&a.decodeErrors, 1)
		return
	}

	// Decode the message using protocol
	name, params, err := a.decodeMessage(data)
	if err != nil {
		atomic.AddUint64(&a.decodeErrors, 1)
		return
	}

	// Dispatch to handler
	a.handlersMu.RLock()
	handler, ok := a.handlers[name]
	defaultHandler := a.defaultHandler
	a.handlersMu.RUnlock()

	if ok {
		handler(name, params, pqm.SentTime, pqm.ReceiveTime)
	} else if defaultHandler != nil {
		defaultHandler(name, params, pqm.SentTime, pqm.ReceiveTime)
	}
}

// decodeMessage decodes a raw message into name and parameters.
func (a *SerialQueueAdapter) decodeMessage(data []byte) (string, map[string]interface{}, error) {
	if len(data) < 1 {
		return "", nil, errors.New("message too short")
	}

	// First byte(s) is command ID (VLQ encoded)
	cmdID, consumed := protocol.DecodeVLQ(data)
	if consumed <= 0 {
		return "", nil, errors.New("invalid command ID encoding")
	}

	// Look up the response format
	format, ok := a.responseFormats[int(cmdID)]
	if !ok {
		return "", nil, fmt.Errorf("unknown response ID: %d", cmdID)
	}

	// Decode parameters
	params, err := protocol.DecodeParams(format, data[consumed:])
	if err != nil {
		return "", nil, fmt.Errorf("decode params: %w", err)
	}

	return format.Name, params, nil
}

// SerialQueue returns the underlying SerialQueue.
func (a *SerialQueueAdapter) SerialQueue() *SerialQueue {
	return a.sq
}

// Dictionary returns the protocol dictionary.
func (a *SerialQueueAdapter) Dictionary() *protocol.Dictionary {
	return a.dict
}
