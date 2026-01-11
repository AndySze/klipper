package mcu

import (
	"context"
	"errors"
	"fmt"
	"io"
	"sync"
	"time"

	"klipper-go-migration/pkg/protocol"
	"klipper-go-migration/pkg/serial"
)

// MessageReader errors
var (
	ErrReaderClosed = errors.New("mcu: reader closed")
	ErrNoHandler    = errors.New("mcu: no handler for message")
)

// Message represents a parsed message from the MCU.
type Message struct {
	// Name is the message name (e.g., "identify_response")
	Name string

	// OID is the object ID for object-specific messages (nil if not applicable)
	OID *int

	// Params contains the parsed parameters
	Params map[string]interface{}

	// SentTime is when the command was sent (host time)
	SentTime time.Time

	// ReceiveTime is when the response was received (host time)
	ReceiveTime time.Time

	// RawPayload is the raw message payload
	RawPayload []byte
}

// MessageHandler is called when a message is received.
type MessageHandler func(msg *Message)

// PendingRequest tracks a command waiting for a response.
type PendingRequest struct {
	// ResponseName is the expected response message name
	ResponseName string

	// OID is the expected object ID (nil for any OID)
	OID *int

	// Response channel receives the response
	Response chan *Message

	// SentTime is when the request was sent
	SentTime time.Time

	// Deadline is the timeout for this request
	Deadline time.Time
}

// Reader manages background message reading from an MCU.
type Reader struct {
	mu   sync.RWMutex
	port *serial.Port
	dict *protocol.Dictionary

	// Handlers for different message types
	handlers map[handlerKey]MessageHandler

	// Pending requests waiting for responses
	pending    map[int]*PendingRequest // keyed by sequence number
	pendingMu  sync.Mutex
	nextSeq    int

	// Background goroutine control
	ctx      context.Context
	cancel   context.CancelFunc
	wg       sync.WaitGroup
	closeErr error

	// Read buffer
	readBuf []byte

	// Default handler for unknown messages
	defaultHandler MessageHandler
}

type handlerKey struct {
	name string
	oid  int  // -1 means any OID
}

// NewReader creates a new message reader.
func NewReader(port *serial.Port, dict *protocol.Dictionary) *Reader {
	ctx, cancel := context.WithCancel(context.Background())
	return &Reader{
		port:     port,
		dict:     dict,
		handlers: make(map[handlerKey]MessageHandler),
		pending:  make(map[int]*PendingRequest),
		ctx:      ctx,
		cancel:   cancel,
		readBuf:  make([]byte, 4096),
		defaultHandler: func(msg *Message) {
			// Default: log unknown messages
		},
	}
}

// Start begins the background message reading goroutine.
func (r *Reader) Start() {
	r.wg.Add(1)
	go r.readLoop()
}

// Stop stops the reader and waits for the background goroutine to exit.
func (r *Reader) Stop() error {
	r.cancel()
	r.wg.Wait()
	return r.closeErr
}

// RegisterHandler registers a handler for a specific message type.
// If oid is nil, the handler receives messages with any OID.
func (r *Reader) RegisterHandler(name string, oid *int, handler MessageHandler) {
	r.mu.Lock()
	defer r.mu.Unlock()

	key := handlerKey{name: name, oid: -1}
	if oid != nil {
		key.oid = *oid
	}

	if handler == nil {
		delete(r.handlers, key)
	} else {
		r.handlers[key] = handler
	}
}

// UnregisterHandler removes a handler for a message type.
func (r *Reader) UnregisterHandler(name string, oid *int) {
	r.RegisterHandler(name, oid, nil)
}

// SetDefaultHandler sets the handler for unhandled messages.
func (r *Reader) SetDefaultHandler(handler MessageHandler) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.defaultHandler = handler
}

// SendCommand sends a command and returns immediately.
// The command should be a raw encoded command (not including message block framing).
func (r *Reader) SendCommand(cmd []byte) error {
	r.pendingMu.Lock()
	seq := r.nextSeq
	r.nextSeq = (r.nextSeq + 1) & protocol.MESSAGE_SEQ_MASK
	r.pendingMu.Unlock()

	msg := protocol.EncodeMsgblock(seq, cmd)
	_, err := r.port.Write(msg)
	return err
}

// SendWithResponse sends a command and waits for a specific response.
func (r *Reader) SendWithResponse(cmd []byte, responseName string, timeout time.Duration) (*Message, error) {
	return r.SendWithResponseOID(cmd, responseName, nil, timeout)
}

// SendWithResponseOID sends a command and waits for a specific response with OID.
func (r *Reader) SendWithResponseOID(cmd []byte, responseName string, oid *int, timeout time.Duration) (*Message, error) {
	r.pendingMu.Lock()
	seq := r.nextSeq
	r.nextSeq = (r.nextSeq + 1) & protocol.MESSAGE_SEQ_MASK

	req := &PendingRequest{
		ResponseName: responseName,
		OID:          oid,
		Response:     make(chan *Message, 1),
		SentTime:     time.Now(),
		Deadline:     time.Now().Add(timeout),
	}
	r.pending[seq] = req
	r.pendingMu.Unlock()

	// Clean up on exit
	defer func() {
		r.pendingMu.Lock()
		delete(r.pending, seq)
		r.pendingMu.Unlock()
	}()

	// Send the command
	msg := protocol.EncodeMsgblock(seq, cmd)
	if _, err := r.port.Write(msg); err != nil {
		return nil, fmt.Errorf("mcu: send command: %w", err)
	}

	// Wait for response
	select {
	case resp := <-req.Response:
		return resp, nil
	case <-time.After(timeout):
		return nil, ErrTimeout
	case <-r.ctx.Done():
		return nil, ErrReaderClosed
	}
}

// readLoop is the background goroutine that reads messages.
func (r *Reader) readLoop() {
	defer r.wg.Done()

	var buffer []byte

	for {
		select {
		case <-r.ctx.Done():
			return
		default:
		}

		// Read from port
		r.port.SetReadTimeout(100 * time.Millisecond)
		n, err := r.port.Read(r.readBuf)
		if err != nil {
			if errors.Is(err, serial.ErrTimeout) {
				continue
			}
			if errors.Is(err, io.EOF) || errors.Is(err, serial.ErrClosed) {
				r.closeErr = err
				return
			}
			// Log error but continue
			continue
		}

		if n == 0 {
			continue
		}

		// Append to buffer
		buffer = append(buffer, r.readBuf[:n]...)

		// Process complete messages
		for len(buffer) >= protocol.MESSAGE_MIN {
			msgLen := r.checkMessage(buffer)
			if msgLen == 0 {
				// Need more data
				break
			}
			if msgLen < 0 {
				// Invalid message, try to resync
				buffer = r.resync(buffer)
				continue
			}

			// Extract and process message
			msgData := buffer[:msgLen]
			buffer = buffer[msgLen:]

			r.processMessage(msgData)
		}
	}
}

// checkMessage checks if we have a valid message at the start of buffer.
// Returns: message length if valid, 0 if need more data, -1 if invalid.
func (r *Reader) checkMessage(buf []byte) int {
	if len(buf) < protocol.MESSAGE_MIN {
		return 0
	}

	msgLen := int(buf[protocol.MESSAGE_POS_LEN])
	if msgLen < protocol.MESSAGE_MIN || msgLen > protocol.MESSAGE_MAX {
		return -1
	}

	if len(buf) < msgLen {
		return 0 // Need more data
	}

	// Check sequence byte format
	msgSeq := buf[protocol.MESSAGE_POS_SEQ]
	if (msgSeq & ^byte(protocol.MESSAGE_SEQ_MASK)) != protocol.MESSAGE_DEST {
		return -1
	}

	// Check sync byte
	if buf[msgLen-1] != protocol.MESSAGE_SYNC {
		return -1
	}

	// Check CRC
	crcHi, crcLo := protocol.CRC16CCITT(buf[:msgLen-protocol.MESSAGE_TRAILER_SIZE])
	if buf[msgLen-3] != crcHi || buf[msgLen-2] != crcLo {
		return -1
	}

	return msgLen
}

// resync attempts to find the next valid message start.
func (r *Reader) resync(buf []byte) []byte {
	// Look for a sync byte followed by a valid message start
	for i := 1; i < len(buf); i++ {
		if buf[i] >= protocol.MESSAGE_MIN && buf[i] <= protocol.MESSAGE_MAX {
			// Possible message start
			return buf[i:]
		}
	}
	// No valid start found, discard all
	return nil
}

// processMessage parses and dispatches a complete message.
func (r *Reader) processMessage(data []byte) {
	receiveTime := time.Now()

	// Parse the message
	payload := data[protocol.MESSAGE_HEADER_SIZE : len(data)-protocol.MESSAGE_TRAILER_SIZE]
	if len(payload) == 0 {
		return
	}

	// Decode message ID
	msgID, pos := protocol.DecodeUint32(payload, 0)

	// Find the message format
	var msgName string
	var msgParams map[string]interface{}

	// Check responses first
	for format, id := range r.dict.Responses {
		if id == int(msgID) {
			msgName = extractMsgName(format)
			msgParams = r.parseParams(format, payload, pos)
			break
		}
	}

	// Check output messages
	if msgName == "" {
		for format, id := range r.dict.Output {
			if id == int(msgID) {
				msgName = extractMsgName(format)
				msgParams = r.parseParams(format, payload, pos)
				break
			}
		}
	}

	if msgName == "" {
		// Unknown message
		msgName = "#unknown"
		msgParams = map[string]interface{}{
			"#msgid": int(msgID),
			"#msg":   payload,
		}
	}

	msg := &Message{
		Name:        msgName,
		Params:      msgParams,
		ReceiveTime: receiveTime,
		RawPayload:  payload,
	}

	// Extract OID if present
	if oid, ok := msgParams["oid"].(int); ok {
		msg.OID = &oid
	}

	// Check for pending requests first
	r.pendingMu.Lock()
	for seq, req := range r.pending {
		if req.ResponseName == msgName {
			if req.OID == nil || (msg.OID != nil && *req.OID == *msg.OID) {
				msg.SentTime = req.SentTime
				select {
				case req.Response <- msg:
				default:
				}
				delete(r.pending, seq)
				r.pendingMu.Unlock()
				return
			}
		}
	}
	r.pendingMu.Unlock()

	// Find registered handler
	r.mu.RLock()
	oidVal := -1
	if msg.OID != nil {
		oidVal = *msg.OID
	}

	// Try specific OID handler first
	handler, ok := r.handlers[handlerKey{name: msgName, oid: oidVal}]
	if !ok {
		// Try wildcard OID handler
		handler, ok = r.handlers[handlerKey{name: msgName, oid: -1}]
	}
	if !ok {
		handler = r.defaultHandler
	}
	r.mu.RUnlock()

	if handler != nil {
		handler(msg)
	}
}

// extractMsgName extracts the message name from a format string.
func extractMsgName(format string) string {
	for i, c := range format {
		if c == ' ' {
			return format[:i]
		}
	}
	return format
}

// parseParams parses message parameters from payload.
func (r *Reader) parseParams(format string, payload []byte, pos int) map[string]interface{} {
	params := make(map[string]interface{})

	// Split format into words, skip the first (message name)
	words := splitWords(format)
	if len(words) <= 1 {
		return params
	}

	for _, word := range words[1:] {
		// Each word should be "name=type"
		eqIdx := -1
		for i, c := range word {
			if c == '=' {
				eqIdx = i
				break
			}
		}
		if eqIdx == -1 {
			continue
		}

		name := word[:eqIdx]
		typeSpec := word[eqIdx+1:]

		// Decode value based on type
		switch typeSpec {
		case "%u", "%i", "%hu", "%hi", "%c":
			if pos < len(payload) {
				val, newPos := protocol.DecodeUint32(payload, pos)
				params[name] = int(val)
				pos = newPos
			}
		case "%s", "%.*s", "%*s":
			if pos < len(payload) {
				length := int(payload[pos])
				pos++
				end := pos + length
				if end > len(payload) {
					end = len(payload)
				}
				params[name] = payload[pos:end]
				pos = end
			}
		}
	}

	return params
}

// splitWords splits a string by spaces.
func splitWords(s string) []string {
	var words []string
	var start int
	inWord := false

	for i, c := range s {
		if c == ' ' {
			if inWord {
				words = append(words, s[start:i])
				inWord = false
			}
		} else {
			if !inWord {
				start = i
				inWord = true
			}
		}
	}
	if inWord {
		words = append(words, s[start:])
	}
	return words
}
