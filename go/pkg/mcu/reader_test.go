package mcu

import (
	"testing"

	"klipper-go-migration/pkg/protocol"
)

func TestCheckMessage(t *testing.T) {
	// Create a mock dictionary
	dict := &protocol.Dictionary{
		Commands:     make(map[string]int),
		Responses:    make(map[string]int),
		Output:       make(map[string]int),
		Enumerations: make(map[string]map[string]int),
		Config:       make(map[string]interface{}),
	}

	r := NewReader(nil, dict)

	tests := []struct {
		name     string
		input    []byte
		expected int // 0 = need more, -1 = invalid, >0 = message length
	}{
		{
			name:     "empty buffer",
			input:    []byte{},
			expected: 0,
		},
		{
			name:     "partial message",
			input:    []byte{5, 0x10}, // length=5, seq=0x10, need more
			expected: 0,
		},
		{
			name:     "invalid length too small",
			input:    []byte{3, 0x10, 0, 0, 0x7e},
			expected: -1,
		},
		{
			name:     "invalid length too large",
			input:    []byte{70, 0x10, 0, 0, 0x7e},
			expected: -1,
		},
		{
			name:     "invalid seq byte",
			input:    []byte{5, 0x00, 0, 0, 0x7e}, // seq should have MESSAGE_DEST set
			expected: -1,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := r.checkMessage(tt.input)
			if result != tt.expected {
				t.Errorf("checkMessage(%v) = %d, want %d", tt.input, result, tt.expected)
			}
		})
	}
}

func TestCheckMessageValidMessage(t *testing.T) {
	dict := &protocol.Dictionary{
		Commands:     make(map[string]int),
		Responses:    make(map[string]int),
		Output:       make(map[string]int),
		Enumerations: make(map[string]map[string]int),
		Config:       make(map[string]interface{}),
	}

	r := NewReader(nil, dict)

	// Create a valid message using protocol.EncodeMsgblock
	payload := []byte{0x00} // minimal payload
	msg := protocol.EncodeMsgblock(0, payload)

	result := r.checkMessage(msg)
	if result != len(msg) {
		t.Errorf("checkMessage(valid msg) = %d, want %d", result, len(msg))
	}
}

func TestResync(t *testing.T) {
	dict := &protocol.Dictionary{
		Commands:     make(map[string]int),
		Responses:    make(map[string]int),
		Output:       make(map[string]int),
		Enumerations: make(map[string]map[string]int),
		Config:       make(map[string]interface{}),
	}

	r := NewReader(nil, dict)

	tests := []struct {
		name      string
		input     []byte
		expectLen int // expected length of result, -1 for nil
	}{
		{
			name:      "garbage followed by valid length",
			input:     []byte{0xFF, 0xFF, 5, 0x10}, // 5 is a valid message length
			expectLen: 2,                           // should skip to index 2
		},
		{
			name:      "all garbage",
			input:     []byte{0xFF, 0xFF, 0xFF},
			expectLen: -1, // should return nil
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			result := r.resync(tt.input)
			if tt.expectLen == -1 {
				if result != nil {
					t.Errorf("resync(%v) = %v, want nil", tt.input, result)
				}
			} else {
				if len(result) != tt.expectLen {
					t.Errorf("resync(%v) len = %d, want %d", tt.input, len(result), tt.expectLen)
				}
			}
		})
	}
}

func TestExtractMsgName(t *testing.T) {
	tests := []struct {
		format   string
		expected string
	}{
		{"identify_response offset=%u data=%.*s", "identify_response"},
		{"get_clock", "get_clock"},
		{"config_endstop oid=%c pin=%c pull_up=%c", "config_endstop"},
	}

	for _, tt := range tests {
		t.Run(tt.format, func(t *testing.T) {
			result := extractMsgName(tt.format)
			if result != tt.expected {
				t.Errorf("extractMsgName(%q) = %q, want %q", tt.format, result, tt.expected)
			}
		})
	}
}

func TestParseParams(t *testing.T) {
	dict := &protocol.Dictionary{
		Commands:     make(map[string]int),
		Responses:    make(map[string]int),
		Output:       make(map[string]int),
		Enumerations: make(map[string]map[string]int),
		Config:       make(map[string]interface{}),
	}

	r := NewReader(nil, dict)

	// Test parsing "identify_response offset=%u data=%.*s"
	format := "identify_response offset=%u data=%.*s"

	// Create payload: offset=0, data="test"
	var payload []byte
	protocol.EncodeUint32(&payload, 0) // offset = 0
	payload = append(payload, 4)       // data length
	payload = append(payload, []byte("test")...)

	params := r.parseParams(format, payload, 0)

	if offset, ok := params["offset"].(int); !ok || offset != 0 {
		t.Errorf("params['offset'] = %v, want 0", params["offset"])
	}

	if data, ok := params["data"].([]byte); !ok || string(data) != "test" {
		t.Errorf("params['data'] = %v, want 'test'", params["data"])
	}
}

func TestNewReader(t *testing.T) {
	dict := &protocol.Dictionary{
		Commands:     make(map[string]int),
		Responses:    make(map[string]int),
		Output:       make(map[string]int),
		Enumerations: make(map[string]map[string]int),
		Config:       make(map[string]interface{}),
	}

	r := NewReader(nil, dict)

	if r == nil {
		t.Fatal("NewReader returned nil")
	}

	if r.handlers == nil {
		t.Error("handlers map not initialized")
	}

	if r.pending == nil {
		t.Error("pending map not initialized")
	}

	if r.ctx == nil {
		t.Error("context not initialized")
	}
}

func TestRegisterHandler(t *testing.T) {
	dict := &protocol.Dictionary{
		Commands:     make(map[string]int),
		Responses:    make(map[string]int),
		Output:       make(map[string]int),
		Enumerations: make(map[string]map[string]int),
		Config:       make(map[string]interface{}),
	}

	r := NewReader(nil, dict)

	called := false
	handler := func(msg *Message) {
		called = true
	}

	// Register handler
	r.RegisterHandler("test_msg", nil, handler)

	// Verify handler is registered
	r.mu.RLock()
	key := handlerKey{name: "test_msg", oid: -1}
	h, ok := r.handlers[key]
	r.mu.RUnlock()

	if !ok {
		t.Error("Handler not registered")
	}

	// Call handler
	h(&Message{})
	if !called {
		t.Error("Handler was not called")
	}

	// Unregister handler
	r.UnregisterHandler("test_msg", nil)

	r.mu.RLock()
	_, ok = r.handlers[key]
	r.mu.RUnlock()

	if ok {
		t.Error("Handler should have been unregistered")
	}
}
