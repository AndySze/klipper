// Package mcu provides MCU communication and handshake protocol.
package mcu

import (
	"bytes"
	"compress/zlib"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"time"

	"klipper-go-migration/pkg/protocol"
	"klipper-go-migration/pkg/serial"
)

// Common errors
var (
	ErrTimeout          = errors.New("mcu: handshake timeout")
	ErrDictionaryMismatch = errors.New("mcu: dictionary CRC mismatch")
	ErrInvalidResponse  = errors.New("mcu: invalid identify response")
	ErrNoResponse       = errors.New("mcu: no response from MCU")
)

// HandshakeConfig holds configuration for the handshake process.
type HandshakeConfig struct {
	// Timeout for the entire handshake process (default: 5 seconds)
	Timeout time.Duration

	// ReadChunkSize for identify data retrieval (default: 40)
	ReadChunkSize int

	// MaxRetries for command retries (default: 5)
	MaxRetries int

	// RetryDelay initial delay between retries (default: 10ms, doubles each retry)
	RetryDelay time.Duration

	// ExpectedCRC if set, verify dictionary CRC matches
	ExpectedCRC uint32
}

// DefaultHandshakeConfig returns a HandshakeConfig with default values.
func DefaultHandshakeConfig() HandshakeConfig {
	return HandshakeConfig{
		Timeout:       5 * time.Second,
		ReadChunkSize: 40,
		MaxRetries:    5,
		RetryDelay:    10 * time.Millisecond,
	}
}

// IdentifyData holds the raw and parsed identify response.
type IdentifyData struct {
	// RawCompressed is the compressed identify data from MCU
	RawCompressed []byte

	// RawJSON is the decompressed JSON dictionary
	RawJSON []byte

	// Dictionary is the parsed dictionary
	Dictionary *protocol.Dictionary

	// Version is the firmware version string
	Version string

	// BuildVersions contains build version info
	BuildVersions string
}

// Handshake performs the MCU identification handshake.
// It sends identify commands, retrieves the dictionary, and verifies it.
func Handshake(port *serial.Port, cfg HandshakeConfig) (*IdentifyData, error) {
	if cfg.Timeout == 0 {
		cfg.Timeout = 5 * time.Second
	}
	if cfg.ReadChunkSize == 0 {
		cfg.ReadChunkSize = 40
	}
	if cfg.MaxRetries == 0 {
		cfg.MaxRetries = 5
	}
	if cfg.RetryDelay == 0 {
		cfg.RetryDelay = 10 * time.Millisecond
	}

	deadline := time.Now().Add(cfg.Timeout)

	// Collect identify data by querying chunks
	var identifyData []byte
	seq := 0

	for time.Now().Before(deadline) {
		offset := len(identifyData)
		cmd := encodeIdentifyCommand(offset, cfg.ReadChunkSize)
		msgBlock := protocol.EncodeMsgblock(seq, cmd)
		seq = (seq + 1) & protocol.MESSAGE_SEQ_MASK

		resp, err := sendWithRetry(port, msgBlock, cfg.MaxRetries, cfg.RetryDelay, deadline)
		if err != nil {
			if errors.Is(err, ErrTimeout) && len(identifyData) == 0 {
				// No response at all - MCU might not be connected
				return nil, ErrNoResponse
			}
			return nil, fmt.Errorf("mcu: identify failed: %w", err)
		}

		// Parse identify_response
		respOffset, data, err := parseIdentifyResponse(resp)
		if err != nil {
			return nil, fmt.Errorf("mcu: parse identify response: %w", err)
		}

		// Verify offset matches what we requested
		if respOffset != offset {
			return nil, fmt.Errorf("mcu: identify offset mismatch: got %d, expected %d", respOffset, offset)
		}

		// Empty data means we're done
		if len(data) == 0 {
			break
		}

		identifyData = append(identifyData, data...)
	}

	if len(identifyData) == 0 {
		return nil, ErrNoResponse
	}

	// Decompress the identify data
	jsonData, err := decompressZlib(identifyData)
	if err != nil {
		return nil, fmt.Errorf("mcu: decompress identify data: %w", err)
	}

	// Parse the dictionary JSON
	dict, err := parseDictionaryJSON(jsonData)
	if err != nil {
		return nil, fmt.Errorf("mcu: parse dictionary: %w", err)
	}

	// Extract version info
	version := ""
	buildVersions := ""
	if v, ok := dict.Config["version"].(string); ok {
		version = v
	}
	if v, ok := dict.Config["build_versions"].(string); ok {
		buildVersions = v
	}

	// Verify CRC if expected
	if cfg.ExpectedCRC != 0 {
		// Calculate CRC of the compressed data
		crc := calculateDictCRC(identifyData)
		if crc != cfg.ExpectedCRC {
			return nil, fmt.Errorf("%w: got 0x%08x, expected 0x%08x", ErrDictionaryMismatch, crc, cfg.ExpectedCRC)
		}
	}

	return &IdentifyData{
		RawCompressed: identifyData,
		RawJSON:       jsonData,
		Dictionary:    dict,
		Version:       version,
		BuildVersions: buildVersions,
	}, nil
}

// encodeIdentifyCommand creates the "identify offset=%u count=%c" command.
// Message ID 1 is the identify command (from DefaultMessages in msgproto.py).
func encodeIdentifyCommand(offset, count int) []byte {
	var cmd []byte
	// Message ID 1 for identify command
	protocol.EncodeUint32(&cmd, 1)
	// offset=%u
	protocol.EncodeUint32(&cmd, int32(offset))
	// count=%c (single byte, but VLQ encoded)
	protocol.EncodeUint32(&cmd, int32(count))
	return cmd
}

// parseIdentifyResponse parses an identify_response message.
// Format: "identify_response offset=%u data=%.*s" with message ID 0
func parseIdentifyResponse(msg []byte) (offset int, data []byte, err error) {
	if len(msg) < protocol.MESSAGE_MIN {
		return 0, nil, ErrInvalidResponse
	}

	// Verify message structure
	msgLen := int(msg[protocol.MESSAGE_POS_LEN])
	if msgLen < protocol.MESSAGE_MIN || msgLen > len(msg) {
		return 0, nil, ErrInvalidResponse
	}

	// Check CRC
	crcHi, crcLo := protocol.CRC16CCITT(msg[:msgLen-protocol.MESSAGE_TRAILER_SIZE])
	if msg[msgLen-3] != crcHi || msg[msgLen-2] != crcLo {
		return 0, nil, fmt.Errorf("mcu: CRC mismatch in response")
	}

	// Check sync byte
	if msg[msgLen-1] != protocol.MESSAGE_SYNC {
		return 0, nil, fmt.Errorf("mcu: missing sync byte")
	}

	// Parse payload (skip header: len + seq)
	payload := msg[protocol.MESSAGE_HEADER_SIZE : msgLen-protocol.MESSAGE_TRAILER_SIZE]
	if len(payload) < 1 {
		return 0, nil, ErrInvalidResponse
	}

	// Parse message ID (should be 0 for identify_response)
	msgID, pos := protocol.DecodeUint32(payload, 0)
	if msgID != 0 {
		return 0, nil, fmt.Errorf("mcu: unexpected message ID %d, expected 0", msgID)
	}

	// Parse offset
	offsetVal, pos := protocol.DecodeUint32(payload, pos)
	offset = int(offsetVal)

	// Parse data (length-prefixed buffer)
	if pos >= len(payload) {
		return offset, nil, nil // Empty data is valid
	}
	dataLen := int(payload[pos])
	pos++
	if pos+dataLen > len(payload) {
		return 0, nil, fmt.Errorf("mcu: data length exceeds payload")
	}
	data = payload[pos : pos+dataLen]

	return offset, data, nil
}

// sendWithRetry sends a command and waits for response with retries.
func sendWithRetry(port *serial.Port, cmd []byte, maxRetries int, retryDelay time.Duration, deadline time.Time) ([]byte, error) {
	delay := retryDelay
	var lastErr error

	for retry := 0; retry <= maxRetries; retry++ {
		if time.Now().After(deadline) {
			return nil, ErrTimeout
		}

		// Send command
		_, err := port.Write(cmd)
		if err != nil {
			lastErr = err
			continue
		}

		// Read response
		resp := make([]byte, protocol.MESSAGE_MAX)
		port.SetReadTimeout(500 * time.Millisecond)
		n, err := port.Read(resp)
		if err != nil {
			if errors.Is(err, serial.ErrTimeout) {
				lastErr = ErrTimeout
				// Retry with exponential backoff
				if retry < maxRetries {
					time.Sleep(delay)
					delay *= 2
				}
				continue
			}
			lastErr = err
			continue
		}

		if n > 0 {
			return resp[:n], nil
		}

		lastErr = ErrNoResponse
		if retry < maxRetries {
			time.Sleep(delay)
			delay *= 2
		}
	}

	if lastErr != nil {
		return nil, lastErr
	}
	return nil, ErrTimeout
}

// decompressZlib decompresses zlib-compressed data.
func decompressZlib(data []byte) ([]byte, error) {
	r, err := zlib.NewReader(bytes.NewReader(data))
	if err != nil {
		return nil, err
	}
	defer r.Close()

	return io.ReadAll(r)
}

// parseDictionaryJSON parses the dictionary from JSON.
func parseDictionaryJSON(data []byte) (*protocol.Dictionary, error) {
	var raw map[string]interface{}
	if err := json.Unmarshal(data, &raw); err != nil {
		return nil, err
	}

	dict := &protocol.Dictionary{
		Commands:     make(map[string]int),
		Responses:    make(map[string]int),
		Output:       make(map[string]int),
		Enumerations: make(map[string]map[string]int),
		Config:       make(map[string]interface{}),
	}

	// Parse commands
	if cmds, ok := raw["commands"].(map[string]interface{}); ok {
		for k, v := range cmds {
			if id, ok := v.(float64); ok {
				dict.Commands[k] = int(id)
			}
		}
	}

	// Parse responses
	if resps, ok := raw["responses"].(map[string]interface{}); ok {
		for k, v := range resps {
			if id, ok := v.(float64); ok {
				dict.Responses[k] = int(id)
			}
		}
	}

	// Parse output
	if out, ok := raw["output"].(map[string]interface{}); ok {
		for k, v := range out {
			if id, ok := v.(float64); ok {
				dict.Output[k] = int(id)
			}
		}
	}

	// Parse config
	if cfg, ok := raw["config"].(map[string]interface{}); ok {
		dict.Config = cfg
	}

	// Parse enumerations
	if enums, ok := raw["enumerations"].(map[string]interface{}); ok {
		for enumName, enumVals := range enums {
			if vals, ok := enumVals.(map[string]interface{}); ok {
				dict.Enumerations[enumName] = make(map[string]int)
				for k, v := range vals {
					if id, ok := v.(float64); ok {
						dict.Enumerations[enumName][k] = int(id)
					}
				}
			}
		}
	}

	return dict, nil
}

// calculateDictCRC calculates the CRC32 of the dictionary data.
// This is used to verify the dictionary matches the expected version.
func calculateDictCRC(data []byte) uint32 {
	// Klipper uses zlib.crc32 for dictionary CRC
	// We'll use the same algorithm
	crc := uint32(0)
	for _, b := range data {
		crc = crc32Update(crc, b)
	}
	return crc
}

// crc32Update updates a CRC32 value with a new byte.
// Uses the same polynomial as zlib.
func crc32Update(crc uint32, b byte) uint32 {
	const polynomial = 0xedb88320
	crc ^= uint32(b)
	for i := 0; i < 8; i++ {
		if crc&1 != 0 {
			crc = (crc >> 1) ^ polynomial
		} else {
			crc >>= 1
		}
	}
	return crc
}
