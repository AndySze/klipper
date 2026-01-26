// Package mcu provides MCU communication and handshake protocol.
package mcu

import (
	"bytes"
	"compress/zlib"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"strconv"
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

	// ResetBeforeIdentify toggles DTR to reset the MCU before identify (default: false)
	ResetBeforeIdentify bool

	// Trace writer for debug output (optional)
	Trace io.Writer
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

	// Step 0: Wait for MCU to stabilize after port open
	// Some serial adapters need time to settle
	if cfg.Trace != nil {
		fmt.Fprintf(cfg.Trace, "Handshake: Waiting for serial port to stabilize\n")
	}
	time.Sleep(200 * time.Millisecond)

	// Step 1: Flush any pending data in the serial buffer
	if cfg.Trace != nil {
		fmt.Fprintf(cfg.Trace, "Handshake: Flushing serial buffer\n")
	}
	if err := port.Flush(); err != nil {
		if cfg.Trace != nil {
			fmt.Fprintf(cfg.Trace, "Handshake: Flush error (non-fatal): %v\n", err)
		}
	}

	// Step 2: Optionally reset the MCU using DTR toggle and/or break signal
	if cfg.ResetBeforeIdentify {
		if cfg.Trace != nil {
			fmt.Fprintf(cfg.Trace, "Handshake: Attempting MCU reset\n")
		}

		// Try DTR toggle first (works on many Arduino-style boards)
		dtrWorked := false
		if err := port.SetDTR(true); err == nil {
			time.Sleep(100 * time.Millisecond)
			if err := port.SetDTR(false); err == nil {
				dtrWorked = true
				time.Sleep(100 * time.Millisecond)
				port.SetDTR(true)
			}
		}
		if cfg.Trace != nil {
			fmt.Fprintf(cfg.Trace, "Handshake: DTR toggle %s\n", map[bool]string{true: "succeeded", false: "failed (not supported)"}[dtrWorked])
		}

		// Also try sending a break signal (some boards reset on this)
		if err := port.SendBreak(); err != nil {
			if cfg.Trace != nil {
				fmt.Fprintf(cfg.Trace, "Handshake: SendBreak error (non-fatal): %v\n", err)
			}
		} else {
			if cfg.Trace != nil {
				fmt.Fprintf(cfg.Trace, "Handshake: Break signal sent\n")
			}
		}

		// Wait a bit for MCU to boot/reset
		time.Sleep(100 * time.Millisecond)

		// Flush again after reset
		port.Flush()
	}

	// Step 3: Drain any pending messages from the MCU
	// The MCU may send startup messages or status updates that we need to clear
	// Do a quick drain - just read whatever is immediately available
	if cfg.Trace != nil {
		fmt.Fprintf(cfg.Trace, "Handshake: Draining pending messages\n")
	}
	drainBuf := make([]byte, protocol.MESSAGE_MAX)
	drainCount := 0
	for drainCount < 50 { // Max 50 reads to clear queued messages from configured MCU
		n, err := port.Read(drainBuf)
		if err != nil || n == 0 {
			break // No more data available, stop draining
		}
		if cfg.Trace != nil {
			fmt.Fprintf(cfg.Trace, "Handshake: Drained %d bytes: %x\n", n, drainBuf[:n])
		}
		drainCount++
		time.Sleep(50 * time.Millisecond) // Allow more data to arrive
	}
	if cfg.Trace != nil {
		fmt.Fprintf(cfg.Trace, "Handshake: Drain complete after %d reads\n", drainCount)
	}

	// Collect identify data by querying chunks
	var identifyData []byte
	seq := 0
	iteration := 0
	unexpectedCount := 0
	const maxUnexpected = 200 // Allow many unexpected messages (MCU may be in configured state sending ADC data)

	if cfg.Trace != nil {
		remaining := time.Until(deadline)
		fmt.Fprintf(cfg.Trace, "Handshake: Starting identify phase, %.1fs remaining\n", remaining.Seconds())
	}

	for time.Now().Before(deadline) {
		iteration++
		offset := len(identifyData)

		// Build identify command manually for debugging
		// Format: identify offset=%u count=%c (message ID 1)
		var cmd []byte
		protocol.EncodeUint32(&cmd, 1) // Message ID 1
		protocol.EncodeUint32(&cmd, int32(offset))
		protocol.EncodeUint32(&cmd, int32(cfg.ReadChunkSize))
		// Use EncodeMsgblock which sets MESSAGE_DEST - required for MCU to process the message
		msgBlock := protocol.EncodeMsgblock(seq, cmd)
		// Don't increment seq yet - only do so on valid response

		if cfg.Trace != nil {
			fmt.Fprintf(cfg.Trace, "Handshake[%d]: TX offset=%d seq=%d: %x (cmd: %x)\n",
				iteration, offset, seq, msgBlock, cmd)
		}

		resp, err := sendWithRetry(port, msgBlock, cfg.MaxRetries, cfg.RetryDelay, deadline, cfg.Trace)
		if err != nil {
			if cfg.Trace != nil {
				fmt.Fprintf(cfg.Trace, "Handshake[%d]: sendWithRetry error: %v\n", iteration, err)
			}
			if errors.Is(err, ErrTimeout) && len(identifyData) == 0 {
				// No response at all - MCU might not be connected
				return nil, ErrNoResponse
			}
			return nil, fmt.Errorf("mcu: identify failed: %w", err)
		}

		if cfg.Trace != nil {
			fmt.Fprintf(cfg.Trace, "Handshake[%d]: RX len=%d: %x\n", iteration, len(resp), resp)
		}

		// Parse identify_response
		respOffset, data, err := parseIdentifyResponse(resp)
		if err != nil {
			unexpectedCount++
			if cfg.Trace != nil {
				fmt.Fprintf(cfg.Trace, "Handshake[%d]: parse error: %v (unexpected #%d, retrying)\n",
					iteration, err, unexpectedCount)
			}
			// If we got a message but it wasn't identify_response, retry
			// This can happen if the MCU sends startup messages after reset
			// But don't retry forever
			if unexpectedCount >= maxUnexpected {
				return nil, fmt.Errorf("mcu: too many unexpected messages (%d), MCU may be in configured state", unexpectedCount)
			}
			continue
		}

		// Got valid identify_response - now increment seq for next request
		seq = (seq + 1) & protocol.MESSAGE_SEQ_MASK

		if cfg.Trace != nil {
			fmt.Fprintf(cfg.Trace, "Handshake[%d]: parsed OK, respOffset=%d, dataLen=%d\n",
				iteration, respOffset, len(data))
		}

		// Verify offset matches what we requested
		if respOffset != offset {
			return nil, fmt.Errorf("mcu: identify offset mismatch: got %d, expected %d", respOffset, offset)
		}

		// Empty data means we're done
		if len(data) == 0 {
			if cfg.Trace != nil {
				fmt.Fprintf(cfg.Trace, "Handshake[%d]: empty data, breaking loop\n", iteration)
			}
			break
		}

		identifyData = append(identifyData, data...)
		if cfg.Trace != nil {
			fmt.Fprintf(cfg.Trace, "Handshake[%d]: identifyData now %d bytes, looping\n",
				iteration, len(identifyData))
		}
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
func sendWithRetry(port *serial.Port, cmd []byte, maxRetries int, retryDelay time.Duration, deadline time.Time, trace io.Writer) ([]byte, error) {
	delay := retryDelay
	var lastErr error

	for retry := 0; retry <= maxRetries; retry++ {
		if time.Now().After(deadline) {
			return nil, ErrTimeout
		}

		// Send command
		n, err := port.Write(cmd)
		if trace != nil {
			fmt.Fprintf(trace, "  sendWithRetry[%d]: Write returned n=%d, err=%v\n", retry, n, err)
		}
		if err != nil {
			lastErr = err
			continue
		}

		// Read response, accumulating bytes until we have a complete message
		resp := make([]byte, 0, protocol.MESSAGE_MAX)
		buf := make([]byte, protocol.MESSAGE_MAX)
		expectedLen := 0
		msgStart := 0 // Start position of current message in resp

		for time.Now().Before(deadline) {
			n, err = port.Read(buf)
			if trace != nil {
				fmt.Fprintf(trace, "  sendWithRetry[%d]: Read returned n=%d, err=%v\n", retry, n, err)
				if n > 0 {
					fmt.Fprintf(trace, "  sendWithRetry[%d]: RX chunk: %x\n", retry, buf[:n])
				}
			}

			if n > 0 {
				resp = append(resp, buf[:n]...)

				// Skip leading sync bytes (may be from previous messages)
				for msgStart < len(resp) && resp[msgStart] == protocol.MESSAGE_SYNC {
					msgStart++
				}

				// Once we have the length byte, we know expected message size
				if expectedLen == 0 && len(resp) > msgStart {
					expectedLen = int(resp[msgStart])
					if trace != nil {
						fmt.Fprintf(trace, "  sendWithRetry[%d]: Expected msg len: %d (msgStart=%d)\n", retry, expectedLen, msgStart)
					}
				}

				// Check if we have a complete message
				if expectedLen > 0 && len(resp) >= msgStart+expectedLen {
					// Verify sync byte
					if resp[msgStart+expectedLen-1] == protocol.MESSAGE_SYNC {
						return resp[msgStart : msgStart+expectedLen], nil
					}
					// Bad sync - try to find next valid message start
					if trace != nil {
						fmt.Fprintf(trace, "  sendWithRetry[%d]: Bad sync at pos %d, got 0x%02x\n",
							retry, msgStart+expectedLen-1, resp[msgStart+expectedLen-1])
					}
				}
			}

			if err != nil {
				if errors.Is(err, serial.ErrTimeout) || err.Error() == "serial: operation timed out" {
					if len(resp) == 0 {
						break // No data at all, retry
					}
					// Got partial data, keep waiting if we haven't hit deadline
					continue
				}
				lastErr = err
				break
			}
		}

		if len(resp) > 0 && trace != nil {
			fmt.Fprintf(trace, "  sendWithRetry[%d]: Accumulated: %x (len=%d, msgStart=%d, expected=%d)\n",
				retry, resp, len(resp), msgStart, expectedLen)
		}

		if len(resp) == 0 {
			lastErr = ErrTimeout
		} else {
			lastErr = ErrNoResponse
		}

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

	// Parse enumerations (supports both simple values and ranges like [start,count])
	if enums, ok := raw["enumerations"].(map[string]interface{}); ok {
		for enumName, enumVals := range enums {
			if vals, ok := enumVals.(map[string]interface{}); ok {
				dict.Enumerations[enumName] = make(map[string]int)
				for k, v := range vals {
					switch tv := v.(type) {
					case float64:
						// Simple value
						dict.Enumerations[enumName][k] = int(tv)
					case []interface{}:
						// Range format: [start, count]
						if len(tv) == 2 {
							start, ok1 := tv[0].(float64)
							count, ok2 := tv[1].(float64)
							if ok1 && ok2 {
								// Extract the root (e.g., "PF" from "PF0")
								keyRoot := k
								for len(keyRoot) > 0 && keyRoot[len(keyRoot)-1] >= '0' && keyRoot[len(keyRoot)-1] <= '9' {
									keyRoot = keyRoot[:len(keyRoot)-1]
								}
								startEnum := 0
								if len(keyRoot) != len(k) {
									if n, err := strconv.Atoi(k[len(keyRoot):]); err == nil {
										startEnum = n
									}
								}
								// Generate all values in range
								for i := 0; i < int(count); i++ {
									enumKey := fmt.Sprintf("%s%d", keyRoot, startEnum+i)
									dict.Enumerations[enumName][enumKey] = int(start) + i
								}
							}
						}
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
