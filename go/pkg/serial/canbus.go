// Package serial provides CAN bus communication for Klipper MCU connections.
//
// This implements Linux SocketCAN support for communicating with CAN-connected
// MCUs (e.g., toolhead boards, EBB36/42, etc.).
//
// CAN ID Assignment:
//   - Admin broadcast: 0x3F0
//   - Admin response:  0x3F1
//   - MCU TX (Host→MCU): 256 + nodeid*2
//   - MCU RX (MCU→Host): 256 + nodeid*2 + 1
//
// Copyright (C) 2025 Go port
// This file may be distributed under the terms of the GNU GPLv3 license.

//go:build linux

package serial

import (
	"encoding/binary"
	"errors"
	"fmt"
	"net"
	"sync"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/unix"
)

// CAN bus constants
const (
	// Admin message IDs
	CANBusAdminID     = 0x3F0
	CANBusAdminRespID = 0x3F1

	// Admin commands
	CANBusCmdQueryUUID  = 0x00
	CANBusCmdSetNodeID  = 0x01
	CANBusCmdGetNodeID  = 0x02
	CANBusCmdRebootNode = 0x03

	// First available node ID
	CANBusNodeIDFirst = 4

	// Maximum CAN frame data length
	CANMaxDataLen = 8

	// CAN frame structure size
	canFrameSize = 16 // sizeof(struct can_frame)
)

// CAN socket constants (from linux/can.h)
const (
	AF_CAN         = 29
	PF_CAN         = AF_CAN
	CAN_RAW        = 1
	CAN_RAW_FILTER = 1
	SOL_CAN_RAW    = 101
)

// Common CAN errors
var (
	ErrCANNotConnected   = errors.New("canbus: not connected")
	ErrCANInvalidUUID    = errors.New("canbus: invalid UUID")
	ErrCANTimeout        = errors.New("canbus: operation timed out")
	ErrCANClosed         = errors.New("canbus: connection closed")
	ErrCANWriteFailed    = errors.New("canbus: write failed")
	ErrCANReadFailed     = errors.New("canbus: read failed")
	ErrCANNotSupported   = errors.New("canbus: not supported on this platform")
	ErrCANInterfaceDown  = errors.New("canbus: interface is down")
	ErrCANNodeIDMismatch = errors.New("canbus: node ID mismatch")
)

// canFrame represents a CAN frame (matches struct can_frame in linux/can.h)
type canFrame struct {
	canID   uint32   // CAN ID + flags
	canDLC  uint8    // frame payload length (0..8)
	pad     uint8    // padding
	res0    uint8    // reserved
	res1    uint8    // reserved
	data    [8]byte  // CAN frame payload
}

// canFilter represents a CAN filter (matches struct can_filter in linux/can.h)
type canFilter struct {
	canID   uint32
	canMask uint32
}

// sockaddrCAN represents struct sockaddr_can
type sockaddrCAN struct {
	family  uint16
	ifindex int32
	addr    [8]byte // union for TP/J1939, unused for raw CAN
}

// CANBusConfig holds CAN bus configuration.
type CANBusConfig struct {
	// Interface name (e.g., "can0", "can1")
	Interface string

	// UUID of the MCU (12 hex characters)
	UUID string

	// Node ID (auto-assigned if 0)
	NodeID int

	// Connection timeout
	ConnectTimeout time.Duration

	// Read timeout for individual operations
	ReadTimeout time.Duration
}

// DefaultCANBusConfig returns a CANBusConfig with default values.
func DefaultCANBusConfig() CANBusConfig {
	return CANBusConfig{
		Interface:      "can0",
		ConnectTimeout: 90 * time.Second,
		ReadTimeout:    5 * time.Second,
	}
}

// CANBus represents a CAN bus connection to an MCU.
type CANBus struct {
	mu       sync.Mutex
	fd       int
	config   CANBusConfig
	txID     uint32 // TX ID (Host → MCU)
	rxID     uint32 // RX ID (MCU → Host)
	closed   bool
	ifindex  int32
	uuid     []byte // 6-byte UUID
	readBuf  []byte // buffer for reassembling messages
}

// NewCANBus creates a new CAN bus connection.
func NewCANBus(config CANBusConfig) (*CANBus, error) {
	if config.Interface == "" {
		config.Interface = "can0"
	}
	if config.ConnectTimeout == 0 {
		config.ConnectTimeout = 90 * time.Second
	}
	if config.ReadTimeout == 0 {
		config.ReadTimeout = 5 * time.Second
	}

	// Parse UUID
	uuid, err := parseCANUUID(config.UUID)
	if err != nil {
		return nil, err
	}

	return &CANBus{
		fd:      -1,
		config:  config,
		uuid:    uuid,
		readBuf: make([]byte, 0, 4096),
	}, nil
}

// parseCANUUID parses a 12-character hex UUID string into 6 bytes.
func parseCANUUID(uuidStr string) ([]byte, error) {
	if uuidStr == "" {
		return nil, ErrCANInvalidUUID
	}

	// Parse as hex
	var uuid uint64
	_, err := fmt.Sscanf(uuidStr, "%x", &uuid)
	if err != nil {
		return nil, fmt.Errorf("%w: %v", ErrCANInvalidUUID, err)
	}

	// Check range (48-bit max)
	if uuid > 0xFFFFFFFFFFFF {
		return nil, fmt.Errorf("%w: UUID too large", ErrCANInvalidUUID)
	}

	// Convert to 6 bytes (big-endian)
	result := make([]byte, 6)
	for i := 0; i < 6; i++ {
		result[i] = byte((uuid >> (40 - i*8)) & 0xFF)
	}

	return result, nil
}

// Connect establishes a CAN bus connection to the MCU.
func (c *CANBus) Connect() error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if c.fd >= 0 {
		return nil // Already connected
	}

	// Get interface index
	ifindex, err := getCANInterfaceIndex(c.config.Interface)
	if err != nil {
		return err
	}
	c.ifindex = ifindex

	// Calculate TX/RX IDs based on node ID
	nodeID := c.config.NodeID
	if nodeID == 0 {
		nodeID = CANBusNodeIDFirst // Default to first node ID
	}
	c.txID = uint32(nodeID*2 + 256)
	c.rxID = c.txID + 1

	// Create SocketCAN socket
	fd, err := syscall.Socket(PF_CAN, syscall.SOCK_RAW, CAN_RAW)
	if err != nil {
		return fmt.Errorf("canbus: failed to create socket: %w", err)
	}

	// Set up RX filter
	filter := canFilter{
		canID:   c.rxID,
		canMask: 0x7FF, // Standard 11-bit ID mask
	}
	_, _, errno := syscall.Syscall6(
		syscall.SYS_SETSOCKOPT,
		uintptr(fd),
		SOL_CAN_RAW,
		CAN_RAW_FILTER,
		uintptr(unsafe.Pointer(&filter)),
		unsafe.Sizeof(filter),
		0,
	)
	if errno != 0 {
		syscall.Close(fd)
		return fmt.Errorf("canbus: failed to set filter: %w", errno)
	}

	// Bind to interface
	addr := sockaddrCAN{
		family:  AF_CAN,
		ifindex: ifindex,
	}
	_, _, errno = syscall.Syscall(
		syscall.SYS_BIND,
		uintptr(fd),
		uintptr(unsafe.Pointer(&addr)),
		unsafe.Sizeof(addr),
	)
	if errno != 0 {
		syscall.Close(fd)
		return fmt.Errorf("canbus: failed to bind: %w", errno)
	}

	c.fd = fd

	// Send SET_NODEID command
	if err := c.sendSetNodeID(); err != nil {
		syscall.Close(fd)
		c.fd = -1
		return err
	}

	return nil
}

// sendSetNodeID sends the SET_NODEID admin command.
func (c *CANBus) sendSetNodeID() error {
	// Build SET_NODEID command: [CMD, UUID[0:6], NODEID]
	cmd := make([]byte, 8)
	cmd[0] = CANBusCmdSetNodeID
	copy(cmd[1:7], c.uuid)
	cmd[7] = byte(c.config.NodeID)
	if c.config.NodeID == 0 {
		cmd[7] = CANBusNodeIDFirst
	}

	// Send to admin ID
	frame := canFrame{
		canID:  CANBusAdminID,
		canDLC: 8,
	}
	copy(frame.data[:], cmd)

	return c.writeFrame(&frame)
}

// writeFrame writes a single CAN frame.
func (c *CANBus) writeFrame(frame *canFrame) error {
	buf := make([]byte, canFrameSize)
	binary.LittleEndian.PutUint32(buf[0:4], frame.canID)
	buf[4] = frame.canDLC
	copy(buf[8:16], frame.data[:])

	n, err := syscall.Write(c.fd, buf)
	if err != nil {
		return fmt.Errorf("%w: %v", ErrCANWriteFailed, err)
	}
	if n != canFrameSize {
		return fmt.Errorf("%w: short write", ErrCANWriteFailed)
	}
	return nil
}

// readFrame reads a single CAN frame.
func (c *CANBus) readFrame(frame *canFrame, timeout time.Duration) error {
	// Set read timeout using poll
	if timeout > 0 {
		pfd := []unix.PollFd{{
			Fd:     int32(c.fd),
			Events: unix.POLLIN,
		}}
		timeoutMs := int(timeout.Milliseconds())
		if timeoutMs <= 0 {
			timeoutMs = 1
		}
		n, err := unix.Poll(pfd, timeoutMs)
		if err != nil {
			return fmt.Errorf("%w: poll error: %v", ErrCANReadFailed, err)
		}
		if n == 0 {
			return ErrCANTimeout
		}
	}

	buf := make([]byte, canFrameSize)
	n, err := syscall.Read(c.fd, buf)
	if err != nil {
		return fmt.Errorf("%w: %v", ErrCANReadFailed, err)
	}
	if n != canFrameSize {
		return fmt.Errorf("%w: short read (%d bytes)", ErrCANReadFailed, n)
	}

	frame.canID = binary.LittleEndian.Uint32(buf[0:4])
	frame.canDLC = buf[4]
	copy(frame.data[:], buf[8:16])

	return nil
}

// Write writes data to the CAN bus, splitting into multiple frames if needed.
func (c *CANBus) Write(data []byte) (int, error) {
	c.mu.Lock()
	defer c.mu.Unlock()

	if c.closed || c.fd < 0 {
		return 0, ErrCANNotConnected
	}

	totalWritten := 0
	remaining := data

	for len(remaining) > 0 {
		// Calculate frame size (max 8 bytes per frame)
		frameLen := len(remaining)
		if frameLen > CANMaxDataLen {
			frameLen = CANMaxDataLen
		}

		frame := canFrame{
			canID:  c.txID,
			canDLC: uint8(frameLen),
		}
		copy(frame.data[:frameLen], remaining[:frameLen])

		if err := c.writeFrame(&frame); err != nil {
			return totalWritten, err
		}

		remaining = remaining[frameLen:]
		totalWritten += frameLen
	}

	return totalWritten, nil
}

// Read reads data from the CAN bus, reassembling from multiple frames.
func (c *CANBus) Read(buf []byte) (int, error) {
	c.mu.Lock()
	defer c.mu.Unlock()

	if c.closed || c.fd < 0 {
		return 0, ErrCANNotConnected
	}

	// If we have buffered data, return it first
	if len(c.readBuf) > 0 {
		n := copy(buf, c.readBuf)
		c.readBuf = c.readBuf[n:]
		return n, nil
	}

	// Read a frame
	var frame canFrame
	if err := c.readFrame(&frame, c.config.ReadTimeout); err != nil {
		return 0, err
	}

	// Check if it's from our MCU (RX ID)
	if frame.canID != c.rxID {
		// Not for us, try again
		return 0, nil
	}

	// Copy data
	dataLen := int(frame.canDLC)
	if dataLen > len(buf) {
		// Buffer too small, save overflow
		n := copy(buf, frame.data[:dataLen])
		c.readBuf = append(c.readBuf, frame.data[n:dataLen]...)
		return n, nil
	}

	return copy(buf, frame.data[:dataLen]), nil
}

// Close closes the CAN bus connection.
func (c *CANBus) Close() error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if c.closed {
		return nil
	}

	c.closed = true
	if c.fd >= 0 {
		err := syscall.Close(c.fd)
		c.fd = -1
		return err
	}
	return nil
}

// Fd returns the file descriptor for use with select/poll.
func (c *CANBus) Fd() int {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.fd
}

// TxID returns the TX CAN ID (Host → MCU).
func (c *CANBus) TxID() uint32 {
	return c.txID
}

// RxID returns the RX CAN ID (MCU → Host).
func (c *CANBus) RxID() uint32 {
	return c.rxID
}

// NodeID returns the assigned node ID.
func (c *CANBus) NodeID() int {
	if c.config.NodeID == 0 {
		return CANBusNodeIDFirst
	}
	return c.config.NodeID
}

// UUID returns the MCU UUID.
func (c *CANBus) UUID() []byte {
	return c.uuid
}

// Interface returns the CAN interface name.
func (c *CANBus) Interface() string {
	return c.config.Interface
}

// getCANInterfaceIndex gets the interface index for a CAN interface.
func getCANInterfaceIndex(ifname string) (int32, error) {
	iface, err := net.InterfaceByName(ifname)
	if err != nil {
		return 0, fmt.Errorf("canbus: interface %s not found: %w", ifname, err)
	}
	return int32(iface.Index), nil
}

// VerifyUUID sends get_canbus_id and verifies the UUID matches.
// This should be called after the MCU connection is established.
func (c *CANBus) VerifyUUID(sendCmd func(string) (map[string]interface{}, error)) error {
	// Send get_canbus_id command
	resp, err := sendCmd("get_canbus_id")
	if err != nil {
		return fmt.Errorf("canbus: failed to get canbus_id: %w", err)
	}

	// Extract canbus_uuid from response
	gotUUID, ok := resp["canbus_uuid"].([]byte)
	if !ok {
		return fmt.Errorf("canbus: invalid canbus_uuid response")
	}

	// Compare with expected UUID
	if len(gotUUID) != len(c.uuid) {
		return fmt.Errorf("%w: length mismatch", ErrCANNodeIDMismatch)
	}
	for i := range c.uuid {
		if gotUUID[i] != c.uuid[i] {
			return fmt.Errorf("%w: UUID mismatch at byte %d", ErrCANNodeIDMismatch, i)
		}
	}

	return nil
}

// QueryCANNodes sends a query to discover CAN nodes on the bus.
// Returns a list of discovered UUIDs.
func QueryCANNodes(ifname string, timeout time.Duration) ([]string, error) {
	// Get interface index
	ifindex, err := getCANInterfaceIndex(ifname)
	if err != nil {
		return nil, err
	}

	// Create socket
	fd, err := syscall.Socket(PF_CAN, syscall.SOCK_RAW, CAN_RAW)
	if err != nil {
		return nil, fmt.Errorf("canbus: failed to create socket: %w", err)
	}
	defer syscall.Close(fd)

	// Set up filter for admin response
	filter := canFilter{
		canID:   CANBusAdminRespID,
		canMask: 0x7FF,
	}
	_, _, errno := syscall.Syscall6(
		syscall.SYS_SETSOCKOPT,
		uintptr(fd),
		SOL_CAN_RAW,
		CAN_RAW_FILTER,
		uintptr(unsafe.Pointer(&filter)),
		unsafe.Sizeof(filter),
		0,
	)
	if errno != 0 {
		return nil, fmt.Errorf("canbus: failed to set filter: %w", errno)
	}

	// Bind to interface
	addr := sockaddrCAN{
		family:  AF_CAN,
		ifindex: ifindex,
	}
	_, _, errno = syscall.Syscall(
		syscall.SYS_BIND,
		uintptr(fd),
		uintptr(unsafe.Pointer(&addr)),
		unsafe.Sizeof(addr),
	)
	if errno != 0 {
		return nil, fmt.Errorf("canbus: failed to bind: %w", errno)
	}

	// Send QUERY_UUID command
	queryCmd := canFrame{
		canID:  CANBusAdminID,
		canDLC: 1,
	}
	queryCmd.data[0] = CANBusCmdQueryUUID

	buf := make([]byte, canFrameSize)
	binary.LittleEndian.PutUint32(buf[0:4], queryCmd.canID)
	buf[4] = queryCmd.canDLC
	copy(buf[8:16], queryCmd.data[:])

	_, err = syscall.Write(fd, buf)
	if err != nil {
		return nil, fmt.Errorf("canbus: failed to send query: %w", err)
	}

	// Collect responses
	var uuids []string
	deadline := time.Now().Add(timeout)

	for time.Now().Before(deadline) {
		// Poll with remaining timeout
		remaining := time.Until(deadline)
		pfd := []unix.PollFd{{
			Fd:     int32(fd),
			Events: unix.POLLIN,
		}}
		n, err := unix.Poll(pfd, int(remaining.Milliseconds()))
		if err != nil {
			break
		}
		if n == 0 {
			break // Timeout
		}

		// Read response
		respBuf := make([]byte, canFrameSize)
		n2, err := syscall.Read(fd, respBuf)
		if err != nil || n2 != canFrameSize {
			continue
		}

		// Parse response
		respID := binary.LittleEndian.Uint32(respBuf[0:4])
		if respID != CANBusAdminRespID {
			continue
		}

		dlc := respBuf[4]
		if dlc < 7 {
			continue // Need at least CMD + 6 byte UUID
		}

		// Check if it's a QUERY_UUID response
		if respBuf[8] == CANBusCmdQueryUUID {
			// Extract UUID (bytes 1-6 of data)
			uuid := fmt.Sprintf("%02x%02x%02x%02x%02x%02x",
				respBuf[9], respBuf[10], respBuf[11],
				respBuf[12], respBuf[13], respBuf[14])

			// Add if not already seen
			found := false
			for _, u := range uuids {
				if u == uuid {
					found = true
					break
				}
			}
			if !found {
				uuids = append(uuids, uuid)
			}
		}
	}

	return uuids, nil
}

// CANBusStats tracks CAN bus communication statistics.
type CANBusTransportStats struct {
	TxFrames  uint64
	RxFrames  uint64
	TxBytes   uint64
	RxBytes   uint64
	TxErrors  uint64
	RxErrors  uint64
	BusOff    uint64
}

// GetStats returns current CAN bus statistics.
func (c *CANBus) GetStats() CANBusTransportStats {
	// In a full implementation, these would be tracked during Read/Write
	return CANBusTransportStats{}
}

// WrapCANBus creates a Port wrapper around a CANBus connection.
// This allows CAN bus to be used with interfaces expecting a Port.
// The returned Port uses the CAN bus file descriptor and marks itself
// as a CAN bus connection (isCAN = true).
func WrapCANBus(canbus *CANBus) *Port {
	if canbus == nil {
		return nil
	}
	return &Port{
		fd:       canbus.fd,
		device:   canbus.config.Interface,
		config:   Config{ReadTimeout: canbus.config.ReadTimeout},
		isSocket: false, // Not a socket
		isCAN:    true,  // Mark as CAN bus
		canbus:   canbus,
	}
}
