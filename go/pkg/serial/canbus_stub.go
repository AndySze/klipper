// CAN bus stub for non-Linux platforms.
//
// SocketCAN is only available on Linux. This stub provides compile-time
// compatibility for other platforms (macOS, Windows, etc.).

//go:build !linux

package serial

import (
	"errors"
	"time"
)

// Common CAN errors (stub versions)
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

// CAN bus constants
const (
	CANBusAdminID       = 0x3F0
	CANBusAdminRespID   = 0x3F1
	CANBusCmdQueryUUID  = 0x00
	CANBusCmdSetNodeID  = 0x01
	CANBusCmdGetNodeID  = 0x02
	CANBusCmdRebootNode = 0x03
	CANBusNodeIDFirst   = 4
	CANMaxDataLen       = 8
)

// CANBusConfig holds CAN bus configuration.
type CANBusConfig struct {
	Interface      string
	UUID           string
	NodeID         int
	ConnectTimeout time.Duration
	ReadTimeout    time.Duration
}

// DefaultCANBusConfig returns a CANBusConfig with default values.
func DefaultCANBusConfig() CANBusConfig {
	return CANBusConfig{
		Interface:      "can0",
		ConnectTimeout: 90 * time.Second,
		ReadTimeout:    5 * time.Second,
	}
}

// CANBus represents a CAN bus connection (stub for non-Linux).
type CANBus struct {
	config CANBusConfig
}

// NewCANBus creates a new CAN bus connection (stub).
func NewCANBus(config CANBusConfig) (*CANBus, error) {
	return nil, ErrCANNotSupported
}

// Connect establishes a CAN bus connection (stub).
func (c *CANBus) Connect() error {
	return ErrCANNotSupported
}

// Write writes data to the CAN bus (stub).
func (c *CANBus) Write(data []byte) (int, error) {
	return 0, ErrCANNotSupported
}

// Read reads data from the CAN bus (stub).
func (c *CANBus) Read(buf []byte) (int, error) {
	return 0, ErrCANNotSupported
}

// Close closes the CAN bus connection (stub).
func (c *CANBus) Close() error {
	return nil
}

// Fd returns the file descriptor (stub).
func (c *CANBus) Fd() int {
	return -1
}

// TxID returns the TX CAN ID (stub).
func (c *CANBus) TxID() uint32 {
	return 0
}

// RxID returns the RX CAN ID (stub).
func (c *CANBus) RxID() uint32 {
	return 0
}

// NodeID returns the assigned node ID (stub).
func (c *CANBus) NodeID() int {
	return 0
}

// UUID returns the MCU UUID (stub).
func (c *CANBus) UUID() []byte {
	return nil
}

// Interface returns the CAN interface name (stub).
func (c *CANBus) Interface() string {
	return ""
}

// VerifyUUID verifies the UUID matches (stub).
func (c *CANBus) VerifyUUID(sendCmd func(string) (map[string]interface{}, error)) error {
	return ErrCANNotSupported
}

// QueryCANNodes discovers CAN nodes on the bus (stub).
func QueryCANNodes(ifname string, timeout time.Duration) ([]string, error) {
	return nil, ErrCANNotSupported
}

// CANBusTransportStats tracks CAN bus statistics.
type CANBusTransportStats struct {
	TxFrames uint64
	RxFrames uint64
	TxBytes  uint64
	RxBytes  uint64
	TxErrors uint64
	RxErrors uint64
	BusOff   uint64
}

// GetStats returns current CAN bus statistics (stub).
func (c *CANBus) GetStats() CANBusTransportStats {
	return CANBusTransportStats{}
}

// WrapCANBus creates a Port wrapper around a CANBus connection (stub).
// This allows CAN bus to be used with interfaces expecting a Port.
func WrapCANBus(canbus *CANBus) *Port {
	return nil
}
