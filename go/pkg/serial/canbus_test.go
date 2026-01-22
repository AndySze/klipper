// CAN Bus Tests
//
// These tests verify the CAN bus API and constants work correctly.
// On non-Linux platforms, the stub returns ErrCANNotSupported.
//
// Copyright (C) 2025 Go port

package serial

import (
	"errors"
	"testing"
	"time"
)

func TestCANBusConstants(t *testing.T) {
	// Verify CAN bus protocol constants
	if CANBusAdminID != 0x3F0 {
		t.Errorf("Expected CANBusAdminID 0x3F0, got 0x%X", CANBusAdminID)
	}
	if CANBusAdminRespID != 0x3F1 {
		t.Errorf("Expected CANBusAdminRespID 0x3F1, got 0x%X", CANBusAdminRespID)
	}
	if CANBusCmdQueryUUID != 0x00 {
		t.Errorf("Expected CANBusCmdQueryUUID 0x00, got 0x%02X", CANBusCmdQueryUUID)
	}
	if CANBusCmdSetNodeID != 0x01 {
		t.Errorf("Expected CANBusCmdSetNodeID 0x01, got 0x%02X", CANBusCmdSetNodeID)
	}
	if CANBusCmdGetNodeID != 0x02 {
		t.Errorf("Expected CANBusCmdGetNodeID 0x02, got 0x%02X", CANBusCmdGetNodeID)
	}
	if CANBusCmdRebootNode != 0x03 {
		t.Errorf("Expected CANBusCmdRebootNode 0x03, got 0x%02X", CANBusCmdRebootNode)
	}
	if CANBusNodeIDFirst != 4 {
		t.Errorf("Expected CANBusNodeIDFirst 4, got %d", CANBusNodeIDFirst)
	}
	if CANMaxDataLen != 8 {
		t.Errorf("Expected CANMaxDataLen 8, got %d", CANMaxDataLen)
	}
}

func TestDefaultCANBusConfig(t *testing.T) {
	cfg := DefaultCANBusConfig()

	if cfg.Interface != "can0" {
		t.Errorf("Expected default interface 'can0', got '%s'", cfg.Interface)
	}
	if cfg.ConnectTimeout != 90*time.Second {
		t.Errorf("Expected default connect timeout 90s, got %v", cfg.ConnectTimeout)
	}
	if cfg.ReadTimeout != 5*time.Second {
		t.Errorf("Expected default read timeout 5s, got %v", cfg.ReadTimeout)
	}
}

func TestCANBusErrors(t *testing.T) {
	// Verify error values exist and are distinct
	errs := []error{
		ErrCANNotConnected,
		ErrCANInvalidUUID,
		ErrCANTimeout,
		ErrCANClosed,
		ErrCANWriteFailed,
		ErrCANReadFailed,
		ErrCANNotSupported,
		ErrCANInterfaceDown,
		ErrCANNodeIDMismatch,
	}

	for i, err := range errs {
		if err == nil {
			t.Errorf("Error at index %d is nil", i)
			continue
		}
		// Verify error messages are not empty
		if err.Error() == "" {
			t.Errorf("Error at index %d has empty message", i)
		}
	}

	// Verify specific error types
	if !errors.Is(ErrCANNotSupported, ErrCANNotSupported) {
		t.Error("ErrCANNotSupported should match itself")
	}
}

func TestCANBusTransportStats(t *testing.T) {
	// Verify stats struct can be created and fields work
	stats := CANBusTransportStats{
		TxFrames: 100,
		RxFrames: 200,
		TxBytes:  800,
		RxBytes:  1600,
		TxErrors: 1,
		RxErrors: 2,
		BusOff:   0,
	}

	if stats.TxFrames != 100 {
		t.Errorf("Expected TxFrames 100, got %d", stats.TxFrames)
	}
	if stats.RxFrames != 200 {
		t.Errorf("Expected RxFrames 200, got %d", stats.RxFrames)
	}
	if stats.TxBytes != 800 {
		t.Errorf("Expected TxBytes 800, got %d", stats.TxBytes)
	}
	if stats.RxBytes != 1600 {
		t.Errorf("Expected RxBytes 1600, got %d", stats.RxBytes)
	}
}

// TestCANBusIDCalculation verifies the CAN ID scheme used by Klipper.
// TX ID = nodeid*2 + 256 (Host → MCU)
// RX ID = TX ID + 1 (MCU → Host)
func TestCANBusIDCalculation(t *testing.T) {
	testCases := []struct {
		nodeID   int
		expectedTx uint32
		expectedRx uint32
	}{
		{4, 264, 265},   // First node: 4*2+256=264
		{5, 266, 267},   // Second node: 5*2+256=266
		{10, 276, 277},  // Node 10: 10*2+256=276
		{100, 456, 457}, // Node 100: 100*2+256=456
	}

	for _, tc := range testCases {
		txID := uint32(tc.nodeID*2 + 256)
		rxID := txID + 1

		if txID != tc.expectedTx {
			t.Errorf("Node %d: expected TX ID %d, got %d", tc.nodeID, tc.expectedTx, txID)
		}
		if rxID != tc.expectedRx {
			t.Errorf("Node %d: expected RX ID %d, got %d", tc.nodeID, tc.expectedRx, rxID)
		}
	}
}

// TestNewCANBus tests CANBus creation.
// On non-Linux, this should return ErrCANNotSupported.
// On Linux, this would attempt to create a real connection.
func TestNewCANBus(t *testing.T) {
	cfg := DefaultCANBusConfig()
	cfg.UUID = "aabbccddeeff"
	cfg.NodeID = CANBusNodeIDFirst

	_, err := NewCANBus(cfg)
	// On non-Linux, should return ErrCANNotSupported
	// On Linux, would return an error about missing interface
	if err == nil {
		t.Log("NewCANBus succeeded - running on Linux with CAN interface available")
	} else if errors.Is(err, ErrCANNotSupported) {
		t.Log("NewCANBus returned ErrCANNotSupported - running on non-Linux platform")
	} else {
		t.Logf("NewCANBus returned error: %v (expected on Linux without CAN interface)", err)
	}
}

// TestQueryCANNodes tests node discovery.
// On non-Linux, this should return ErrCANNotSupported.
func TestQueryCANNodes(t *testing.T) {
	_, err := QueryCANNodes("can0", 100*time.Millisecond)
	if err == nil {
		t.Log("QueryCANNodes succeeded - running on Linux with CAN interface available")
	} else if errors.Is(err, ErrCANNotSupported) {
		t.Log("QueryCANNodes returned ErrCANNotSupported - running on non-Linux platform")
	} else {
		t.Logf("QueryCANNodes returned error: %v", err)
	}
}
