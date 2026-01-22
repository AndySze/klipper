// CAN Bus IDs Tests
//
// Copyright (C) 2025 Go port

package hosth4

import (
	"testing"
)

func TestCANBusIDs_AddUUID(t *testing.T) {
	cids := &CANBusIDs{
		interface_: "can0",
		nodes:      make(map[string]*CANNode),
	}

	// Test adding first UUID
	nodeID, err := cids.AddUUID("aabbccddeeff", "can0")
	if err != nil {
		t.Fatalf("AddUUID failed: %v", err)
	}
	if nodeID != CANBusNodeIDFirst {
		t.Errorf("Expected node ID %d, got %d", CANBusNodeIDFirst, nodeID)
	}

	// Test adding second UUID
	nodeID2, err := cids.AddUUID("112233445566", "can0")
	if err != nil {
		t.Fatalf("AddUUID failed: %v", err)
	}
	if nodeID2 != CANBusNodeIDFirst+1 {
		t.Errorf("Expected node ID %d, got %d", CANBusNodeIDFirst+1, nodeID2)
	}

	// Test duplicate UUID
	_, err = cids.AddUUID("aabbccddeeff", "can0")
	if err == nil {
		t.Error("Expected error for duplicate UUID, got nil")
	}
}

func TestCANBusIDs_GetNodeID(t *testing.T) {
	cids := &CANBusIDs{
		interface_: "can0",
		nodes:      make(map[string]*CANNode),
	}

	// Add a UUID
	expectedID, err := cids.AddUUID("aabbccddeeff", "can0")
	if err != nil {
		t.Fatalf("AddUUID failed: %v", err)
	}

	// Test getting node ID
	nodeID, err := cids.GetNodeID("aabbccddeeff")
	if err != nil {
		t.Fatalf("GetNodeID failed: %v", err)
	}
	if nodeID != expectedID {
		t.Errorf("Expected node ID %d, got %d", expectedID, nodeID)
	}

	// Test unknown UUID
	_, err = cids.GetNodeID("unknown")
	if err == nil {
		t.Error("Expected error for unknown UUID, got nil")
	}
}

func TestCANBusIDs_RegisterNode(t *testing.T) {
	cids := &CANBusIDs{
		interface_: "can0",
		nodes:      make(map[string]*CANNode),
	}

	// Test registering node
	err := cids.RegisterNode("aabbccddeeff", 10, "test_node")
	if err != nil {
		t.Fatalf("RegisterNode failed: %v", err)
	}

	// Verify registration
	node, err := cids.GetNodeByUUID("aabbccddeeff")
	if err != nil {
		t.Fatalf("GetNodeByUUID failed: %v", err)
	}
	if node.nodeID != 10 {
		t.Errorf("Expected node ID 10, got %d", node.nodeID)
	}
	if node.name != "test_node" {
		t.Errorf("Expected name 'test_node', got '%s'", node.name)
	}

	// Test duplicate registration
	err = cids.RegisterNode("aabbccddeeff", 11, "test_node2")
	if err == nil {
		t.Error("Expected error for duplicate registration, got nil")
	}
}

func TestCANBusIDs_GetNodeByID(t *testing.T) {
	cids := &CANBusIDs{
		interface_: "can0",
		nodes:      make(map[string]*CANNode),
	}

	// Register a node
	err := cids.RegisterNode("aabbccddeeff", 10, "test_node")
	if err != nil {
		t.Fatalf("RegisterNode failed: %v", err)
	}

	// Test getting node by ID
	node, err := cids.GetNodeByID(10)
	if err != nil {
		t.Fatalf("GetNodeByID failed: %v", err)
	}
	if node.uuid != "aabbccddeeff" {
		t.Errorf("Expected UUID 'aabbccddeeff', got '%s'", node.uuid)
	}

	// Test unknown ID
	_, err = cids.GetNodeByID(999)
	if err == nil {
		t.Error("Expected error for unknown ID, got nil")
	}
}

func TestCANBusIDs_AssignNodeID(t *testing.T) {
	cids := &CANBusIDs{
		interface_: "can0",
		nodes:      make(map[string]*CANNode),
	}

	// Register a node
	err := cids.RegisterNode("aabbccddeeff", 10, "test_node")
	if err != nil {
		t.Fatalf("RegisterNode failed: %v", err)
	}

	// Test assigning new node ID
	err = cids.AssignNodeID("aabbccddeeff", 20)
	if err != nil {
		t.Fatalf("AssignNodeID failed: %v", err)
	}

	// Verify new ID
	nodeID, err := cids.GetNodeID("aabbccddeeff")
	if err != nil {
		t.Fatalf("GetNodeID failed: %v", err)
	}
	if nodeID != 20 {
		t.Errorf("Expected node ID 20, got %d", nodeID)
	}

	// Test assigning to unknown UUID
	err = cids.AssignNodeID("unknown", 30)
	if err == nil {
		t.Error("Expected error for unknown UUID, got nil")
	}
}

func TestCANBusIDs_NodeCount(t *testing.T) {
	cids := &CANBusIDs{
		interface_: "can0",
		nodes:      make(map[string]*CANNode),
	}

	// Initially empty
	if count := cids.NodeCount(); count != 0 {
		t.Errorf("Expected 0 nodes, got %d", count)
	}

	// Add nodes
	cids.AddUUID("aabbccddeeff", "can0")
	if count := cids.NodeCount(); count != 1 {
		t.Errorf("Expected 1 node, got %d", count)
	}

	cids.AddUUID("112233445566", "can0")
	if count := cids.NodeCount(); count != 2 {
		t.Errorf("Expected 2 nodes, got %d", count)
	}
}

func TestCANBusIDs_GetStatus(t *testing.T) {
	cids := &CANBusIDs{
		interface_: "can0",
		nodes:      make(map[string]*CANNode),
	}

	// Add a node
	cids.AddUUID("aabbccddeeff", "can0")

	// Get status
	status := cids.GetStatus()

	if iface, ok := status["interface"].(string); !ok || iface != "can0" {
		t.Errorf("Expected interface 'can0', got %v", status["interface"])
	}

	nodes, ok := status["nodes"].([]map[string]any)
	if !ok {
		t.Fatalf("Expected nodes to be []map[string]any, got %T", status["nodes"])
	}
	if len(nodes) != 1 {
		t.Errorf("Expected 1 node, got %d", len(nodes))
	}
}

func TestCANBusIDs_Interface(t *testing.T) {
	cids := &CANBusIDs{
		interface_: "can1",
		nodes:      make(map[string]*CANNode),
	}

	if iface := cids.Interface(); iface != "can1" {
		t.Errorf("Expected interface 'can1', got '%s'", iface)
	}
}

func TestCANBusConstants(t *testing.T) {
	// Verify constants match expected values
	if CANBusAdminID != 0x3F0 {
		t.Errorf("Expected CANBusAdminID 0x3F0, got 0x%X", CANBusAdminID)
	}
	if CANBusAdminRespID != 0x3F1 {
		t.Errorf("Expected CANBusAdminRespID 0x3F1, got 0x%X", CANBusAdminRespID)
	}
	if CANBusNodeIDFirst != 4 {
		t.Errorf("Expected CANBusNodeIDFirst 4, got %d", CANBusNodeIDFirst)
	}
}
