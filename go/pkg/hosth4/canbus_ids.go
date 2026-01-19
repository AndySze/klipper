// CAN Bus IDs - port of klippy/extras/canbus_ids.py
//
// Support for CAN bus identifier assignment
//
// Copyright (C) 2020 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// CAN bus constants.
const (
	canBusAdminID     = 0x3F0 // Admin message ID
	canBusAdminRespID = 0x3F1 // Admin response ID
)

// CAN bus admin commands.
const (
	canBusCmdQueryUUID  = 0x00
	canBusCmdSetNodeID  = 0x01
	canBusCmdGetNodeID  = 0x02
	canBusCmdRebootNode = 0x03
)

// CANBusIDs manages CAN bus node identification.
type CANBusIDs struct {
	rt          *runtime
	interface_  string
	nodes       map[string]*CANNode
	mu          sync.Mutex
}

// CANNode represents a CAN bus node.
type CANNode struct {
	uuid   string
	nodeID int
	name   string
}

// CANBusIDsConfig holds configuration for CANBusIDs.
type CANBusIDsConfig struct {
	Interface string
}

// newCANBusIDs creates a new CAN bus ID manager.
func newCANBusIDs(rt *runtime, cfg CANBusIDsConfig) (*CANBusIDs, error) {
	if cfg.Interface == "" {
		cfg.Interface = "can0"
	}

	cids := &CANBusIDs{
		rt:         rt,
		interface_: cfg.Interface,
		nodes:      make(map[string]*CANNode),
	}

	log.Printf("canbus_ids: initialized on %s", cfg.Interface)
	return cids, nil
}

// RegisterNode registers a CAN node by UUID.
func (cids *CANBusIDs) RegisterNode(uuid string, nodeID int, name string) error {
	cids.mu.Lock()
	defer cids.mu.Unlock()

	if _, exists := cids.nodes[uuid]; exists {
		return fmt.Errorf("canbus_ids: node %s already registered", uuid)
	}

	cids.nodes[uuid] = &CANNode{
		uuid:   uuid,
		nodeID: nodeID,
		name:   name,
	}

	log.Printf("canbus_ids: registered node %s (id=%d, name=%s)", uuid, nodeID, name)
	return nil
}

// GetNodeByUUID returns a node by its UUID.
func (cids *CANBusIDs) GetNodeByUUID(uuid string) (*CANNode, error) {
	cids.mu.Lock()
	defer cids.mu.Unlock()

	node, exists := cids.nodes[uuid]
	if !exists {
		return nil, fmt.Errorf("canbus_ids: node %s not found", uuid)
	}
	return node, nil
}

// GetNodeByID returns a node by its node ID.
func (cids *CANBusIDs) GetNodeByID(nodeID int) (*CANNode, error) {
	cids.mu.Lock()
	defer cids.mu.Unlock()

	for _, node := range cids.nodes {
		if node.nodeID == nodeID {
			return node, nil
		}
	}
	return nil, fmt.Errorf("canbus_ids: node with id %d not found", nodeID)
}

// QueryNodes sends a query to discover CAN nodes.
func (cids *CANBusIDs) QueryNodes() error {
	// In real implementation, this would send CAN admin messages
	log.Printf("canbus_ids: querying nodes on %s", cids.interface_)
	return nil
}

// AssignNodeID assigns a node ID to a UUID.
func (cids *CANBusIDs) AssignNodeID(uuid string, nodeID int) error {
	cids.mu.Lock()
	defer cids.mu.Unlock()

	node, exists := cids.nodes[uuid]
	if !exists {
		return fmt.Errorf("canbus_ids: node %s not found", uuid)
	}

	node.nodeID = nodeID
	log.Printf("canbus_ids: assigned node ID %d to %s", nodeID, uuid)
	return nil
}

// GetStatus returns the CAN bus ID manager status.
func (cids *CANBusIDs) GetStatus() map[string]any {
	cids.mu.Lock()
	defer cids.mu.Unlock()

	nodeList := make([]map[string]any, 0, len(cids.nodes))
	for _, node := range cids.nodes {
		nodeList = append(nodeList, map[string]any{
			"uuid":    node.uuid,
			"node_id": node.nodeID,
			"name":    node.name,
		})
	}

	return map[string]any{
		"interface": cids.interface_,
		"nodes":     nodeList,
	}
}
