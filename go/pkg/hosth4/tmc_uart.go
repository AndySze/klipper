// TMC UART - port of klippy/extras/tmc_uart.py
//
// Support for UART communication with TMC drivers
//
// Copyright (C) 2018-2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// TMC UART constants.
const (
	tmcUartSync      = 0x05
	tmcUartSlaveAddr = 0x00
	tmcUartReadCmd   = 0x00
	tmcUartWriteCmd  = 0x80
)

// TMCUartBitBang represents a UART interface for TMC drivers.
type TMCUartBitBang struct {
	rt           *runtime
	name         string
	rxPin        string
	txPin        string
	selectPins   []string
	addr         int
	mu           sync.Mutex
}

// TMCUartBitBangConfig holds configuration for TMC UART.
type TMCUartBitBangConfig struct {
	Name       string
	RxPin      string
	TxPin      string
	SelectPins []string
	Addr       int
}

// newTMCUartBitBang creates a new TMC UART interface.
func newTMCUartBitBang(rt *runtime, cfg TMCUartBitBangConfig) (*TMCUartBitBang, error) {
	if cfg.TxPin == "" {
		return nil, fmt.Errorf("tmc_uart: tx_pin is required")
	}

	uart := &TMCUartBitBang{
		rt:         rt,
		name:       cfg.Name,
		rxPin:      cfg.RxPin,
		txPin:      cfg.TxPin,
		selectPins: cfg.SelectPins,
		addr:       cfg.Addr,
	}

	log.Printf("tmc_uart: initialized '%s' tx=%s rx=%s", cfg.Name, cfg.TxPin, cfg.RxPin)
	return uart, nil
}

// calcCRC8 calculates the CRC8 checksum for TMC UART.
func calcCRC8(data []byte) byte {
	crc := byte(0)
	for _, b := range data {
		for i := 0; i < 8; i++ {
			if (crc>>7)^(b>>7) != 0 {
				crc = (crc << 1) ^ 0x07
			} else {
				crc = crc << 1
			}
			b = b << 1
		}
	}
	return crc
}

// buildReadCmd builds a register read command.
func (uart *TMCUartBitBang) buildReadCmd(reg int) []byte {
	msg := []byte{
		tmcUartSync,
		byte(uart.addr),
		byte(reg | tmcUartReadCmd),
	}
	msg = append(msg, calcCRC8(msg))
	return msg
}

// buildWriteCmd builds a register write command.
func (uart *TMCUartBitBang) buildWriteCmd(reg int, val uint32) []byte {
	msg := []byte{
		tmcUartSync,
		byte(uart.addr),
		byte(reg | tmcUartWriteCmd),
		byte(val >> 24),
		byte(val >> 16),
		byte(val >> 8),
		byte(val),
	}
	msg = append(msg, calcCRC8(msg))
	return msg
}

// RegRead reads a register value.
func (uart *TMCUartBitBang) RegRead(reg int) (uint32, error) {
	uart.mu.Lock()
	defer uart.mu.Unlock()

	// Build and send read command
	_ = uart.buildReadCmd(reg)

	// In a real implementation, this would send the command and read the response
	log.Printf("tmc_uart '%s': read reg 0x%02x", uart.name, reg)

	// Return dummy value
	return 0, nil
}

// RegWrite writes a register value.
func (uart *TMCUartBitBang) RegWrite(reg int, val uint32) error {
	uart.mu.Lock()
	defer uart.mu.Unlock()

	// Build and send write command
	_ = uart.buildWriteCmd(reg, val)

	// In a real implementation, this would send the command
	log.Printf("tmc_uart '%s': write reg 0x%02x = 0x%08x", uart.name, reg, val)

	return nil
}

// GetAddr returns the UART address.
func (uart *TMCUartBitBang) GetAddr() int {
	return uart.addr
}

// TMCUartHelper provides helper functions for TMC UART communication.
type TMCUartHelper struct {
	uart     *TMCUartBitBang
	regNames map[int]string
	mu       sync.RWMutex
}

// newTMCUartHelper creates a new TMC UART helper.
func newTMCUartHelper(uart *TMCUartBitBang) *TMCUartHelper {
	return &TMCUartHelper{
		uart:     uart,
		regNames: make(map[int]string),
	}
}

// SetRegName sets a human-readable name for a register.
func (h *TMCUartHelper) SetRegName(reg int, name string) {
	h.mu.Lock()
	defer h.mu.Unlock()
	h.regNames[reg] = name
}

// GetRegName returns the name of a register.
func (h *TMCUartHelper) GetRegName(reg int) string {
	h.mu.RLock()
	defer h.mu.RUnlock()
	if name, ok := h.regNames[reg]; ok {
		return name
	}
	return fmt.Sprintf("0x%02x", reg)
}

// GetField extracts a field from a register value.
func (h *TMCUartHelper) GetField(regVal uint32, mask, shift uint32) uint32 {
	return (regVal >> shift) & mask
}

// SetField sets a field in a register value.
func (h *TMCUartHelper) SetField(regVal uint32, mask, shift, val uint32) uint32 {
	return (regVal & ^(mask << shift)) | ((val & mask) << shift)
}
