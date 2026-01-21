// SamdSercom - port of klippy/extras/samd_sercom.py
//
// SAMD SERCOM pin configuration
//
// Copyright (C) 2019 Florian Heilmann <Florian.Heilmann@gmx.net>
// Copyright (C) 2026 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// SamdSercom configures SERCOM pins on SAMD microcontrollers.
// This module generates MCU config commands for set_sercom_pin.
type SamdSercom struct {
	sercom  string
	txPin   string
	rxPin   string
	clkPin  string
	mcuName string
	mu      sync.Mutex
}

// SamdSercomConfig holds configuration for SAMD SERCOM
type SamdSercomConfig struct {
	Sercom  string
	TxPin   string
	RxPin   string // optional
	ClkPin  string
	MCUName string // MCU name (default "mcu")
}

// NewSamdSercom creates a new SAMD SERCOM configuration
func NewSamdSercom(cfg SamdSercomConfig) (*SamdSercom, error) {
	if cfg.Sercom == "" {
		return nil, fmt.Errorf("samd_sercom: sercom must be specified")
	}
	if cfg.TxPin == "" {
		return nil, fmt.Errorf("samd_sercom: tx_pin must be specified")
	}
	if cfg.ClkPin == "" {
		return nil, fmt.Errorf("samd_sercom: clk_pin must be specified")
	}

	mcuName := cfg.MCUName
	if mcuName == "" {
		mcuName = "mcu"
	}

	ss := &SamdSercom{
		sercom:  cfg.Sercom,
		txPin:   cfg.TxPin,
		rxPin:   cfg.RxPin,
		clkPin:  cfg.ClkPin,
		mcuName: mcuName,
	}

	log.Printf("samd_sercom: initialized sercom=%s tx=%s clk=%s rx=%s mcu=%s",
		cfg.Sercom, cfg.TxPin, cfg.ClkPin, cfg.RxPin, mcuName)

	return ss, nil
}

// GetConfigCommands returns the MCU config commands for this SERCOM
func (ss *SamdSercom) GetConfigCommands() []string {
	ss.mu.Lock()
	defer ss.mu.Unlock()

	cmds := []string{
		// Configure TX pin
		fmt.Sprintf("set_sercom_pin bus=%s sercom_pin_type=tx pin=%s",
			ss.sercom, ss.txPin),
		// Configure CLK pin
		fmt.Sprintf("set_sercom_pin bus=%s sercom_pin_type=clk pin=%s",
			ss.sercom, ss.clkPin),
	}

	// Configure RX pin if specified
	if ss.rxPin != "" {
		cmds = append(cmds, fmt.Sprintf(
			"set_sercom_pin bus=%s sercom_pin_type=rx pin=%s",
			ss.sercom, ss.rxPin))
	}

	return cmds
}

// GetSercom returns the SERCOM name
func (ss *SamdSercom) GetSercom() string {
	return ss.sercom
}

// GetMCUName returns the MCU name
func (ss *SamdSercom) GetMCUName() string {
	return ss.mcuName
}

// GetStatus returns the SERCOM status
func (ss *SamdSercom) GetStatus() map[string]interface{} {
	return map[string]interface{}{
		"sercom":   ss.sercom,
		"tx_pin":   ss.txPin,
		"rx_pin":   ss.rxPin,
		"clk_pin":  ss.clkPin,
		"mcu_name": ss.mcuName,
	}
}
