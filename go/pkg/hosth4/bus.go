// Bus - port of klippy/extras/bus.py
//
// Helper code for SPI and I2C bus communication
//
// Copyright (C) 2018,2019 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// MCU_SPI represents an SPI bus connection.
type MCU_SPI struct {
	mcuName      string
	bus          string
	pin          string
	mode         int
	speed        int
	swPins       []string // Software SPI pins [miso, mosi, sclk]
	csActiveHigh bool
	oid          int
	mu           sync.Mutex
}

// SPIConfig holds configuration for an SPI connection.
type SPIConfig struct {
	MCUName      string
	Bus          string
	Pin          string   // CS pin
	Mode         int      // SPI mode (0-3)
	Speed        int      // Speed in Hz
	SwPins       []string // Software SPI pins [miso, mosi, sclk]
	CSActiveHigh bool
}

// DefaultSPIConfig returns default SPI configuration.
func DefaultSPIConfig() SPIConfig {
	return SPIConfig{
		Mode:  0,
		Speed: 100000,
	}
}

// newMCU_SPI creates a new SPI connection.
func newMCU_SPI(cfg SPIConfig) (*MCU_SPI, error) {
	if cfg.Speed < 100000 {
		return nil, fmt.Errorf("spi_speed must be at least 100000")
	}

	spi := &MCU_SPI{
		mcuName:      cfg.MCUName,
		bus:          cfg.Bus,
		pin:          cfg.Pin,
		mode:         cfg.Mode,
		speed:        cfg.Speed,
		swPins:       cfg.SwPins,
		csActiveHigh: cfg.CSActiveHigh,
	}

	log.Printf("spi: initialized on %s bus=%s pin=%s mode=%d speed=%d",
		cfg.MCUName, cfg.Bus, cfg.Pin, cfg.Mode, cfg.Speed)
	return spi, nil
}

// GetOID returns the object ID.
func (spi *MCU_SPI) GetOID() int {
	return spi.oid
}

// GetMCUName returns the MCU name.
func (spi *MCU_SPI) GetMCUName() string {
	return spi.mcuName
}

// IsSoftwareSPI returns true if using software SPI.
func (spi *MCU_SPI) IsSoftwareSPI() bool {
	return len(spi.swPins) > 0
}

// Send sends data over SPI without reading response.
func (spi *MCU_SPI) Send(data []byte) error {
	spi.mu.Lock()
	defer spi.mu.Unlock()

	// Note: Actual MCU command sending would happen here
	log.Printf("spi: send %d bytes", len(data))
	return nil
}

// Transfer sends data and receives response.
func (spi *MCU_SPI) Transfer(data []byte) ([]byte, error) {
	spi.mu.Lock()
	defer spi.mu.Unlock()

	// Note: Actual MCU command sending would happen here
	log.Printf("spi: transfer %d bytes", len(data))

	// Return dummy response
	return make([]byte, len(data)), nil
}

// MCU_I2C represents an I2C bus connection.
type MCU_I2C struct {
	mcuName string
	bus     string
	address int
	speed   int
	swPins  []string // Software I2C pins [scl, sda]
	oid     int
	mu      sync.Mutex
}

// I2CConfig holds configuration for an I2C connection.
type I2CConfig struct {
	MCUName string
	Bus     string
	Address int
	Speed   int
	SwPins  []string // Software I2C pins [scl, sda]
}

// DefaultI2CConfig returns default I2C configuration.
func DefaultI2CConfig() I2CConfig {
	return I2CConfig{
		Speed: 100000,
	}
}

// newMCU_I2C creates a new I2C connection.
func newMCU_I2C(cfg I2CConfig) (*MCU_I2C, error) {
	if cfg.Address < 0 || cfg.Address > 127 {
		return nil, fmt.Errorf("i2c_address must be 0-127")
	}
	if cfg.Speed < 100000 {
		return nil, fmt.Errorf("i2c_speed must be at least 100000")
	}

	i2c := &MCU_I2C{
		mcuName: cfg.MCUName,
		bus:     cfg.Bus,
		address: cfg.Address,
		speed:   cfg.Speed,
		swPins:  cfg.SwPins,
	}

	log.Printf("i2c: initialized on %s bus=%s addr=0x%02x speed=%d",
		cfg.MCUName, cfg.Bus, cfg.Address, cfg.Speed)
	return i2c, nil
}

// GetOID returns the object ID.
func (i2c *MCU_I2C) GetOID() int {
	return i2c.oid
}

// GetMCUName returns the MCU name.
func (i2c *MCU_I2C) GetMCUName() string {
	return i2c.mcuName
}

// GetAddress returns the I2C address.
func (i2c *MCU_I2C) GetAddress() int {
	return i2c.address
}

// IsSoftwareI2C returns true if using software I2C.
func (i2c *MCU_I2C) IsSoftwareI2C() bool {
	return len(i2c.swPins) > 0
}

// Write writes data to the I2C device.
func (i2c *MCU_I2C) Write(data []byte) error {
	i2c.mu.Lock()
	defer i2c.mu.Unlock()

	// Note: Actual MCU command sending would happen here
	log.Printf("i2c 0x%02x: write %d bytes", i2c.address, len(data))
	return nil
}

// WriteNoAck writes data without waiting for ACK.
func (i2c *MCU_I2C) WriteNoAck(data []byte) error {
	i2c.mu.Lock()
	defer i2c.mu.Unlock()

	// Note: Actual MCU command sending would happen here
	log.Printf("i2c 0x%02x: write_noack %d bytes", i2c.address, len(data))
	return nil
}

// Read reads data from the I2C device.
func (i2c *MCU_I2C) Read(reg []byte, readLen int) ([]byte, error) {
	i2c.mu.Lock()
	defer i2c.mu.Unlock()

	// Note: Actual MCU command sending would happen here
	log.Printf("i2c 0x%02x: read %d bytes from reg", i2c.address, readLen)

	// Return dummy response
	return make([]byte, readLen), nil
}

// MCU_bus_digital_out represents a GPIO that updates on a command queue.
type MCU_bus_digital_out struct {
	mcuName string
	pin     string
	value   bool
	oid     int
	mu      sync.Mutex
}

// BusDigitalOutConfig holds configuration for a bus-synchronized digital output.
type BusDigitalOutConfig struct {
	MCUName      string
	Pin          string
	InitialValue bool
}

// newMCU_bus_digital_out creates a new bus-synchronized digital output.
func newMCU_bus_digital_out(cfg BusDigitalOutConfig) (*MCU_bus_digital_out, error) {
	out := &MCU_bus_digital_out{
		mcuName: cfg.MCUName,
		pin:     cfg.Pin,
		value:   cfg.InitialValue,
	}

	log.Printf("bus_digital_out: initialized on %s pin=%s", cfg.MCUName, cfg.Pin)
	return out, nil
}

// GetOID returns the object ID.
func (out *MCU_bus_digital_out) GetOID() int {
	return out.oid
}

// GetMCUName returns the MCU name.
func (out *MCU_bus_digital_out) GetMCUName() string {
	return out.mcuName
}

// Update updates the digital output value.
func (out *MCU_bus_digital_out) Update(value bool) error {
	out.mu.Lock()
	defer out.mu.Unlock()

	out.value = value

	// Note: Actual MCU command sending would happen here
	log.Printf("bus_digital_out %s: set to %v", out.pin, value)
	return nil
}

// GetValue returns the current value.
func (out *MCU_bus_digital_out) GetValue() bool {
	out.mu.Lock()
	defer out.mu.Unlock()
	return out.value
}

// ResolveBusName resolves a bus name to its enumeration value.
func ResolveBusName(enumerations map[string]map[string]int, param, bus string) (string, error) {
	enums, ok := enumerations[param]
	if !ok {
		enums = enumerations["bus"]
	}

	if enums == nil {
		if bus == "" {
			return "", nil
		}
		return bus, nil
	}

	if bus == "" {
		// Find default (value 0)
		for k, v := range enums {
			if v == 0 {
				return k, nil
			}
		}
		return "", fmt.Errorf("must specify %s", param)
	}

	if _, ok := enums[bus]; !ok {
		return "", fmt.Errorf("unknown %s '%s'", param, bus)
	}

	return bus, nil
}
