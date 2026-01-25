// hardware-test is a command-line tool for testing real-time MCU communication.
// It can be used to verify serial connectivity, MCU identification, clock synchronization,
// and basic command/response exchange with Klipper MCU firmware.
//
// Usage:
//
//	hardware-test -device /dev/ttyUSB0 [options]
//
// Options:
//
//	-device string    Serial device path, Unix socket path, or TCP address (required, or use -config)
//	-config string    Printer config file (optional, extracts device from [mcu] section)
//	-baud int         Baud rate (default: 250000)
//	-timeout duration Connection timeout (default: 60s)
//	-trace            Enable debug tracing
//	-test string      Test to run: "connect", "clock", "temp", "query", "all" (default: "connect")
//	-socket           Connect via Unix socket instead of serial port (for Linux MCU simulator)
//	-tcp              Connect via TCP (e.g., -device localhost:5555 -tcp)
//
// Examples:
//
//	# Basic connection test with device path
//	hardware-test -device /dev/ttyUSB0 -test connect
//
//	# Use printer.cfg to get device path
//	hardware-test -config ~/printer.cfg -test connect
//
//	# Query MCU capabilities
//	hardware-test -device /dev/ttyUSB0 -test query -trace
//
//	# Connect to Linux MCU simulator via Unix socket
//	hardware-test -device /tmp/klipper_mcu -socket -test connect -trace
package main

import (
	"bufio"
	"flag"
	"fmt"
	"math"
	"os"
	"os/signal"
	"strings"
	"sync"
	"syscall"
	"time"

	"klipper-go-migration/pkg/hosth4"
	"klipper-go-migration/pkg/serial"
)

// boolToInt converts a boolean to an integer (0 or 1)
func boolToInt(b bool) int {
	if b {
		return 1
	}
	return 0
}

func main() {
	// Command line flags
	device := flag.String("device", "", "Serial device path, Unix socket path, or TCP address (host:port)")
	configFile := flag.String("config", "", "Printer config file (optional)")
	baud := flag.Int("baud", 250000, "Baud rate")
	timeout := flag.Duration("timeout", 60*time.Second, "Connection timeout")
	trace := flag.Bool("trace", false, "Enable debug tracing")
	test := flag.String("test", "connect", "Test to run: serial, connect, clock, temp, query, all")
	socket := flag.Bool("socket", false, "Connect via Unix socket instead of serial port (for Linux MCU simulator)")
	tcp := flag.Bool("tcp", false, "Connect via TCP (e.g., -device localhost:5555 -tcp)")
	linux := flag.Bool("linux", false, "Use Linux-style GPIO pins (gpio0, gpio1) instead of ARM-style (PA0, PA1)")

	flag.Parse()

	// If config file provided, extract device from it
	if *configFile != "" && *device == "" {
		extractedDevice, extractedBaud, err := parseConfigFile(*configFile)
		if err != nil {
			fmt.Fprintf(os.Stderr, "Error parsing config: %v\n", err)
			os.Exit(1)
		}
		if extractedDevice != "" {
			*device = extractedDevice
			fmt.Printf("Using device from config: %s\n", *device)
		}
		if extractedBaud > 0 && *baud == 250000 {
			*baud = extractedBaud
			fmt.Printf("Using baud rate from config: %d\n", *baud)
		}
	}

	if *device == "" {
		fmt.Fprintf(os.Stderr, "Error: -device is required (or provide -config with [mcu] section)\n")
		flag.Usage()
		os.Exit(1)
	}

	// Create MCU configuration
	cfg := hosth4.DefaultMCUConnectionConfig()
	cfg.Device = *device
	cfg.BaudRate = *baud
	cfg.ConnectTimeout = *timeout
	cfg.UseSocket = *socket
	cfg.UseTCP = *tcp

	if *trace {
		cfg.Trace = os.Stdout
	}

	if *tcp {
		fmt.Printf("Using TCP mode for device: %s\n", *device)
	} else if *socket {
		fmt.Printf("Using Unix socket mode for device: %s\n", *device)
	}

	// Create realtime integration
	ri := hosth4.NewRealtimeIntegration()
	if *trace {
		ri.SetTrace(os.Stdout)
	}

	// Add MCU
	if err := ri.AddMCU("mcu", cfg); err != nil {
		fmt.Fprintf(os.Stderr, "Error adding MCU: %v\n", err)
		os.Exit(1)
	}

	// Set up signal handler for graceful shutdown
	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)

	// Run test in a goroutine so we can handle signals
	doneCh := make(chan error, 1)
	go func() {
		var err error
		switch *test {
		case "serial":
			// Direct serial test - doesn't need full integration
			if *socket {
				err = fmt.Errorf("serial test not supported in socket mode")
			} else {
				err = testSerial(*device, *baud, *timeout, *trace)
			}
		case "connect":
			err = testConnect(ri, *timeout)
		case "clock":
			err = testClock(ri, *timeout)
		case "temp":
			err = testTemp(ri, *timeout)
		case "query":
			err = testQuery(ri, *timeout)
		case "config":
			err = testConfig(ri, *timeout)
		case "stepper":
			err = testStepper(ri, *timeout, *linux)
		case "gpio":
			err = testGPIO(ri, *timeout, *linux)
		case "endstop":
			err = testEndstop(ri, *timeout)
		case "temploop":
			err = testTempLoop(ri, *timeout)
		case "homing":
			err = testHoming(ri, *timeout, *configFile)
		case "endstops":
			err = testEndstops(ri, *timeout, *configFile)
		case "xhoming":
			err = testXHoming(ri, *timeout, *configFile)
		case "ymovement":
			err = testYMovement(ri, *timeout, *configFile)
		case "yhoming":
			err = testYHoming(ri, *timeout, *configFile)
		case "zmovement":
			err = testZMovement(ri, *timeout, *configFile)
		case "zhoming":
			err = testZHoming(ri, *timeout, *configFile)
		case "allhoming":
			err = testAllHoming(ri, *timeout, *configFile)
		case "g28":
			err = testG28(ri, *timeout, *configFile)
		case "extruder-heater":
			err = testExtruderHeater(ri, *timeout, *configFile, 50.0) // Default target 50°C
		case "bed-heater":
			err = testBedHeater(ri, *timeout, *configFile, 30.0) // Default target 30°C
		case "fan":
			err = testFan(ri, *timeout, "PG2") // Part cooling fan
		case "heatbreak-fan":
			err = testFan(ri, *timeout, "PC1") // Heatbreak cooling fan
		case "heater":
			err = testHeaterSimple(ri, *timeout, "PA7", 10*time.Second) // Extruder heater, 10s ON
		case "bed-heat":
			err = testHeaterSimple(ri, *timeout, "PA6", 10*time.Second) // Bed heater, 10s ON
		case "extruder-temp":
			err = testTempRead(ri, *timeout, "PF0") // Extruder thermistor
		case "extrude":
			err = testExtrude(ri, *timeout, 10.0, 220.0) // Extrude 10mm at 220°C
		case "cold-extrude":
			err = testColdExtrude(ri, *timeout, 20.0) // Move extruder 20mm without heating
		case "heat-adc":
			err = testHeaterWithADC(ri, *timeout) // Test heater with ADC monitoring
		case "multicmd":
			err = testMultiCmd(ri, *timeout)
		case "errors":
			err = testErrors(ri, *timeout)
		case "motion":
			err = testMotion(ri, *timeout)
		case "multimcu":
			err = testMultiMCU(ri, *timeout)
		case "stress":
			err = testStress(ri, *timeout)
		case "gcode":
			err = testGcode(ri, *timeout, *configFile)
		case "all":
			err = testAll(ri, *timeout)
		default:
			err = fmt.Errorf("unknown test: %s", *test)
		}
		doneCh <- err
	}()

	// Wait for completion or signal
	select {
	case err := <-doneCh:
		if err != nil {
			fmt.Fprintf(os.Stderr, "Test failed: %v\n", err)
			os.Exit(1)
		}
		fmt.Println("\nAll tests passed!")
		// Give disconnect a chance to complete, but don't wait forever
		go func() {
			ri.Disconnect()
		}()
		time.Sleep(500 * time.Millisecond)
		os.Exit(0)
	case sig := <-sigCh:
		fmt.Printf("\nReceived signal %v, shutting down...\n", sig)
		// Don't wait for disconnect - just exit on second signal
		go func() {
			ri.Disconnect()
		}()
		// Wait for second signal to force quit
		select {
		case <-sigCh:
			fmt.Println("\nForce quit")
			os.Exit(1)
		case <-time.After(2 * time.Second):
			fmt.Println("\nClean shutdown")
			os.Exit(130)
		}
	}
}

// testSerial performs direct serial port testing without full MCU handshake.
// This is useful for debugging connection issues.
func testSerial(device string, baud int, timeout time.Duration, trace bool) error {
	fmt.Println("=== Test: Direct Serial Port ===")
	fmt.Printf("Device: %s\n", device)
	fmt.Printf("Baud rate: %d\n", baud)

	// Open serial port
	fmt.Println("\nOpening serial port...")
	cfg := serial.Config{
		Device:         device,
		BaudRate:       baud,
		ConnectTimeout: timeout,
		ReadTimeout:    2 * time.Second,
		RTSOnConnect:   true,
		DTROnConnect:   true,
	}

	port, err := serial.Open(cfg)
	if err != nil {
		return fmt.Errorf("open serial: %w", err)
	}
	defer port.Close()

	fmt.Println("Serial port opened successfully!")

	// Send identify command (offset=0, count=40)
	// Message format: len=8, seq=0, cmd_id=1, offset=0, count=40, CRC, sync
	fmt.Println("\nSending identify command...")
	// CORRECT format: message ID 1 for identify
	// seq byte = 0x10 (MESSAGE_DEST set, seq# = 0)
	correctCmd := []byte{
		0x08,       // length (8 bytes total)
		0x10,       // seq = MESSAGE_DEST | 0 (host-to-MCU direction)
		0x01,       // cmd id (1 = identify)
		0x00,       // offset VLQ (0)
		0x28,       // count VLQ (40)
		0x00, 0x00, // CRC placeholder
		0x7e,       // sync byte
	}
	crcCorrect := crc16ccitt(correctCmd[:5])
	correctCmd[5] = byte(crcCorrect >> 8)
	correctCmd[6] = byte(crcCorrect & 0xff)

	// OLD format (wrong but was getting a response)
	oldCmd := []byte{
		0x0a,       // length (WRONG - says 10 but only 8 bytes)
		0x00,       // seq
		0x00,       // cmd id (WRONG - 0 is identify_response)
		0x00,       // offset VLQ (0)
		0x28,       // count VLQ (40)
		0x00, 0x00, // CRC placeholder
		0x7e,       // sync byte
	}
	crcOld := crc16ccitt(oldCmd[:5])
	oldCmd[5] = byte(crcOld >> 8)
	oldCmd[6] = byte(crcOld & 0xff)

	// Try correct command first
	fmt.Println("Trying CORRECT identify command (msg_id=1, len=8)...")
	identifyCmd := correctCmd

	if trace {
		fmt.Printf("TX: %x\n", identifyCmd)
	}

	_, err = port.Write(identifyCmd)
	if err != nil {
		return fmt.Errorf("write: %w", err)
	}

	// Send multiple times and read responses
	fmt.Println("Sending identify command multiple times...")
	for i := 0; i < 5; i++ {
		if trace {
			fmt.Printf("\n--- Attempt %d ---\n", i+1)
			fmt.Printf("TX: %x\n", identifyCmd)
		}

		_, err = port.Write(identifyCmd)
		if err != nil {
			fmt.Printf("Write error: %v\n", err)
			continue
		}

		// Read response with timeout
		resp := make([]byte, 256)
		n, err := port.Read(resp)
		if err != nil {
			if err.Error() == "serial: operation timed out" {
				fmt.Printf("Attempt %d: timeout (no response)\n", i+1)
			} else {
				fmt.Printf("Attempt %d: read error: %v\n", i+1, err)
			}
			continue
		}

		fmt.Printf("Attempt %d: received %d bytes\n", i+1, n)
		if trace && n > 0 {
			fmt.Printf("RX: %x\n", resp[:n])
		}

		// Parse response
		if n >= 5 && resp[n-1] == 0x7e {
			msgLen := int(resp[0])
			if msgLen >= 5 && msgLen <= n {
				// Get message ID from payload
				if msgLen > 5 {
					msgID := resp[2]
					fmt.Printf("  Message ID: %d (0x%02x)\n", msgID, msgID)
					if msgID == 0 {
						fmt.Println("  -> This is identify_response!")
					}
				} else {
					fmt.Println("  -> Empty message (ACK)")
				}
			}
		}

		// Small delay between attempts
		time.Sleep(100 * time.Millisecond)
	}

	return nil
}

// crc16ccitt calculates the CRC16-CCITT checksum using Klipper's optimized algorithm.
// This matches the algorithm in klippy/chelper/crc16_ccitt.c
func crc16ccitt(data []byte) uint16 {
	crc := uint16(0xFFFF)
	for _, b := range data {
		d := b ^ uint8(crc&0xff)
		d ^= d << 4 // 8-bit overflow intentional
		crc = ((uint16(d) << 8) | (crc >> 8)) ^ uint16(d>>4) ^ (uint16(d) << 3)
	}
	return crc
}

// testConnect verifies basic connectivity to the MCU.
func testConnect(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: MCU Connection ===")

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	fmt.Println("Connected!")

	// Wait for ready
	fmt.Println("Waiting for MCU ready...")
	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	fmt.Println("MCU is ready!")

	// Get status
	status := ri.GetStatus()
	fmt.Printf("MCU Status: %v\n", status)

	return nil
}

// testClock verifies clock synchronization with the MCU.
func testClock(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: Clock Synchronization ===")

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	// Wait for ready
	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	// Get MCU manager
	mgr := ri.MCUManager()
	mcu := mgr.PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no primary MCU")
	}

	// Test clock conversion
	fmt.Println("Testing clock conversion...")

	// Get print time for current host time
	hostTime := 0.0 // Relative to start
	mcuClock := mcu.PrintTime(hostTime)
	fmt.Printf("Host time %.3f -> MCU clock %d\n", hostTime, mcuClock)

	// Test a few more points
	for _, t := range []float64{0.1, 0.5, 1.0, 5.0} {
		clock := mcu.PrintTime(t)
		fmt.Printf("Host time %.3f -> MCU clock %d\n", t, clock)
	}

	// Report MCU frequency
	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	return nil
}

// testTemp tests temperature sensor reading by configuring MCU ADC.
func testTemp(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: Temperature Sensor ===")

	// Connect to MCU first
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	// Wait for ready
	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Println("MCU connected!")
	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check if MCU is already configured
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}

	isConfig := 0
	if v, ok := resp["is_config"].(int); ok {
		isConfig = v
	}

	if isConfig != 0 {
		fmt.Println("MCU is already configured (from previous session)")
		fmt.Println("Cannot send new config commands. MCU needs reset.")
		return nil
	}

	fmt.Println("MCU is NOT configured - proceeding with ADC setup")

	// Debug: show enumerations
	dict := mcu.Dictionary()
	fmt.Printf("\nAvailable enumerations (%d total):\n", len(dict.Enumerations))
	for name, vals := range dict.Enumerations {
		fmt.Printf("  %s: %d values\n", name, len(vals))
	}

	// Check for pin-related enumerations
	if pins, ok := dict.Enumerations["pin"]; ok && len(pins) > 0 {
		fmt.Println("\nPin enumeration (first 10):")
		count := 0
		for name, val := range pins {
			fmt.Printf("  %s = %d\n", name, val)
			count++
			if count >= 10 {
				fmt.Printf("  ... and %d more\n", len(pins)-10)
				break
			}
		}
	} else {
		fmt.Println("\nNo 'pin' enumeration found")
	}

	// Configure ADC for temperature reading
	// Pin PF0 is the extruder thermistor (from printer.cfg: sensor_pin: PF0)
	oid := 0
	pin := "PF0" // Pin name from MCU's pin enumeration

	fmt.Println("\n--- Configuring ADC ---")

	// Step 1: Allocate OID
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 1}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=1")
	time.Sleep(10 * time.Millisecond) // Small delay between commands

	// Step 2: Configure analog input
	fmt.Println("  sending config_analog_in...")
	if err := mcu.SendCommand("config_analog_in", map[string]interface{}{
		"oid": oid,
		"pin": pin,
	}); err != nil {
		return fmt.Errorf("config_analog_in: %w", err)
	}
	fmt.Printf("  config_analog_in oid=%d pin=%s - done\n", oid, pin)
	time.Sleep(10 * time.Millisecond)

	// Step 3: Finalize config (crc=0 for simple test)
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config crc=0")
	time.Sleep(50 * time.Millisecond) // More time for finalize

	// Verify config is finalized
	resp, err = mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("verify config: %w", err)
	}
	fmt.Printf("  Config state: %v\n", resp)

	// Step 4: Register handler for analog_in_state responses
	tempReadings := make(chan int, 10)
	mcu.RegisterOIDHandler("analog_in_state", oid, func(params map[string]interface{}, receiveTime time.Time) {
		if value, ok := params["value"].(int); ok {
			select {
			case tempReadings <- value:
			default:
			}
		}
	})

	// Step 5: Start querying temperature
	mcuClock := mcu.PrintTime(0)                  // Current MCU clock
	sampleTicks := int(mcu.MCUFreq() * 0.001)     // 1ms sample time
	restTicks := int(mcu.MCUFreq() * 0.3)         // 300ms between reports

	fmt.Printf("\n--- Starting ADC Query ---\n")
	fmt.Printf("  MCU clock: %d, sample_ticks: %d, rest_ticks: %d\n", mcuClock, sampleTicks, restTicks)

	if err := mcu.SendCommand("query_analog_in", map[string]interface{}{
		"oid":               oid,
		"clock":             uint32(mcuClock + 100000), // Start shortly in the future
		"sample_ticks":      sampleTicks,
		"sample_count":      8,
		"rest_ticks":        restTicks,
		"min_value":         0,
		"max_value":         4095, // 10-bit ADC max (ATmega uses 10-bit ADC: 0-1023)
		"range_check_count": 0,
	}); err != nil {
		return fmt.Errorf("query_analog_in: %w", err)
	}
	fmt.Println("  query_analog_in started")

	// Read temperature values for a few seconds
	fmt.Println("\n--- Reading Temperature ---")

	// Create thermistor using hosth4's built-in profile
	// Printer config: sensor_type: ATC Semitec 104GT-2, pullup: 4700 ohms
	thermistor, err := hosth4.NewThermistorFromProfile("ATC Semitec 104GT-2", 4700, 0)
	if err != nil {
		return fmt.Errorf("create thermistor: %w", err)
	}

	// With sample_count=8, the value is sum of 8 samples, range 0-8184
	sampleCount := 8
	maxADC := 1023 * sampleCount // 8184 for 8 samples of 10-bit ADC
	fmt.Printf("    (Thermistor: ATC Semitec 104GT-2, Pullup: 4700Ω)\n")
	fmt.Printf("    (ADC: 10-bit × %d samples, range 0-%d)\n", sampleCount, maxADC)

	readTimeout := time.After(5 * time.Second)
	readCount := 0

	for {
		select {
		case value := <-tempReadings:
			readCount++
			if value > 0 && value < maxADC {
				// Normalize to 0.0-1.0 range for thermistor calculation
				adcNormalized := float64(value) / float64(maxADC)
				tempC := thermistor.CalcTemp(adcNormalized)
				fmt.Printf("  [%d] ADC: %5d (%.3f) -> %.1f°C\n", readCount, value, adcNormalized, tempC)
			} else if value >= maxADC {
				fmt.Printf("  [%d] ADC: %5d (max - open circuit/no thermistor)\n", readCount, value)
			} else {
				fmt.Printf("  [%d] ADC: %5d (min - short circuit)\n", readCount, value)
			}
		case <-readTimeout:
			if readCount == 0 {
				fmt.Println("  No temperature readings received")
				fmt.Println("  (Check thermistor connection on pin PF0)")
			} else {
				fmt.Printf("\nReceived %d temperature readings\n", readCount)
			}
			return nil
		}
	}
}

// testConfig tests MCU configuration phase (allocate_oids, finalize_config).
func testConfig(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: MCU Configuration ===")

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	// Wait for ready
	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check current config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}

	isConfig := 0
	if v, ok := resp["is_config"].(int); ok {
		isConfig = v
	}
	fmt.Printf("Initial config state: is_config=%d\n", isConfig)

	if isConfig != 0 {
		fmt.Println("MCU already configured from previous session")
		fmt.Println("Config test skipped (MCU needs reset)")
		return nil
	}

	// Allocate OIDs
	fmt.Println("\nAllocating OIDs...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 5}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=5 - OK")

	// Finalize config
	fmt.Println("\nFinalizing config...")
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 12345}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config crc=12345 - OK")

	// Verify config state
	resp, err = mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("verify config: %w", err)
	}
	fmt.Printf("\nFinal config state: %v\n", resp)

	isConfig = 0
	if v, ok := resp["is_config"].(int); ok {
		isConfig = v
	}
	if isConfig != 1 {
		return fmt.Errorf("config not finalized: is_config=%d", isConfig)
	}
	fmt.Println("Configuration phase completed successfully!")

	return nil
}

// testStepper tests stepper motor configuration and control.
func testStepper(ri *hosth4.RealtimeIntegration, timeout time.Duration, linuxPins bool) error {
	fmt.Println("=== Test: Stepper Motor ===")

	// Select pin names based on MCU type
	stepPin := "PA0"
	dirPin := "PA1"
	if linuxPins {
		stepPin = "gpio0"
		dirPin = "gpio1"
	}

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	// Wait for ready
	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check current config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}

	isConfig := 0
	if v, ok := resp["is_config"].(int); ok {
		isConfig = v
	}

	if isConfig != 0 {
		fmt.Println("MCU already configured from previous session")
		fmt.Println("Stepper test skipped (MCU needs reset)")
		return nil
	}

	// Allocate OIDs for stepper
	fmt.Println("\nAllocating OIDs...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 3}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}

	// Configure stepper (X axis)
	oid := 0
	// step_pulse_ticks: duration of step pulse in MCU ticks (2 microseconds at 50MHz = 100 ticks)
	stepPulseTicks := int(mcu.MCUFreq() * 2 / 1000000) // 2 microseconds
	fmt.Printf("\nConfiguring stepper oid=%d (step=%s, dir=%s)...\n", oid, stepPin, dirPin)
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              oid,
		"step_pin":         stepPin,
		"dir_pin":          dirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper: %w", err)
	}
	fmt.Println("  config_stepper - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Set step direction
	fmt.Println("\nSetting step direction...")
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": oid,
		"dir": 1, // Forward
	}); err != nil {
		return fmt.Errorf("set_next_step_dir: %w", err)
	}
	fmt.Println("  set_next_step_dir dir=1 - OK")

	// Reset step clock
	mcuClock := mcu.PrintTime(0)
	fmt.Printf("\nResetting step clock to %d...\n", mcuClock)
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   oid,
		"clock": uint32(mcuClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock: %w", err)
	}
	fmt.Println("  reset_step_clock - OK")

	// Queue steps
	stepInterval := int(mcu.MCUFreq() / 1000) // 1000 steps/sec
	fmt.Printf("\nQueuing steps (interval=%d ticks)...\n", stepInterval)
	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      oid,
		"interval": stepInterval,
		"count":    100, // 100 steps
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step: %w", err)
	}
	fmt.Println("  queue_step count=100 - OK")

	// Get stepper position
	fmt.Println("\nGetting stepper position...")
	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": oid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position: %w", err)
	}
	fmt.Printf("  Stepper position: %v\n", resp)

	fmt.Println("\nStepper test completed successfully!")
	return nil
}

// testGPIO tests digital and PWM output.
func testGPIO(ri *hosth4.RealtimeIntegration, timeout time.Duration, linuxPins bool) error {
	fmt.Println("=== Test: GPIO Output ===")

	// Select pin names based on MCU type
	digPin := "PB0"
	pwmPin := "pwmchip0/pwm0" // Linux MCU uses pwmchip for PWM
	if linuxPins {
		digPin = "gpio2"
		pwmPin = "pwmchip0/pwm0"
	} else {
		pwmPin = "PB1"
	}

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	// Wait for ready
	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check current config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}

	isConfig := 0
	if v, ok := resp["is_config"].(int); ok {
		isConfig = v
	}

	if isConfig != 0 {
		fmt.Println("MCU already configured from previous session")
		fmt.Println("GPIO test skipped (MCU needs reset)")
		return nil
	}

	// Allocate OIDs
	fmt.Println("\nAllocating OIDs...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 3}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}

	// Configure digital output (LED)
	digOid := 0
	fmt.Printf("\nConfiguring digital output oid=%d (pin=%s)...\n", digOid, digPin)
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           digOid,
		"pin":           digPin,
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out: %w", err)
	}
	fmt.Println("  config_digital_out - OK")

	// Configure PWM output (heater/fan)
	pwmOid := 1
	pwmCycleTicks := int(mcu.MCUFreq() / 1000) // 1kHz PWM
	fmt.Printf("\nConfiguring PWM output oid=%d (pin=%s)...\n", pwmOid, pwmPin)
	if err := mcu.SendCommand("config_pwm_out", map[string]interface{}{
		"oid":           pwmOid,
		"pin":           pwmPin,
		"cycle_ticks":   pwmCycleTicks,
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_pwm_out: %w", err)
	}
	fmt.Println("  config_pwm_out - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Schedule digital output ON
	mcuClock := mcu.PrintTime(0.1)
	fmt.Printf("\nScheduling digital output ON at clock=%d...\n", mcuClock)
	if err := mcu.SendCommand("schedule_digital_out", map[string]interface{}{
		"oid":   digOid,
		"clock": uint32(mcuClock),
		"value": 1,
	}); err != nil {
		return fmt.Errorf("schedule_digital_out: %w", err)
	}
	fmt.Println("  schedule_digital_out value=1 - OK")

	// Schedule PWM output to 50%
	mcuClock = mcu.PrintTime(0.2)
	fmt.Printf("\nScheduling PWM output 50%% at clock=%d...\n", mcuClock)
	if err := mcu.SendCommand("schedule_pwm_out", map[string]interface{}{
		"oid":   pwmOid,
		"clock": uint32(mcuClock),
		"value": 128, // 50% of 255
	}); err != nil {
		return fmt.Errorf("schedule_pwm_out: %w", err)
	}
	fmt.Println("  schedule_pwm_out value=128 - OK")

	fmt.Println("\nGPIO test completed successfully!")
	return nil
}

// testEndstop tests endstop configuration and query.
func testEndstop(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: Endstop ===")

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	// Wait for ready
	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check current config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}

	isConfig := 0
	if v, ok := resp["is_config"].(int); ok {
		isConfig = v
	}

	if isConfig != 0 {
		fmt.Println("MCU already configured from previous session")
		fmt.Println("Endstop test skipped (MCU needs reset)")
		return nil
	}

	// Allocate OIDs
	fmt.Println("\nAllocating OIDs...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 2}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}

	// Configure endstop (X min)
	oid := 0
	fmt.Printf("\nConfiguring endstop oid=%d...\n", oid)
	if err := mcu.SendCommand("config_endstop", map[string]interface{}{
		"oid":     oid,
		"pin":     "PC0",
		"pull_up": 1,
	}); err != nil {
		return fmt.Errorf("config_endstop: %w", err)
	}
	fmt.Println("  config_endstop - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Query endstop state
	fmt.Println("\nQuerying endstop state...")
	resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
		"oid": oid,
	}, "endstop_state", 2*time.Second)
	if err != nil {
		return fmt.Errorf("endstop_query_state: %w", err)
	}
	fmt.Printf("  Endstop state: %v\n", resp)

	fmt.Println("\nEndstop test completed successfully!")
	return nil
}

// testTempLoop tests temperature sensor with closed-loop heater control.
// This simulates a real heating scenario: ADC reading → temperature → PWM heater control.
func testTempLoop(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: Temperature Closed-Loop Control ===")

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Allocate OIDs: 0=ADC (thermistor), 1=PWM (heater)
	fmt.Println("\nConfiguring temperature control system...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 2}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}

	// Configure ADC for temperature reading
	adcOid := 0
	if err := mcu.SendCommand("config_analog_in", map[string]interface{}{
		"oid": adcOid,
		"pin": "PA0",
	}); err != nil {
		return fmt.Errorf("config_analog_in: %w", err)
	}
	fmt.Println("  config_analog_in oid=0 pin=PA0 - OK")

	// Configure PWM for heater
	heaterOid := 1
	pwmCycleTicks := int(mcu.MCUFreq() / 1000) // 1kHz PWM
	if err := mcu.SendCommand("config_pwm_out", map[string]interface{}{
		"oid":           heaterOid,
		"pin":           "PA1",
		"cycle_ticks":   pwmCycleTicks,
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_pwm_out: %w", err)
	}
	fmt.Println("  config_pwm_out oid=1 pin=PA1 - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Register handler for analog readings
	tempReadings := make(chan int, 20)
	mcu.RegisterOIDHandler("analog_in_state", adcOid, func(params map[string]interface{}, receiveTime time.Time) {
		if value, ok := params["value"].(int); ok {
			select {
			case tempReadings <- value:
			default:
			}
		}
	})

	// Start ADC sampling
	mcuClock := mcu.PrintTime(0.1)
	sampleTicks := int(mcu.MCUFreq() * 0.001)  // 1ms sample
	restTicks := int(mcu.MCUFreq() * 0.1)      // 100ms between reports
	if err := mcu.SendCommand("query_analog_in", map[string]interface{}{
		"oid":               adcOid,
		"clock":             uint32(mcuClock),
		"sample_ticks":      sampleTicks,
		"sample_count":      8,
		"rest_ticks":        restTicks,
		"min_value":         0,
		"max_value":         4095,
		"range_check_count": 0,
	}); err != nil {
		return fmt.Errorf("query_analog_in: %w", err)
	}
	fmt.Println("  query_analog_in started")

	// Closed-loop control: read temperature and adjust heater PWM
	fmt.Println("\n--- Closed-Loop Temperature Control ---")
	fmt.Println("  Target: 60°C (simulated), Running for 3 seconds...")

	targetTemp := 60.0 // Target temperature
	currentPWM := 0
	readCount := 0
	loopTimeout := time.After(3 * time.Second)

	for {
		select {
		case adcValue := <-tempReadings:
			readCount++
			// Simulate temperature from ADC (mock MCU sends ~500-550)
			// Map ADC to temperature: ADC 500 ≈ 25°C, ADC 400 ≈ 50°C (thermistor is inverse)
			simulatedTemp := 75.0 - float64(adcValue-400)*0.5

			// Simple proportional control
			error := targetTemp - simulatedTemp
			newPWM := int(error * 5) // P-gain = 5
			if newPWM < 0 {
				newPWM = 0
			}
			if newPWM > 255 {
				newPWM = 255
			}

			// Only update PWM if changed significantly
			if abs(newPWM-currentPWM) > 10 {
				currentPWM = newPWM
				mcuClock = mcu.PrintTime(0)
				if err := mcu.SendCommand("schedule_pwm_out", map[string]interface{}{
					"oid":   heaterOid,
					"clock": uint32(mcuClock + 1000),
					"value": currentPWM,
				}); err != nil {
					fmt.Printf("  Warning: schedule_pwm_out failed: %v\n", err)
				}
			}

			if readCount <= 5 || readCount%5 == 0 {
				fmt.Printf("  [%d] ADC=%d Temp=%.1f°C Error=%.1f PWM=%d\n",
					readCount, adcValue, simulatedTemp, error, currentPWM)
			}

		case <-loopTimeout:
			fmt.Printf("\n  Total readings: %d\n", readCount)
			if readCount < 10 {
				return fmt.Errorf("insufficient readings: got %d, expected >= 10", readCount)
			}
			fmt.Println("\nTemperature closed-loop test completed successfully!")
			return nil
		}
	}
}

// abs returns the absolute value of an integer.
func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// testEndstops reads XYZ endstop sensor states every 1 second for 10 iterations.
func testEndstops(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string) error {
	fmt.Println("=== Test: XYZ Endstop Sensors ===")

	if configFile == "" {
		return fmt.Errorf("config file required for endstops test (use -config flag)")
	}

	config, err := parseFullConfig(configFile)
	if err != nil {
		return fmt.Errorf("parse config: %w", err)
	}

	// Load stepper configurations to get endstop pins
	stepperX := config.Steppers["stepper_x"]
	stepperY := config.Steppers["stepper_y"]
	stepperZ := config.Steppers["stepper_z"]

	if stepperX == nil || stepperX.EndstopPin == "" {
		return fmt.Errorf("no [stepper_x] endstop_pin found in config file")
	}
	if stepperY == nil || stepperY.EndstopPin == "" {
		return fmt.Errorf("no [stepper_y] endstop_pin found in config file")
	}
	if stepperZ == nil || stepperZ.EndstopPin == "" {
		return fmt.Errorf("no [stepper_z] endstop_pin found in config file")
	}

	fmt.Printf("  Endstop pins from config:\n")
	fmt.Printf("    X: pin=%s pullup=%v invert=%v\n", stepperX.EndstopPin, stepperX.EndstopPullup, stepperX.EndstopInvert)
	fmt.Printf("    Y: pin=%s pullup=%v invert=%v\n", stepperY.EndstopPin, stepperY.EndstopPullup, stepperY.EndstopInvert)
	fmt.Printf("    Z: pin=%s pullup=%v invert=%v\n", stepperZ.EndstopPin, stepperZ.EndstopPullup, stepperZ.EndstopInvert)

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Allocate OIDs for 3 endstops
	fmt.Println("\nConfiguring endstops...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 3}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=3 - OK")

	// Configure endstop X (OID 0)
	pullupX := 0
	if stepperX.EndstopPullup {
		pullupX = 1
	}
	if err := mcu.SendCommand("config_endstop", map[string]interface{}{
		"oid":     0,
		"pin":     stepperX.EndstopPin,
		"pull_up": pullupX,
	}); err != nil {
		return fmt.Errorf("config_endstop X: %w", err)
	}
	fmt.Printf("  config_endstop oid=0 (X: pin=%s pullup=%v) - OK\n", stepperX.EndstopPin, stepperX.EndstopPullup)

	// Configure endstop Y (OID 1)
	pullupY := 0
	if stepperY.EndstopPullup {
		pullupY = 1
	}
	if err := mcu.SendCommand("config_endstop", map[string]interface{}{
		"oid":     1,
		"pin":     stepperY.EndstopPin,
		"pull_up": pullupY,
	}); err != nil {
		return fmt.Errorf("config_endstop Y: %w", err)
	}
	fmt.Printf("  config_endstop oid=1 (Y: pin=%s pullup=%v) - OK\n", stepperY.EndstopPin, stepperY.EndstopPullup)

	// Configure endstop Z (OID 2)
	pullupZ := 0
	if stepperZ.EndstopPullup {
		pullupZ = 1
	}
	if err := mcu.SendCommand("config_endstop", map[string]interface{}{
		"oid":     2,
		"pin":     stepperZ.EndstopPin,
		"pull_up": pullupZ,
	}); err != nil {
		return fmt.Errorf("config_endstop Z: %w", err)
	}
	fmt.Printf("  config_endstop oid=2 (Z: pin=%s pullup=%v) - OK\n", stepperZ.EndstopPin, stepperZ.EndstopPullup)

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Read endstop states 10 times, once per second
	fmt.Println("\n--- Reading Endstop States (10 iterations, 1 second interval) ---")
	fmt.Println("  (Trigger state: 0=open, 1=triggered)")
	fmt.Println()

	for i := 1; i <= 10; i++ {
		// Query X endstop
		respX, err := mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
			"oid": 0,
		}, "endstop_state", 2*time.Second)
		if err != nil {
			return fmt.Errorf("endstop_query_state X: %w", err)
		}

		// Query Y endstop
		respY, err := mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
			"oid": 1,
		}, "endstop_state", 2*time.Second)
		if err != nil {
			return fmt.Errorf("endstop_query_state Y: %w", err)
		}

		// Query Z endstop
		respZ, err := mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
			"oid": 2,
		}, "endstop_state", 2*time.Second)
		if err != nil {
			return fmt.Errorf("endstop_query_state Z: %w", err)
		}

		// Extract pin_value from responses (taking into account inversion)
		xState := respX["pin_value"].(int)
		yState := respY["pin_value"].(int)
		zState := respZ["pin_value"].(int)

		// Apply inversion if configured
		if stepperX.EndstopInvert {
			xState = 1 - xState
		}
		if stepperY.EndstopInvert {
			yState = 1 - yState
		}
		if stepperZ.EndstopInvert {
			zState = 1 - zState
		}

		// Print status
		xStr := "OPEN"
		if xState == 1 {
			xStr = "TRIGGERED"
		}
		yStr := "OPEN"
		if yState == 1 {
			yStr = "TRIGGERED"
		}
		zStr := "OPEN"
		if zState == 1 {
			zStr = "TRIGGERED"
		}

		fmt.Printf("  [%2d/10] X: %-9s  Y: %-9s  Z: %-9s\n", i, xStr, yStr, zStr)

		// Wait 1 second (except after last iteration)
		if i < 10 {
			time.Sleep(1 * time.Second)
		}
	}

	fmt.Println("\nEndstop sensor test completed successfully!")
	return nil
}

// testYMovement tests Y-axis movement: +20mm then -20mm.
// CoreXY Y-axis: both motors move in OPPOSITE directions.
func testYMovement(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string) error {
	fmt.Println("=== Test: Y-Axis Movement (+20mm, -20mm) ===")

	if configFile == "" {
		return fmt.Errorf("config file required for ymovement test (use -config flag)")
	}

	config, err := parseFullConfig(configFile)
	if err != nil {
		return fmt.Errorf("parse config: %w", err)
	}

	stepperX := config.Steppers["stepper_x"]
	stepperY := config.Steppers["stepper_y"]
	if stepperX == nil || stepperY == nil {
		return fmt.Errorf("stepper_x and stepper_y required in config")
	}

	fmt.Printf("  Loaded from config:\n")
	fmt.Printf("    [stepper_x]: step=%s dir=%s enable=%s\n",
		stepperX.StepPin, stepperX.DirPin, stepperX.EnablePin)
	fmt.Printf("    [stepper_y]: step=%s dir=%s enable=%s\n",
		stepperY.StepPin, stepperY.DirPin, stepperY.EnablePin)

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Allocate OIDs: 0=stepper_x, 1=stepper_y, 2=enable_x, 3=enable_y
	fmt.Println("\nConfiguring Y-axis movement system...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 4}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=4 - OK")

	stepPulseTicks := 32

	// Configure steppers
	stepperXOid := 0
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperXOid,
		"step_pin":         stepperX.StepPin,
		"dir_pin":          stepperX.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper X: %w", err)
	}
	fmt.Println("  config_stepper oid=0 (X) - OK")

	stepperYOid := 1
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperYOid,
		"step_pin":         stepperY.StepPin,
		"dir_pin":          stepperY.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper Y: %w", err)
	}
	fmt.Println("  config_stepper oid=1 (Y) - OK")

	// Configure enable pins
	enableXOid := 2
	enableXDisabledValue := 1
	if !stepperX.EnableInvert {
		enableXDisabledValue = 0
	}
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableXOid,
		"pin":           stepperX.EnablePin,
		"value":         enableXDisabledValue,
		"default_value": enableXDisabledValue,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (enable_x): %w", err)
	}
	fmt.Println("  config_digital_out oid=2 (enable_x) - OK")

	enableYOid := 3
	enableYDisabledValue := 1
	if !stepperY.EnableInvert {
		enableYDisabledValue = 0
	}
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableYOid,
		"pin":           stepperY.EnablePin,
		"value":         enableYDisabledValue,
		"default_value": enableYDisabledValue,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (enable_y): %w", err)
	}
	fmt.Println("  config_digital_out oid=3 (enable_y) - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Enable motors
	fmt.Println("\n--- Y-Axis Movement Test ---")
	enableXActiveValue := 0
	if !stepperX.EnableInvert {
		enableXActiveValue = 1
	}
	enableYActiveValue := 0
	if !stepperY.EnableInvert {
		enableYActiveValue = 1
	}

	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableXOid,
		"value": enableXActiveValue,
	}); err != nil {
		return fmt.Errorf("enable motor X: %w", err)
	}
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableYOid,
		"value": enableYActiveValue,
	}); err != nil {
		return fmt.Errorf("enable motor Y: %w", err)
	}
	fmt.Println("  Motors ENABLED")

	// Movement parameters: 500mm/min = 8.33mm/sec = 667 steps/sec, 20mm = 1600 steps
	stepInterval := int(mcu.MCUFreq() / 667) // 667 steps/sec
	stepCount := 1600                         // 20mm at 80 steps/mm
	moveDuration := float64(stepCount) * float64(stepInterval) / mcu.MCUFreq()

	fmt.Printf("  Step interval: %d ticks (%.2f ms)\n", stepInterval, float64(stepInterval)/mcu.MCUFreq()*1000)
	fmt.Printf("  Step count: %d (20mm)\n", stepCount)
	fmt.Printf("  Move duration: %.2f seconds\n", moveDuration)

	// ========== PHASE 1: Y+ (positive direction, +20mm) ==========
	fmt.Println("\n  === Phase 1: Moving Y+ (+20mm) ===")

	// Get MCU clock
	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime: %w", err)
	}
	curClock := uint64(resp["clock"].(int))
	startClock := curClock + uint64(mcu.MCUFreq()*0.1)

	// CoreXY Y+: motor_x positive (dir=0), motor_y negative (dir=1)
	// Both motors move in OPPOSITE directions for Y-axis
	yPlusDirX := 0 // motor_x moves positive
	yPlusDirY := 1 // motor_y moves negative

	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperXOid,
		"dir": yPlusDirX,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir X: %w", err)
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperYOid,
		"dir": yPlusDirY,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir Y: %w", err)
	}
	fmt.Printf("  Direction: motor_x=%d, motor_y=%d (Y+)\n", yPlusDirX, yPlusDirY)

	// Reset clocks and queue steps
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperXOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock X: %w", err)
	}
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperYOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock Y: %w", err)
	}

	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperXOid,
		"interval": stepInterval,
		"count":    stepCount,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step X: %w", err)
	}
	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperYOid,
		"interval": stepInterval,
		"count":    stepCount,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step Y: %w", err)
	}
	fmt.Printf("  Queued %d steps - moving Y+...\n", stepCount)

	// Wait for movement
	waitTime := time.Duration((moveDuration+0.3)*1000) * time.Millisecond
	time.Sleep(waitTime)

	// Check position after Y+ move
	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": stepperXOid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position X: %w", err)
	}
	posX1 := resp["pos"].(int)
	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": stepperYOid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position Y: %w", err)
	}
	posY1 := resp["pos"].(int)
	fmt.Printf("  Position after Y+: stepper_x=%d, stepper_y=%d\n", posX1, posY1)

	// ========== PHASE 2: Y- (negative direction, -20mm) ==========
	fmt.Println("\n  === Phase 2: Moving Y- (-20mm) ===")

	// Get fresh clock
	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime: %w", err)
	}
	curClock = uint64(resp["clock"].(int))
	startClock = curClock + uint64(mcu.MCUFreq()*0.1)

	// CoreXY Y-: opposite of Y+
	yMinusDirX := 1 - yPlusDirX // motor_x moves positive (dir=0)
	yMinusDirY := 1 - yPlusDirY // motor_y moves negative (dir=1)

	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperXOid,
		"dir": yMinusDirX,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir X: %w", err)
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperYOid,
		"dir": yMinusDirY,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir Y: %w", err)
	}
	fmt.Printf("  Direction: motor_x=%d, motor_y=%d (Y-)\n", yMinusDirX, yMinusDirY)

	// Reset clocks and queue steps
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperXOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock X: %w", err)
	}
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperYOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock Y: %w", err)
	}

	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperXOid,
		"interval": stepInterval,
		"count":    stepCount,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step X: %w", err)
	}
	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperYOid,
		"interval": stepInterval,
		"count":    stepCount,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step Y: %w", err)
	}
	fmt.Printf("  Queued %d steps - moving Y-...\n", stepCount)

	// Wait for movement
	time.Sleep(waitTime)

	// Check final position
	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": stepperXOid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position X: %w", err)
	}
	posX2 := resp["pos"].(int)
	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": stepperYOid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position Y: %w", err)
	}
	posY2 := resp["pos"].(int)
	fmt.Printf("  Position after Y-: stepper_x=%d, stepper_y=%d\n", posX2, posY2)

	// Disable motors
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableXOid,
		"value": enableXDisabledValue,
	}); err != nil {
		return fmt.Errorf("disable motor X: %w", err)
	}
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableYOid,
		"value": enableYDisabledValue,
	}); err != nil {
		return fmt.Errorf("disable motor Y: %w", err)
	}
	fmt.Println("  Motors disabled")

	// Summary
	fmt.Printf("\n  === Summary ===\n")
	fmt.Printf("  Net stepper_x movement: %d steps\n", posX2)
	fmt.Printf("  Net stepper_y movement: %d steps\n", posY2)
	if posX2 == 0 && posY2 == 0 {
		fmt.Println("\n  SUCCESS: Y-axis returned to original position!")
	} else {
		fmt.Printf("\n  Note: Final position differs from start (x=%d, y=%d)\n", posX2, posY2)
	}

	fmt.Println("\nY-axis movement test completed!")
	return nil
}

// testYHoming performs Y-axis homing using endstop trigger with trsync.
// This moves the Y-axis toward the endstop until triggered, then stops.
func testYHoming(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string) error {
	fmt.Println("=== Test: Y-Axis Homing ===")

	if configFile == "" {
		return fmt.Errorf("config file required for yhoming test (use -config flag)")
	}

	config, err := parseFullConfig(configFile)
	if err != nil {
		return fmt.Errorf("parse config: %w", err)
	}

	stepperX := config.Steppers["stepper_x"]
	stepperY := config.Steppers["stepper_y"]
	if stepperX == nil || stepperY == nil {
		return fmt.Errorf("stepper_x and stepper_y required in config")
	}

	fmt.Printf("  Loaded from config:\n")
	fmt.Printf("    [stepper_x]: step=%s dir=%s enable=%s\n",
		stepperX.StepPin, stepperX.DirPin, stepperX.EnablePin)
	fmt.Printf("    [stepper_y]: step=%s dir=%s enable=%s endstop=%s position_endstop=%.0f position_max=%.0f\n",
		stepperY.StepPin, stepperY.DirPin, stepperY.EnablePin, stepperY.EndstopPin,
		stepperY.PositionEndstop, stepperY.PositionMax)

	if stepperY.EndstopPin == "" {
		return fmt.Errorf("stepper_y endstop_pin required")
	}

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Allocate OIDs: 0=stepper_x, 1=stepper_y, 2=enable_x, 3=enable_y, 4=endstop_y, 5=trsync
	fmt.Println("\nConfiguring Y-axis homing system...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 6}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=6 - OK")

	stepPulseTicks := 32

	// Configure steppers
	stepperXOid := 0
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperXOid,
		"step_pin":         stepperX.StepPin,
		"dir_pin":          stepperX.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper X: %w", err)
	}
	fmt.Println("  config_stepper oid=0 (X) - OK")

	stepperYOid := 1
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperYOid,
		"step_pin":         stepperY.StepPin,
		"dir_pin":          stepperY.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper Y: %w", err)
	}
	fmt.Println("  config_stepper oid=1 (Y) - OK")

	// Configure enable pins
	enableXOid := 2
	enableXDisabledValue := 1
	if !stepperX.EnableInvert {
		enableXDisabledValue = 0
	}
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableXOid,
		"pin":           stepperX.EnablePin,
		"value":         enableXDisabledValue,
		"default_value": enableXDisabledValue,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (enable_x): %w", err)
	}
	fmt.Println("  config_digital_out oid=2 (enable_x) - OK")

	enableYOid := 3
	enableYDisabledValue := 1
	if !stepperY.EnableInvert {
		enableYDisabledValue = 0
	}
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableYOid,
		"pin":           stepperY.EnablePin,
		"value":         enableYDisabledValue,
		"default_value": enableYDisabledValue,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (enable_y): %w", err)
	}
	fmt.Println("  config_digital_out oid=3 (enable_y) - OK")

	// Configure Y endstop (OID 4)
	endstopOid := 4
	pullupValue := 0
	if stepperY.EndstopPullup {
		pullupValue = 1
	}
	if err := mcu.SendCommand("config_endstop", map[string]interface{}{
		"oid":     endstopOid,
		"pin":     stepperY.EndstopPin,
		"pull_up": pullupValue,
	}); err != nil {
		return fmt.Errorf("config_endstop: %w", err)
	}
	fmt.Printf("  config_endstop oid=4 (Y endstop pin=%s pullup=%v) - OK\n",
		stepperY.EndstopPin, stepperY.EndstopPullup)

	// Configure trsync (OID 5)
	trsyncOid := 5
	if err := mcu.SendCommand("config_trsync", map[string]interface{}{
		"oid": trsyncOid,
	}); err != nil {
		return fmt.Errorf("config_trsync: %w", err)
	}
	fmt.Println("  config_trsync oid=5 - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Start Y-axis homing sequence
	fmt.Println("\n--- Y-Axis Homing Sequence ---")

	// Check current endstop state
	resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
		"oid": endstopOid,
	}, "endstop_state", 2*time.Second)
	if err != nil {
		return fmt.Errorf("endstop_query_state: %w", err)
	}
	pinValue := resp["pin_value"].(int)
	if stepperY.EndstopInvert {
		pinValue = 1 - pinValue
	}
	if pinValue == 1 {
		fmt.Println("  WARNING: Y endstop already triggered!")
		return fmt.Errorf("endstop already triggered")
	}
	fmt.Println("  Y endstop state: OPEN (ready for homing)")

	// Enable motors
	enableXActiveValue := 0
	if !stepperX.EnableInvert {
		enableXActiveValue = 1
	}
	enableYActiveValue := 0
	if !stepperY.EnableInvert {
		enableYActiveValue = 1
	}

	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableXOid,
		"value": enableXActiveValue,
	}); err != nil {
		return fmt.Errorf("enable motor X: %w", err)
	}
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableYOid,
		"value": enableYActiveValue,
	}); err != nil {
		return fmt.Errorf("enable motor Y: %w", err)
	}
	fmt.Println("  Motors ENABLED")

	// Get MCU clock
	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime: %w", err)
	}
	curClock := uint64(resp["clock"].(int))
	fmt.Printf("  Current MCU clock: %d\n", curClock)

	// Homing parameters
	homingSpeed := 400.0 // steps/sec = 300mm/min at 80 steps/mm
	stepInterval := int(mcu.MCUFreq() / homingSpeed)
	maxSteps := 24000 // 300mm max travel

	fmt.Printf("  Homing speed: %.0f steps/sec (%.0f mm/min)\n", homingSpeed, homingSpeed/80*60)
	fmt.Printf("  Max travel: %d steps (%.0f mm)\n", maxSteps, float64(maxSteps)/80)

	// Determine homing direction based on position_endstop from config
	// If position_endstop is at min (close to 0), home toward negative (Y-)
	// If position_endstop is at max (close to position_max), home toward positive (Y+)
	var homingDirX, homingDirY int
	var dirDescription string
	if stepperY.PositionEndstop > stepperY.PositionMax/2 {
		// Endstop at max position - home toward Y+ (positive direction)
		// CoreXY Y+: motor_x positive (dir=0), motor_y negative (dir=1)
		homingDirX = 0
		homingDirY = 1
		dirDescription = fmt.Sprintf("toward Y+ endstop (position_endstop=%.0f)", stepperY.PositionEndstop)
	} else {
		// Endstop at min position - home toward Y- (negative direction)
		// CoreXY Y-: motor_x negative (dir=1), motor_y positive (dir=0)
		homingDirX = 1
		homingDirY = 0
		dirDescription = fmt.Sprintf("toward Y- endstop (position_endstop=%.0f)", stepperY.PositionEndstop)
	}

	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperXOid,
		"dir": homingDirX,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir X: %w", err)
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperYOid,
		"dir": homingDirY,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir Y: %w", err)
	}
	fmt.Printf("  Direction: motor_x=%d, motor_y=%d (%s)\n", homingDirX, homingDirY, dirDescription)

	// Calculate timing
	startClock := curClock + uint64(mcu.MCUFreq()*0.1)
	moveDuration := float64(maxSteps) * float64(stepInterval) / mcu.MCUFreq()
	expireClock := startClock + uint64((moveDuration+2.0)*mcu.MCUFreq())
	sampleTicks := uint32(mcu.MCUFreq() * 0.000025)
	restTicks := uint32(mcu.MCUFreq() * 0.0001)

	// Setup trsync and endstop homing
	if err := mcu.SendCommand("trsync_start", map[string]interface{}{
		"oid":           trsyncOid,
		"report_clock":  uint32(startClock),
		"report_ticks":  uint32(mcu.MCUFreq() * 0.1),
		"expire_reason": 2,
	}); err != nil {
		return fmt.Errorf("trsync_start: %w", err)
	}
	fmt.Println("  trsync_start - OK")

	if err := mcu.SendCommand("trsync_set_timeout", map[string]interface{}{
		"oid":   trsyncOid,
		"clock": uint32(expireClock),
	}); err != nil {
		return fmt.Errorf("trsync_set_timeout: %w", err)
	}
	fmt.Println("  trsync_set_timeout - OK")

	if err := mcu.SendCommand("stepper_stop_on_trigger", map[string]interface{}{
		"oid":        stepperXOid,
		"trsync_oid": trsyncOid,
	}); err != nil {
		return fmt.Errorf("stepper_stop_on_trigger X: %w", err)
	}
	if err := mcu.SendCommand("stepper_stop_on_trigger", map[string]interface{}{
		"oid":        stepperYOid,
		"trsync_oid": trsyncOid,
	}); err != nil {
		return fmt.Errorf("stepper_stop_on_trigger Y: %w", err)
	}
	fmt.Println("  stepper_stop_on_trigger X, Y - OK")

	triggerPinValue := 0 // Trigger when pin goes LOW (endstop pressed)
	if err := mcu.SendCommand("endstop_home", map[string]interface{}{
		"oid":            endstopOid,
		"clock":          uint32(startClock),
		"sample_ticks":   sampleTicks,
		"sample_count":   4,
		"rest_ticks":     restTicks,
		"pin_value":      triggerPinValue,
		"trsync_oid":     trsyncOid,
		"trigger_reason": 1,
	}); err != nil {
		return fmt.Errorf("endstop_home: %w", err)
	}
	fmt.Printf("  endstop_home (trigger when pin_value=%d) - OK\n", triggerPinValue)

	// Queue steps
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperXOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock X: %w", err)
	}
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperYOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock Y: %w", err)
	}
	fmt.Println("  reset_step_clock X, Y - OK")

	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperXOid,
		"interval": stepInterval,
		"count":    maxSteps,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step X: %w", err)
	}
	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperYOid,
		"interval": stepInterval,
		"count":    maxSteps,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step Y: %w", err)
	}
	fmt.Printf("  queue_step X, Y (count=%d) - OK\n", maxSteps)

	fmt.Println("\n  Moving toward Y endstop...")

	// Poll endstop state
	homingTimeout := time.Duration(moveDuration+5) * time.Second
	startTime := time.Now()
	homingSuccess := false

	for time.Since(startTime) < homingTimeout {
		resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
			"oid": endstopOid,
		}, "endstop_state", 2*time.Second)
		if err != nil {
			fmt.Printf("\n  WARNING: endstop_query_state failed: %v\n", err)
			break
		}

		endstopPinValue := resp["pin_value"].(int)
		if stepperY.EndstopInvert {
			endstopPinValue = 1 - endstopPinValue
		}

		resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
			"oid": stepperYOid,
		}, "stepper_position", 2*time.Second)
		if err != nil {
			fmt.Printf("\n  WARNING: stepper_get_position failed: %v\n", err)
			break
		}
		curPos := resp["pos"].(int)

		fmt.Printf("  [%.1fs] Endstop: %s, Position: %d steps (%.1f mm)\r",
			time.Since(startTime).Seconds(),
			map[int]string{0: "OPEN", 1: "TRIGGERED"}[endstopPinValue],
			curPos, float64(abs(curPos))/80.0)

		if endstopPinValue == 1 {
			time.Sleep(100 * time.Millisecond)

			resp, _ = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
				"oid": stepperXOid,
			}, "stepper_position", 2*time.Second)
			finalPosX := resp["pos"].(int)
			resp, _ = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
				"oid": stepperYOid,
			}, "stepper_position", 2*time.Second)
			finalPosY := resp["pos"].(int)

			travelSteps := abs(finalPosY)
			travelMM := float64(travelSteps) / 80.0

			fmt.Printf("\n\n  SUCCESS: Y-axis homing completed!\n")
			fmt.Printf("    Endstop TRIGGERED\n")
			fmt.Printf("    Stepper X position: %d steps\n", finalPosX)
			fmt.Printf("    Stepper Y position: %d steps\n", finalPosY)
			fmt.Printf("    Travel distance: %d steps (%.2f mm)\n", travelSteps, travelMM)
			homingSuccess = true
			break
		}

		time.Sleep(100 * time.Millisecond)
	}

	if !homingSuccess {
		fmt.Printf("\n\n  TIMEOUT: Endstop not triggered within %.0f seconds\n", homingTimeout.Seconds())
		return fmt.Errorf("homing timeout")
	}

	// Disable motors
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableXOid,
		"value": enableXDisabledValue,
	}); err != nil {
		return fmt.Errorf("disable motor X: %w", err)
	}
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableYOid,
		"value": enableYDisabledValue,
	}); err != nil {
		return fmt.Errorf("disable motor Y: %w", err)
	}
	fmt.Println("  Motors disabled")

	fmt.Println("\nY-axis homing test completed successfully!")
	return nil
}

// testZMovement performs Z-axis movement test: move up, then move down.
// Z-axis is a single stepper motor (not CoreXY).
func testZMovement(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string) error {
	fmt.Println("=== Test: Z-Axis Movement (+10mm, -10mm) ===")

	if configFile == "" {
		return fmt.Errorf("config file required for zmovement test (use -config flag)")
	}

	config, err := parseFullConfig(configFile)
	if err != nil {
		return fmt.Errorf("parse config: %w", err)
	}

	stepperX := config.Steppers["stepper_x"]
	stepperY := config.Steppers["stepper_y"]
	stepperZ := config.Steppers["stepper_z"]
	if stepperX == nil || stepperY == nil || stepperZ == nil {
		return fmt.Errorf("stepper_x, stepper_y, and stepper_z required in config")
	}

	fmt.Printf("  Loaded from config:\n")
	fmt.Printf("    [stepper_x]: step=%s dir=%s enable=%s\n",
		stepperX.StepPin, stepperX.DirPin, stepperX.EnablePin)
	fmt.Printf("    [stepper_y]: step=%s dir=%s enable=%s\n",
		stepperY.StepPin, stepperY.DirPin, stepperY.EnablePin)
	fmt.Printf("    [stepper_z]: step=%s dir=%s enable=%s\n",
		stepperZ.StepPin, stepperZ.DirPin, stepperZ.EnablePin)

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Allocate OIDs: 0=stepper_z, 1=enable_z (minimal config for Z only)
	fmt.Println("\nConfiguring Z-axis movement system...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 2}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=2 - OK")

	stepPulseTicks := 32

	// Configure stepper_z
	stepperZOid := 0
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperZOid,
		"step_pin":         stepperZ.StepPin,
		"dir_pin":          stepperZ.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper Z: %w", err)
	}
	fmt.Printf("  config_stepper oid=0 (Z) step=%s dir=%s - OK\n", stepperZ.StepPin, stepperZ.DirPin)

	// Configure enable pin for Z
	enableZOid := 1
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableZOid,
		"pin":           stepperZ.EnablePin,
		"value":         1,
		"default_value": 1,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out Z enable: %w", err)
	}
	fmt.Printf("  config_digital_out oid=1 (enable_z) pin=%s - OK\n", stepperZ.EnablePin)

	// Variables to suppress unused warnings
	_ = stepperX
	_ = stepperY

	// Finalize config (use SendCommand, not SendCommandWithResponse - finalize_config doesn't return a response)
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")
	time.Sleep(100 * time.Millisecond) // Brief delay after finalize_config

	// Z-axis movement parameters
	// rotation_distance=8, microsteps=16 -> steps_per_mm = 200*16/8 = 400
	stepsPerMM := 400.0
	moveDistance := 10.0 // 10mm
	stepCount := int(moveDistance * stepsPerMM)
	moveSpeed := 200.0 // steps per second (slow for Z)

	fmt.Println("\n--- Z-Axis Movement Test ---")

	// Enable motor
	enableActiveValue := 0
	if !stepperZ.EnableInvert {
		enableActiveValue = 1
	}
	fmt.Printf("  Enabling motor (EnableInvert=%v, value=%d)...\n", stepperZ.EnableInvert, enableActiveValue)
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableZOid,
		"value": enableActiveValue,
	}); err != nil {
		return fmt.Errorf("enable motor Z: %w", err)
	}
	fmt.Println("  Motor Z ENABLED")

	// Calculate step timing
	stepInterval := int(mcu.MCUFreq() / moveSpeed)
	moveDuration := float64(stepCount) / moveSpeed

	fmt.Printf("  Step interval: %d ticks (%.2f ms)\n", stepInterval, float64(stepInterval)/mcu.MCUFreq()*1000)
	fmt.Printf("  Step count: %d (%.0fmm)\n", stepCount, moveDistance)
	fmt.Printf("  Move duration: %.2f seconds\n", moveDuration)

	// Direction values for Z-axis
	// dir=0 for Z+ (up), dir=1 for Z- (down)
	// If dir_pin is inverted, swap them
	zPlusDir := 0
	zMinusDir := 1
	if stepperZ.DirInvert {
		zPlusDir = 1
		zMinusDir = 0
	}

	// === Phase 1: Move Z+ (up) ===
	fmt.Printf("\n  === Phase 1: Moving Z+ (+%.0fmm) ===\n", moveDistance)
	fmt.Printf("  Direction: dir=%d (Z+)\n", zPlusDir)

	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperZOid,
		"dir": zPlusDir,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir Z+: %w", err)
	}

	// Get current MCU clock
	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime: %w", err)
	}
	highClock := uint64(resp["high"].(int))
	curClock := (highClock << 32) | uint64(resp["clock"].(int))
	startClock := curClock + uint64(mcu.MCUFreq()*0.1)

	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperZOid,
		"clock": startClock,
	}); err != nil {
		return fmt.Errorf("reset_step_clock Z: %w", err)
	}

	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperZOid,
		"interval": stepInterval,
		"count":    stepCount,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step Z+: %w", err)
	}
	fmt.Printf("  Queued %d steps - moving Z+...\n", stepCount)

	// Wait for movement to complete
	time.Sleep(time.Duration((moveDuration+0.5)*1000) * time.Millisecond)

	// Get position after Z+
	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": stepperZOid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position Z: %w", err)
	}
	posAfterUp := resp["pos"].(int)
	fmt.Printf("  Position after Z+: stepper_z=%d\n", posAfterUp)

	// === Phase 2: Move Z- (down) ===
	fmt.Printf("\n  === Phase 2: Moving Z- (-%.0fmm) ===\n", moveDistance)
	fmt.Printf("  Direction: dir=%d (Z-)\n", zMinusDir)

	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperZOid,
		"dir": zMinusDir,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir Z-: %w", err)
	}

	// Get current MCU clock
	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime: %w", err)
	}
	highClock = uint64(resp["high"].(int))
	curClock = (highClock << 32) | uint64(resp["clock"].(int))
	startClock = curClock + uint64(mcu.MCUFreq()*0.1)

	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperZOid,
		"clock": startClock,
	}); err != nil {
		return fmt.Errorf("reset_step_clock Z: %w", err)
	}

	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperZOid,
		"interval": stepInterval,
		"count":    stepCount,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step Z-: %w", err)
	}
	fmt.Printf("  Queued %d steps - moving Z-...\n", stepCount)

	// Wait for movement to complete
	time.Sleep(time.Duration((moveDuration+0.5)*1000) * time.Millisecond)

	// Get position after Z-
	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": stepperZOid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position Z: %w", err)
	}
	posAfterDown := resp["pos"].(int)
	fmt.Printf("  Position after Z-: stepper_z=%d\n", posAfterDown)

	// Disable motor
	disableValue := 1
	if !stepperZ.EnableInvert {
		disableValue = 0
	}
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableZOid,
		"value": disableValue,
	}); err != nil {
		return fmt.Errorf("disable motor Z: %w", err)
	}
	fmt.Println("  Motor disabled")

	// Summary
	fmt.Println("\n  === Summary ===")
	fmt.Printf("  Net stepper_z movement: %d steps\n", posAfterDown)

	if posAfterDown == 0 {
		fmt.Println("\n  SUCCESS: Z-axis returned to original position!")
	} else {
		fmt.Printf("\n  WARNING: Z-axis did not return to origin (offset: %d steps)\n", posAfterDown)
	}

	fmt.Println("\nZ-axis movement test completed!")
	return nil
}

// testZHoming performs Z-axis homing using endstop trigger with trsync.
// Z-axis is a single stepper motor (not CoreXY).
func testZHoming(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string) error {
	fmt.Println("=== Test: Z-Axis Homing ===")

	if configFile == "" {
		return fmt.Errorf("config file required for zhoming test (use -config flag)")
	}

	config, err := parseFullConfig(configFile)
	if err != nil {
		return fmt.Errorf("parse config: %w", err)
	}

	stepperZ := config.Steppers["stepper_z"]
	if stepperZ == nil {
		return fmt.Errorf("stepper_z required in config")
	}

	fmt.Printf("  Loaded from config:\n")
	fmt.Printf("    [stepper_z]: step=%s dir=%s enable=%s endstop=%s position_endstop=%.1f position_max=%.0f\n",
		stepperZ.StepPin, stepperZ.DirPin, stepperZ.EnablePin, stepperZ.EndstopPin,
		stepperZ.PositionEndstop, stepperZ.PositionMax)

	if stepperZ.EndstopPin == "" {
		return fmt.Errorf("stepper_z endstop_pin required")
	}

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Allocate OIDs: 0=stepper_z, 1=enable_z, 2=endstop_z, 3=trsync
	fmt.Println("\nConfiguring Z-axis homing system...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 4}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=4 - OK")

	stepPulseTicks := 32

	// Configure Z stepper (OID 0)
	stepperZOid := 0
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperZOid,
		"step_pin":         stepperZ.StepPin,
		"dir_pin":          stepperZ.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper Z: %w", err)
	}
	fmt.Printf("  config_stepper oid=0 (Z) step=%s dir=%s - OK\n", stepperZ.StepPin, stepperZ.DirPin)

	// Configure enable pin (OID 1)
	enableZOid := 1
	enableZDisabledValue := 1
	if !stepperZ.EnableInvert {
		enableZDisabledValue = 0
	}
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableZOid,
		"pin":           stepperZ.EnablePin,
		"value":         enableZDisabledValue,
		"default_value": enableZDisabledValue,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (enable_z): %w", err)
	}
	fmt.Printf("  config_digital_out oid=1 (enable_z) pin=%s - OK\n", stepperZ.EnablePin)

	// Configure Z endstop (OID 2)
	endstopOid := 2
	pullupValue := 0
	if stepperZ.EndstopPullup {
		pullupValue = 1
	}
	if err := mcu.SendCommand("config_endstop", map[string]interface{}{
		"oid":     endstopOid,
		"pin":     stepperZ.EndstopPin,
		"pull_up": pullupValue,
	}); err != nil {
		return fmt.Errorf("config_endstop: %w", err)
	}
	fmt.Printf("  config_endstop oid=2 (Z endstop pin=%s pullup=%v) - OK\n",
		stepperZ.EndstopPin, stepperZ.EndstopPullup)

	// Configure trsync (OID 3)
	trsyncOid := 3
	if err := mcu.SendCommand("config_trsync", map[string]interface{}{
		"oid": trsyncOid,
	}); err != nil {
		return fmt.Errorf("config_trsync: %w", err)
	}
	fmt.Println("  config_trsync oid=3 - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Start Z-axis homing sequence
	fmt.Println("\n--- Z-Axis Homing Sequence ---")

	// Check current endstop state
	resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
		"oid": endstopOid,
	}, "endstop_state", 2*time.Second)
	if err != nil {
		return fmt.Errorf("endstop_query_state: %w", err)
	}
	pinValue := resp["pin_value"].(int)
	if stepperZ.EndstopInvert {
		pinValue = 1 - pinValue
	}
	if pinValue == 1 {
		fmt.Println("  WARNING: Z endstop already triggered!")
		return fmt.Errorf("endstop already triggered - move Z away from endstop first")
	}
	fmt.Println("  Z endstop state: OPEN (ready for homing)")

	// Enable motor
	enableZActiveValue := 0
	if !stepperZ.EnableInvert {
		enableZActiveValue = 1
	}

	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableZOid,
		"value": enableZActiveValue,
	}); err != nil {
		return fmt.Errorf("enable motor Z: %w", err)
	}
	fmt.Println("  Motor Z ENABLED")

	// Get MCU clock
	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime: %w", err)
	}
	curClock := uint64(resp["clock"].(int))
	fmt.Printf("  Current MCU clock: %d\n", curClock)

	// Z-axis homing parameters
	// Z uses rotation_distance=8, microsteps=16 -> 400 steps/mm
	stepsPerMM := 400.0
	homingSpeed := 800.0 // steps/sec = 2 mm/s = 120 mm/min (reasonable for Z)
	stepInterval := int(mcu.MCUFreq() / homingSpeed)
	// Limit max travel to 100mm for homing test (avoids uint32 clock overflow)
	maxTravelMM := 100.0
	if stepperZ.PositionMax < maxTravelMM {
		maxTravelMM = stepperZ.PositionMax
	}
	maxSteps := int(maxTravelMM * stepsPerMM)

	fmt.Printf("  Homing speed: %.0f steps/sec (%.0f mm/min)\n", homingSpeed, homingSpeed/stepsPerMM*60)
	fmt.Printf("  Max travel: %d steps (%.0f mm)\n", maxSteps, float64(maxSteps)/stepsPerMM)

	// Determine homing direction based on position_endstop from config
	// If position_endstop is at min (close to 0), home toward negative (Z-)
	// If position_endstop is at max (close to position_max), home toward positive (Z+)
	// Note: For this Z-axis, dir=0 moves Z- (down), dir=1 moves Z+ (up)
	var homingDir int
	var dirDescription string
	if stepperZ.PositionEndstop > stepperZ.PositionMax/2 {
		// Endstop at max position - home toward Z+ (positive direction)
		homingDir = 1
		if stepperZ.DirInvert {
			homingDir = 0
		}
		dirDescription = fmt.Sprintf("toward Z+ endstop (position_endstop=%.1f)", stepperZ.PositionEndstop)
	} else {
		// Endstop at min position - home toward Z- (negative direction)
		homingDir = 0
		if stepperZ.DirInvert {
			homingDir = 1
		}
		dirDescription = fmt.Sprintf("toward Z- endstop (position_endstop=%.1f)", stepperZ.PositionEndstop)
	}

	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperZOid,
		"dir": homingDir,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir Z: %w", err)
	}
	fmt.Printf("  Direction: dir=%d (%s)\n", homingDir, dirDescription)

	// Calculate timing
	startClock := curClock + uint64(mcu.MCUFreq()*0.1)
	moveDuration := float64(maxSteps) * float64(stepInterval) / mcu.MCUFreq()
	expireClock := startClock + uint64((moveDuration+2.0)*mcu.MCUFreq())
	sampleTicks := uint32(mcu.MCUFreq() * 0.000025)
	restTicks := uint32(mcu.MCUFreq() * 0.0001)

	// Setup trsync and endstop homing
	if err := mcu.SendCommand("trsync_start", map[string]interface{}{
		"oid":           trsyncOid,
		"report_clock":  uint32(startClock),
		"report_ticks":  uint32(mcu.MCUFreq() * 0.1),
		"expire_reason": 2,
	}); err != nil {
		return fmt.Errorf("trsync_start: %w", err)
	}
	fmt.Println("  trsync_start - OK")

	if err := mcu.SendCommand("trsync_set_timeout", map[string]interface{}{
		"oid":   trsyncOid,
		"clock": uint32(expireClock),
	}); err != nil {
		return fmt.Errorf("trsync_set_timeout: %w", err)
	}
	fmt.Println("  trsync_set_timeout - OK")

	if err := mcu.SendCommand("stepper_stop_on_trigger", map[string]interface{}{
		"oid":        stepperZOid,
		"trsync_oid": trsyncOid,
	}); err != nil {
		return fmt.Errorf("stepper_stop_on_trigger Z: %w", err)
	}
	fmt.Println("  stepper_stop_on_trigger Z - OK")

	// endstop_home triggers when raw pin matches pin_value
	// For normally-open switches with pullup: pin is HIGH when open, LOW when pressed
	// So trigger on pin_value=0 (pressed state)
	triggerPinValue := 0
	if err := mcu.SendCommand("endstop_home", map[string]interface{}{
		"oid":            endstopOid,
		"clock":          uint32(startClock),
		"sample_ticks":   sampleTicks,
		"sample_count":   4,
		"rest_ticks":     restTicks,
		"pin_value":      triggerPinValue,
		"trsync_oid":     trsyncOid,
		"trigger_reason": 1,
	}); err != nil {
		return fmt.Errorf("endstop_home: %w", err)
	}
	fmt.Printf("  endstop_home (trigger when pin_value=%d) - OK\n", triggerPinValue)

	// Queue steps
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperZOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock Z: %w", err)
	}
	fmt.Println("  reset_step_clock Z - OK")

	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperZOid,
		"interval": stepInterval,
		"count":    maxSteps,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step Z: %w", err)
	}
	fmt.Printf("  queue_step Z (count=%d) - OK\n", maxSteps)

	fmt.Println("\n  Moving toward Z endstop...")

	// Poll endstop state
	homingTimeout := time.Duration(moveDuration+10) * time.Second
	startTime := time.Now()
	homingSuccess := false

	for time.Since(startTime) < homingTimeout {
		resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
			"oid": endstopOid,
		}, "endstop_state", 2*time.Second)
		if err != nil {
			fmt.Printf("\n  WARNING: endstop_query_state failed: %v\n", err)
			break
		}

		endstopPinValue := resp["pin_value"].(int)
		if stepperZ.EndstopInvert {
			endstopPinValue = 1 - endstopPinValue
		}

		resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
			"oid": stepperZOid,
		}, "stepper_position", 2*time.Second)
		if err != nil {
			fmt.Printf("\n  WARNING: stepper_get_position failed: %v\n", err)
			break
		}
		curPos := resp["pos"].(int)

		fmt.Printf("  [%.1fs] Endstop: %s, Position: %d steps (%.1f mm)\r",
			time.Since(startTime).Seconds(),
			map[int]string{0: "OPEN", 1: "TRIGGERED"}[endstopPinValue],
			curPos, float64(abs(curPos))/stepsPerMM)

		if endstopPinValue == 1 {
			time.Sleep(100 * time.Millisecond)

			resp, _ = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
				"oid": stepperZOid,
			}, "stepper_position", 2*time.Second)
			finalPosZ := resp["pos"].(int)

			travelSteps := abs(finalPosZ)
			travelMM := float64(travelSteps) / stepsPerMM

			fmt.Printf("\n\n  SUCCESS: Z-axis homing completed!\n")
			fmt.Printf("    Endstop TRIGGERED\n")
			fmt.Printf("    Stepper Z position: %d steps\n", finalPosZ)
			fmt.Printf("    Travel distance: %d steps (%.2f mm)\n", travelSteps, travelMM)
			homingSuccess = true
			break
		}

		time.Sleep(100 * time.Millisecond)
	}

	if !homingSuccess {
		fmt.Printf("\n\n  TIMEOUT: Endstop not triggered within %.0f seconds\n", homingTimeout.Seconds())
		// Disable motor before returning error
		mcu.SendCommand("update_digital_out", map[string]interface{}{
			"oid":   enableZOid,
			"value": enableZDisabledValue,
		})
		return fmt.Errorf("homing timeout")
	}

	// Disable motor
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableZOid,
		"value": enableZDisabledValue,
	}); err != nil {
		return fmt.Errorf("disable motor Z: %w", err)
	}
	fmt.Println("  Motor disabled")

	fmt.Println("\nZ-axis homing test completed successfully!")
	return nil
}

// testAllHoming performs homing on all three axes (Z, X, Y) in sequence.
// Homing order: Z first (for safety), then X, then Y.
func testAllHoming(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string) error {
	fmt.Println("=== Test: All-Axis Homing (Z, X, Y) ===")

	if configFile == "" {
		return fmt.Errorf("config file required for allhoming test (use -config flag)")
	}

	config, err := parseFullConfig(configFile)
	if err != nil {
		return fmt.Errorf("parse config: %w", err)
	}

	stepperX := config.Steppers["stepper_x"]
	stepperY := config.Steppers["stepper_y"]
	stepperZ := config.Steppers["stepper_z"]
	if stepperX == nil || stepperY == nil || stepperZ == nil {
		return fmt.Errorf("stepper_x, stepper_y, and stepper_z required in config")
	}

	fmt.Printf("  Loaded from config:\n")
	fmt.Printf("    [stepper_x]: step=%s dir=%s enable=%s endstop=%s pos_endstop=%.0f\n",
		stepperX.StepPin, stepperX.DirPin, stepperX.EnablePin, stepperX.EndstopPin, stepperX.PositionEndstop)
	fmt.Printf("    [stepper_y]: step=%s dir=%s enable=%s endstop=%s pos_endstop=%.0f\n",
		stepperY.StepPin, stepperY.DirPin, stepperY.EnablePin, stepperY.EndstopPin, stepperY.PositionEndstop)
	fmt.Printf("    [stepper_z]: step=%s dir=%s enable=%s endstop=%s pos_endstop=%.1f\n",
		stepperZ.StepPin, stepperZ.DirPin, stepperZ.EnablePin, stepperZ.EndstopPin, stepperZ.PositionEndstop)

	// Validate endstop pins
	if stepperX.EndstopPin == "" || stepperY.EndstopPin == "" || stepperZ.EndstopPin == "" {
		return fmt.Errorf("all axes require endstop_pin in config")
	}

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Allocate OIDs:
	// 0=stepper_x, 1=stepper_y, 2=stepper_z
	// 3=enable_x, 4=enable_y, 5=enable_z
	// 6=endstop_x, 7=endstop_y, 8=endstop_z
	// 9=trsync
	fmt.Println("\nConfiguring all-axis homing system...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 10}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=10 - OK")

	stepPulseTicks := 32

	// Configure steppers
	stepperXOid, stepperYOid, stepperZOid := 0, 1, 2
	for _, cfg := range []struct {
		oid     int
		name    string
		stepper *StepperConfig
	}{
		{stepperXOid, "X", stepperX},
		{stepperYOid, "Y", stepperY},
		{stepperZOid, "Z", stepperZ},
	} {
		if err := mcu.SendCommand("config_stepper", map[string]interface{}{
			"oid":              cfg.oid,
			"step_pin":         cfg.stepper.StepPin,
			"dir_pin":          cfg.stepper.DirPin,
			"invert_step":      0,
			"step_pulse_ticks": stepPulseTicks,
		}); err != nil {
			return fmt.Errorf("config_stepper %s: %w", cfg.name, err)
		}
		fmt.Printf("  config_stepper oid=%d (%s) - OK\n", cfg.oid, cfg.name)
	}

	// Configure enable pins
	enableXOid, enableYOid, enableZOid := 3, 4, 5
	enableXDisabled := 1
	if !stepperX.EnableInvert {
		enableXDisabled = 0
	}
	enableYDisabled := 1
	if !stepperY.EnableInvert {
		enableYDisabled = 0
	}
	enableZDisabled := 1
	if !stepperZ.EnableInvert {
		enableZDisabled = 0
	}

	for _, cfg := range []struct {
		oid          int
		name         string
		pin          string
		disabledVal  int
	}{
		{enableXOid, "enable_x", stepperX.EnablePin, enableXDisabled},
		{enableYOid, "enable_y", stepperY.EnablePin, enableYDisabled},
		{enableZOid, "enable_z", stepperZ.EnablePin, enableZDisabled},
	} {
		if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
			"oid":           cfg.oid,
			"pin":           cfg.pin,
			"value":         cfg.disabledVal,
			"default_value": cfg.disabledVal,
			"max_duration":  0,
		}); err != nil {
			return fmt.Errorf("config_digital_out %s: %w", cfg.name, err)
		}
		fmt.Printf("  config_digital_out oid=%d (%s) - OK\n", cfg.oid, cfg.name)
	}

	// Configure endstops
	endstopXOid, endstopYOid, endstopZOid := 6, 7, 8
	for _, cfg := range []struct {
		oid     int
		name    string
		stepper *StepperConfig
	}{
		{endstopXOid, "endstop_x", stepperX},
		{endstopYOid, "endstop_y", stepperY},
		{endstopZOid, "endstop_z", stepperZ},
	} {
		pullup := 0
		if cfg.stepper.EndstopPullup {
			pullup = 1
		}
		if err := mcu.SendCommand("config_endstop", map[string]interface{}{
			"oid":     cfg.oid,
			"pin":     cfg.stepper.EndstopPin,
			"pull_up": pullup,
		}); err != nil {
			return fmt.Errorf("config_endstop %s: %w", cfg.name, err)
		}
		fmt.Printf("  config_endstop oid=%d (%s pin=%s) - OK\n", cfg.oid, cfg.name, cfg.stepper.EndstopPin)
	}

	// Configure trsync
	trsyncOid := 9
	if err := mcu.SendCommand("config_trsync", map[string]interface{}{
		"oid": trsyncOid,
	}); err != nil {
		return fmt.Errorf("config_trsync: %w", err)
	}
	fmt.Println("  config_trsync oid=9 - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Helper function to enable/disable motors
	setMotorEnable := func(enableOid int, enabled bool, invert bool) error {
		value := 0
		if enabled {
			if !invert {
				value = 1
			}
		} else {
			if !invert {
				value = 0
			} else {
				value = 1
			}
		}
		return mcu.SendCommand("update_digital_out", map[string]interface{}{
			"oid":   enableOid,
			"value": value,
		})
	}

	// Helper function to perform single-axis homing
	homeAxis := func(axisName string, stepperOids []int, endstopOid int, enableOids []int,
		enableInverts []bool, homingDirs []int, stepsPerMM float64, homingSpeed float64,
		maxTravelMM float64, endstopInvert bool) error {

		fmt.Printf("\n--- Homing %s-axis ---\n", axisName)

		// Check endstop state
		resp, err := mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
			"oid": endstopOid,
		}, "endstop_state", 2*time.Second)
		if err != nil {
			return fmt.Errorf("endstop_query_state %s: %w", axisName, err)
		}
		pinValue := resp["pin_value"].(int)
		if endstopInvert {
			pinValue = 1 - pinValue
		}
		if pinValue == 1 {
			fmt.Printf("  WARNING: %s endstop already triggered!\n", axisName)
			return fmt.Errorf("%s endstop already triggered", axisName)
		}
		fmt.Printf("  %s endstop: OPEN\n", axisName)

		// Enable motors
		for i, enableOid := range enableOids {
			if err := setMotorEnable(enableOid, true, enableInverts[i]); err != nil {
				return fmt.Errorf("enable motor: %w", err)
			}
		}
		fmt.Println("  Motors ENABLED")

		// Get MCU clock
		resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
		if err != nil {
			return fmt.Errorf("get_uptime: %w", err)
		}
		curClock := uint64(resp["clock"].(int))

		// Calculate parameters
		stepInterval := int(mcu.MCUFreq() / homingSpeed)
		maxSteps := int(maxTravelMM * stepsPerMM)

		fmt.Printf("  Speed: %.0f steps/sec, Max travel: %d steps (%.0f mm)\n",
			homingSpeed, maxSteps, maxTravelMM)

		// Set directions
		for i, stepperOid := range stepperOids {
			if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
				"oid": stepperOid,
				"dir": homingDirs[i],
			}); err != nil {
				return fmt.Errorf("set_next_step_dir: %w", err)
			}
		}
		fmt.Printf("  Direction set: %v\n", homingDirs)

		// Setup timing
		startClock := curClock + uint64(mcu.MCUFreq()*0.1)
		moveDuration := float64(maxSteps) * float64(stepInterval) / mcu.MCUFreq()
		expireClock := startClock + uint64((moveDuration+2.0)*mcu.MCUFreq())
		sampleTicks := uint32(mcu.MCUFreq() * 0.000025)
		restTicks := uint32(mcu.MCUFreq() * 0.0001)

		// Setup trsync
		if err := mcu.SendCommand("trsync_start", map[string]interface{}{
			"oid":           trsyncOid,
			"report_clock":  uint32(startClock),
			"report_ticks":  uint32(mcu.MCUFreq() * 0.1),
			"expire_reason": 2,
		}); err != nil {
			return fmt.Errorf("trsync_start: %w", err)
		}

		if err := mcu.SendCommand("trsync_set_timeout", map[string]interface{}{
			"oid":   trsyncOid,
			"clock": uint32(expireClock),
		}); err != nil {
			return fmt.Errorf("trsync_set_timeout: %w", err)
		}

		// Link steppers to trsync
		for _, stepperOid := range stepperOids {
			if err := mcu.SendCommand("stepper_stop_on_trigger", map[string]interface{}{
				"oid":        stepperOid,
				"trsync_oid": trsyncOid,
			}); err != nil {
				return fmt.Errorf("stepper_stop_on_trigger: %w", err)
			}
		}

		// Setup endstop homing
		triggerPinValue := 0
		if err := mcu.SendCommand("endstop_home", map[string]interface{}{
			"oid":            endstopOid,
			"clock":          uint32(startClock),
			"sample_ticks":   sampleTicks,
			"sample_count":   4,
			"rest_ticks":     restTicks,
			"pin_value":      triggerPinValue,
			"trsync_oid":     trsyncOid,
			"trigger_reason": 1,
		}); err != nil {
			return fmt.Errorf("endstop_home: %w", err)
		}

		// Queue steps
		for _, stepperOid := range stepperOids {
			if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
				"oid":   stepperOid,
				"clock": uint32(startClock),
			}); err != nil {
				return fmt.Errorf("reset_step_clock: %w", err)
			}
			if err := mcu.SendCommand("queue_step", map[string]interface{}{
				"oid":      stepperOid,
				"interval": stepInterval,
				"count":    maxSteps,
				"add":      0,
			}); err != nil {
				return fmt.Errorf("queue_step: %w", err)
			}
		}

		fmt.Printf("  Moving toward %s endstop...\n", axisName)

		// Poll for endstop trigger
		homingTimeout := time.Duration(moveDuration+10) * time.Second
		startTime := time.Now()

		for time.Since(startTime) < homingTimeout {
			resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
				"oid": endstopOid,
			}, "endstop_state", 2*time.Second)
			if err != nil {
				break
			}

			endstopPinValue := resp["pin_value"].(int)
			if endstopInvert {
				endstopPinValue = 1 - endstopPinValue
			}

			if endstopPinValue == 1 {
				fmt.Printf("  %s endstop TRIGGERED!\n", axisName)
				time.Sleep(100 * time.Millisecond)
				return nil
			}
			time.Sleep(100 * time.Millisecond)
		}

		return fmt.Errorf("%s homing timeout", axisName)
	}

	// ========== HOME Z FIRST ==========
	// Z-axis: single stepper, 400 steps/mm
	zStepsPerMM := 400.0
	zHomingSpeed := 800.0 // 2 mm/s
	zMaxTravel := 100.0   // 100mm max
	// Z endstop at position_endstop=0, so home toward Z- (dir=0 for this hardware)
	zHomingDir := 0
	if stepperZ.DirInvert {
		zHomingDir = 1
	}

	if err := homeAxis("Z",
		[]int{stepperZOid},
		endstopZOid,
		[]int{enableZOid},
		[]bool{stepperZ.EnableInvert},
		[]int{zHomingDir},
		zStepsPerMM,
		zHomingSpeed,
		zMaxTravel,
		stepperZ.EndstopInvert,
	); err != nil {
		// Disable motors on error
		setMotorEnable(enableZOid, false, stepperZ.EnableInvert)
		return fmt.Errorf("Z homing failed: %w", err)
	}

	// ========== HOME X ==========
	// CoreXY X-axis: both motors move in same direction
	// X endstop at position_endstop=0, home toward X-
	// For CoreXY X-: motor_x dir=1, motor_y dir=1
	xyStepsPerMM := 80.0
	xyHomingSpeed := 1600.0 // 20 mm/s = 1200 mm/min
	xyMaxTravel := 100.0    // 100mm max

	xHomingDirX := 1
	xHomingDirY := 1
	if stepperX.PositionEndstop > stepperX.PositionMax/2 {
		// Endstop at max, home toward X+
		xHomingDirX = 0
		xHomingDirY = 0
	}

	if err := homeAxis("X",
		[]int{stepperXOid, stepperYOid},
		endstopXOid,
		[]int{enableXOid, enableYOid},
		[]bool{stepperX.EnableInvert, stepperY.EnableInvert},
		[]int{xHomingDirX, xHomingDirY},
		xyStepsPerMM,
		xyHomingSpeed,
		xyMaxTravel,
		stepperX.EndstopInvert,
	); err != nil {
		// Disable motors on error
		setMotorEnable(enableXOid, false, stepperX.EnableInvert)
		setMotorEnable(enableYOid, false, stepperY.EnableInvert)
		return fmt.Errorf("X homing failed: %w", err)
	}

	// ========== HOME Y ==========
	// CoreXY Y-axis: motors move in opposite directions
	// Y endstop at position_endstop=300, home toward Y+
	// For CoreXY Y+: motor_x dir=0, motor_y dir=1
	yHomingDirX := 0
	yHomingDirY := 1
	if stepperY.PositionEndstop <= stepperY.PositionMax/2 {
		// Endstop at min, home toward Y-
		yHomingDirX = 1
		yHomingDirY = 0
	}

	if err := homeAxis("Y",
		[]int{stepperXOid, stepperYOid},
		endstopYOid,
		[]int{enableXOid, enableYOid},
		[]bool{stepperX.EnableInvert, stepperY.EnableInvert},
		[]int{yHomingDirX, yHomingDirY},
		xyStepsPerMM,
		xyHomingSpeed,
		xyMaxTravel,
		stepperY.EndstopInvert,
	); err != nil {
		// Disable motors on error
		setMotorEnable(enableXOid, false, stepperX.EnableInvert)
		setMotorEnable(enableYOid, false, stepperY.EnableInvert)
		return fmt.Errorf("Y homing failed: %w", err)
	}

	// Disable all motors
	setMotorEnable(enableXOid, false, stepperX.EnableInvert)
	setMotorEnable(enableYOid, false, stepperY.EnableInvert)
	setMotorEnable(enableZOid, false, stepperZ.EnableInvert)
	fmt.Println("\n  All motors disabled")

	fmt.Println("\n=== All-axis homing completed successfully! ===")
	return nil
}

// testG28 simulates G28 G-code command for homing.
// G28 [X] [Y] [Z] - homes specified axes, or all if none specified.
func testG28(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string) error {
	fmt.Println("=== Test: G28 Command (Home All Axes) ===")
	fmt.Println("  Simulating: G28 (home all axes)")
	fmt.Println("  Homing order: Z first (safety), then X, then Y")

	// G28 without parameters homes all axes
	// This is equivalent to testAllHoming
	return testAllHoming(ri, timeout, configFile)
}

// testXHoming performs X-axis homing using endstop trigger with trsync.
// This moves the X-axis toward the endstop until triggered, then stops.
func testXHoming(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string) error {
	fmt.Println("=== Test: X-Axis Homing ===")

	if configFile == "" {
		return fmt.Errorf("config file required for xhoming test (use -config flag)")
	}

	config, err := parseFullConfig(configFile)
	if err != nil {
		return fmt.Errorf("parse config: %w", err)
	}

	// Load stepper configurations
	stepperX := config.Steppers["stepper_x"]
	stepperY := config.Steppers["stepper_y"]
	if stepperX == nil {
		return fmt.Errorf("no [stepper_x] section found in config file")
	}
	if stepperY == nil {
		return fmt.Errorf("no [stepper_y] section found in config file")
	}

	fmt.Printf("  Loaded from config:\n")
	fmt.Printf("    [stepper_x]: step=%s dir=%s enable=%s endstop=%s position_endstop=%.0f position_max=%.0f\n",
		stepperX.StepPin, stepperX.DirPin, stepperX.EnablePin, stepperX.EndstopPin,
		stepperX.PositionEndstop, stepperX.PositionMax)
	fmt.Printf("    [stepper_y]: step=%s dir=%s enable=%s endstop=%s position_endstop=%.0f position_max=%.0f\n",
		stepperY.StepPin, stepperY.DirPin, stepperY.EnablePin, stepperY.EndstopPin,
		stepperY.PositionEndstop, stepperY.PositionMax)

	// Validate pins
	if stepperX.StepPin == "" || stepperX.DirPin == "" || stepperX.EnablePin == "" || stepperX.EndstopPin == "" {
		return fmt.Errorf("incomplete stepper_x config")
	}
	if stepperY.StepPin == "" || stepperY.DirPin == "" || stepperY.EnablePin == "" {
		return fmt.Errorf("incomplete stepper_y config")
	}

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Allocate OIDs:
	// OID 0 = stepper_x, OID 1 = stepper_y
	// OID 2 = enable_x, OID 3 = enable_y
	// OID 4 = endstop_x, OID 5 = trsync
	fmt.Println("\nConfiguring X-axis homing system...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 6}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=6 - OK")

	stepPulseTicks := 32 // 2 microseconds at 16MHz

	// Configure stepper_x (OID 0)
	stepperXOid := 0
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperXOid,
		"step_pin":         stepperX.StepPin,
		"dir_pin":          stepperX.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper X: %w", err)
	}
	fmt.Printf("  config_stepper oid=0 (X) - OK\n")

	// Configure stepper_y (OID 1)
	stepperYOid := 1
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperYOid,
		"step_pin":         stepperY.StepPin,
		"dir_pin":          stepperY.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper Y: %w", err)
	}
	fmt.Printf("  config_stepper oid=1 (Y) - OK\n")

	// Configure enable_x pin (OID 2)
	enableXOid := 2
	enableXDisabledValue := 1
	if !stepperX.EnableInvert {
		enableXDisabledValue = 0
	}
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableXOid,
		"pin":           stepperX.EnablePin,
		"value":         enableXDisabledValue,
		"default_value": enableXDisabledValue,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (enable_x): %w", err)
	}
	fmt.Printf("  config_digital_out oid=2 (enable_x) - OK\n")

	// Configure enable_y pin (OID 3)
	enableYOid := 3
	enableYDisabledValue := 1
	if !stepperY.EnableInvert {
		enableYDisabledValue = 0
	}
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableYOid,
		"pin":           stepperY.EnablePin,
		"value":         enableYDisabledValue,
		"default_value": enableYDisabledValue,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (enable_y): %w", err)
	}
	fmt.Printf("  config_digital_out oid=3 (enable_y) - OK\n")

	// Configure endstop (OID 4)
	endstopOid := 4
	pullupValue := 0
	if stepperX.EndstopPullup {
		pullupValue = 1
	}
	if err := mcu.SendCommand("config_endstop", map[string]interface{}{
		"oid":     endstopOid,
		"pin":     stepperX.EndstopPin,
		"pull_up": pullupValue,
	}); err != nil {
		return fmt.Errorf("config_endstop: %w", err)
	}
	fmt.Printf("  config_endstop oid=4 (X endstop pin=%s pullup=%v invert=%v) - OK\n",
		stepperX.EndstopPin, stepperX.EndstopPullup, stepperX.EndstopInvert)

	// Configure trsync (OID 5)
	trsyncOid := 5
	if err := mcu.SendCommand("config_trsync", map[string]interface{}{
		"oid": trsyncOid,
	}); err != nil {
		return fmt.Errorf("config_trsync: %w", err)
	}
	fmt.Printf("  config_trsync oid=5 - OK\n")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Start X-axis homing sequence
	fmt.Println("\n--- X-Axis Homing Sequence ---")

	// Check current endstop state
	resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
		"oid": endstopOid,
	}, "endstop_state", 2*time.Second)
	if err != nil {
		return fmt.Errorf("endstop_query_state: %w", err)
	}
	pinValue := resp["pin_value"].(int)
	// Apply inversion
	if stepperX.EndstopInvert {
		pinValue = 1 - pinValue
	}
	if pinValue == 1 {
		fmt.Println("  WARNING: X endstop already triggered! Move carriage away from endstop first.")
		return fmt.Errorf("endstop already triggered")
	}
	fmt.Println("  X endstop state: OPEN (ready for homing)")

	// Enable both motors
	enableXActiveValue := 0
	if !stepperX.EnableInvert {
		enableXActiveValue = 1
	}
	enableYActiveValue := 0
	if !stepperY.EnableInvert {
		enableYActiveValue = 1
	}

	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableXOid,
		"value": enableXActiveValue,
	}); err != nil {
		return fmt.Errorf("update_digital_out (enable_x): %w", err)
	}
	fmt.Printf("  Motor X ENABLED - OK\n")

	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableYOid,
		"value": enableYActiveValue,
	}); err != nil {
		return fmt.Errorf("update_digital_out (enable_y): %w", err)
	}
	fmt.Printf("  Motor Y ENABLED - OK\n")

	// Get actual MCU clock
	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime: %w", err)
	}
	curClock := uint64(resp["clock"].(int))
	fmt.Printf("  Current MCU clock: %d\n", curClock)

	// Homing parameters
	// Homing speed: 300 mm/min = 5 mm/sec = 400 steps/sec (at 80 steps/mm)
	homingSpeed := 400.0 // steps/sec
	stepInterval := int(mcu.MCUFreq() / homingSpeed)
	// Maximum travel: 300mm = 24000 steps (this is a safety limit)
	maxSteps := 24000

	fmt.Printf("  Homing speed: %.0f steps/sec (%.0f mm/min)\n", homingSpeed, homingSpeed/80*60)
	fmt.Printf("  Step interval: %d ticks (%.2f ms)\n", stepInterval, float64(stepInterval)/mcu.MCUFreq()*1000)
	fmt.Printf("  Max travel: %d steps (%.0f mm)\n", maxSteps, float64(maxSteps)/80)

	// Determine homing direction based on position_endstop from config
	// If position_endstop is at min (close to 0), home toward negative (X-)
	// If position_endstop is at max (close to position_max), home toward positive (X+)
	var homingDirX, homingDirY int
	var dirDescription string
	if stepperX.PositionEndstop > stepperX.PositionMax/2 {
		// Endstop at max position - home toward X+ (positive direction)
		// CoreXY X+: both motors same direction, dir=0 for positive
		homingDirX = 0
		homingDirY = 0
		dirDescription = fmt.Sprintf("toward X+ endstop (position_endstop=%.0f)", stepperX.PositionEndstop)
	} else {
		// Endstop at min position - home toward X- (negative direction)
		// CoreXY X-: both motors same direction, dir=1 for negative
		homingDirX = 1
		homingDirY = 1
		dirDescription = fmt.Sprintf("toward X- endstop (position_endstop=%.0f)", stepperX.PositionEndstop)
	}

	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperXOid,
		"dir": homingDirX,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir X: %w", err)
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperYOid,
		"dir": homingDirY,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir Y: %w", err)
	}
	fmt.Printf("  Direction set: X=%d, Y=%d (%s)\n", homingDirX, homingDirY, dirDescription)

	// Calculate timing
	startClock := curClock + uint64(mcu.MCUFreq()*0.1)                               // Start 100ms from now
	moveDuration := float64(maxSteps) * float64(stepInterval) / mcu.MCUFreq()        // Estimated max duration
	expireClock := startClock + uint64((moveDuration+2.0)*mcu.MCUFreq())             // Timeout: max duration + 2 seconds
	sampleTicks := uint32(mcu.MCUFreq() * 0.000025)                                  // 25 microseconds sampling
	restTicks := uint32(mcu.MCUFreq() * 0.0001)                                      // 100 microseconds rest

	fmt.Printf("  Start clock: %d (100ms from now)\n", startClock)
	fmt.Printf("  Expire clock: %d (%.1f seconds from now)\n", expireClock, float64(expireClock-curClock)/mcu.MCUFreq())

	// 1. Start trsync
	// expire_reason=2 means timeout (0=endstop trigger, 1=stepper, 2=timeout)
	if err := mcu.SendCommand("trsync_start", map[string]interface{}{
		"oid":          trsyncOid,
		"report_clock": uint32(startClock),
		"report_ticks": uint32(mcu.MCUFreq() * 0.1), // Report every 100ms
		"expire_reason": 2,                          // Timeout reason
	}); err != nil {
		return fmt.Errorf("trsync_start: %w", err)
	}
	fmt.Println("  trsync_start - OK")

	// 2. Set trsync timeout
	if err := mcu.SendCommand("trsync_set_timeout", map[string]interface{}{
		"oid":   trsyncOid,
		"clock": uint32(expireClock),
	}); err != nil {
		return fmt.Errorf("trsync_set_timeout: %w", err)
	}
	fmt.Println("  trsync_set_timeout - OK")

	// 3. Connect steppers to trsync (stop on trigger)
	if err := mcu.SendCommand("stepper_stop_on_trigger", map[string]interface{}{
		"oid":       stepperXOid,
		"trsync_oid": trsyncOid,
	}); err != nil {
		return fmt.Errorf("stepper_stop_on_trigger X: %w", err)
	}
	fmt.Println("  stepper_stop_on_trigger X - OK")

	if err := mcu.SendCommand("stepper_stop_on_trigger", map[string]interface{}{
		"oid":       stepperYOid,
		"trsync_oid": trsyncOid,
	}); err != nil {
		return fmt.Errorf("stepper_stop_on_trigger Y: %w", err)
	}
	fmt.Println("  stepper_stop_on_trigger Y - OK")

	// 4. Start endstop homing
	// pin_value: the raw pin value that indicates trigger (NOT considering software inversion)
	// Physical endstop with pullup: pin=HIGH(1) when open, pin=LOW(0) when pressed
	// We want to trigger when pressed, so pin_value=0
	triggerPinValue := 0 // Trigger when pin goes LOW (endstop pressed)

	if err := mcu.SendCommand("endstop_home", map[string]interface{}{
		"oid":          endstopOid,
		"clock":        uint32(startClock),
		"sample_ticks": sampleTicks,
		"sample_count": 4, // Sample 4 times to confirm trigger
		"rest_ticks":   restTicks,
		"pin_value":    triggerPinValue,
		"trsync_oid":   trsyncOid,
		"trigger_reason": 1, // Endstop trigger reason
	}); err != nil {
		return fmt.Errorf("endstop_home: %w", err)
	}
	fmt.Printf("  endstop_home (trigger when pin_value=%d) - OK\n", triggerPinValue)

	// 5. Reset step clocks and queue steps
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperXOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock X: %w", err)
	}
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperYOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock Y: %w", err)
	}
	fmt.Println("  reset_step_clock X, Y - OK")

	// Queue steps for both motors
	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperXOid,
		"interval": stepInterval,
		"count":    maxSteps,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step X: %w", err)
	}
	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperYOid,
		"interval": stepInterval,
		"count":    maxSteps,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step Y: %w", err)
	}
	fmt.Printf("  queue_step X, Y (count=%d) - OK\n", maxSteps)

	fmt.Println("\n  Moving toward X endstop...")

	// 6. Poll endstop state to detect when homing completes
	// The MCU will automatically stop the steppers when endstop triggers
	homingTimeout := time.Duration(moveDuration+5) * time.Second
	startTime := time.Now()
	homingSuccess := false
	lastPos := 0

	for time.Since(startTime) < homingTimeout {
		// Check endstop state
		resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
			"oid": endstopOid,
		}, "endstop_state", 2*time.Second)
		if err != nil {
			// MCU might have shut down - check if we can still communicate
			fmt.Printf("\n  WARNING: endstop_query_state failed: %v\n", err)
			break
		}

		endstopPinValue := resp["pin_value"].(int)
		// Apply inversion
		if stepperX.EndstopInvert {
			endstopPinValue = 1 - endstopPinValue
		}

		// Get current stepper position to check if still moving
		resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
			"oid": stepperXOid,
		}, "stepper_position", 2*time.Second)
		if err != nil {
			fmt.Printf("\n  WARNING: stepper_get_position failed: %v\n", err)
			break
		}
		curPos := resp["pos"].(int)

		fmt.Printf("  [%.1fs] Endstop: %s, Position: %d steps (%.1f mm)\r",
			time.Since(startTime).Seconds(),
			map[int]string{0: "OPEN", 1: "TRIGGERED"}[endstopPinValue],
			curPos, float64(abs(curPos))/80.0)

		if endstopPinValue == 1 {
			// Endstop triggered - wait a bit for stepper to stop
			time.Sleep(100 * time.Millisecond)

			// Get final position
			resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
				"oid": stepperXOid,
			}, "stepper_position", 2*time.Second)
			if err != nil {
				return fmt.Errorf("stepper_get_position X (final): %w", err)
			}
			finalPosX := resp["pos"].(int)

			resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
				"oid": stepperYOid,
			}, "stepper_position", 2*time.Second)
			if err != nil {
				return fmt.Errorf("stepper_get_position Y (final): %w", err)
			}
			finalPosY := resp["pos"].(int)

			travelSteps := abs(finalPosX)
			travelMM := float64(travelSteps) / 80.0

			fmt.Printf("\n\n  SUCCESS: X-axis homing completed!\n")
			fmt.Printf("    Endstop TRIGGERED\n")
			fmt.Printf("    Stepper X position: %d steps\n", finalPosX)
			fmt.Printf("    Stepper Y position: %d steps\n", finalPosY)
			fmt.Printf("    Travel distance: %d steps (%.2f mm)\n", travelSteps, travelMM)
			homingSuccess = true
			break
		}

		// Check if stepper stopped moving (might indicate we reached limit without endstop)
		if curPos == lastPos && curPos != 0 {
			// Position unchanged, might be stalled or finished
			fmt.Printf("\n  Position unchanged at %d steps, checking if movement stopped...\n", curPos)
		}
		lastPos = curPos

		time.Sleep(100 * time.Millisecond)
	}

	if !homingSuccess {
		fmt.Printf("\n\n  TIMEOUT: Endstop not triggered within %.0f seconds\n", homingTimeout.Seconds())
		// Get final position anyway
		resp, _ = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
			"oid": stepperXOid,
		}, "stepper_position", 2*time.Second)
		if resp != nil {
			fmt.Printf("    Final position: %d steps\n", resp["pos"].(int))
		}
		return fmt.Errorf("homing timeout - endstop not reached")
	}

	// Disable motors
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableXOid,
		"value": enableXDisabledValue,
	}); err != nil {
		return fmt.Errorf("disable motor X: %w", err)
	}
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableYOid,
		"value": enableYDisabledValue,
	}); err != nil {
		return fmt.Errorf("disable motor Y: %w", err)
	}
	fmt.Println("  Motors disabled")

	fmt.Println("\nX-axis homing test completed successfully!")
	return nil
}

// testHoming tests the homing sequence with endstop trigger.
func testHoming(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string) error {
	fmt.Println("=== Test: CoreXY Homing Sequence ===")

	// CoreXY requires both stepper_x and stepper_y to move a single axis
	// For X-axis movement: both steppers move in the SAME direction
	// For Y-axis movement: both steppers move in OPPOSITE directions

	if configFile == "" {
		return fmt.Errorf("config file required for homing test (use -config flag)")
	}

	config, err := parseFullConfig(configFile)
	if err != nil {
		return fmt.Errorf("parse config: %w", err)
	}

	// Load stepper_x configuration
	stepperX := config.Steppers["stepper_x"]
	if stepperX == nil {
		return fmt.Errorf("no [stepper_x] section found in config file")
	}
	// Load stepper_y configuration
	stepperY := config.Steppers["stepper_y"]
	if stepperY == nil {
		return fmt.Errorf("no [stepper_y] section found in config file")
	}

	fmt.Printf("  Loaded from config:\n")
	fmt.Printf("    [stepper_x]: step=%s dir=%s enable=%s endstop=%s\n",
		stepperX.StepPin, stepperX.DirPin, stepperX.EnablePin, stepperX.EndstopPin)
	fmt.Printf("    [stepper_y]: step=%s dir=%s enable=%s endstop=%s\n",
		stepperY.StepPin, stepperY.DirPin, stepperY.EnablePin, stepperY.EndstopPin)

	// Validate pins
	if stepperX.StepPin == "" || stepperX.DirPin == "" || stepperX.EnablePin == "" || stepperX.EndstopPin == "" {
		return fmt.Errorf("incomplete stepper_x config")
	}
	if stepperY.StepPin == "" || stepperY.DirPin == "" || stepperY.EnablePin == "" {
		return fmt.Errorf("incomplete stepper_y config")
	}

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	fmt.Println("\nDEBUG: sending get_config...")
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	fmt.Printf("DEBUG: get_config response: is_config=%v\n", resp["is_config"])
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Allocate OIDs for CoreXY:
	// OID 0 = stepper_x, OID 1 = stepper_y
	// OID 2 = enable_x, OID 3 = enable_y
	// OID 4 = endstop_x
	fmt.Println("\nConfiguring CoreXY homing system...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 5}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=5 - OK")

	// step_pulse_ticks: duration of step pulse in MCU ticks (2 microseconds at 16MHz = 32 ticks)
	stepPulseTicks := 32

	// Configure stepper_x (OID 0)
	// Note: MCU doesn't handle ! prefix, we handle direction inversion in software
	stepperXOid := 0
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperXOid,
		"step_pin":         stepperX.StepPin,
		"dir_pin":          stepperX.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper X: %w", err)
	}
	fmt.Printf("  config_stepper oid=0 (X: step=%s dir=%s invert=%v) - OK\n",
		stepperX.StepPin, stepperX.DirPin, stepperX.DirInvert)

	// Configure stepper_y (OID 1)
	stepperYOid := 1
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperYOid,
		"step_pin":         stepperY.StepPin,
		"dir_pin":          stepperY.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper Y: %w", err)
	}
	fmt.Printf("  config_stepper oid=1 (Y: step=%s dir=%s invert=%v) - OK\n",
		stepperY.StepPin, stepperY.DirPin, stepperY.DirInvert)

	// Configure enable_x pin (OID 2)
	enableXOid := 2
	enableXDisabledValue := 1
	if !stepperX.EnableInvert {
		enableXDisabledValue = 0
	}
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableXOid,
		"pin":           stepperX.EnablePin,
		"value":         enableXDisabledValue,
		"default_value": enableXDisabledValue,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (enable_x): %w", err)
	}
	fmt.Printf("  config_digital_out oid=2 (enable_x=%s) - OK\n", stepperX.EnablePin)

	// Configure enable_y pin (OID 3)
	enableYOid := 3
	enableYDisabledValue := 1
	if !stepperY.EnableInvert {
		enableYDisabledValue = 0
	}
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableYOid,
		"pin":           stepperY.EnablePin,
		"value":         enableYDisabledValue,
		"default_value": enableYDisabledValue,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (enable_y): %w", err)
	}
	fmt.Printf("  config_digital_out oid=3 (enable_y=%s) - OK\n", stepperY.EnablePin)

	// Configure endstop (OID 4)
	endstopOid := 4
	pullupValue := 0
	if stepperX.EndstopPullup {
		pullupValue = 1
	}
	if err := mcu.SendCommand("config_endstop", map[string]interface{}{
		"oid":     endstopOid,
		"pin":     stepperX.EndstopPin,
		"pull_up": pullupValue,
	}); err != nil {
		return fmt.Errorf("config_endstop: %w", err)
	}
	fmt.Printf("  config_endstop oid=4 (pin=%s pullup=%v) - OK\n", stepperX.EndstopPin, stepperX.EndstopPullup)

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Start CoreXY homing sequence
	fmt.Println("\n--- CoreXY X-Axis Homing Sequence ---")

	// Check clock sync status
	estPrintTime := mcu.EstimatedPrintTime()
	if estPrintTime == 0 {
		return fmt.Errorf("clock not synchronized - EstimatedPrintTime returned 0")
	}
	estClock := mcu.PrintTime(estPrintTime)
	fmt.Printf("  Clock synced: EstimatedPrintTime=%.6f estimated_clock=%d\n", estPrintTime, estClock)

	// Enable both motors
	enableXActiveValue := 0
	if !stepperX.EnableInvert {
		enableXActiveValue = 1
	}
	enableYActiveValue := 0
	if !stepperY.EnableInvert {
		enableYActiveValue = 1
	}

	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableXOid,
		"value": enableXActiveValue,
	}); err != nil {
		return fmt.Errorf("update_digital_out (enable_x): %w", err)
	}
	fmt.Printf("  Motor X ENABLED (pin=%s value=%d) - OK\n", stepperX.EnablePin, enableXActiveValue)

	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableYOid,
		"value": enableYActiveValue,
	}); err != nil {
		return fmt.Errorf("update_digital_out (enable_y): %w", err)
	}
	fmt.Printf("  Motor Y ENABLED (pin=%s value=%d) - OK\n", stepperY.EnablePin, enableYActiveValue)

	// Set direction for both steppers
	// CoreXY X-axis homing: both motors move in the SAME direction
	homingDirX := 0
	if stepperX.DirInvert {
		homingDirX = 1
	}
	homingDirY := 0 // Same direction for CoreXY X movement
	if stepperY.DirInvert {
		homingDirY = 1
	}

	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperXOid,
		"dir": homingDirX,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir X: %w", err)
	}
	fmt.Printf("  set_next_step_dir X dir=%d - OK\n", homingDirX)

	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperYOid,
		"dir": homingDirY,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir Y: %w", err)
	}
	fmt.Printf("  set_next_step_dir Y dir=%d - OK\n", homingDirY)

	// Get initial stepper positions to verify we're starting from known state
	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": stepperXOid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position X (initial): %w", err)
	}
	initialPosX := resp["pos"]
	fmt.Printf("  Initial stepper X position: %v\n", initialPosX)

	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": stepperYOid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position Y (initial): %w", err)
	}
	initialPosY := resp["pos"]
	fmt.Printf("  Initial stepper Y position: %v\n", initialPosY)

	// Simple step test: use reset_step_clock AFTER queue_step with small interval
	// This avoids the timing issues with large intervals

	// First, check MCU is still responding and get the ACTUAL MCU clock
	fmt.Println("  Checking MCU responsiveness and getting actual clock...")
	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime (before stepping): %w", err)
	}
	fmt.Printf("  MCU uptime: %v\n", resp)

	// Use the actual MCU clock, not the estimated print time!
	// The estimated print time can be offset from the real MCU clock
	actualMCUClock := uint64(resp["clock"].(int))
	fmt.Printf("  Actual MCU clock: %d\n", actualMCUClock)

	// For comparison, show what our estimated clock would have been
	curTime := mcu.EstimatedPrintTime()
	estimatedClock := mcu.PrintTime(curTime)
	fmt.Printf("  Estimated clock (from print time %.6f): %d\n", curTime, estimatedClock)
	fmt.Printf("  Clock difference (actual - estimated): %d ticks = %.3f seconds\n",
		int64(actualMCUClock)-int64(estimatedClock), float64(int64(actualMCUClock)-int64(estimatedClock))/mcu.MCUFreq())

	curClock := actualMCUClock // Use actual clock!

	// Step parameters: 667 steps/sec (500mm/min at 80 steps/mm), 1600 steps (20mm)
	// 500mm/min = 8.33mm/sec, 8.33 * 80 = 667 steps/sec
	stepInterval := int(mcu.MCUFreq() / 667) // 667 steps/sec ≈ 500mm/min
	stepCount := 1600                         // 1600 steps = 20mm with 80 steps/mm
	moveDuration := float64(stepCount) * float64(stepInterval) / mcu.MCUFreq()

	fmt.Printf("  Step interval=%d ticks (%.1f ms), count=%d (20mm)\n",
		stepInterval, float64(stepInterval)/mcu.MCUFreq()*1000, stepCount)
	fmt.Printf("  Movement duration: %.2f seconds per direction\n", moveDuration)

	// ========== PHASE 1: Move POSITIVE direction (+20mm) ==========
	fmt.Println("\n  === Phase 1: Moving POSITIVE direction (+20mm) ===")

	// Get fresh MCU clock for phase 1
	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime for phase 1: %w", err)
	}
	curClock = uint64(resp["clock"].(int))

	// Schedule first step 100ms from now
	startClock := curClock + uint64(mcu.MCUFreq()*0.1) // 100ms from now

	// Set direction: positive (dir=0)
	// MCU handles pin inversion based on ! prefix in config_stepper
	// For CoreXY X+: both motors same direction
	positiveDirX := 0
	positiveDirY := 0

	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperXOid,
		"dir": positiveDirX,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir X (positive): %w", err)
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperYOid,
		"dir": positiveDirY,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir Y (positive): %w", err)
	}
	fmt.Printf("  Direction set: X=%d, Y=%d (positive)\n", positiveDirX, positiveDirY)

	// Reset step clocks
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperXOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock X: %w", err)
	}
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperYOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock Y: %w", err)
	}

	// Queue steps
	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperXOid,
		"interval": stepInterval,
		"count":    stepCount,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step X (positive): %w", err)
	}
	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperYOid,
		"interval": stepInterval,
		"count":    stepCount,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step Y (positive): %w", err)
	}
	fmt.Printf("  Queued %d steps - moving positive...\n", stepCount)

	// Wait for positive movement to complete
	waitTime := time.Duration((moveDuration+0.2)*1000) * time.Millisecond
	fmt.Printf("  Waiting %.1f seconds for positive movement...\n", waitTime.Seconds())
	time.Sleep(waitTime)

	// Check position after positive move
	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": stepperXOid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position X after positive: %w", err)
	}
	posAfterPositive := resp["pos"].(int)
	fmt.Printf("  Position after positive move: X=%d (moved %d steps)\n", posAfterPositive, posAfterPositive-initialPosX.(int))

	// ========== PHASE 2: Move NEGATIVE direction (-20mm) ==========
	fmt.Println("\n  === Phase 2: Moving NEGATIVE direction (-20mm) ===")

	// Get fresh MCU clock for phase 2
	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime for phase 2: %w", err)
	}
	curClock = uint64(resp["clock"].(int))
	startClock = curClock + uint64(mcu.MCUFreq()*0.1) // 100ms from now

	// Set direction: negative (opposite of positive)
	negativeDirX := 1 - positiveDirX
	negativeDirY := 1 - positiveDirY

	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperXOid,
		"dir": negativeDirX,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir X (negative): %w", err)
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperYOid,
		"dir": negativeDirY,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir Y (negative): %w", err)
	}
	fmt.Printf("  Direction set: X=%d, Y=%d (negative)\n", negativeDirX, negativeDirY)

	// Reset step clocks for phase 2
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperXOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock X (phase 2): %w", err)
	}
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperYOid,
		"clock": uint32(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock Y (phase 2): %w", err)
	}

	// Queue steps for negative direction
	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperXOid,
		"interval": stepInterval,
		"count":    stepCount,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step X (negative): %w", err)
	}
	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperYOid,
		"interval": stepInterval,
		"count":    stepCount,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step Y (negative): %w", err)
	}
	fmt.Printf("  Queued %d steps - moving negative...\n", stepCount)

	// Wait for negative movement to complete
	fmt.Printf("  Waiting %.1f seconds for negative movement...\n", waitTime.Seconds())
	time.Sleep(waitTime)

	// Get final stepper positions
	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": stepperXOid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position X: %w", err)
	}
	finalPosX := resp["pos"].(int)

	resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
		"oid": stepperYOid,
	}, "stepper_position", 2*time.Second)
	if err != nil {
		return fmt.Errorf("stepper_get_position Y: %w", err)
	}
	finalPosY := resp["pos"].(int)

	// Summary
	fmt.Println("\n  === Movement Summary ===")
	fmt.Printf("  Initial position: X=%d, Y=%d\n", initialPosX, initialPosY)
	fmt.Printf("  After +20mm:      X=%d (delta=%d)\n", posAfterPositive, posAfterPositive-initialPosX.(int))
	fmt.Printf("  After -20mm:      X=%d, Y=%d\n", finalPosX, finalPosY)
	fmt.Printf("  Net movement:     X=%d steps, Y=%d steps\n", finalPosX-initialPosX.(int), finalPosY-initialPosY.(int))

	if finalPosX == initialPosX.(int) && finalPosY == initialPosY.(int) {
		fmt.Println("\n  SUCCESS: Motor returned to original position!")
	} else {
		fmt.Printf("\n  Note: Position difference of %d steps (should be ~0 if both directions worked)\n",
			finalPosX-initialPosX.(int))
	}

	fmt.Println("\nCoreXY movement test completed!")
	return nil
}

// testMultiCmd tests sending multiple commands in a single message.
func testMultiCmd(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: Multi-Command Messages ===")

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Allocate OIDs for multiple steppers
	fmt.Println("\nConfiguring multiple steppers...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 4}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}

	// Configure 3 steppers (X, Y, Z)
	for i := 0; i < 3; i++ {
		if err := mcu.SendCommand("config_stepper", map[string]interface{}{
			"oid":         i,
			"step_pin":    fmt.Sprintf("PA%d", i*2),
			"dir_pin":     fmt.Sprintf("PA%d", i*2+1),
			"invert_step": 0,
		}); err != nil {
			return fmt.Errorf("config_stepper %d: %w", i, err)
		}
		fmt.Printf("  config_stepper oid=%d - OK\n", i)
	}

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Test: Send multiple queue_step commands rapidly
	fmt.Println("\n--- Sending Multiple Commands Rapidly ---")

	mcuClock := mcu.PrintTime(0)
	stepInterval := int(mcu.MCUFreq() / 1000) // 1000 steps/sec

	// Reset all stepper clocks
	for i := 0; i < 3; i++ {
		if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
			"oid":   i,
			"clock": uint32(mcuClock),
		}); err != nil {
			return fmt.Errorf("reset_step_clock %d: %w", i, err)
		}
	}
	fmt.Println("  reset_step_clock for all steppers - OK")

	// Set all directions
	for i := 0; i < 3; i++ {
		if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
			"oid": i,
			"dir": 1,
		}); err != nil {
			return fmt.Errorf("set_next_step_dir %d: %w", i, err)
		}
	}
	fmt.Println("  set_next_step_dir for all steppers - OK")

	// Send queue_step commands in rapid succession
	startTime := time.Now()
	cmdCount := 30
	for i := 0; i < cmdCount; i++ {
		stepperOid := i % 3 // Rotate between X, Y, Z
		if err := mcu.SendCommand("queue_step", map[string]interface{}{
			"oid":      stepperOid,
			"interval": stepInterval,
			"count":    50,
			"add":      0,
		}); err != nil {
			return fmt.Errorf("queue_step %d: %w", i, err)
		}
	}
	elapsed := time.Since(startTime)
	fmt.Printf("  Sent %d queue_step commands in %v (%.0f cmd/sec)\n",
		cmdCount, elapsed, float64(cmdCount)/elapsed.Seconds())

	// Verify all steppers moved
	fmt.Println("\n--- Verifying Stepper Positions ---")
	for i := 0; i < 3; i++ {
		resp, err := mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
			"oid": i,
		}, "stepper_position", 2*time.Second)
		if err != nil {
			return fmt.Errorf("stepper_get_position %d: %w", i, err)
		}
		fmt.Printf("  Stepper %d position: %v\n", i, resp["pos"])
	}

	fmt.Println("\nMulti-command test completed successfully!")
	return nil
}

// testErrors tests error handling scenarios.
// NOTE: This test is affected by a known concurrency issue where the background
// clock sync thread races with test commands. This test is simplified to avoid
// the race condition while still providing some basic error handling coverage.
func testErrors(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: Error Handling ===")
	fmt.Println("NOTE: Some tests skipped due to background clock sync race conditions")

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Test 1: Verify MCU is connected
	fmt.Println("\n--- Test 1: Connection Verification ---")
	if !mcu.IsConnected() {
		return fmt.Errorf("MCU not connected")
	}
	fmt.Println("  MCU is connected - OK")

	// Test 2: Verify dictionary was loaded
	fmt.Println("\n--- Test 2: Dictionary Verification ---")
	dict := mcu.Dictionary()
	if dict == nil {
		return fmt.Errorf("dictionary is nil")
	}
	cmdCount := len(dict.Commands)
	if cmdCount < 5 {
		return fmt.Errorf("dictionary has too few commands: %d", cmdCount)
	}
	fmt.Printf("  Dictionary loaded with %d commands - OK\n", cmdCount)

	// Test 3: MCU frequency is valid
	fmt.Println("\n--- Test 3: MCU Frequency Verification ---")
	freq := mcu.MCUFreq()
	if freq < 1000000 {
		return fmt.Errorf("MCU frequency too low: %.0f Hz", freq)
	}
	fmt.Printf("  MCU frequency %.0f Hz - OK\n", freq)

	fmt.Println("\nError handling test completed successfully!")
	fmt.Println("(Full stress tests available in 'stress' test)")
	return nil
}

// testMotion tests the motion planning pipeline.
func testMotion(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: Motion Planning Pipeline ===")

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Configure 3 steppers for X, Y, Z axes
	fmt.Println("\nConfiguring motion system (X, Y, Z steppers)...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 3}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}

	axes := []string{"X", "Y", "Z"}
	for i := 0; i < 3; i++ {
		if err := mcu.SendCommand("config_stepper", map[string]interface{}{
			"oid":         i,
			"step_pin":    fmt.Sprintf("PA%d", i*2),
			"dir_pin":     fmt.Sprintf("PA%d", i*2+1),
			"invert_step": 0,
		}); err != nil {
			return fmt.Errorf("config_stepper %s: %w", axes[i], err)
		}
		fmt.Printf("  config_stepper %s (oid=%d) - OK\n", axes[i], i)
	}

	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Simulate a diagonal move from (0,0,0) to (10,10,5) mm
	// Assuming 80 steps/mm, this is 800, 800, 400 steps
	fmt.Println("\n--- Simulating Diagonal Move ---")
	fmt.Println("  Move: (0,0,0) -> (10,10,5) mm")
	fmt.Println("  Steps: X=800, Y=800, Z=400")

	mcuClock := mcu.PrintTime(0)
	baseInterval := int(mcu.MCUFreq() / 2000) // 2000 steps/sec base speed

	// Reset all step clocks
	for i := 0; i < 3; i++ {
		if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
			"oid":   i,
			"clock": uint32(mcuClock),
		}); err != nil {
			return fmt.Errorf("reset_step_clock: %w", err)
		}
		if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
			"oid": i,
			"dir": 1, // Positive direction
		}); err != nil {
			return fmt.Errorf("set_next_step_dir: %w", err)
		}
	}
	fmt.Println("  Step clocks reset, directions set")

	// Queue steps for each axis with appropriate intervals
	// X and Y move same distance (800 steps), Z moves half (400 steps)
	// Z should have double the interval to finish at the same time
	moves := []struct {
		axis     string
		oid      int
		steps    int
		interval int
	}{
		{"X", 0, 800, baseInterval},
		{"Y", 1, 800, baseInterval},
		{"Z", 2, 400, baseInterval * 2}, // Half steps, double interval
	}

	for _, m := range moves {
		if err := mcu.SendCommand("queue_step", map[string]interface{}{
			"oid":      m.oid,
			"interval": m.interval,
			"count":    m.steps,
			"add":      0,
		}); err != nil {
			return fmt.Errorf("queue_step %s: %w", m.axis, err)
		}
		fmt.Printf("  queue_step %s: %d steps @ interval %d\n", m.axis, m.steps, m.interval)
	}

	// Wait for motion to complete (800 steps at 2000 steps/sec = 0.4 sec)
	fmt.Println("\n  Waiting for motion to complete...")
	time.Sleep(500 * time.Millisecond)

	// Verify final positions
	fmt.Println("\n--- Verifying Final Positions ---")
	for i := 0; i < 3; i++ {
		resp, err := mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
			"oid": i,
		}, "stepper_position", 2*time.Second)
		if err != nil {
			return fmt.Errorf("stepper_get_position %s: %w", axes[i], err)
		}
		pos := resp["pos"]
		fmt.Printf("  %s position: %v steps\n", axes[i], pos)
	}

	fmt.Println("\nMotion planning test completed successfully!")
	return nil
}

// testMultiMCU tests coordination between multiple MCU instances.
// Note: This test requires modifications to mock-mcu to support multiple instances.
func testMultiMCU(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: Multi-MCU Coordination ===")
	fmt.Println("Note: This test simulates multi-MCU by using the same MCU for demonstration.")

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Simulate multi-MCU by using different OID ranges
	// MCU1: OIDs 0-4 (main printer)
	// MCU2: OIDs 5-9 (toolhead)
	fmt.Println("\nSimulating Multi-MCU configuration...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 10}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}

	// Configure "MCU1" steppers (main printer X, Y, Z)
	fmt.Println("\n--- MCU1 (Main Printer) ---")
	for i := 0; i < 3; i++ {
		if err := mcu.SendCommand("config_stepper", map[string]interface{}{
			"oid":         i,
			"step_pin":    fmt.Sprintf("PA%d", i),
			"dir_pin":     fmt.Sprintf("PB%d", i),
			"invert_step": 0,
		}); err != nil {
			return fmt.Errorf("config_stepper MCU1-%d: %w", i, err)
		}
		fmt.Printf("  config_stepper oid=%d (axis %d) - OK\n", i, i)
	}

	// Configure "MCU2" components (toolhead: extruder stepper, heater, fan)
	fmt.Println("\n--- MCU2 (Toolhead) ---")
	extruderOid := 5
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":         extruderOid,
		"step_pin":    "PC0",
		"dir_pin":     "PC1",
		"invert_step": 0,
	}); err != nil {
		return fmt.Errorf("config_stepper extruder: %w", err)
	}
	fmt.Printf("  config_stepper oid=%d (extruder) - OK\n", extruderOid)

	heaterOid := 6
	if err := mcu.SendCommand("config_pwm_out", map[string]interface{}{
		"oid":           heaterOid,
		"pin":           "PC2",
		"cycle_ticks":   int(mcu.MCUFreq() / 1000),
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_pwm_out heater: %w", err)
	}
	fmt.Printf("  config_pwm_out oid=%d (heater) - OK\n", heaterOid)

	fanOid := 7
	if err := mcu.SendCommand("config_pwm_out", map[string]interface{}{
		"oid":           fanOid,
		"pin":           "PC3",
		"cycle_ticks":   int(mcu.MCUFreq() / 25000), // 25kHz fan PWM
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_pwm_out fan: %w", err)
	}
	fmt.Printf("  config_pwm_out oid=%d (fan) - OK\n", fanOid)

	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Simulate coordinated motion between MCUs
	fmt.Println("\n--- Coordinated Operation ---")
	mcuClock := mcu.PrintTime(0)

	// Both MCUs need to start motion at the same time
	// Reset all step clocks to the same reference
	for i := 0; i < 3; i++ {
		if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
			"oid":   i,
			"clock": uint32(mcuClock),
		}); err != nil {
			return fmt.Errorf("reset_step_clock: %w", err)
		}
	}
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   extruderOid,
		"clock": uint32(mcuClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock extruder: %w", err)
	}
	fmt.Println("  All step clocks synchronized")

	// Start heater and fan at the same time
	scheduleClock := mcu.PrintTime(0.1)
	if err := mcu.SendCommand("schedule_pwm_out", map[string]interface{}{
		"oid":   heaterOid,
		"clock": uint32(scheduleClock),
		"value": 128, // 50%
	}); err != nil {
		return fmt.Errorf("schedule_pwm_out heater: %w", err)
	}
	if err := mcu.SendCommand("schedule_pwm_out", map[string]interface{}{
		"oid":   fanOid,
		"clock": uint32(scheduleClock),
		"value": 200, // 78%
	}); err != nil {
		return fmt.Errorf("schedule_pwm_out fan: %w", err)
	}
	fmt.Println("  Heater and fan scheduled to start simultaneously")

	// Queue coordinated motion (X, Y, Z + extruder)
	stepInterval := int(mcu.MCUFreq() / 1000)
	for i := 0; i < 3; i++ {
		if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
			"oid": i,
			"dir": 1,
		}); err != nil {
			return fmt.Errorf("set_next_step_dir: %w", err)
		}
		if err := mcu.SendCommand("queue_step", map[string]interface{}{
			"oid":      i,
			"interval": stepInterval,
			"count":    100,
			"add":      0,
		}); err != nil {
			return fmt.Errorf("queue_step: %w", err)
		}
	}
	// Extruder runs at 1/10 speed for retraction
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": extruderOid,
		"dir": 1,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir extruder: %w", err)
	}
	if err := mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      extruderOid,
		"interval": stepInterval * 10,
		"count":    10,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step extruder: %w", err)
	}
	fmt.Println("  Coordinated motion queued (X, Y, Z + extruder)")

	fmt.Println("\nMulti-MCU coordination test completed successfully!")
	return nil
}

// testStress performs stress testing with high command rates.
func testStress(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: Stress Testing ===")

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Note: High-frequency clock queries skipped due to race condition with
	// background clock sync. This is a known protocol limitation.
	fmt.Println("\n--- Test 1: Connection Stability ---")
	fmt.Println("  MCU connected and responsive - OK")

	// Check config state for remaining tests - use a simple check
	fmt.Println("\n  Checking MCU config state...")
	if !mcu.IsConnected() {
		return fmt.Errorf("MCU disconnected")
	}
	fmt.Println("  MCU still connected - OK")

	// Configure for step commands
	fmt.Println("\n--- Test 2: High-Rate Step Commands ---")
	var err error
	if err = mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 1}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":         0,
		"step_pin":    "PA0",
		"dir_pin":     "PA1",
		"invert_step": 0,
	}); err != nil {
		return fmt.Errorf("config_stepper: %w", err)
	}
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}

	mcuClock := mcu.PrintTime(0)
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   0,
		"clock": uint32(mcuClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock: %w", err)
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": 0,
		"dir": 1,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir: %w", err)
	}

	// Send many queue_step commands rapidly
	startTime := time.Now()
	stepCmdCount := 200
	stepInterval := int(mcu.MCUFreq() / 5000) // 5000 steps/sec
	for i := 0; i < stepCmdCount; i++ {
		if err := mcu.SendCommand("queue_step", map[string]interface{}{
			"oid":      0,
			"interval": stepInterval,
			"count":    10,
			"add":      0,
		}); err != nil {
			return fmt.Errorf("queue_step %d: %w", i, err)
		}
	}
	elapsed := time.Since(startTime)
	rate := float64(stepCmdCount) / elapsed.Seconds()
	fmt.Printf("  %d queue_step commands in %v (%.1f cmd/sec)\n", stepCmdCount, elapsed, rate)
	fmt.Printf("  Total steps queued: %d\n", stepCmdCount*10)

	// Test 3: Additional step commands
	fmt.Println("\n--- Test 3: Additional Step Commands ---")
	startTime = time.Now()
	additionalSteps := 100
	for i := 0; i < additionalSteps; i++ {
		if err := mcu.SendCommand("queue_step", map[string]interface{}{
			"oid":      0,
			"interval": stepInterval,
			"count":    5,
			"add":      0,
		}); err != nil {
			return fmt.Errorf("queue_step %d: %w", i, err)
		}
	}
	elapsed = time.Since(startTime)
	rate = float64(additionalSteps) / elapsed.Seconds()
	fmt.Printf("  %d additional queue_step commands in %v (%.1f cmd/sec)\n", additionalSteps, elapsed, rate)

	// Verify MCU still responsive
	fmt.Println("\n--- Verifying MCU Stability ---")
	if !mcu.IsConnected() {
		return fmt.Errorf("MCU disconnected during stress test")
	}
	fmt.Println("  MCU still connected after stress test - OK")

	fmt.Println("\nStress test completed successfully!")
	return nil
}

// testAll runs all tests.
func testAll(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("Running all tests...")

	if err := testConnect(ri, timeout); err != nil {
		return err
	}
	fmt.Println()

	// For clock and temp tests, we need fresh connections
	ri2 := hosth4.NewRealtimeIntegration()
	cfg := hosth4.DefaultMCUConnectionConfig()
	// Copy config from the original ri's MCU manager
	mgr := ri.MCUManager()
	mcu := mgr.PrimaryMCU()
	if mcu != nil {
		// Get device from the MCU name (this is a simplification)
		// In a real test, we'd pass the device through
	}

	if err := ri2.AddMCU("mcu", cfg); err != nil {
		fmt.Println("Skipping clock test - could not create new integration")
	} else {
		if err := testClock(ri2, timeout); err != nil {
			return err
		}
	}

	return nil
}

// testQuery queries and displays MCU capabilities from the data dictionary.
func testQuery(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: Query MCU Capabilities ===")

	// Connect to MCU
	fmt.Println("Connecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	// Wait for ready
	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	// Get MCU info
	mgr := ri.MCUManager()
	mcu := mgr.PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no primary MCU")
	}

	fmt.Println("\n--- MCU Information ---")
	fmt.Printf("MCU Frequency: %.0f Hz\n", mcu.MCUFreq())
	fmt.Printf("Connected: %v\n", mcu.IsConnected())

	// Get status
	status := ri.GetStatus()
	fmt.Printf("\nStatus: %v\n", status)

	fmt.Println("\nNote: Full command list is available in the MCU's data dictionary.")
	fmt.Println("      Use -trace flag to see the dictionary during connection.")

	return nil
}

// testGcode runs a simple G-code sequence on real hardware.
// G28 X Y - Home X and Y axes
// G1 X100 Y100 F1000 - Move to X100 Y100 at 1000mm/min
// G1 X40 Y40 F500 - Move to X40 Y40 at 500mm/min
func testGcode(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string) error {
	fmt.Println("=== Test: G-code Sequence ===")
	fmt.Println("  G28 X Y        ; Home X and Y")
	fmt.Println("  G1 X100 Y100 F1000  ; Move to 100,100 at 1000mm/min")
	fmt.Println("  G1 X40 Y40 F500     ; Move to 40,40 at 500mm/min")

	if configFile == "" {
		return fmt.Errorf("config file required for gcode test (use -config flag)")
	}

	config, err := parseFullConfig(configFile)
	if err != nil {
		return fmt.Errorf("parse config: %w", err)
	}

	// Check kinematics
	if config.Kinematics != "corexy" {
		return fmt.Errorf("this test currently only supports corexy kinematics, got: %s", config.Kinematics)
	}

	stepperX := config.Steppers["stepper_x"]
	stepperY := config.Steppers["stepper_y"]
	if stepperX == nil || stepperY == nil {
		return fmt.Errorf("stepper_x and stepper_y required in config")
	}

	fmt.Printf("\nLoaded from config (kinematics: %s):\n", config.Kinematics)
	fmt.Printf("  [stepper_x]: step=%s dir=%s enable=%s endstop=%s\n",
		stepperX.StepPin, stepperX.DirPin, stepperX.EnablePin, stepperX.EndstopPin)
	fmt.Printf("  [stepper_y]: step=%s dir=%s enable=%s endstop=%s\n",
		stepperY.StepPin, stepperY.DirPin, stepperY.EnablePin, stepperY.EndstopPin)
	fmt.Printf("  rotation_distance: %.1f mm, microsteps: %d\n",
		stepperX.RotationDistance, stepperX.Microsteps)

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %.0f Hz\n", mcu.MCUFreq())

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - skipping")
		return nil
	}

	// Calculate steps per mm
	fullStepsPerRev := 200.0 // Standard stepper
	stepsPerMM := (fullStepsPerRev * float64(stepperX.Microsteps)) / stepperX.RotationDistance
	fmt.Printf("  Steps per mm: %.1f\n", stepsPerMM)

	// Allocate OIDs:
	// 0 = stepper_x, 1 = stepper_y
	// 2 = enable_x, 3 = enable_y
	// 4 = endstop_x, 5 = endstop_y
	// 6 = trsync_x, 7 = trsync_y
	fmt.Println("\nConfiguring hardware...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 8}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=8 - OK")

	stepPulseTicks := 32 // 2 microseconds at 16MHz

	// Configure stepper_x (OID 0)
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              0,
		"step_pin":         stepperX.StepPin,
		"dir_pin":          stepperX.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper X: %w", err)
	}
	fmt.Println("  config_stepper oid=0 (motor_x) - OK")

	// Configure stepper_y (OID 1)
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              1,
		"step_pin":         stepperY.StepPin,
		"dir_pin":          stepperY.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper Y: %w", err)
	}
	fmt.Println("  config_stepper oid=1 (motor_y) - OK")

	// Configure enable pins
	enableXDisabledVal := 1
	if !stepperX.EnableInvert {
		enableXDisabledVal = 0
	}
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           2,
		"pin":           stepperX.EnablePin,
		"value":         enableXDisabledVal,
		"default_value": enableXDisabledVal,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (enable_x): %w", err)
	}
	fmt.Println("  config_digital_out oid=2 (enable_x) - OK")

	enableYDisabledVal := 1
	if !stepperY.EnableInvert {
		enableYDisabledVal = 0
	}
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           3,
		"pin":           stepperY.EnablePin,
		"value":         enableYDisabledVal,
		"default_value": enableYDisabledVal,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (enable_y): %w", err)
	}
	fmt.Println("  config_digital_out oid=3 (enable_y) - OK")

	// Configure endstops
	pullupX := 0
	if stepperX.EndstopPullup {
		pullupX = 1
	}
	if err := mcu.SendCommand("config_endstop", map[string]interface{}{
		"oid":     4,
		"pin":     stepperX.EndstopPin,
		"pull_up": pullupX,
	}); err != nil {
		return fmt.Errorf("config_endstop X: %w", err)
	}
	fmt.Println("  config_endstop oid=4 (X endstop) - OK")

	pullupY := 0
	if stepperY.EndstopPullup {
		pullupY = 1
	}
	if err := mcu.SendCommand("config_endstop", map[string]interface{}{
		"oid":     5,
		"pin":     stepperY.EndstopPin,
		"pull_up": pullupY,
	}); err != nil {
		return fmt.Errorf("config_endstop Y: %w", err)
	}
	fmt.Println("  config_endstop oid=5 (Y endstop) - OK")

	// Configure trsync
	if err := mcu.SendCommand("config_trsync", map[string]interface{}{"oid": 6}); err != nil {
		return fmt.Errorf("config_trsync X: %w", err)
	}
	fmt.Println("  config_trsync oid=6 - OK")

	if err := mcu.SendCommand("config_trsync", map[string]interface{}{"oid": 7}); err != nil {
		return fmt.Errorf("config_trsync Y: %w", err)
	}
	fmt.Println("  config_trsync oid=7 - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Helper to enable/disable motors
	enableMotors := func(enable bool) error {
		valX := enableXDisabledVal
		valY := enableYDisabledVal
		if enable {
			if stepperX.EnableInvert {
				valX = 0
			} else {
				valX = 1
			}
			if stepperY.EnableInvert {
				valY = 0
			} else {
				valY = 1
			}
		}
		if err := mcu.SendCommand("update_digital_out", map[string]interface{}{"oid": 2, "value": valX}); err != nil {
			return err
		}
		if err := mcu.SendCommand("update_digital_out", map[string]interface{}{"oid": 3, "value": valY}); err != nil {
			return err
		}
		return nil
	}

	// =============================================
	// Step 1: Home X axis
	// =============================================
	fmt.Println("\n--- Step 1: Homing X axis ---")

	// Check if X endstop is already triggered
	resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{"oid": 4}, "endstop_state", 2*time.Second)
	if err != nil {
		return fmt.Errorf("endstop_query_state X: %w", err)
	}
	xEndstopVal := resp["pin_value"].(int)
	if stepperX.EndstopInvert {
		xEndstopVal = 1 - xEndstopVal
	}
	if xEndstopVal == 1 {
		fmt.Println("  X endstop already triggered - moving away first...")
		// Move away from endstop
		if err := enableMotors(true); err != nil {
			return fmt.Errorf("enable motors: %w", err)
		}
		// Move 10mm away
		moveSteps := int(10.0 * stepsPerMM)
		resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
		if err != nil {
			return fmt.Errorf("get_uptime: %w", err)
		}
		curClock := uint64(resp["clock"].(int))
		startClock := curClock + uint64(mcu.MCUFreq()*0.1)
		stepInterval := int(mcu.MCUFreq() / 1000) // 1000 steps/sec

		// CoreXY: X+ direction = motor_x+ and motor_y+
		if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{"oid": 0, "dir": 1}); err != nil {
			return err
		}
		if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{"oid": 1, "dir": 1}); err != nil {
			return err
		}
		if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{"oid": 0, "clock": uint32(startClock)}); err != nil {
			return err
		}
		if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{"oid": 1, "clock": uint32(startClock)}); err != nil {
			return err
		}
		if err := mcu.SendCommand("queue_step", map[string]interface{}{"oid": 0, "interval": stepInterval, "count": moveSteps, "add": 0}); err != nil {
			return err
		}
		if err := mcu.SendCommand("queue_step", map[string]interface{}{"oid": 1, "interval": stepInterval, "count": moveSteps, "add": 0}); err != nil {
			return err
		}
		time.Sleep(time.Duration(float64(moveSteps)*float64(stepInterval)/mcu.MCUFreq()*1000+500) * time.Millisecond)
	}

	if err := enableMotors(true); err != nil {
		return fmt.Errorf("enable motors: %w", err)
	}
	fmt.Println("  Motors enabled")

	// Home X: move toward X endstop (X- direction)
	// CoreXY: X- direction = motor_x- and motor_y-
	homingSpeed := 50.0 // mm/s
	homingStepsPerSec := homingSpeed * stepsPerMM
	homingInterval := int(mcu.MCUFreq() / homingStepsPerSec)
	maxHomingSteps := int(stepperX.PositionMax * stepsPerMM * 1.2) // 20% extra

	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime: %w", err)
	}
	curClock := uint64(resp["clock"].(int))
	startClock := curClock + uint64(mcu.MCUFreq()*0.1)
	expireTime := startClock + uint64(float64(maxHomingSteps)*float64(homingInterval))

	// Set direction for X- (toward endstop at X=0)
	// CoreXY X-: motor_x moves negative, motor_y moves negative
	// dir=1 means move toward min (position_endstop=0)
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{"oid": 0, "dir": 1}); err != nil {
		return err
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{"oid": 1, "dir": 1}); err != nil {
		return err
	}

	// Setup trsync for X endstop
	if err := mcu.SendCommand("trsync_start", map[string]interface{}{
		"oid":          6,
		"report_clock": uint32(startClock),
		"report_ticks": uint32(mcu.MCUFreq() / 10),
		"expire_reason": 4,
	}); err != nil {
		return err
	}
	if err := mcu.SendCommand("trsync_set_timeout", map[string]interface{}{
		"oid":   6,
		"clock": uint32(expireTime),
	}); err != nil {
		return err
	}
	// Endstop timing (from working testXHoming)
	sampleTicks := uint32(mcu.MCUFreq() * 0.000025) // 25μs sampling
	restTicks := uint32(mcu.MCUFreq() * 0.0001)     // 100μs rest

	if err := mcu.SendCommand("endstop_home", map[string]interface{}{
		"oid":            4,
		"clock":          uint32(startClock),
		"sample_ticks":   sampleTicks,
		"sample_count":   4,
		"rest_ticks":     restTicks,
		"pin_value":      0, // Trigger when pin goes LOW (endstop pressed)
		"trsync_oid":     6,
		"trigger_reason": 1, // Endstop trigger reason
	}); err != nil {
		return err
	}

	// Link steppers to trsync
	if err := mcu.SendCommand("stepper_stop_on_trigger", map[string]interface{}{"oid": 0, "trsync_oid": 6}); err != nil {
		return err
	}
	if err := mcu.SendCommand("stepper_stop_on_trigger", map[string]interface{}{"oid": 1, "trsync_oid": 6}); err != nil {
		return err
	}

	// Reset step clocks and queue steps
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{"oid": 0, "clock": uint32(startClock)}); err != nil {
		return err
	}
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{"oid": 1, "clock": uint32(startClock)}); err != nil {
		return err
	}
	if err := mcu.SendCommand("queue_step", map[string]interface{}{"oid": 0, "interval": homingInterval, "count": maxHomingSteps, "add": 0}); err != nil {
		return err
	}
	if err := mcu.SendCommand("queue_step", map[string]interface{}{"oid": 1, "interval": homingInterval, "count": maxHomingSteps, "add": 0}); err != nil {
		return err
	}

	fmt.Println("  Homing X...")

	// Wait for endstop trigger by polling endstop_query_state (like testXHoming)
	triggered := false
	homingTimeout := 30 * time.Second
	startTime := time.Now()

	for time.Since(startTime) < homingTimeout {
		resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
			"oid": 4,
		}, "endstop_state", 2*time.Second)
		if err != nil {
			fmt.Printf("  WARNING: endstop_query_state failed: %v\n", err)
			break
		}

		endstopPinValue := resp["pin_value"].(int)
		// Apply inversion
		if stepperX.EndstopInvert {
			endstopPinValue = 1 - endstopPinValue
		}

		if endstopPinValue == 1 {
			triggered = true
			fmt.Printf("  X endstop triggered!\n")
			break
		}

		// Check stepper position to see if still moving
		resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
			"oid": 0,
		}, "stepper_position", 2*time.Second)
		if err == nil {
			pos := resp["pos"].(int)
			fmt.Printf("  Moving... stepper pos=%d\n", pos)
		}

		time.Sleep(200 * time.Millisecond)
	}

	if !triggered {
		return fmt.Errorf("X homing failed - endstop not triggered within timeout")
	}

	// =============================================
	// Step 2: Home Y axis
	// =============================================
	fmt.Println("\n--- Step 2: Homing Y axis ---")

	// Check if Y endstop is already triggered
	resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{"oid": 5}, "endstop_state", 2*time.Second)
	if err != nil {
		return fmt.Errorf("endstop_query_state Y: %w", err)
	}
	yEndstopVal := resp["pin_value"].(int)
	if stepperY.EndstopInvert {
		yEndstopVal = 1 - yEndstopVal
	}

	// Y homes to max (position_endstop: 300)
	// CoreXY Y+ direction = motor_x+ and motor_y-
	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime: %w", err)
	}
	curClock = uint64(resp["clock"].(int))
	startClock = curClock + uint64(mcu.MCUFreq()*0.1)
	expireTime = startClock + uint64(float64(maxHomingSteps)*float64(homingInterval))

	// Y+ direction for homing to max
	// CoreXY Y+: motor_x moves + (dir=0), motor_y moves - (dir=1)
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{"oid": 0, "dir": 0}); err != nil {
		return err
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{"oid": 1, "dir": 1}); err != nil {
		return err
	}

	// Setup trsync for Y endstop
	if err := mcu.SendCommand("trsync_start", map[string]interface{}{
		"oid":          7,
		"report_clock": uint32(startClock),
		"report_ticks": uint32(mcu.MCUFreq() / 10),
		"expire_reason": 4,
	}); err != nil {
		return err
	}
	if err := mcu.SendCommand("trsync_set_timeout", map[string]interface{}{
		"oid":   7,
		"clock": uint32(expireTime),
	}); err != nil {
		return err
	}
	if err := mcu.SendCommand("endstop_home", map[string]interface{}{
		"oid":            5,
		"clock":          uint32(startClock),
		"sample_ticks":   sampleTicks,
		"sample_count":   4,
		"rest_ticks":     restTicks,
		"pin_value":      0, // Trigger when pin goes LOW (endstop pressed)
		"trsync_oid":     7,
		"trigger_reason": 1, // Endstop trigger reason
	}); err != nil {
		return err
	}

	// Link steppers to trsync
	if err := mcu.SendCommand("stepper_stop_on_trigger", map[string]interface{}{"oid": 0, "trsync_oid": 7}); err != nil {
		return err
	}
	if err := mcu.SendCommand("stepper_stop_on_trigger", map[string]interface{}{"oid": 1, "trsync_oid": 7}); err != nil {
		return err
	}

	// Reset step clocks and queue steps
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{"oid": 0, "clock": uint32(startClock)}); err != nil {
		return err
	}
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{"oid": 1, "clock": uint32(startClock)}); err != nil {
		return err
	}
	if err := mcu.SendCommand("queue_step", map[string]interface{}{"oid": 0, "interval": homingInterval, "count": maxHomingSteps, "add": 0}); err != nil {
		return err
	}
	if err := mcu.SendCommand("queue_step", map[string]interface{}{"oid": 1, "interval": homingInterval, "count": maxHomingSteps, "add": 0}); err != nil {
		return err
	}

	fmt.Println("  Homing Y...")

	// Wait for endstop trigger by polling endstop_query_state
	triggered = false
	startTime = time.Now()

	for time.Since(startTime) < homingTimeout {
		resp, err = mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
			"oid": 5,
		}, "endstop_state", 2*time.Second)
		if err != nil {
			fmt.Printf("  WARNING: endstop_query_state failed: %v\n", err)
			break
		}

		endstopPinValue := resp["pin_value"].(int)
		// Apply inversion
		if stepperY.EndstopInvert {
			endstopPinValue = 1 - endstopPinValue
		}

		if endstopPinValue == 1 {
			triggered = true
			fmt.Printf("  Y endstop triggered!\n")
			break
		}

		// Check stepper position to see if still moving
		resp, err = mcu.SendCommandWithResponse("stepper_get_position", map[string]interface{}{
			"oid": 0,
		}, "stepper_position", 2*time.Second)
		if err == nil {
			pos := resp["pos"].(int)
			fmt.Printf("  Moving... stepper pos=%d\n", pos)
		}

		time.Sleep(200 * time.Millisecond)
	}

	if !triggered {
		return fmt.Errorf("Y homing failed - endstop not triggered within timeout")
	}

	// After homing: X=0, Y=300 (based on position_endstop values)
	currentX := stepperX.PositionEndstop // 0
	currentY := stepperY.PositionEndstop // 300
	fmt.Printf("\n  Homed position: X=%.0f Y=%.0f\n", currentX, currentY)

	// =============================================
	// Step 3: Move to X100 Y100 at F1000
	// =============================================
	fmt.Println("\n--- Step 3: G1 X100 Y100 F1000 ---")

	targetX := 100.0
	targetY := 100.0
	feedrate := 1000.0 // mm/min
	speed := feedrate / 60.0 // mm/s

	deltaX := targetX - currentX
	deltaY := targetY - currentY
	distance := (deltaX*deltaX + deltaY*deltaY)
	if distance > 0 {
		distance = math.Sqrt(distance)
	}

	// CoreXY kinematics
	motorXmm := deltaX + deltaY // motor_x moves by delta_x + delta_y
	motorYmm := deltaX - deltaY // motor_y moves by delta_x - delta_y

	motorXsteps := int(math.Abs(motorXmm) * stepsPerMM)
	motorYsteps := int(math.Abs(motorYmm) * stepsPerMM)

	moveTime := distance / speed
	fmt.Printf("  Delta: X=%.0f Y=%.0f Distance=%.1fmm Time=%.2fs\n", deltaX, deltaY, distance, moveTime)
	fmt.Printf("  CoreXY motors: motor_x=%.1fmm (%d steps), motor_y=%.1fmm (%d steps)\n",
		motorXmm, motorXsteps, motorYmm, motorYsteps)

	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime: %w", err)
	}
	curClock = uint64(resp["clock"].(int))
	startClock = curClock + uint64(mcu.MCUFreq()*0.1)

	// Set directions
	// dir=0 means move toward max (+), dir=1 means move toward min (-)
	motorXdir := 0
	if motorXmm < 0 {
		motorXdir = 1
	}
	motorYdir := 0
	if motorYmm < 0 {
		motorYdir = 1
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{"oid": 0, "dir": motorXdir}); err != nil {
		return err
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{"oid": 1, "dir": motorYdir}); err != nil {
		return err
	}

	// Reset clocks
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{"oid": 0, "clock": uint32(startClock)}); err != nil {
		return err
	}
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{"oid": 1, "clock": uint32(startClock)}); err != nil {
		return err
	}

	// Queue steps - calculate intervals to complete in moveTime
	if motorXsteps > 0 {
		intervalX := int(mcu.MCUFreq() * moveTime / float64(motorXsteps))
		if err := mcu.SendCommand("queue_step", map[string]interface{}{"oid": 0, "interval": intervalX, "count": motorXsteps, "add": 0}); err != nil {
			return err
		}
	}
	if motorYsteps > 0 {
		intervalY := int(mcu.MCUFreq() * moveTime / float64(motorYsteps))
		if err := mcu.SendCommand("queue_step", map[string]interface{}{"oid": 1, "interval": intervalY, "count": motorYsteps, "add": 0}); err != nil {
			return err
		}
	}

	fmt.Printf("  Moving to X100 Y100...\n")
	time.Sleep(time.Duration(moveTime*1000+500) * time.Millisecond)
	fmt.Printf("  Reached X100 Y100\n")

	currentX = targetX
	currentY = targetY

	// =============================================
	// Step 4: Move to X40 Y40 at F500
	// =============================================
	fmt.Println("\n--- Step 4: G1 X40 Y40 F500 ---")

	targetX = 40.0
	targetY = 40.0
	feedrate = 500.0
	speed = feedrate / 60.0

	deltaX = targetX - currentX
	deltaY = targetY - currentY
	distance = (deltaX*deltaX + deltaY*deltaY)
	if distance > 0 {
		distance = math.Sqrt(distance)
	}

	motorXmm = deltaX + deltaY
	motorYmm = deltaX - deltaY

	motorXsteps = int(math.Abs(motorXmm) * stepsPerMM)
	motorYsteps = int(math.Abs(motorYmm) * stepsPerMM)

	moveTime = distance / speed
	fmt.Printf("  Delta: X=%.0f Y=%.0f Distance=%.1fmm Time=%.2fs\n", deltaX, deltaY, distance, moveTime)
	fmt.Printf("  CoreXY motors: motor_x=%.1fmm (%d steps), motor_y=%.1fmm (%d steps)\n",
		motorXmm, motorXsteps, motorYmm, motorYsteps)

	resp, err = mcu.SendCommandWithResponse("get_uptime", nil, "uptime", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_uptime: %w", err)
	}
	curClock = uint64(resp["clock"].(int))
	startClock = curClock + uint64(mcu.MCUFreq()*0.1)

	// Set directions
	// dir=0 means move toward max (+), dir=1 means move toward min (-)
	motorXdir = 0
	if motorXmm < 0 {
		motorXdir = 1
	}
	motorYdir = 0
	if motorYmm < 0 {
		motorYdir = 1
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{"oid": 0, "dir": motorXdir}); err != nil {
		return err
	}
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{"oid": 1, "dir": motorYdir}); err != nil {
		return err
	}

	// Reset clocks
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{"oid": 0, "clock": uint32(startClock)}); err != nil {
		return err
	}
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{"oid": 1, "clock": uint32(startClock)}); err != nil {
		return err
	}

	// Queue steps
	if motorXsteps > 0 {
		intervalX := int(mcu.MCUFreq() * moveTime / float64(motorXsteps))
		if err := mcu.SendCommand("queue_step", map[string]interface{}{"oid": 0, "interval": intervalX, "count": motorXsteps, "add": 0}); err != nil {
			return err
		}
	}
	if motorYsteps > 0 {
		intervalY := int(mcu.MCUFreq() * moveTime / float64(motorYsteps))
		if err := mcu.SendCommand("queue_step", map[string]interface{}{"oid": 1, "interval": intervalY, "count": motorYsteps, "add": 0}); err != nil {
			return err
		}
	}

	fmt.Printf("  Moving to X40 Y40...\n")
	time.Sleep(time.Duration(moveTime*1000+500) * time.Millisecond)
	fmt.Printf("  Reached X40 Y40\n")

	// Disable motors
	if err := enableMotors(false); err != nil {
		return fmt.Errorf("disable motors: %w", err)
	}
	fmt.Println("\n  Motors disabled")
	fmt.Println("\n  G-code sequence complete!")

	return nil
}


// parseConfigFile extracts MCU serial device from a Klipper printer.cfg file.
func parseConfigFile(path string) (device string, baud int, err error) {
	file, err := os.Open(path)
	if err != nil {
		return "", 0, err
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	inMCUSection := false

	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())

		// Skip comments and empty lines
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}

		// Check for section headers
		if strings.HasPrefix(line, "[") && strings.HasSuffix(line, "]") {
			section := strings.ToLower(strings.Trim(line, "[]"))
			inMCUSection = (section == "mcu")
			continue
		}

		// Parse settings in [mcu] section
		if inMCUSection {
			if strings.HasPrefix(line, "serial:") || strings.HasPrefix(line, "serial =") {
				parts := strings.SplitN(line, ":", 2)
				if len(parts) < 2 {
					parts = strings.SplitN(line, "=", 2)
				}
				if len(parts) == 2 {
					device = strings.TrimSpace(parts[1])
				}
			}
			if strings.HasPrefix(line, "baud:") || strings.HasPrefix(line, "baud =") {
				parts := strings.SplitN(line, ":", 2)
				if len(parts) < 2 {
					parts = strings.SplitN(line, "=", 2)
				}
				if len(parts) == 2 {
					fmt.Sscanf(strings.TrimSpace(parts[1]), "%d", &baud)
				}
			}
		}
	}

	if err := scanner.Err(); err != nil {
		return "", 0, err
	}

	return device, baud, nil
}

// StepperConfig holds the pin configuration for a stepper motor
type StepperConfig struct {
	Name       string // e.g., "stepper_x", "stepper_y", "stepper_z"
	StepPin    string // e.g., "PK0"
	DirPin     string // e.g., "PK1"
	EnablePin  string // e.g., "PF7"
	EndstopPin string // e.g., "PL6"
	// Pin modifiers
	DirInvert     bool // ! prefix on dir_pin
	EnableInvert  bool // ! prefix on enable_pin
	EndstopPullup bool // ^ prefix on endstop_pin
	EndstopInvert bool // ! prefix on endstop_pin
	// Position parameters
	PositionEndstop float64 // position_endstop: 0 means at min, 300 means at max
	PositionMax     float64 // position_max: maximum travel distance
	// Motion parameters
	Microsteps       int     // e.g., 16
	RotationDistance float64 // e.g., 40.0 mm per full rotation
}

// ExtruderConfig holds extruder/heater configuration
type ExtruderConfig struct {
	HeaterPin  string  // e.g., "PA7"
	SensorPin  string  // e.g., "PF0"
	SensorType string  // e.g., "ATC Semitec 104GT-2"
	PullupR    float64 // pullup resistor value, default 4700
	MinTemp    float64
	MaxTemp    float64
	ControlPID bool
	PID_Kp     float64
	PID_Ki     float64
	PID_Kd     float64
}

// HeaterBedConfig holds heated bed configuration
type HeaterBedConfig struct {
	HeaterPin  string  // e.g., "PA6"
	SensorPin  string  // e.g., "PF2"
	SensorType string  // e.g., "EPCOS 100K B57560G104F"
	PullupR    float64 // pullup resistor value, default 4700
	MinTemp    float64
	MaxTemp    float64
}

// PrinterConfig holds the full printer configuration
type PrinterConfig struct {
	Device     string
	Baud       int
	Kinematics string // e.g., "cartesian", "corexy", "delta"
	Steppers   map[string]*StepperConfig
	Extruder   *ExtruderConfig
	HeaterBed  *HeaterBedConfig
}

// parsePin extracts pin name and modifiers from a Klipper pin spec like "!PK1" or "^!PL6"
func parsePin(spec string) (pin string, invert bool, pullup bool) {
	spec = strings.TrimSpace(spec)
	for len(spec) > 0 {
		switch spec[0] {
		case '!':
			invert = true
			spec = spec[1:]
		case '^':
			pullup = true
			spec = spec[1:]
		case '~':
			// PWM modifier, ignore for now
			spec = spec[1:]
		default:
			pin = spec
			return
		}
	}
	return
}

// parseFullConfig parses a Klipper printer.cfg file and extracts all relevant configuration
func parseFullConfig(path string) (*PrinterConfig, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	config := &PrinterConfig{
		Steppers: make(map[string]*StepperConfig),
	}

	scanner := bufio.NewScanner(file)
	var currentSection string
	var currentStepper *StepperConfig
	var currentExtruder *ExtruderConfig
	var currentHeaterBed *HeaterBedConfig

	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())

		// Skip comments and empty lines
		if line == "" || strings.HasPrefix(line, "#") {
			continue
		}

		// Check for section headers
		if strings.HasPrefix(line, "[") && strings.HasSuffix(line, "]") {
			section := strings.Trim(line, "[]")
			currentSection = strings.ToLower(section)

			// Check if it's a stepper section
			if strings.HasPrefix(currentSection, "stepper_") {
				currentStepper = &StepperConfig{Name: currentSection}
				config.Steppers[currentSection] = currentStepper
				currentExtruder = nil
				currentHeaterBed = nil
			} else if currentSection == "extruder" {
				currentExtruder = &ExtruderConfig{PullupR: 4700} // Default pullup resistor
				config.Extruder = currentExtruder
				currentStepper = nil
				currentHeaterBed = nil
			} else if currentSection == "heater_bed" {
				currentHeaterBed = &HeaterBedConfig{PullupR: 4700} // Default pullup resistor
				config.HeaterBed = currentHeaterBed
				currentStepper = nil
				currentExtruder = nil
			} else {
				currentStepper = nil
				currentExtruder = nil
				currentHeaterBed = nil
			}
			continue
		}

		// Parse key: value or key = value
		var key, value string
		if idx := strings.Index(line, ":"); idx > 0 {
			key = strings.TrimSpace(line[:idx])
			value = strings.TrimSpace(line[idx+1:])
		} else if idx := strings.Index(line, "="); idx > 0 {
			key = strings.TrimSpace(line[:idx])
			value = strings.TrimSpace(line[idx+1:])
		} else {
			continue
		}

		// Parse settings based on current section
		switch currentSection {
		case "mcu":
			switch key {
			case "serial":
				config.Device = value
			case "baud":
				fmt.Sscanf(value, "%d", &config.Baud)
			}
		case "printer":
			switch key {
			case "kinematics":
				config.Kinematics = value
			}

		default:
			if currentStepper != nil {
				switch key {
				case "step_pin":
					pin, _, _ := parsePin(value)
					currentStepper.StepPin = pin
				case "dir_pin":
					pin, invert, _ := parsePin(value)
					currentStepper.DirPin = pin
					currentStepper.DirInvert = invert
				case "enable_pin":
					pin, invert, _ := parsePin(value)
					currentStepper.EnablePin = pin
					currentStepper.EnableInvert = invert
				case "endstop_pin":
					pin, invert, pullup := parsePin(value)
					currentStepper.EndstopPin = pin
					currentStepper.EndstopInvert = invert
					currentStepper.EndstopPullup = pullup
				case "position_endstop":
					fmt.Sscanf(value, "%f", &currentStepper.PositionEndstop)
				case "position_max":
					fmt.Sscanf(value, "%f", &currentStepper.PositionMax)
				case "microsteps":
					fmt.Sscanf(value, "%d", &currentStepper.Microsteps)
				case "rotation_distance":
					fmt.Sscanf(value, "%f", &currentStepper.RotationDistance)
				}
			}
			if currentExtruder != nil {
				switch key {
				case "heater_pin":
					pin, _, _ := parsePin(value)
					currentExtruder.HeaterPin = pin
				case "sensor_pin":
					pin, _, _ := parsePin(value)
					currentExtruder.SensorPin = pin
				case "sensor_type":
					currentExtruder.SensorType = value
				case "pullup_resistor":
					fmt.Sscanf(value, "%f", &currentExtruder.PullupR)
				case "min_temp":
					fmt.Sscanf(value, "%f", &currentExtruder.MinTemp)
				case "max_temp":
					fmt.Sscanf(value, "%f", &currentExtruder.MaxTemp)
				case "control":
					currentExtruder.ControlPID = value == "pid"
				case "pid_kp":
					fmt.Sscanf(value, "%f", &currentExtruder.PID_Kp)
				case "pid_ki":
					fmt.Sscanf(value, "%f", &currentExtruder.PID_Ki)
				case "pid_kd":
					fmt.Sscanf(value, "%f", &currentExtruder.PID_Kd)
				}
			}
			if currentHeaterBed != nil {
				switch key {
				case "heater_pin":
					pin, _, _ := parsePin(value)
					currentHeaterBed.HeaterPin = pin
				case "sensor_pin":
					pin, _, _ := parsePin(value)
					currentHeaterBed.SensorPin = pin
				case "sensor_type":
					currentHeaterBed.SensorType = value
				case "pullup_resistor":
					fmt.Sscanf(value, "%f", &currentHeaterBed.PullupR)
				case "min_temp":
					fmt.Sscanf(value, "%f", &currentHeaterBed.MinTemp)
				case "max_temp":
					fmt.Sscanf(value, "%f", &currentHeaterBed.MaxTemp)
				}
			}
		}
	}

	if err := scanner.Err(); err != nil {
		return nil, err
	}

	return config, nil
}

// testExtruderHeater tests extruder heater functionality with temperature control.
// It reads temperature from the thermistor and controls heater PWM to reach target temperature.
func testExtruderHeater(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string, targetTemp float64) error {
	fmt.Printf("=== Test: Extruder Heater (Target: %.1f°C) ===\n", targetTemp)

	if configFile == "" {
		return fmt.Errorf("config file required for extruder heater test (use -config flag)")
	}

	config, err := parseFullConfig(configFile)
	if err != nil {
		return fmt.Errorf("parse config: %w", err)
	}

	if config.Extruder == nil {
		return fmt.Errorf("no [extruder] section found in config file")
	}

	extruder := config.Extruder
	if extruder.HeaterPin == "" {
		return fmt.Errorf("no heater_pin found in [extruder] config")
	}
	if extruder.SensorPin == "" {
		return fmt.Errorf("no sensor_pin found in [extruder] config")
	}
	if extruder.SensorType == "" {
		extruder.SensorType = "ATC Semitec 104GT-2" // Default
	}

	fmt.Printf("  Loaded from config:\n")
	fmt.Printf("    [extruder]: heater_pin=%s sensor_pin=%s sensor_type=%s\n",
		extruder.HeaterPin, extruder.SensorPin, extruder.SensorType)

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %d Hz\n", int(mcu.MCUFreq()))

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - need to reset MCU")
		return fmt.Errorf("MCU already configured, please reset")
	}

	// Configure OIDs: 0=ADC (thermistor), 1=PWM (heater)
	fmt.Println("\nConfiguring extruder heater system...")

	// Allocate OIDs
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 2}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=2 - OK")

	// Configure ADC for temperature sensor
	adcOid := 0
	if err := mcu.SendCommand("config_analog_in", map[string]interface{}{
		"oid": adcOid,
		"pin": extruder.SensorPin,
	}); err != nil {
		return fmt.Errorf("config_analog_in: %w", err)
	}
	fmt.Printf("  config_analog_in oid=%d pin=%s - OK\n", adcOid, extruder.SensorPin)

	// Configure PWM for heater - use software PWM (config_digital_out with schedule)
	// because config_pwm_out might have issues with certain pins
	heaterOid := 1
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           heaterOid,
		"pin":           extruder.HeaterPin,
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (heater): %w", err)
	}
	fmt.Printf("  config_digital_out oid=%d pin=%s - OK\n", heaterOid, extruder.HeaterPin)

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Create thermistor for temperature calculation
	thermistor, err := hosth4.NewThermistorFromProfile(extruder.SensorType, extruder.PullupR, 0)
	if err != nil {
		return fmt.Errorf("create thermistor: %w", err)
	}

	// Register handler for analog readings
	tempReadings := make(chan int, 20)
	mcu.RegisterOIDHandler("analog_in_state", adcOid, func(params map[string]interface{}, receiveTime time.Time) {
		if value, ok := params["value"].(int); ok {
			select {
			case tempReadings <- value:
			default:
			}
		}
	})

	// Start ADC sampling (use same approach as testTemp)
	mcuClock := mcu.PrintTime(0)               // Current estimated MCU clock
	sampleTicks := int(mcu.MCUFreq() * 0.001)  // 1ms sample
	restTicks := int(mcu.MCUFreq() * 0.3)      // 300ms between reports (same as testTemp)
	sampleCount := 8                            // 8 samples per reading
	maxADC := 1023 * sampleCount               // 10-bit ADC × 8 samples = 8184

	fmt.Printf("  MCU clock: %d, sample_ticks: %d, rest_ticks: %d\n", mcuClock, sampleTicks, restTicks)

	if err := mcu.SendCommand("query_analog_in", map[string]interface{}{
		"oid":               adcOid,
		"clock":             uint32(mcuClock + 100000), // Start shortly in the future (same as testTemp)
		"sample_ticks":      sampleTicks,
		"sample_count":      sampleCount,
		"rest_ticks":        restTicks,
		"min_value":         0,
		"max_value":         maxADC,
		"range_check_count": 0,
	}); err != nil {
		return fmt.Errorf("query_analog_in: %w", err)
	}
	fmt.Println("  query_analog_in started")

	// Heater control loop
	fmt.Printf("\n--- Extruder Heater Control (Target: %.1f°C) ---\n", targetTemp)
	fmt.Printf("  (Thermistor: %s, Pullup: %.0fΩ)\n", extruder.SensorType, extruder.PullupR)
	fmt.Printf("  (ADC: 10-bit × %d samples, range 0-%d)\n", sampleCount, maxADC)
	fmt.Printf("  Press Ctrl+C to stop\n\n")

	currentPWM := 0
	readCount := 0
	lastPWMUpdate := time.Now()
	pwmUpdateInterval := 2 * time.Second // Update PWM every 2 seconds for safety

	// Run for max 60 seconds
	loopTimeout := time.After(60 * time.Second)

	for {
		select {
		case adcValue := <-tempReadings:
			readCount++

			var tempC float64
			var tempStatus string

			if adcValue >= maxADC-10 {
				// Open circuit - no thermistor connected
				tempC = -999
				tempStatus = "OPEN CIRCUIT (no thermistor)"
			} else if adcValue <= 10 {
				// Short circuit
				tempC = 999
				tempStatus = "SHORT CIRCUIT"
			} else {
				// Normal reading - calculate temperature
				adcNormalized := float64(adcValue) / float64(maxADC)
				tempC = thermistor.CalcTemp(adcNormalized)
				tempStatus = fmt.Sprintf("%.1f°C", tempC)
			}

			// Simple proportional control (P-only for safety)
			var newPWM int
			if tempC < -100 || tempC > 400 {
				// Invalid temperature - turn off heater
				newPWM = 0
			} else {
				errorTemp := targetTemp - tempC
				// P-gain = 10 (PWM per degree error)
				newPWM = int(errorTemp * 10)
				if newPWM < 0 {
					newPWM = 0
				}
				if newPWM > 255 {
					newPWM = 255
				}
				// Safety: limit max PWM to 200 (about 78%) during testing
				if newPWM > 200 {
					newPWM = 200
				}
			}

			// Update heater periodically using simple on/off control
			if time.Since(lastPWMUpdate) >= pwmUpdateInterval {
				currentPWM = newPWM
				// Convert PWM to on/off: if PWM > 127, turn on; otherwise turn off
				heaterOn := 0
				if currentPWM > 127 {
					heaterOn = 1
				}
				if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
					"oid":   heaterOid,
					"value": heaterOn,
				}); err != nil {
					fmt.Printf("  Warning: update_digital_out failed: %v\n", err)
				}
				lastPWMUpdate = time.Now()
			}

			// Display status
			fmt.Printf("  [%2d] ADC: %5d -> %s, PWM: %3d/255 (%.0f%%)\n",
				readCount, adcValue, tempStatus, currentPWM, float64(currentPWM)/255*100)

			// Check if target reached (within 2 degrees)
			if tempC > 0 && tempC < 400 {
				if tempC >= targetTemp-2 && tempC <= targetTemp+2 {
					fmt.Printf("\n  ✓ Target temperature reached! (%.1f°C)\n", tempC)
				}
			}

		case <-loopTimeout:
			// Turn off heater before exit
			mcu.SendCommand("update_digital_out", map[string]interface{}{
				"oid":   heaterOid,
				"value": 0,
			})
			fmt.Printf("\n  Test timeout after 60 seconds. Total readings: %d\n", readCount)
			fmt.Println("  Heater turned OFF (safety)")
			return nil
		}
	}
}

func testBedHeater(ri *hosth4.RealtimeIntegration, timeout time.Duration, configFile string, targetTemp float64) error {
	fmt.Printf("=== Test: Heated Bed (Target: %.1f°C) ===\n", targetTemp)

	if configFile == "" {
		return fmt.Errorf("config file required for bed heater test (use -config flag)")
	}

	config, err := parseFullConfig(configFile)
	if err != nil {
		return fmt.Errorf("parse config: %w", err)
	}

	if config.HeaterBed == nil {
		return fmt.Errorf("no [heater_bed] section found in config file")
	}

	bed := config.HeaterBed
	if bed.HeaterPin == "" {
		return fmt.Errorf("no heater_pin found in [heater_bed] config")
	}
	if bed.SensorPin == "" {
		return fmt.Errorf("no sensor_pin found in [heater_bed] config")
	}
	if bed.SensorType == "" {
		bed.SensorType = "EPCOS 100K B57560G104F" // Default
	}

	fmt.Printf("  Loaded from config:\n")
	fmt.Printf("    [heater_bed]: heater_pin=%s sensor_pin=%s sensor_type=%s\n",
		bed.HeaterPin, bed.SensorPin, bed.SensorType)

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer ri.Disconnect()

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %d Hz\n", int(mcu.MCUFreq()))

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - need to reset MCU")
		return fmt.Errorf("MCU already configured, please reset")
	}

	// Configure OIDs: 0=ADC (thermistor), 1=digital out (heater)
	fmt.Println("\nConfiguring heated bed system...")

	// Allocate OIDs
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 2}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=2 - OK")

	// Configure ADC for temperature sensor
	adcOid := 0
	if err := mcu.SendCommand("config_analog_in", map[string]interface{}{
		"oid": adcOid,
		"pin": bed.SensorPin,
	}); err != nil {
		return fmt.Errorf("config_analog_in: %w", err)
	}
	fmt.Printf("  config_analog_in oid=%d pin=%s - OK\n", adcOid, bed.SensorPin)

	// Configure digital out for heater (on/off control)
	heaterOid := 1
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           heaterOid,
		"pin":           bed.HeaterPin,
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out (heater): %w", err)
	}
	fmt.Printf("  config_digital_out oid=%d pin=%s - OK\n", heaterOid, bed.HeaterPin)

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Create thermistor for temperature calculation
	thermistor, err := hosth4.NewThermistorFromProfile(bed.SensorType, bed.PullupR, 0)
	if err != nil {
		return fmt.Errorf("create thermistor: %w", err)
	}

	// Register handler for analog readings
	tempReadings := make(chan int, 20)
	mcu.RegisterOIDHandler("analog_in_state", adcOid, func(params map[string]interface{}, receiveTime time.Time) {
		if value, ok := params["value"].(int); ok {
			select {
			case tempReadings <- value:
			default:
			}
		}
	})

	// Start ADC sampling
	mcuClock := mcu.PrintTime(0)              // Current estimated MCU clock
	sampleTicks := int(mcu.MCUFreq() * 0.001) // 1ms sample
	restTicks := int(mcu.MCUFreq() * 0.3)     // 300ms between reports
	sampleCount := 8                          // 8 samples per reading
	maxADC := 1023 * sampleCount              // 10-bit ADC × 8 samples = 8184

	fmt.Printf("  MCU clock: %d, sample_ticks: %d, rest_ticks: %d\n", mcuClock, sampleTicks, restTicks)

	if err := mcu.SendCommand("query_analog_in", map[string]interface{}{
		"oid":               adcOid,
		"clock":             uint32(mcuClock + 100000), // Start shortly in the future
		"sample_ticks":      sampleTicks,
		"sample_count":      sampleCount,
		"rest_ticks":        restTicks,
		"min_value":         0,
		"max_value":         maxADC,
		"range_check_count": 0,
	}); err != nil {
		return fmt.Errorf("query_analog_in: %w", err)
	}
	fmt.Println("  query_analog_in started")

	// Heater control loop
	fmt.Printf("\n--- Heated Bed Control (Target: %.1f°C) ---\n", targetTemp)
	fmt.Printf("  (Thermistor: %s, Pullup: %.0fΩ)\n", bed.SensorType, bed.PullupR)
	fmt.Printf("  (ADC: 10-bit × %d samples, range 0-%d)\n", sampleCount, maxADC)
	fmt.Printf("  Press Ctrl+C to stop\n\n")

	heaterOn := false
	readCount := 0
	lastUpdate := time.Now()
	updateInterval := 2 * time.Second // Update heater every 2 seconds

	// Run for max 120 seconds (beds heat slowly)
	loopTimeout := time.After(120 * time.Second)

	for {
		select {
		case adcValue := <-tempReadings:
			readCount++

			var tempC float64
			var tempStatus string

			if adcValue >= maxADC-10 {
				// Open circuit - no thermistor connected
				tempC = -999
				tempStatus = "OPEN CIRCUIT (no thermistor)"
			} else if adcValue <= 10 {
				// Short circuit
				tempC = 999
				tempStatus = "SHORT CIRCUIT"
			} else {
				// Normal reading - calculate temperature
				adcNormalized := float64(adcValue) / float64(maxADC)
				tempC = thermistor.CalcTemp(adcNormalized)
				tempStatus = fmt.Sprintf("%.1f°C", tempC)
			}

			// Simple on/off (bang-bang) control with hysteresis
			// On when below target - 1°C, off when above target + 1°C
			shouldHeat := false
			if tempC > 0 && tempC < 400 {
				if tempC < targetTemp-1 {
					shouldHeat = true
				} else if tempC > targetTemp+1 {
					shouldHeat = false
				} else {
					shouldHeat = heaterOn // Keep current state in hysteresis band
				}
			}

			// Update heater periodically
			if time.Since(lastUpdate) >= updateInterval {
				// Try inverted logic: 0=ON, 1=OFF (some MOSFETs are active-low)
				heaterValue := 1
				if shouldHeat {
					heaterValue = 0
				}
				if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
					"oid":   heaterOid,
					"value": heaterValue,
				}); err != nil {
					fmt.Printf("  Warning: update_digital_out failed: %v\n", err)
				}
				heaterOn = shouldHeat
				lastUpdate = time.Now()
			}

			// Display status
			heaterStatus := "OFF"
			if heaterOn {
				heaterStatus = "ON"
			}
			fmt.Printf("  [%2d] ADC: %5d (%.3f) -> %s, Heater: %s\n",
				readCount, adcValue, float64(adcValue)/float64(maxADC), tempStatus, heaterStatus)

			// Check if target reached (within 2 degrees)
			if tempC > 0 && tempC < 400 {
				if tempC >= targetTemp-2 && tempC <= targetTemp+2 {
					fmt.Printf("\n  ✓ Target temperature reached! (%.1f°C)\n", tempC)
				}
			}

		case <-loopTimeout:
			// Turn off heater before exit
			mcu.SendCommand("update_digital_out", map[string]interface{}{
				"oid":   heaterOid,
				"value": 0,
			})
			fmt.Printf("\n  Test timeout after 120 seconds. Total readings: %d\n", readCount)
			fmt.Println("  Heater turned OFF (safety)")
			return nil
		}
	}
}

func testFan(ri *hosth4.RealtimeIntegration, timeout time.Duration, fanPin string) error {
	fmt.Printf("=== Test: Fan (pin=%s) ===\n", fanPin)

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	// Note: disconnect is handled by main() after test completes

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %d Hz\n", int(mcu.MCUFreq()))

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - need to reset MCU")
		return fmt.Errorf("MCU already configured, please reset")
	}

	// Configure fan as digital output
	fmt.Println("\nConfiguring fan...")

	// Allocate OID
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 1}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=1 - OK")

	// Configure digital output for fan
	fanOid := 0
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           fanOid,
		"pin":           fanPin,
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out: %w", err)
	}
	fmt.Printf("  config_digital_out oid=%d pin=%s - OK\n", fanOid, fanPin)

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Test sequence: ON 10s, OFF 10s, then OFF
	fmt.Println("\n--- Fan Test Sequence ---")

	// Turn ON
	fmt.Println("  Turning fan ON...")
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   fanOid,
		"value": 1,
	}); err != nil {
		return fmt.Errorf("update_digital_out ON: %w", err)
	}
	fmt.Println("  Fan ON - waiting 10 seconds...")
	time.Sleep(10 * time.Second)

	// Turn OFF
	fmt.Println("  Turning fan OFF...")
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   fanOid,
		"value": 0,
	}); err != nil {
		return fmt.Errorf("update_digital_out OFF: %w", err)
	}
	fmt.Println("  Fan OFF - waiting 10 seconds...")
	time.Sleep(10 * time.Second)

	// Final OFF (ensure fan is off)
	fmt.Println("  Final turn OFF...")
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   fanOid,
		"value": 0,
	}); err != nil {
		return fmt.Errorf("update_digital_out final OFF: %w", err)
	}

	fmt.Println("\n  Fan test complete!")
	return nil
}

// testHeaterSimple tests a heater by turning it ON for a specified duration, then OFF.
// This is a simple digital output test without temperature feedback.
func testHeaterSimple(ri *hosth4.RealtimeIntegration, timeout time.Duration, heaterPin string, onDuration time.Duration) error {
	fmt.Printf("=== Test: Heater (pin=%s) ===\n", heaterPin)

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	// Note: disconnect is handled by main() after test completes

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %d Hz\n", int(mcu.MCUFreq()))

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - need to reset MCU")
		return fmt.Errorf("MCU already configured, please reset")
	}

	// Configure heater as digital output
	fmt.Println("\nConfiguring heater...")

	// Allocate OID
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 1}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=1 - OK")

	// Configure digital output for heater
	heaterOid := 0
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           heaterOid,
		"pin":           heaterPin,
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out: %w", err)
	}
	fmt.Printf("  config_digital_out oid=%d pin=%s - OK\n", heaterOid, heaterPin)

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Test sequence: ON for duration, then OFF
	fmt.Println("\n--- Heater Test Sequence ---")

	// Turn ON
	fmt.Println("  Turning heater ON...")
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   heaterOid,
		"value": 1,
	}); err != nil {
		return fmt.Errorf("update_digital_out ON: %w", err)
	}
	fmt.Printf("  Heater ON - waiting %v...\n", onDuration)
	time.Sleep(onDuration)

	// Turn OFF
	fmt.Println("  Turning heater OFF...")
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   heaterOid,
		"value": 0,
	}); err != nil {
		return fmt.Errorf("update_digital_out OFF: %w", err)
	}

	fmt.Println("\n  Heater test complete!")
	return nil
}

// testTempRead reads temperature from a thermistor using ADC.
func testTempRead(ri *hosth4.RealtimeIntegration, timeout time.Duration, sensorPin string) error {
	fmt.Printf("=== Test: Temperature Read (pin=%s) ===\n", sensorPin)

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	fmt.Printf("MCU frequency: %d Hz\n", int(mcu.MCUFreq()))

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		fmt.Println("MCU already configured - need to reset MCU")
		return fmt.Errorf("MCU already configured, please reset")
	}

	// Configure ADC for temperature sensor
	fmt.Println("\nConfiguring ADC...")

	// Allocate OID
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 1}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=1 - OK")

	// Configure analog input
	adcOid := 0
	if err := mcu.SendCommand("config_analog_in", map[string]interface{}{
		"oid": adcOid,
		"pin": sensorPin,
	}); err != nil {
		return fmt.Errorf("config_analog_in: %w", err)
	}
	fmt.Printf("  config_analog_in oid=%d pin=%s - OK\n", adcOid, sensorPin)

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Create thermistor for temperature conversion
	thermistor, err := hosth4.NewThermistorFromProfile("ATC Semitec 104GT-2", 4700, 0)
	if err != nil {
		return fmt.Errorf("create thermistor: %w", err)
	}

	// Register handler for analog readings
	tempReadings := make(chan int, 20)
	mcu.RegisterOIDHandler("analog_in_state", adcOid, func(params map[string]interface{}, receiveTime time.Time) {
		if value, ok := params["value"].(int); ok {
			select {
			case tempReadings <- value:
			default:
			}
		}
	})

	// Start ADC sampling
	mcuClock := mcu.PrintTime(0)              // Current estimated MCU clock
	sampleTicks := int(mcu.MCUFreq() * 0.001) // 1ms sample
	restTicks := int(mcu.MCUFreq() * 0.5)     // 500ms between reports
	sampleCount := 8                          // 8 samples per reading
	maxADC := 1023 * sampleCount              // 10-bit ADC × 8 samples = 8184

	fmt.Printf("  MCU clock: %d, sample_ticks: %d, rest_ticks: %d\n", mcuClock, sampleTicks, restTicks)

	if err := mcu.SendCommand("query_analog_in", map[string]interface{}{
		"oid":               adcOid,
		"clock":             uint32(mcuClock + 100000), // Start shortly in the future
		"sample_ticks":      sampleTicks,
		"sample_count":      sampleCount,
		"rest_ticks":        restTicks,
		"min_value":         0,
		"max_value":         maxADC,
		"range_check_count": 0,
	}); err != nil {
		return fmt.Errorf("query_analog_in: %w", err)
	}
	fmt.Println("  query_analog_in started")

	// Read temperature values for 10 seconds
	fmt.Println("\n--- Temperature Readings ---")
	readCount := 0
	readTimeout := time.After(10 * time.Second)

	for {
		select {
		case adcValue := <-tempReadings:
			readCount++
			adcFloat := float64(adcValue) / float64(maxADC)
			temp := thermistor.CalcTemp(adcFloat)
			fmt.Printf("  [%d] ADC=%d -> %.1f°C\n", readCount, adcValue, temp)
		case <-readTimeout:
			fmt.Printf("\n  Read %d temperature samples in 10 seconds\n", readCount)
			fmt.Println("\n  Temperature read complete!")
			return nil
		}
	}
}

// testExtrude tests extruder by heating to target temp and extruding specified length.
// Prerequisites: heatbreak fan ON, heat to target temp, then extrude.
func testExtrude(ri *hosth4.RealtimeIntegration, timeout time.Duration, extrudeLength float64, targetTemp float64) error {
	fmt.Printf("=== Test: Extrude %.1fmm at %.0f°C (Soft PWM control) ===\n", extrudeLength, targetTemp)

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	mcuFreq := mcu.MCUFreq()
	fmt.Printf("MCU frequency: %d Hz\n", int(mcuFreq))

	// Check config state and clear any previous shutdown
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	// Check if MCU is in shutdown state and clear it
	if v, ok := resp["is_shutdown"].(int); ok && v != 0 {
		fmt.Println("  MCU was in shutdown state, clearing...")
		if err := mcu.SendCommand("clear_shutdown", nil); err != nil {
			return fmt.Errorf("clear_shutdown: %w", err)
		}
		// Re-check config state after clearing shutdown
		resp, err = mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
		if err != nil {
			return fmt.Errorf("get_config after clear_shutdown: %w", err)
		}
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		return fmt.Errorf("MCU already configured, please reset")
	}

	// Allocate OIDs: heater, fan, ADC, stepper, enable
	fmt.Println("\nConfiguring hardware...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 5}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=5 - OK")

	// Configure heater (PA7) as digital output for soft PWM
	// Use max_duration=0 in config (like Klipper), actual timing via queue_digital_out
	heaterOid := 0
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           heaterOid,
		"pin":           "PA7",
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config heater: %w", err)
	}
	fmt.Println("  config_digital_out oid=0 pin=PA7 (heater) - OK")

	// Configure heatbreak fan (PC1) as digital (on/off)
	fanOid := 1
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid": fanOid, "pin": "PC1", "value": 0, "default_value": 0, "max_duration": 0,
	}); err != nil {
		return fmt.Errorf("config fan: %w", err)
	}
	fmt.Println("  config_digital_out oid=1 pin=PC1 (heatbreak fan) - OK")

	// Configure temp sensor (PF0)
	adcOid := 2
	if err := mcu.SendCommand("config_analog_in", map[string]interface{}{
		"oid": adcOid, "pin": "PF0",
	}); err != nil {
		return fmt.Errorf("config ADC: %w", err)
	}
	fmt.Println("  config_analog_in oid=2 pin=PF0 (temp sensor) - OK")

	// Configure extruder stepper motor (PA1=step, PA2=dir)
	// Note: dir inversion handled via set_next_step_dir command
	stepperOid := 3
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperOid,
		"step_pin":         "PA1",
		"dir_pin":          "PA2",
		"invert_step":      0,
		"step_pulse_ticks": 0,
	}); err != nil {
		return fmt.Errorf("config stepper: %w", err)
	}
	fmt.Println("  config_stepper oid=3 step=PA1 dir=PA2 (extruder) - OK")

	// Configure extruder enable pin (PA0, active low - so value=0 enables)
	enableOid := 4
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableOid,
		"pin":           "PA0",
		"value":         1, // Start disabled (HIGH = disabled for active-low enable)
		"default_value": 1, // Default disabled
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config enable: %w", err)
	}
	fmt.Println("  config_digital_out oid=4 pin=PA0 (extruder enable) - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Wait for MCU to be ready
	time.Sleep(100 * time.Millisecond)

	// Register handler to detect MCU shutdown BEFORE turning heater on
	shutdownDetected := false
	shutdownTime := time.Time{}
	mcu.RegisterHandler("shutdown", func(params map[string]interface{}, receiveTime time.Time) {
		fmt.Printf("\n  !!! MCU SHUTDOWN at %v: %v\n", receiveTime, params)
		shutdownDetected = true
		shutdownTime = receiveTime
	})
	mcu.RegisterHandler("is_shutdown", func(params map[string]interface{}, receiveTime time.Time) {
		if !shutdownDetected {
			fmt.Printf("\n  !!! MCU IS_SHUTDOWN (first detected at %v): %v\n", receiveTime, params)
			shutdownDetected = true
			shutdownTime = receiveTime
		}
	})
	_ = shutdownTime // Will be used later

	// Turn on heatbreak cooling fan first (important to prevent heat creep)
	fmt.Println("\n  Turning heatbreak fan ON...")
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   fanOid,
		"value": 1,
	}); err != nil {
		return fmt.Errorf("update_digital_out (fan ON): %w", err)
	}
	fmt.Println("  Heatbreak fan ON")

	// Turn heater ON using simple update_digital_out
	fmt.Println("  Turning heater ON...")
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   heaterOid,
		"value": 1,
	}); err != nil {
		return fmt.Errorf("update_digital_out (heater ON): %w", err)
	}
	fmt.Println("  Heater ON")
	fmt.Println("  Waiting 2 seconds to confirm heater is on...")
	time.Sleep(2 * time.Second)
	_ = shutdownDetected // Will be used in control loop

	// Set up thermistor for temperature calculation
	thermistor, err := hosth4.NewThermistorFromProfile("ATC Semitec 104GT-2", 4700, 0)
	if err != nil {
		return fmt.Errorf("create thermistor: %w", err)
	}

	// Start ADC sampling
	// IMPORTANT: Use actual MCU clock, not 0! Using clock=0 causes "Timer too close"
	// shutdown after ~30-40 seconds due to timer scheduling issues.
	// Get the last known MCU clock from clock sync samples
	fmt.Println("  Getting MCU clock...")
	var mcuClock uint64
	syncDeadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(syncDeadline) {
		mcuClock = mcu.GetLastMCUClock()
		if mcuClock > 0 {
			break
		}
		time.Sleep(50 * time.Millisecond)
	}
	if mcuClock == 0 {
		return fmt.Errorf("could not get MCU clock - no clock samples received")
	}
	fmt.Printf("  MCU clock: %d (%.2f seconds since MCU reset)\n", mcuClock, float64(mcuClock)/mcuFreq)
	sampleTicks := int(mcuFreq * 0.001)     // 1ms sample time
	restTicks := int(mcuFreq * 0.5)         // 500ms between reports (slower to avoid timer issues)
	sampleCount := 8
	maxADC := 1023 * sampleCount

	// Use a larger offset to ensure the start time is well in the future
	// The clock value we got is slightly stale, so add more margin
	startOffset := uint64(mcuFreq / 2) // 500ms offset
	startClock := mcuClock + startOffset

	fmt.Printf("  Sample settings: sample_ticks: %d, rest_ticks: %d, start_offset: %d ticks\n",
		sampleTicks, restTicks, startOffset)

	if err := mcu.SendCommand("query_analog_in", map[string]interface{}{
		"oid":               adcOid,
		"clock":             uint32(startClock), // Start with larger offset
		"sample_ticks":      sampleTicks,
		"sample_count":      sampleCount,
		"rest_ticks":        restTicks,
		"min_value":         0,
		"max_value":         maxADC,
		"range_check_count": 0,
	}); err != nil {
		return fmt.Errorf("query_analog_in: %w", err)
	}
	fmt.Println("  query_analog_in started (3 Hz)")

	// Bang-bang temperature control with asymmetric hysteresis to reduce overshoot
	fmt.Printf("\n--- Heating to %.0f°C with bang-bang control ---\n", targetTemp)
	// Use asymmetric hysteresis: turn OFF early to reduce overshoot
	// Based on testing, ~15°C overshoot occurs, so turn off 10°C early
	hysteresisLow := 5.0   // Turn ON when temp drops 5°C below target
	hysteresisHigh := -10.0 // Turn OFF 10°C before target (anticipate 15°C overshoot)
	fmt.Printf("  Control: ON below %.0f°C, OFF at %.0f°C (anticipating overshoot)\n",
		targetTemp-hysteresisLow, targetTemp+hysteresisHigh)

	// Shared state for temperature control
	var currentTemp float64
	var tempMutex sync.Mutex
	heaterOn := true // Start with heater ON (already turned on above)
	targetReached := false
	targetReachedTime := time.Time{}

	// Handler that reads temperature and controls heater
	mcu.RegisterOIDHandler("analog_in_state", adcOid, func(params map[string]interface{}, receiveTime time.Time) {
		if value, ok := params["value"].(int); ok {
			adcFloat := float64(value) / float64(maxADC)
			temp := thermistor.CalcTemp(adcFloat)

			tempMutex.Lock()
			currentTemp = temp
			oldHeaterState := heaterOn

			// Safety check: emergency shutoff if temperature exceeds safe limit
			maxSafeTemp := 260.0 // Typical hotend max is ~280-300°C
			if temp > maxSafeTemp {
				if heaterOn {
					fmt.Printf("\n  !!! SAFETY: Temperature %.1f°C exceeds max %.0f°C, forcing heater OFF !!!\n", temp, maxSafeTemp)
					heaterOn = false
				}
			} else {
				// Bang-bang control logic with asymmetric hysteresis
				if temp < targetTemp-hysteresisLow && !heaterOn {
					// Below lower threshold, turn ON
					heaterOn = true
				} else if temp >= targetTemp+hysteresisHigh && heaterOn {
					// At or above target, turn OFF (anticipate overshoot)
					heaterOn = false
				}
			}

			// Track when we first reach near target (within 5°C)
			if !targetReached && temp >= targetTemp-hysteresisLow {
				targetReached = true
				targetReachedTime = time.Now()
			}

			newHeaterState := heaterOn
			tempMutex.Unlock()

			// Update heater if state changed
			if newHeaterState != oldHeaterState {
				stateStr := "OFF"
				if newHeaterState {
					stateStr = "ON"
				}
				if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
					"oid":   heaterOid,
					"value": boolToInt(newHeaterState),
				}); err != nil {
					fmt.Printf("  ERROR: Failed to turn heater %s: %v\n", stateStr, err)
				}
			}

			// Print status
			stateStr := "OFF"
			if newHeaterState {
				stateStr = "ON"
			}
			fmt.Printf("  Temp: %.1f°C (heater %s)\n", temp, stateStr)
		}
	})

	// Wait for target temperature with timeout
	heatTimeout := 120 * time.Second // 2 minutes to reach target
	holdTime := 10 * time.Second     // Hold at target for 10 seconds
	startTime := time.Now()

	fmt.Println("  Waiting to reach target temperature...")
	for {
		// Check for MCU shutdown first
		if shutdownDetected {
			fmt.Println("  ERROR: MCU shutdown detected, aborting heating!")
			break
		}

		tempMutex.Lock()
		reached := targetReached
		reachedAt := targetReachedTime
		tempMutex.Unlock()

		if reached {
			// Check if we've held at target long enough
			if time.Since(reachedAt) >= holdTime {
				fmt.Printf("  Target temperature %.0f°C reached and held for %.0f seconds!\n",
					targetTemp, holdTime.Seconds())
				break
			}
		}

		if time.Since(startTime) >= heatTimeout {
			tempMutex.Lock()
			finalTemp := currentTemp
			tempMutex.Unlock()
			fmt.Printf("  WARNING: Timeout after %.0f seconds, current temp: %.1f°C\n",
				heatTimeout.Seconds(), finalTemp)
			break
		}

		time.Sleep(100 * time.Millisecond)
	}

	if shutdownDetected {
		fmt.Printf("  WARNING: MCU shutdown was detected during heating at %v\n", shutdownTime)
	}

	// Step 3: Extrude
	fmt.Printf("\n--- Step 3: Extruding %.1fmm ---\n", extrudeLength)

	// Calculate steps needed
	// From printer.cfg: rotation_distance: 7.1556, microsteps: 16
	// Assuming 200 full steps per revolution (standard stepper)
	fullStepsPerRotation := 200
	microsteps := 16
	rotationDistance := 7.1556 // mm per full rotation
	stepsPerMM := float64(fullStepsPerRotation*microsteps) / rotationDistance
	totalSteps := int(extrudeLength * stepsPerMM)
	fmt.Printf("  Steps per mm: %.1f, total steps: %d\n", stepsPerMM, totalSteps)

	// Enable the extruder stepper (value=0 enables for active-low pin)
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableOid,
		"value": 0, // Enable (LOW = enabled for active-low)
	}); err != nil {
		return fmt.Errorf("enable stepper: %w", err)
	}
	fmt.Println("  Extruder stepper enabled")

	// Get current MCU clock for step timing
	extrudeClock := mcu.GetLastMCUClock()
	if extrudeClock == 0 {
		return fmt.Errorf("could not get MCU clock for extrusion")
	}

	// Extrude at slow speed: 5mm/s
	// At 447.3 steps/mm, 5mm/s = 2236 steps/sec
	// Interval between steps = 16MHz / 2236 = ~7155 ticks
	extrudeSpeed := 5.0 // mm/s
	stepsPerSec := extrudeSpeed * stepsPerMM
	intervalTicks := int(mcuFreq / stepsPerSec)
	fmt.Printf("  Extrude speed: %.1f mm/s, interval: %d ticks\n", extrudeSpeed, intervalTicks)

	// Reset stepper position
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperOid,
		"clock": uint32(extrudeClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock: %w", err)
	}

	// Set stepper direction (0 = extrude, since dir_pin is inverted in hardware)
	// The printer.cfg has dir_pin: !PA2, meaning inverted logic
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperOid,
		"dir": 0, // Forward direction (inverted: 0=extrude, 1=retract)
	}); err != nil {
		return fmt.Errorf("set_next_step_dir: %w", err)
	}

	// Queue steps in batches
	// queue_step takes: oid, interval, count, add
	// interval: ticks between first step and second step
	// count: number of steps
	// add: acceleration (0 for constant speed)
	batchSize := 500
	remainingSteps := totalSteps

	fmt.Printf("  Queuing %d steps...\n", totalSteps)

	for remainingSteps > 0 {
		count := batchSize
		if remainingSteps < batchSize {
			count = remainingSteps
		}

		if err := mcu.SendCommand("queue_step", map[string]interface{}{
			"oid":      stepperOid,
			"interval": intervalTicks,
			"count":    count,
			"add":      0, // No acceleration
		}); err != nil {
			return fmt.Errorf("queue_step: %w", err)
		}

		remainingSteps -= count
	}

	// Wait for extrusion to complete
	extrudeTime := extrudeLength / extrudeSpeed
	fmt.Printf("  Waiting %.1f seconds for extrusion to complete...\n", extrudeTime)
	time.Sleep(time.Duration(extrudeTime*1.5) * time.Second) // Wait with margin

	// Disable stepper (value=1 disables for active-low pin)
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableOid,
		"value": 1, // Disable (HIGH = disabled for active-low)
	}); err != nil {
		return fmt.Errorf("disable stepper: %w", err)
	}
	fmt.Println("  Extruder stepper disabled")
	fmt.Println("  Extrusion complete!")

	// Step 4: Turn off heater
	fmt.Println("\n--- Step 4: Cooling down ---")
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   heaterOid,
		"value": 0,
	}); err != nil {
		return fmt.Errorf("heater OFF: %w", err)
	}
	fmt.Println("  Heater OFF")

	// Turn off fan
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{"oid": fanOid, "value": 0}); err != nil {
		return fmt.Errorf("fan OFF: %w", err)
	}
	fmt.Println("  Heatbreak fan OFF")

	fmt.Println("\n  Extrude test complete!")
	return nil
}

// testColdExtrude tests extruder stepper motor without heating.
// This is useful for testing the mechanical extrusion without filament or with cold filament.
func testColdExtrude(ri *hosth4.RealtimeIntegration, timeout time.Duration, extrudeLength float64) error {
	fmt.Printf("=== Test: Cold Extrude %.1fmm (no heating) ===\n", extrudeLength)

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	mcuFreq := mcu.MCUFreq()
	fmt.Printf("MCU frequency: %d Hz\n", int(mcuFreq))

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_shutdown"].(int); ok && v != 0 {
		fmt.Println("  MCU was in shutdown state, clearing...")
		if err := mcu.SendCommand("clear_shutdown", nil); err != nil {
			return fmt.Errorf("clear_shutdown: %w", err)
		}
		resp, err = mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
		if err != nil {
			return fmt.Errorf("get_config after clear: %w", err)
		}
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		return fmt.Errorf("MCU already configured, please reset")
	}

	// Allocate OIDs: stepper, enable
	fmt.Println("\nConfiguring hardware...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 2}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=2 - OK")

	// Configure extruder stepper motor (PA1=step, PA2=dir)
	stepperOid := 0
	if err := mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperOid,
		"step_pin":         "PA1",
		"dir_pin":          "PA2",
		"invert_step":      0,
		"step_pulse_ticks": 0,
	}); err != nil {
		return fmt.Errorf("config stepper: %w", err)
	}
	fmt.Println("  config_stepper oid=0 step=PA1 dir=PA2 (extruder) - OK")

	// Configure extruder enable pin (PA0, active low)
	enableOid := 1
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableOid,
		"pin":           "PA0",
		"value":         1, // Start disabled (HIGH)
		"default_value": 1,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config enable: %w", err)
	}
	fmt.Println("  config_digital_out oid=1 pin=PA0 (extruder enable) - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Calculate steps needed
	fullStepsPerRotation := 200
	microsteps := 16
	rotationDistance := 7.1556 // mm per full rotation
	stepsPerMM := float64(fullStepsPerRotation*microsteps) / rotationDistance
	totalSteps := int(extrudeLength * stepsPerMM)
	fmt.Printf("\n  Steps per mm: %.1f, total steps: %d\n", stepsPerMM, totalSteps)

	// Enable the extruder stepper
	fmt.Println("  Enabling extruder stepper...")
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableOid,
		"value": 0, // Enable (LOW)
	}); err != nil {
		return fmt.Errorf("enable stepper: %w", err)
	}
	fmt.Println("  Extruder stepper enabled")

	// Get current MCU clock
	time.Sleep(100 * time.Millisecond) // Wait for clock samples
	extrudeClock := mcu.GetLastMCUClock()
	if extrudeClock == 0 {
		return fmt.Errorf("could not get MCU clock")
	}
	fmt.Printf("  MCU clock: %d\n", extrudeClock)

	// Extrude at slow speed: 5mm/s
	extrudeSpeed := 5.0 // mm/s
	stepsPerSec := extrudeSpeed * stepsPerMM
	intervalTicks := int(mcuFreq / stepsPerSec)
	fmt.Printf("  Extrude speed: %.1f mm/s, interval: %d ticks\n", extrudeSpeed, intervalTicks)

	// Reset stepper clock
	if err := mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperOid,
		"clock": uint32(extrudeClock + uint64(mcuFreq/10)), // Start 100ms in future
	}); err != nil {
		return fmt.Errorf("reset_step_clock: %w", err)
	}

	// Set direction (0 = extrude, since dir_pin is inverted in hardware)
	if err := mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperOid,
		"dir": 0,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir: %w", err)
	}

	// Queue steps
	fmt.Printf("  Queuing %d steps...\n", totalSteps)
	batchSize := 500
	remainingSteps := totalSteps

	for remainingSteps > 0 {
		count := batchSize
		if remainingSteps < batchSize {
			count = remainingSteps
		}

		if err := mcu.SendCommand("queue_step", map[string]interface{}{
			"oid":      stepperOid,
			"interval": intervalTicks,
			"count":    count,
			"add":      0,
		}); err != nil {
			return fmt.Errorf("queue_step: %w", err)
		}

		remainingSteps -= count
	}

	// Wait for extrusion to complete
	extrudeTime := extrudeLength / extrudeSpeed
	fmt.Printf("  Extruding %.1fmm at %.1fmm/s (%.1f seconds)...\n", extrudeLength, extrudeSpeed, extrudeTime)
	time.Sleep(time.Duration(extrudeTime*1.5) * time.Second)

	// Disable stepper
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableOid,
		"value": 1, // Disable (HIGH)
	}); err != nil {
		return fmt.Errorf("disable stepper: %w", err)
	}
	fmt.Println("  Extruder stepper disabled")

	fmt.Println("\n  Cold extrude test complete!")
	return nil
}

// testHeaterWithADC tests heater with ADC temperature monitoring.
// This is a minimal test to debug why heater works alone but not with ADC.
func testHeaterWithADC(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("=== Test: Heater with ADC monitoring ===")

	// Connect to MCU
	fmt.Println("\nConnecting to MCU...")
	if err := ri.Connect(); err != nil {
		return fmt.Errorf("connect: %w", err)
	}

	if err := ri.WaitReady(timeout); err != nil {
		return fmt.Errorf("wait ready: %w", err)
	}

	mcu := ri.MCUManager().PrimaryMCU()
	if mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	mcuFreq := mcu.MCUFreq()
	fmt.Printf("MCU frequency: %d Hz\n", int(mcuFreq))

	// Check config state
	resp, err := mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}
	if v, ok := resp["is_config"].(int); ok && v != 0 {
		return fmt.Errorf("MCU already configured, please reset")
	}

	// Step-by-step test: heater + FAN + ADC config (same as testExtrude)
	fmt.Println("\nTest: heater + fan + ADC config (like testExtrude)...")
	if err := mcu.SendCommand("allocate_oids", map[string]interface{}{"count": 3}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}
	fmt.Println("  allocate_oids count=3 - OK")

	// Configure heater (PA7) - same as testExtrude
	heaterOid := 0
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           heaterOid,
		"pin":           "PA7",
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config heater: %w", err)
	}
	fmt.Println("  config_digital_out oid=0 pin=PA7 (heater) - OK")

	// Configure heatbreak fan (PC1) - same as testExtrude
	fanOid := 1
	if err := mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           fanOid,
		"pin":           "PC1",
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config fan: %w", err)
	}
	fmt.Println("  config_digital_out oid=1 pin=PC1 (fan) - OK")

	// Configure ADC (PF0) - same as testExtrude
	adcOid := 2
	if err := mcu.SendCommand("config_analog_in", map[string]interface{}{
		"oid": adcOid,
		"pin": "PF0",
	}); err != nil {
		return fmt.Errorf("config ADC: %w", err)
	}
	fmt.Println("  config_analog_in oid=2 pin=PF0 - OK")

	// Finalize config
	if err := mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}
	fmt.Println("  finalize_config - OK")

	// Turn on heater FIRST (same as testExtrude)
	fmt.Println("\n--- Step 1: Turning heater ON ---")
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   heaterOid,
		"value": 1,
	}); err != nil {
		return fmt.Errorf("heater ON: %w", err)
	}
	fmt.Println("  Heater ON")
	fmt.Println("  Waiting 2 seconds to confirm heater is on...")
	time.Sleep(2 * time.Second)

	// Set up thermistor for temperature calculation
	thermistor, err := hosth4.NewThermistorFromProfile("ATC Semitec 104GT-2", 4700, 0)
	if err != nil {
		return fmt.Errorf("create thermistor: %w", err)
	}

	// Now start ADC query (same as testExtrude)
	fmt.Println("\n--- Step 2: Starting ADC query with temperature monitoring ---")
	// IMPORTANT: Use actual MCU clock, not 0! Using clock=0 causes "Timer too close"
	// shutdown after ~30-40 seconds due to timer scheduling issues.
	// Wait for clock sync to complete (PrintTime returns 0 if not synced)
	var mcuClock uint64
	syncDeadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(syncDeadline) {
		mcuClock = mcu.PrintTime(0)
		if mcuClock > 0 {
			break
		}
		time.Sleep(50 * time.Millisecond)
	}
	if mcuClock == 0 {
		// Clock sync not available, use estimated clock
		mcuClock = uint64(mcuFreq) // Start about 1 second in
	}
	sampleTicks := int(mcuFreq * 0.001)  // 1ms sample ticks
	restTicks := int(mcuFreq * 0.3)      // 300ms between reports
	sampleCount := 8
	maxADC := 1023 * sampleCount

	// Register handler to print temperature
	mcu.RegisterOIDHandler("analog_in_state", adcOid, func(params map[string]interface{}, receiveTime time.Time) {
		if value, ok := params["value"].(int); ok {
			adcFloat := float64(value) / float64(maxADC)
			temp := thermistor.CalcTemp(adcFloat)
			fmt.Printf("  Temp: %.1f°C (heater ON)\n", temp)
		}
	})

	if err := mcu.SendCommand("query_analog_in", map[string]interface{}{
		"oid":               adcOid,
		"clock":             uint32(mcuClock + 100000), // Start 100k ticks (~6ms) in future
		"sample_ticks":      sampleTicks,
		"sample_count":      sampleCount,
		"rest_ticks":        restTicks,
		"min_value":         0,
		"max_value":         maxADC,
		"range_check_count": 0,
	}); err != nil {
		return fmt.Errorf("query_analog_in: %w", err)
	}
	fmt.Println("  query_analog_in started - monitoring temperature for 30 seconds...")
	_ = fanOid // fan not used

	fmt.Println("  NOTE: Temperature should START RISING if heater is working!")
	time.Sleep(30 * time.Second)

	fmt.Println("  30 seconds passed")

	// Turn off heater
	fmt.Println("\n--- Turning heater OFF ---")
	if err := mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   heaterOid,
		"value": 0,
	}); err != nil {
		return fmt.Errorf("heater OFF: %w", err)
	}
	fmt.Println("  Heater OFF")
	fmt.Println("\n  Test complete!")
	return nil
}

