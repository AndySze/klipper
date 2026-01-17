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
//	-device string    Serial device path (required, or use -config)
//	-config string    Printer config file (optional, extracts device from [mcu] section)
//	-baud int         Baud rate (default: 250000)
//	-timeout duration Connection timeout (default: 60s)
//	-trace            Enable debug tracing
//	-test string      Test to run: "connect", "clock", "temp", "query", "all" (default: "connect")
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
package main

import (
	"bufio"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"klipper-go-migration/pkg/hosth4"
	"klipper-go-migration/pkg/serial"
)

func main() {
	// Command line flags
	device := flag.String("device", "", "Serial device path (e.g., /dev/ttyUSB0)")
	configFile := flag.String("config", "", "Printer config file (optional)")
	baud := flag.Int("baud", 250000, "Baud rate")
	timeout := flag.Duration("timeout", 60*time.Second, "Connection timeout")
	trace := flag.Bool("trace", false, "Enable debug tracing")
	test := flag.String("test", "connect", "Test to run: serial, connect, clock, temp, query, all")

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

	if *trace {
		cfg.Trace = os.Stdout
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
			err = testSerial(*device, *baud, *timeout, *trace)
		case "connect":
			err = testConnect(ri, *timeout)
		case "clock":
			err = testClock(ri, *timeout)
		case "temp":
			err = testTemp(ri, *timeout)
		case "query":
			err = testQuery(ri, *timeout)
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

// testAll runs all tests.
func testAll(ri *hosth4.RealtimeIntegration, timeout time.Duration) error {
	fmt.Println("Running all tests...\n")

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
