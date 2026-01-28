// klipper-go is the Go implementation of the Klipper host software.
// It connects to MCU firmware, provides Moonraker-compatible API for Fluidd/Mainsail,
// and executes G-code commands for 3D printing.
//
// Usage:
//
//	klipper-go -config ~/printer.cfg [options]
//
// Options:
//
//	-config string    Printer configuration file (required)
//	-moonraker string Moonraker API server address (default ":7125")
//	-trace            Enable debug tracing
//	-logfile string   Log file path (default: stdout)
//
// Examples:
//
//	# Start with default Moonraker port
//	klipper-go -config ~/printer.cfg
//
//	# Start with custom Moonraker port
//	klipper-go -config ~/printer.cfg -moonraker :7126
//
//	# Start with debug tracing
//	klipper-go -config ~/printer.cfg -trace
package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"runtime"
	"strings"
	"syscall"
	"time"

	"klipper-go-migration/pkg/config"
	"klipper-go-migration/pkg/gcode"
	"klipper-go-migration/pkg/hosth4"
)

func main() {
	// Command line flags
	configFile := flag.String("config", "", "Printer configuration file (required)")
	moonrakerAddr := flag.String("moonraker", ":7125", "Moonraker API server address")
	trace := flag.Bool("trace", false, "Enable debug tracing")
	logFile := flag.String("logfile", "", "Log file path (default: stdout)")
	reset := flag.Bool("reset", false, "Reset MCU on connect (DTR signal)")
	defaultSerialQueue := runtime.GOOS != "darwin"
	serialqueue := flag.Bool("serialqueue", defaultSerialQueue, "Use chelper serialqueue transport (off by default on macOS)")

	flag.Parse()

	// Validate required flags
	if *configFile == "" {
		fmt.Fprintf(os.Stderr, "Error: -config is required\n")
		flag.Usage()
		os.Exit(1)
	}

	// Set up logging
	if *logFile != "" {
		f, err := os.OpenFile(*logFile, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
		if err != nil {
			fmt.Fprintf(os.Stderr, "Error opening log file: %v\n", err)
			os.Exit(1)
		}
		defer f.Close()
		log.SetOutput(f)
	}

	log.Println("========================================")
	log.Println("Klipper-Go Host Starting")
	log.Println("========================================")

	// Parse full config file
	printerCfg, err := config.ParsePrinterConfig(*configFile)
	if err != nil {
		log.Fatalf("Error parsing config: %v", err)
	}
	if printerCfg.Device == "" {
		log.Fatal("No serial device found in config file [mcu] section")
	}

	log.Printf("Config: %s", *configFile)
	log.Printf("Device: %s", printerCfg.Device)
	log.Printf("Baud: %d", printerCfg.Baud)
	log.Printf("Kinematics: %s", printerCfg.Kinematics)
	log.Printf("Moonraker: %s", *moonrakerAddr)

	// Log configured steppers
	for name, stepper := range printerCfg.Steppers {
		log.Printf("  %s: step=%s dir=%s enable=%s endstop=%s",
			name, stepper.StepPin, stepper.DirPin, stepper.EnablePin, stepper.EndstopPin)
	}
	if printerCfg.Extruder != nil {
		log.Printf("  extruder: heater=%s sensor=%s (%s)",
			printerCfg.Extruder.HeaterPin, printerCfg.Extruder.SensorPin, printerCfg.Extruder.SensorType)
	}
	if printerCfg.HeaterBed != nil {
		log.Printf("  heater_bed: heater=%s sensor=%s (%s)",
			printerCfg.HeaterBed.HeaterPin, printerCfg.HeaterBed.SensorPin, printerCfg.HeaterBed.SensorType)
	}

	// Create MCU configuration
	cfg := hosth4.DefaultMCUConnectionConfig()
	cfg.Device = printerCfg.Device
	cfg.BaudRate = printerCfg.Baud
	cfg.ConnectTimeout = 60 * time.Second
	cfg.ResetOnConnect = *reset
	cfg.UseSerialQueue = *serialqueue

	if *trace {
		cfg.Trace = os.Stdout
	}

	// Create realtime integration
	ri := hosth4.NewRealtimeIntegration()
	if *trace {
		ri.SetTrace(os.Stdout)
	}

	// Configure Moonraker API server
	if *moonrakerAddr != "" {
		ri.SetMoonrakerAddr(*moonrakerAddr)
	}

	// Add MCU
	if err := ri.AddMCU("mcu", cfg); err != nil {
		log.Fatalf("Error adding MCU: %v", err)
	}

	// Set up signal handler for graceful shutdown
	sigCh := make(chan os.Signal, 1)
	signal.Notify(sigCh, syscall.SIGINT, syscall.SIGTERM)
	shutdownCh := make(chan struct{})
	go func() {
		<-sigCh
		close(shutdownCh)
	}()

	// Run startup in a goroutine so Ctrl+C works even if connect/handshake blocks.
	type startupResult struct {
		executor *gcode.Executor
		err      error
	}
	startCh := make(chan startupResult, 1)
	go func() {
		// Connect to MCU
		log.Println("Connecting to MCU...")
		if err := ri.Connect(); err != nil {
			startCh <- startupResult{err: fmt.Errorf("failed to connect to MCU: %w", err)}
			return
		}

		// Wait for MCU to be ready
		log.Println("Waiting for MCU ready...")
		if err := ri.WaitReady(60 * time.Second); err != nil {
			startCh <- startupResult{err: fmt.Errorf("MCU not ready: %w", err)}
			return
		}

		// Get MCU info
		mcu := ri.MCUManager().GetMCU("mcu")
		if mcu != nil {
			log.Printf("MCU frequency: %.0f Hz", mcu.MCUFreq())
		}

		// Create G-code executor
		executor := gcode.NewExecutor(printerCfg, ri)

		// Configure hardware on MCU
		log.Println("Configuring printer hardware...")
		if err := executor.Configure(); err != nil {
			startCh <- startupResult{err: fmt.Errorf("failed to configure hardware: %w", err)}
			return
		}
		log.Println("Hardware configuration complete")

		// Set up Moonraker callbacks
		if moonraker := ri.Moonraker(); moonraker != nil {
			// G-code execution callback
			moonraker.SetGCodeCallback(func(gcodeCmd string) error {
				log.Printf("G-code received: %s", gcodeCmd)
				// Execute each line
				for _, line := range strings.Split(gcodeCmd, "\n") {
					line = strings.TrimSpace(line)
					if line == "" {
						continue
					}
					if err := executor.Execute(line); err != nil {
						log.Printf("G-code error: %v", err)
						return err
					}
				}
				return nil
			})

			// Temperature reading callback
			moonraker.SetTempCallback(func(name string) (temp, target float64) {
				return executor.GetTemperature(name)
			})

			// Heater power callback
			moonraker.SetHeaterPowerCallback(func(name string) float64 {
				return executor.GetHeaterPower(name)
			})

			// Position callback
			moonraker.SetPositionCallback(func() [4]float64 {
				return executor.GetPosition()
			})

			// Homed axes callback
			moonraker.SetHomedAxesCallback(func() string {
				return executor.GetHomedAxesString()
			})

			// Feedrate callback
			moonraker.SetFeedrateCallback(func() float64 {
				return executor.GetFeedrate()
			})

			// Coordinate mode callback
			moonraker.SetAbsoluteCallback(func() (coords, extrude bool) {
				return executor.IsAbsoluteCoords(), executor.IsAbsoluteExtrude()
			})

			log.Println("Moonraker callbacks configured")
		}

		startCh <- startupResult{executor: executor, err: nil}
	}()

	select {
	case <-shutdownCh:
		log.Printf("Received shutdown signal, exiting...")
		_ = ri.Disconnect()
		return
	case res := <-startCh:
		if res.err != nil {
			_ = ri.Disconnect()
			log.Fatalf("%v", res.err)
		}
		_ = res.executor
	}
	defer ri.Disconnect()

	log.Println("========================================")
	log.Println("Klipper-Go Host Ready")
	log.Printf("Moonraker API: http://localhost%s", *moonrakerAddr)
	log.Println("Press Ctrl+C to stop")
	log.Println("========================================")

	// Main loop - wait for shutdown signal
	<-shutdownCh

	log.Println("")
	log.Println("Shutting down...")
	log.Println("Klipper-Go Host stopped")
}
