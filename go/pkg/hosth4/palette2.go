// Palette2 - port of klippy/extras/palette2.py
//
// Palette 2 MMU support, Firmware 9.0.9 and newer supported only!
//
// Copyright (C) 2021 Clifford Roche <clifford.roche@gmail.com>
// Copyright (C) 2026 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"bufio"
	"fmt"
	"log"
	"strconv"
	"strings"
	"sync"
	"time"

	"klipper-go-migration/pkg/serial"
)

// Palette2 protocol commands
const (
	p2CmdHeartbeat      = "O99"
	p2CmdCut            = "O10 D5"
	p2CmdFilename       = "O51"
	p2CmdFilenamesDone  = "O52"
	p2CmdFirmware       = "O50"
	p2CmdPing           = "O31"
	p2CmdSmartLoadStop  = "O102 D1"
)

// Palette2 clear sequence
var p2CmdClear = []string{
	"O10 D5",
	"O10 D0 D0 D0 DFFE1",
	"O10 D1 D0 D0 DFFE1",
	"O10 D2 D0 D0 DFFE1",
	"O10 D3 D0 D0 DFFE1",
	"O10 D4 D0 D0 D0069",
}

// Timing constants
const (
	p2HeartbeatSend    = 5.0  // seconds
	p2HeartbeatTimeout = 11.0 // (HeartbeatSend * 2) + 1
	p2SetupTimeout     = 10.0
	p2SerialTimer      = 100 * time.Millisecond
	p2AutoloadTimer    = 5 * time.Second
)

// Palette2 manages connection to Palette 2 MMU
type Palette2 struct {
	// Configuration
	serialPort           string
	baud                 int
	feedrateSplice       float64
	feedrateNormal       float64
	autoLoadSpeed        int
	autoCancelVariation  *float64

	// Serial connection
	port         *serial.Port
	readBuffer   string
	readQueue    chan string
	writeQueue   chan string

	// State
	isPrinting         bool
	isSetupComplete    bool
	isSplicing         bool
	isLoading          bool
	remainingLoadLen   int
	heartbeat          time.Time
	signalDisconnect   bool

	// Omega protocol state
	omegaHeader          [9]string
	omegaHeaderCounter   int
	omegaAlgorithms      []string
	omegaAlgorithmsCounter int
	omegaSplices         [][2]string // (drive, distance)
	omegaSplicesCounter  int
	omegaPings           []PingInfo
	omegaPongs           []PingInfo
	omegaCurrentPing     string
	omegaDrives          []string
	omegaLastCommand     string

	// Timers control
	stopCh        chan struct{}
	smartLoadStop chan struct{}

	mu sync.Mutex
}

// PingInfo holds ping/pong information
type PingInfo struct {
	Number  int
	Percent float64
}

// Palette2Config holds configuration for Palette 2
type Palette2Config struct {
	SerialPort          string
	Baud                int
	FeedrateSplice      float64  // 0.0-1.0, default 0.8
	FeedrateNormal      float64  // 0.0-1.0, default 1.0
	AutoLoadSpeed       int      // mm/s, default 2
	AutoCancelVariation *float64 // 0.01-0.2, nil to disable
}

// DefaultPalette2Config returns default configuration
func DefaultPalette2Config() Palette2Config {
	return Palette2Config{
		Baud:           115200,
		FeedrateSplice: 0.8,
		FeedrateNormal: 1.0,
		AutoLoadSpeed:  2,
	}
}

// NewPalette2 creates a new Palette 2 instance
func NewPalette2(cfg Palette2Config) (*Palette2, error) {
	if cfg.SerialPort == "" {
		return nil, fmt.Errorf("palette2: serial port must be specified")
	}
	if cfg.Baud == 0 {
		cfg.Baud = 115200
	}
	if cfg.FeedrateSplice <= 0 || cfg.FeedrateSplice > 1 {
		cfg.FeedrateSplice = 0.8
	}
	if cfg.FeedrateNormal <= 0 || cfg.FeedrateNormal > 1 {
		cfg.FeedrateNormal = 1.0
	}
	if cfg.AutoLoadSpeed <= 0 {
		cfg.AutoLoadSpeed = 2
	}

	p := &Palette2{
		serialPort:          cfg.SerialPort,
		baud:                cfg.Baud,
		feedrateSplice:      cfg.FeedrateSplice,
		feedrateNormal:      cfg.FeedrateNormal,
		autoLoadSpeed:       cfg.AutoLoadSpeed,
		autoCancelVariation: cfg.AutoCancelVariation,
		readQueue:           make(chan string, 100),
		writeQueue:          make(chan string, 100),
	}

	p.reset()

	log.Printf("palette2: initialized serial=%s baud=%d", cfg.SerialPort, cfg.Baud)
	return p, nil
}

// reset clears the Palette 2 state
func (p *Palette2) reset() {
	p.isSetupComplete = false
	p.isSplicing = false
	p.isLoading = false
	p.remainingLoadLen = 0
	p.omegaAlgorithms = nil
	p.omegaAlgorithmsCounter = 0
	p.omegaSplices = nil
	p.omegaSplicesCounter = 0
	p.omegaPings = nil
	p.omegaPongs = nil
	p.omegaCurrentPing = ""
	p.omegaHeader = [9]string{}
	p.omegaHeaderCounter = 0
	p.omegaLastCommand = ""
	p.omegaDrives = nil
}

// Connect connects to the Palette 2 device
func (p *Palette2) Connect() error {
	p.mu.Lock()
	defer p.mu.Unlock()

	if p.port != nil {
		return fmt.Errorf("palette2: already connected, disconnect first")
	}

	log.Printf("palette2: connecting to %s at %d baud", p.serialPort, p.baud)

	cfg := serial.Config{
		Device:         p.serialPort,
		BaudRate:       p.baud,
		ConnectTimeout: 10 * time.Second,
		ReadTimeout:    100 * time.Millisecond,
	}

	port, err := serial.Open(cfg)
	if err != nil {
		return fmt.Errorf("palette2: unable to connect: %w", err)
	}

	p.port = port
	p.signalDisconnect = false
	p.stopCh = make(chan struct{})

	// Clear queues
	for len(p.readQueue) > 0 {
		<-p.readQueue
	}
	for len(p.writeQueue) > 0 {
		<-p.writeQueue
	}

	// Start background tasks
	go p.readLoop()
	go p.writeLoop()
	go p.heartbeatLoop()

	// Initial handshake
	p.writeQueue <- "\n"
	p.writeQueue <- p2CmdFirmware

	return nil
}

// Disconnect disconnects from the Palette 2 device
func (p *Palette2) Disconnect() {
	p.mu.Lock()
	defer p.mu.Unlock()

	log.Printf("palette2: disconnecting")

	if p.stopCh != nil {
		close(p.stopCh)
		p.stopCh = nil
	}

	if p.smartLoadStop != nil {
		close(p.smartLoadStop)
		p.smartLoadStop = nil
	}

	if p.port != nil {
		p.port.Close()
		p.port = nil
	}

	p.isPrinting = false
	p.heartbeat = time.Time{}
}

// IsConnected returns whether the device is connected
func (p *Palette2) IsConnected() bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.port != nil
}

// Clear clears the Palette 2 input and output
func (p *Palette2) Clear() error {
	p.mu.Lock()
	defer p.mu.Unlock()

	if p.port == nil {
		return fmt.Errorf("palette2: not connected")
	}

	log.Printf("palette2: clearing input and output")
	for _, cmd := range p2CmdClear {
		p.writeQueue <- cmd
	}
	return nil
}

// Cut cuts the outgoing filament
func (p *Palette2) Cut() error {
	p.mu.Lock()
	defer p.mu.Unlock()

	if p.port == nil {
		return fmt.Errorf("palette2: not connected")
	}

	log.Printf("palette2: cutting filament")
	p.writeQueue <- p2CmdCut
	return nil
}

// SendCommand sends a raw command to the Palette 2
func (p *Palette2) SendCommand(cmd string) error {
	p.mu.Lock()
	defer p.mu.Unlock()

	if p.port == nil {
		return fmt.Errorf("palette2: not connected")
	}

	p.writeQueue <- cmd
	return nil
}

// readLoop handles reading from the serial port
func (p *Palette2) readLoop() {
	reader := bufio.NewReader(p.port)

	for {
		select {
		case <-p.stopCh:
			return
		default:
		}

		// Set read deadline for non-blocking behavior
		p.port.SetReadTimeout(100 * time.Millisecond)

		line, err := reader.ReadString('\n')
		if err != nil {
			// Check if we're shutting down
			select {
			case <-p.stopCh:
				return
			default:
			}
			// Timeout is expected, continue
			continue
		}

		line = strings.TrimSpace(line)
		if line == "" {
			continue
		}

		// Check for heartbeat
		if line == p2CmdHeartbeat {
			p.mu.Lock()
			p.heartbeat = time.Now()
			p.mu.Unlock()
			continue
		}

		if strings.Contains(line, "Connection Okay") {
			p.mu.Lock()
			p.heartbeat = time.Now()
			p.mu.Unlock()
			continue
		}

		// Process Omega commands
		if len(line) > 0 && line[0] == 'O' {
			p.processP2Command(line)
		}
	}
}

// writeLoop handles writing to the serial port
func (p *Palette2) writeLoop() {
	for {
		select {
		case <-p.stopCh:
			return
		case cmd := <-p.writeQueue:
			if cmd == "" {
				continue
			}

			p.mu.Lock()
			p.omegaLastCommand = cmd
			p.mu.Unlock()

			cmd = strings.TrimSpace(cmd) + "\n"
			if p.port != nil {
				_, err := p.port.Write([]byte(cmd))
				if err != nil {
					log.Printf("palette2: write error: %v", err)
					p.signalDisconnect = true
				}
			}
		}
	}
}

// heartbeatLoop sends periodic heartbeats
func (p *Palette2) heartbeatLoop() {
	ticker := time.NewTicker(time.Duration(p2HeartbeatSend * float64(time.Second)))
	defer ticker.Stop()

	for {
		select {
		case <-p.stopCh:
			return
		case <-ticker.C:
			p.writeQueue <- p2CmdHeartbeat

			p.mu.Lock()
			hb := p.heartbeat
			printing := p.isPrinting
			setup := p.isSetupComplete
			p.mu.Unlock()

			// Check heartbeat timeout
			if !hb.IsZero() && time.Since(hb) > time.Duration(p2HeartbeatTimeout*float64(time.Second)) {
				log.Printf("palette2: heartbeat timeout")
				if !printing || setup {
					go p.Disconnect()
					return
				}
			}
		}
	}
}

// processP2Command processes an incoming Omega command
func (p *Palette2) processP2Command(line string) {
	parts := strings.Fields(line)
	if len(parts) == 0 {
		return
	}

	ocode := parts[0]
	params := parts[1:]

	// Validate parameters (must start with D or U)
	for _, param := range params {
		if len(param) == 0 || (param[0] != 'D' && param[0] != 'U') {
			log.Printf("palette2: invalid omega parameters: %s", line)
			return
		}
	}

	log.Printf("palette2: received %s", line)

	switch ocode {
	case "O20":
		p.handleO20(params)
	case "O34":
		p.handleO34(params)
	case "O40":
		p.handleO40(params)
	case "O50":
		p.handleO50(params)
	case "O53":
		p.handleO53(params)
	case "O88":
		p.handleO88(params)
	case "O97":
		p.handleO97(params)
	case "O100":
		p.handleO100(params)
	case "O102":
		p.handleO102(params)
	}
}

// handleO20 handles data request from Palette 2
func (p *Palette2) handleO20(params []string) {
	p.mu.Lock()
	defer p.mu.Unlock()

	if !p.isPrinting {
		return
	}

	if len(params) == 0 {
		return
	}

	// First print, we can ignore
	if params[0] == "D5" {
		log.Printf("palette2: first print on Palette")
		return
	}

	n, err := strconv.Atoi(params[0][1:])
	if err != nil {
		log.Printf("palette2: O20 invalid parameters")
		return
	}

	switch n {
	case 0: // Request header
		if p.omegaHeaderCounter < len(p.omegaHeader) {
			log.Printf("palette2: sending omega header %d", p.omegaHeaderCounter)
			p.writeQueue <- p.omegaHeader[p.omegaHeaderCounter]
			p.omegaHeaderCounter++
		}
	case 1: // Request splice info
		if p.omegaSplicesCounter < len(p.omegaSplices) {
			log.Printf("palette2: sending splice info %d", p.omegaSplicesCounter)
			splice := p.omegaSplices[p.omegaSplicesCounter]
			p.writeQueue <- fmt.Sprintf("O30 D%s D%s", splice[0], splice[1])
			p.omegaSplicesCounter++
		}
	case 2: // Request ping info
		log.Printf("palette2: sending current ping info %s", p.omegaCurrentPing)
		p.writeQueue <- p.omegaCurrentPing
	case 4: // Request algorithm info
		if p.omegaAlgorithmsCounter < len(p.omegaAlgorithms) {
			log.Printf("palette2: sending algorithm info %d", p.omegaAlgorithmsCounter)
			p.writeQueue <- p.omegaAlgorithms[p.omegaAlgorithmsCounter]
			p.omegaAlgorithmsCounter++
		}
	case 8: // Resend last command
		log.Printf("palette2: resending last command")
		p.writeQueue <- p.omegaLastCommand
	}
}

// handleO34 handles ping/pong from Palette 2
func (p *Palette2) handleO34(params []string) {
	p.mu.Lock()
	defer p.mu.Unlock()

	if !p.isPrinting {
		return
	}

	if len(params) < 3 {
		return
	}

	percent, err := strconv.ParseFloat(params[1][1:], 64)
	if err != nil {
		return
	}

	if params[0] == "D1" {
		// Ping
		number := len(p.omegaPings) + 1
		log.Printf("palette2: ping %d, %.0f%%", number, percent)
		p.omegaPings = append(p.omegaPings, PingInfo{Number: number, Percent: percent})
		p.checkPingVariation(percent)
	} else if params[0] == "D2" {
		// Pong
		number := len(p.omegaPongs) + 1
		log.Printf("palette2: pong %d, %.0f%%", number, percent)
		p.omegaPongs = append(p.omegaPongs, PingInfo{Number: number, Percent: percent})
	}
}

// checkPingVariation checks if ping variation is within acceptable range
func (p *Palette2) checkPingVariation(percent float64) {
	if p.autoCancelVariation == nil {
		return
	}

	pingMax := 100.0 + (*p.autoCancelVariation * 100.0)
	pingMin := 100.0 - (*p.autoCancelVariation * 100.0)

	if percent < pingMin || percent > pingMax {
		log.Printf("palette2: ping variation too high (%.0f%%), cancelling print", percent)
		// Note: In actual implementation, this would trigger CANCEL_PRINT
	}
}

// handleO40 handles resume request
func (p *Palette2) handleO40(params []string) {
	log.Printf("palette2: resume request from Palette 2")
	// Note: In actual implementation, this would trigger resume_command
}

// handleO50 handles firmware response
func (p *Palette2) handleO50(params []string) {
	if len(params) > 0 {
		fw := params[0][1:]
		log.Printf("palette2: firmware version %s detected", fw)

		if fw < "9.0.9" {
			log.Printf("palette2: WARNING - firmware version too old, need 9.0.9+")
		}
	}
}

// handleO53 handles file selection
func (p *Palette2) handleO53(params []string) {
	if len(params) > 1 && params[0] == "D1" {
		idx, err := strconv.ParseInt(params[1][1:], 16, 32)
		if err != nil {
			log.Printf("palette2: O53 invalid parameters")
			return
		}
		log.Printf("palette2: file selected index %d", idx)
		// Note: In actual implementation, this would trigger SDCARD_PRINT_FILE
	}
}

// handleO88 handles error from Palette 2
func (p *Palette2) handleO88(params []string) {
	log.Printf("palette2: error detected")
	if len(params) > 0 {
		errCode, err := strconv.ParseInt(params[0][1:], 16, 32)
		if err == nil {
			log.Printf("palette2: error code %d", errCode)
		}
	}
}

// handleO97 handles status updates
func (p *Palette2) handleO97(params []string) {
	p.mu.Lock()
	defer p.mu.Unlock()

	if len(params) == 0 {
		return
	}

	// Check for print cancelling
	if len(params) >= 2 && params[0] == "U0" && params[1] == "D2" {
		log.Printf("palette2: print cancelling")
		// Note: Trigger CANCEL_PRINT in actual implementation
		return
	}

	// Check for print cancelled
	if len(params) >= 2 && params[0] == "U0" && params[1] == "D3" {
		log.Printf("palette2: print cancelled")
		p.reset()
		return
	}

	// Check for loading offset
	if params[0] == "U39" {
		if len(params) >= 2 {
			remaining, err := strconv.Atoi(params[1][1:])
			if err == nil {
				p.remainingLoadLen = remaining
				log.Printf("palette2: loading filament remaining %d", remaining)
				if remaining >= 0 {
					log.Printf("palette2: smart load complete")
					p.isLoading = false
				}
			}
		} else {
			log.Printf("palette2: waiting for user to load filament")
			p.isLoading = true
		}
		return
	}

	// Check for feedrate changes
	if len(params) >= 2 && params[0] == "U25" {
		if params[1] == "D0" {
			log.Printf("palette2: splice starting, setting feedrate to %.0f%%", p.feedrateSplice*100)
			p.isSplicing = true
			// Note: Trigger M220 in actual implementation
		} else if params[1] == "D1" {
			log.Printf("palette2: splice done, setting feedrate to %.0f%%", p.feedrateNormal*100)
			p.isSplicing = false
			// Note: Trigger M220 in actual implementation
		}
	}
}

// handleO100 handles pause request
func (p *Palette2) handleO100(params []string) {
	p.mu.Lock()
	defer p.mu.Unlock()

	log.Printf("palette2: pause request from Palette 2")
	p.isSetupComplete = true
	// Note: Trigger pause_command in actual implementation
}

// handleO102 handles smart load
func (p *Palette2) handleO102(params []string) {
	p.mu.Lock()
	defer p.mu.Unlock()

	if p.smartLoadStop != nil {
		return // Already running
	}

	log.Printf("palette2: smart load starting")
	p.smartLoadStop = make(chan struct{})
	go p.smartLoadLoop()
}

// smartLoadLoop handles automatic filament loading
func (p *Palette2) smartLoadLoop() {
	ticker := time.NewTicker(p2AutoloadTimer)
	defer ticker.Stop()

	for {
		select {
		case <-p.stopCh:
			return
		case <-p.smartLoadStop:
			return
		case <-ticker.C:
			p.mu.Lock()
			splicing := p.isSplicing
			remaining := p.remainingLoadLen
			p.mu.Unlock()

			if !splicing && remaining < 0 {
				extrude := -remaining
				if extrude > 50 {
					extrude = remaining / 2
				}
				if extrude <= 10 {
					extrude = 1
				}
				log.Printf("palette2: smart loading %dmm filament", extrude)
				// Note: Trigger G92 E0 and G1 E%d in actual implementation
			}
		}
	}
}

// SetOmegaHeader sets an omega header value (called from G-code processing)
func (p *Palette2) SetOmegaHeader(index int, value string) {
	p.mu.Lock()
	defer p.mu.Unlock()

	if index >= 0 && index < len(p.omegaHeader) {
		p.omegaHeader[index] = value

		// Parse drives from O25
		if index == 4 && len(value) > 4 {
			drives := strings.Fields(value[4:])
			p.omegaDrives = nil
			for idx, d := range drives {
				if len(d) >= 2 && d[:2] == "D1" {
					p.omegaDrives = append(p.omegaDrives, fmt.Sprintf("U%d", 60+idx))
				} else if d != "D0" {
					p.omegaDrives = append(p.omegaDrives, d)
				}
			}
			log.Printf("palette2: omega drives: %v", p.omegaDrives)
		}
	}
}

// AddSplice adds a splice to the list
func (p *Palette2) AddSplice(drive, distance string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.omegaSplices = append(p.omegaSplices, [2]string{drive, distance})
}

// AddAlgorithm adds an algorithm to the list
func (p *Palette2) AddAlgorithm(algo string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.omegaAlgorithms = append(p.omegaAlgorithms, algo)
}

// SetCurrentPing sets the current ping command
func (p *Palette2) SetCurrentPing(ping string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.omegaCurrentPing = ping
}

// StartPrint marks the start of a print
func (p *Palette2) StartPrint() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.reset()
	p.isPrinting = true
}

// EndPrint marks the end of a print
func (p *Palette2) EndPrint() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.isPrinting = false
}

// GetStatus returns the Palette 2 status
func (p *Palette2) GetStatus() map[string]interface{} {
	p.mu.Lock()
	defer p.mu.Unlock()

	status := map[string]interface{}{
		"connected":             p.port != nil,
		"is_printing":           p.isPrinting,
		"is_splicing":           p.isSplicing,
		"is_loading":            p.isLoading,
		"remaining_load_length": p.remainingLoadLen,
		"ping":                  nil,
	}

	if len(p.omegaPings) > 0 {
		lastPing := p.omegaPings[len(p.omegaPings)-1]
		status["ping"] = map[string]interface{}{
			"number":  lastPing.Number,
			"percent": lastPing.Percent,
		}
	}

	return status
}
