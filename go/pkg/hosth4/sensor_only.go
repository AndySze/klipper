// Runtime support for sensor-only configs (kinematics: none).
// Used for LED, manual_stepper, and load_cell configs without standard kinematics.
//
// Copyright (C) 2026  Klipper Go Migration
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"klipper-go-migration/pkg/chelper"
	"klipper-go-migration/pkg/protocol"
)

// ledState holds the state for a single LED device.
type ledState struct {
	name       string
	ledType    string // "led", "neopixel", "dotstar", "pca9533", "pca9632"
	oid        int
	chainCount int
	colors     []ledColor // per-chain colors (or single for non-chain types)
}

// parseLEDColor parses an LED color value from config, returning 0.0 as default.
// Returns an error if the value is present but cannot be parsed.
func parseLEDColor(sec map[string]string, key string) (float64, error) {
	s := strings.TrimSpace(sec[key])
	if s == "" {
		return 0.0, nil
	}
	v, err := strconv.ParseFloat(s, 64)
	if err != nil {
		return 0, fmt.Errorf("bad LED color %s=%q: %w", key, s, err)
	}
	return v, nil
}

// sensorOnlyRuntime is a minimal runtime for configs without stepper kinematics.
type sensorOnlyRuntime struct {
	cfg     *configWrapper
	dict    *protocol.Dictionary
	formats map[string]*protocol.MessageFormat
	mcuFreq float64

	sq     *chelper.SerialQueue
	cqMain *chelper.CommandQueue
	rawOut *os.File

	leds map[string]*ledState

	// OID assignments for LED types
	oidPWM      int
	oidNeopixel int
	oidSPI      int
	oidI2C1     int
	oidI2C2     int

	// Clock tracking for LED commands
	nextClock int

	// Previous state tracking for change detection
	prevPWMColor   float64
	prevNeopixel   []byte
	prevDotstar    []byte
	prevPCA9533    byte
	prevPCA9632    [5]byte // PWM0-3 + LEDOUT
}

// newSensorOnlyRuntime creates a runtime for kinematics:none configs.
func newSensorOnlyRuntime(cfgPath string, cfg *configWrapper, dict *protocol.Dictionary, formats map[string]*protocol.MessageFormat, mcuFreq float64) (*runtime, error) {
	tmpDir := filepath.Join("out", "go_migration_tmp")
	if err := os.MkdirAll(tmpDir, 0o755); err != nil {
		return nil, err
	}
	rawPath := filepath.Join(tmpDir, "serial-"+randSuffix()+".bin")
	f, err := os.OpenFile(rawPath, os.O_CREATE|os.O_TRUNC|os.O_RDWR, 0o644)
	if err != nil {
		return nil, err
	}

	sq, err := chelper.NewSerialQueue(int(f.Fd()), 'f', 0, "serialq mcu")
	if err != nil {
		f.Close()
		return nil, err
	}
	sq.SetClockEst(1e12, chelper.Monotonic(), 0, 0)

	cqMain := chelper.NewCommandQueue()
	if cqMain == nil {
		sq.Free()
		f.Close()
		return nil, fmt.Errorf("alloc main command queue failed")
	}

	sor := &sensorOnlyRuntime{
		cfg:     cfg,
		dict:    dict,
		formats: formats,
		mcuFreq: mcuFreq,
		sq:      sq,
		cqMain:  cqMain,
		rawOut:  f,
		leds:    make(map[string]*ledState),
		// Initial clock at 0.2s (matching Python debugoutput timing)
		nextClock:    int(mcuFreq * 0.2),
		prevPWMColor: -1, // Invalid initial value to force first update
	}

	// Parse LED configurations and assign OIDs
	// OID layout matches Python expected output:
	// oid=0: neopixel
	// oid=1: SPI for dotstar
	// oid=2: I2C for pca9533
	// oid=3: I2C (software) for pca9632
	// oid=4: digital_out PWM for [led]
	sor.oidNeopixel = 0
	sor.oidSPI = 1
	sor.oidI2C1 = 2
	sor.oidI2C2 = 3
	sor.oidPWM = 4

	if err := sor.parseLEDConfigs(); err != nil {
		sq.Free()
		f.Close()
		return nil, err
	}

	// Create a minimal runtime that wraps the sensor-only runtime
	r := &runtime{
		cfg:            cfg,
		dict:           dict,
		formats:        formats,
		mcuFreq:        mcuFreq,
		sq:             sq,
		cqMain:         cqMain,
		rawFile:        f,
		rawPath:        rawPath,
		sensorOnly:     sor,
		kinematicsNone: true,
	}

	return r, nil
}

func (sor *sensorOnlyRuntime) parseLEDConfigs() error {
	cfg := sor.cfg

	// Parse [led lled] section
	for name, sec := range cfg.sections {
		if strings.HasPrefix(name, "led ") {
			ledName := strings.TrimPrefix(name, "led ")
			chainCount := 1
			ls := &ledState{
				name:       ledName,
				ledType:    "led",
				oid:        sor.oidPWM,
				chainCount: chainCount,
				colors:     make([]ledColor, chainCount),
			}
			// Parse initial colors
			var err error
			if ls.colors[0].R, err = parseLEDColor(sec, "initial_red"); err != nil {
				return fmt.Errorf("led %s: %w", ledName, err)
			}
			if ls.colors[0].G, err = parseLEDColor(sec, "initial_green"); err != nil {
				return fmt.Errorf("led %s: %w", ledName, err)
			}
			if ls.colors[0].B, err = parseLEDColor(sec, "initial_blue"); err != nil {
				return fmt.Errorf("led %s: %w", ledName, err)
			}
			if ls.colors[0].W, err = parseLEDColor(sec, "initial_white"); err != nil {
				return fmt.Errorf("led %s: %w", ledName, err)
			}
			sor.leds[ledName] = ls
		}
	}

	// Parse [neopixel nled] section
	for name, sec := range cfg.sections {
		if strings.HasPrefix(name, "neopixel ") {
			ledName := strings.TrimPrefix(name, "neopixel ")
			chainCount := 1
			if s := sec["chain_count"]; s != "" {
				fmt.Sscanf(s, "%d", &chainCount)
			}
			ls := &ledState{
				name:       ledName,
				ledType:    "neopixel",
				oid:        sor.oidNeopixel,
				chainCount: chainCount,
				colors:     make([]ledColor, chainCount),
			}
			// Parse initial colors (applies to all LEDs in chain)
			initR, err := parseLEDColor(sec, "initial_red")
			if err != nil {
				return fmt.Errorf("neopixel %s: %w", ledName, err)
			}
			initG, err := parseLEDColor(sec, "initial_green")
			if err != nil {
				return fmt.Errorf("neopixel %s: %w", ledName, err)
			}
			initB, err := parseLEDColor(sec, "initial_blue")
			if err != nil {
				return fmt.Errorf("neopixel %s: %w", ledName, err)
			}
			initW, err := parseLEDColor(sec, "initial_white")
			if err != nil {
				return fmt.Errorf("neopixel %s: %w", ledName, err)
			}
			for i := range ls.colors {
				ls.colors[i] = ledColor{initR, initG, initB, initW}
			}
			sor.leds[ledName] = ls
		}
	}

	// Parse [dotstar dled] section
	for name, sec := range cfg.sections {
		if strings.HasPrefix(name, "dotstar ") {
			ledName := strings.TrimPrefix(name, "dotstar ")
			chainCount := 1
			if s := sec["chain_count"]; s != "" {
				fmt.Sscanf(s, "%d", &chainCount)
			}
			ls := &ledState{
				name:       ledName,
				ledType:    "dotstar",
				oid:        sor.oidSPI,
				chainCount: chainCount,
				colors:     make([]ledColor, chainCount),
			}
			// Parse initial colors
			initR, err := parseLEDColor(sec, "initial_red")
			if err != nil {
				return fmt.Errorf("dotstar %s: %w", ledName, err)
			}
			initG, err := parseLEDColor(sec, "initial_green")
			if err != nil {
				return fmt.Errorf("dotstar %s: %w", ledName, err)
			}
			initB, err := parseLEDColor(sec, "initial_blue")
			if err != nil {
				return fmt.Errorf("dotstar %s: %w", ledName, err)
			}
			for i := range ls.colors {
				ls.colors[i] = ledColor{initR, initG, initB, 0}
			}
			sor.leds[ledName] = ls
		}
	}

	// Parse [pca9533 p5led] section
	for name, sec := range cfg.sections {
		if strings.HasPrefix(name, "pca9533 ") {
			ledName := strings.TrimPrefix(name, "pca9533 ")
			ls := &ledState{
				name:       ledName,
				ledType:    "pca9533",
				oid:        sor.oidI2C1,
				chainCount: 1,
				colors:     make([]ledColor, 1),
			}
			var err error
			if ls.colors[0].R, err = parseLEDColor(sec, "initial_red"); err != nil {
				return fmt.Errorf("pca9533 %s: %w", ledName, err)
			}
			if ls.colors[0].G, err = parseLEDColor(sec, "initial_green"); err != nil {
				return fmt.Errorf("pca9533 %s: %w", ledName, err)
			}
			if ls.colors[0].B, err = parseLEDColor(sec, "initial_blue"); err != nil {
				return fmt.Errorf("pca9533 %s: %w", ledName, err)
			}
			if ls.colors[0].W, err = parseLEDColor(sec, "initial_white"); err != nil {
				return fmt.Errorf("pca9533 %s: %w", ledName, err)
			}
			sor.leds[ledName] = ls
		}
	}

	// Parse [pca9632 p6led] section
	for name, sec := range cfg.sections {
		if strings.HasPrefix(name, "pca9632 ") {
			ledName := strings.TrimPrefix(name, "pca9632 ")
			ls := &ledState{
				name:       ledName,
				ledType:    "pca9632",
				oid:        sor.oidI2C2,
				chainCount: 1,
				colors:     make([]ledColor, 1),
			}
			var err error
			if ls.colors[0].R, err = parseLEDColor(sec, "initial_red"); err != nil {
				return fmt.Errorf("pca9632 %s: %w", ledName, err)
			}
			if ls.colors[0].G, err = parseLEDColor(sec, "initial_green"); err != nil {
				return fmt.Errorf("pca9632 %s: %w", ledName, err)
			}
			if ls.colors[0].B, err = parseLEDColor(sec, "initial_blue"); err != nil {
				return fmt.Errorf("pca9632 %s: %w", ledName, err)
			}
			if ls.colors[0].W, err = parseLEDColor(sec, "initial_white"); err != nil {
				return fmt.Errorf("pca9632 %s: %w", ledName, err)
			}
			sor.leds[ledName] = ls
		}
	}

	return nil
}

// initLEDs sends the initialization commands for all LEDs (called after finalize_config).
// This matches Python's klippy:connect event handling for LEDs.
func (sor *sensorOnlyRuntime) initLEDs(r *runtime) error {
	// Initialization order (matching Python):
	// 1. PWM LED (lled): queue_digital_out
	// 2. PCA9533 (p5led): i2c_write for PWM0, PWM1, then update_leds
	// 3. Neopixel (nled): neopixel_update
	// 4. Dotstar (dled): spi_send
	// 5. PCA9632 (p6led): i2c_write for MODE1, MODE2, then PWM and LEDOUT

	clock := sor.nextClock
	cycleTicks := int(sor.mcuFreq * 0.01) // 10ms cycle

	// 1. PWM LED init
	if ls, ok := sor.leds["lled"]; ok {
		onTicks := int(float64(cycleTicks) * ls.colors[0].R)
		cmd := fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", ls.oid, clock, onTicks)
		if err := r.sendLine(cmd, r.cqMain, 0, 0); err != nil {
			return err
		}
		sor.prevPWMColor = ls.colors[0].R
	}

	// 2. PCA9533 init (PWM0=85, PWM1=170, then LED select)
	if ls, ok := sor.leds["p5led"]; ok {
		// PWM0 register (0x02) = 0x55 (85)
		if err := r.sendLine(fmt.Sprintf("i2c_write oid=%d data=%s", ls.oid, formatBytes([]byte{0x02, 0x55})), r.cqMain, 0, 0); err != nil {
			return err
		}
		// PWM1 register (0x04) = 0xAA (170)
		if err := r.sendLine(fmt.Sprintf("i2c_write oid=%d data=%s", ls.oid, formatBytes([]byte{0x04, 0xaa})), r.cqMain, 0, 0); err != nil {
			return err
		}
	}

	// 3. Neopixel init
	if ls, ok := sor.leds["nled"]; ok {
		data := make([]byte, ls.chainCount*3)
		for i, c := range ls.colors {
			data[i*3] = byte(c.G*255 + 0.5)   // G
			data[i*3+1] = byte(c.R*255 + 0.5) // R
			data[i*3+2] = byte(c.B*255 + 0.5) // B
		}
		cmd := fmt.Sprintf("neopixel_update oid=%d pos=0 data=%s", ls.oid, formatBytes(data))
		if err := r.sendLine(cmd, r.cqMain, 0, 0); err != nil {
			return err
		}
		sor.prevNeopixel = append([]byte{}, data...)
	}

	// 4. Dotstar init
	if ls, ok := sor.leds["dled"]; ok {
		data := make([]byte, 4+ls.chainCount*4+4)
		// Start frame: 0x00 0x00 0x00 0x00
		for i, c := range ls.colors {
			idx := 4 + i*4
			data[idx] = 0xff // max brightness
			data[idx+1] = byte(c.B*255 + 0.5)
			data[idx+2] = byte(c.G*255 + 0.5)
			data[idx+3] = byte(c.R*255 + 0.5)
		}
		// End frame: 0xff 0xff 0xff 0xff
		endIdx := 4 + ls.chainCount*4
		data[endIdx] = 0xff
		data[endIdx+1] = 0xff
		data[endIdx+2] = 0xff
		data[endIdx+3] = 0xff
		cmd := fmt.Sprintf("spi_send oid=%d data=%s", ls.oid, formatBytes(data))
		if err := r.sendLine(cmd, r.cqMain, 0, 0); err != nil {
			return err
		}
		sor.prevDotstar = append([]byte{}, data...)
	}

	// 5. PCA9533 LED select (after neopixel/dotstar)
	if ls, ok := sor.leds["p5led"]; ok {
		rmap := []byte{0, 2, 3, 1, 1}
		red := rmap[int(ls.colors[0].R*4)]
		green := rmap[int(ls.colors[0].G*4)]
		blue := rmap[int(ls.colors[0].B*4)]
		white := rmap[int(ls.colors[0].W*4)]
		ls0 := (white << 6) | (blue << 4) | (green << 2) | red
		cmd := fmt.Sprintf("i2c_write oid=%d data=%s", ls.oid, formatBytes([]byte{0x05, ls0}))
		if err := r.sendLine(cmd, r.cqMain, 0, 0); err != nil {
			return err
		}
		sor.prevPCA9533 = ls0
	}

	// 6. PCA9632 init (MODE1, MODE2, then PWM and LEDOUT)
	if ls, ok := sor.leds["p6led"]; ok {
		// MODE1 register (0x00) = 0x00
		if err := r.sendLine(fmt.Sprintf("i2c_write oid=%d data=%s", ls.oid, formatBytes([]byte{0x00, 0x00})), r.cqMain, 0, 0); err != nil {
			return err
		}
		// MODE2 register (0x01) = 0x15
		if err := r.sendLine(fmt.Sprintf("i2c_write oid=%d data=%s", ls.oid, formatBytes([]byte{0x01, 0x15})), r.cqMain, 0, 0); err != nil {
			return err
		}

		// PWM registers and LEDOUT
		led0 := byte(ls.colors[0].R*255 + 0.5)
		led1 := byte(ls.colors[0].G*255 + 0.5)
		led2 := byte(ls.colors[0].B*255 + 0.5)
		led3 := byte(ls.colors[0].W*255 + 0.5)

		// LEDOUT: set PWM mode (0x02) for non-zero channels
		var ledout byte
		if led0 != 0 {
			ledout |= 0x02 << 0
		}
		if led1 != 0 {
			ledout |= 0x02 << 2
		}
		if led2 != 0 {
			ledout |= 0x02 << 4
		}
		if led3 != 0 {
			ledout |= 0x02 << 6
		}

		// Send PWM values
		regAddrs := []struct {
			reg byte
			val byte
		}{
			{0x02, led0},
			{0x03, led1},
			{0x04, led2},
			{0x05, led3},
			{0x08, ledout},
		}
		for _, rv := range regAddrs {
			if err := r.sendLine(fmt.Sprintf("i2c_write oid=%d data=%s", ls.oid, formatBytes([]byte{rv.reg, rv.val})), r.cqMain, 0, 0); err != nil {
				return err
			}
		}

		// Store previous values
		sor.prevPCA9632 = [5]byte{led0, led1, led2, led3, ledout}
	}

	// Advance clock for next commands
	sor.nextClock = clock + int(sor.mcuFreq*0.05) // 50ms

	return nil
}

// cmdSetLED handles SET_LED commands.
func (sor *sensorOnlyRuntime) cmdSetLED(r *runtime, params map[string]string) error {
	ledName := strings.TrimSpace(params["LED"])
	if ledName == "" {
		return fmt.Errorf("SET_LED: missing LED parameter")
	}

	ls, ok := sor.leds[ledName]
	if !ok {
		return fmt.Errorf("SET_LED: unknown LED %q", ledName)
	}

	// Parse color values
	var red, green, blue, white float64
	if s := params["RED"]; s != "" {
		fmt.Sscanf(s, "%f", &red)
	}
	if s := params["GREEN"]; s != "" {
		fmt.Sscanf(s, "%f", &green)
	}
	if s := params["BLUE"]; s != "" {
		fmt.Sscanf(s, "%f", &blue)
	}
	if s := params["WHITE"]; s != "" {
		fmt.Sscanf(s, "%f", &white)
	}

	// Parse INDEX (1-based, default all LEDs)
	index := -1
	if s := params["INDEX"]; s != "" {
		fmt.Sscanf(s, "%d", &index)
		index-- // Convert to 0-based
	}

	// Parse TRANSMIT (default 1)
	transmit := true
	if s := params["TRANSMIT"]; s != "" {
		transmit = s == "1"
	}

	// Update color state
	color := ledColor{R: red, G: green, B: blue, W: white}
	if index < 0 {
		// Set all LEDs
		for i := range ls.colors {
			ls.colors[i] = color
		}
	} else if index < len(ls.colors) {
		ls.colors[index] = color
	}

	// Transmit if requested
	if transmit {
		return sor.transmitLED(r, ls)
	}
	return nil
}

// transmitLED sends LED color data to the MCU.
func (sor *sensorOnlyRuntime) transmitLED(r *runtime, ls *ledState) error {
	clock := sor.nextClock

	switch ls.ledType {
	case "led":
		// PWM LED: queue_digital_out
		cycleTicks := int(sor.mcuFreq * 0.01) // 10ms cycle = 160000 ticks at 16MHz
		onTicks := int(float64(cycleTicks) * ls.colors[0].R)
		// Only send if color changed
		if ls.colors[0].R != sor.prevPWMColor {
			cmd := fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", ls.oid, clock, onTicks)
			if err := r.sendLine(cmd, r.cqMain, 0, 0); err != nil {
				return err
			}
			sor.prevPWMColor = ls.colors[0].R
			sor.nextClock += int(sor.mcuFreq * 0.1) // Advance 100ms for next command
		}

	case "neopixel":
		// Neopixel: neopixel_update with GRB color order
		data := make([]byte, ls.chainCount*3)
		for i, c := range ls.colors {
			data[i*3] = byte(c.G*255 + 0.5)   // G
			data[i*3+1] = byte(c.R*255 + 0.5) // R
			data[i*3+2] = byte(c.B*255 + 0.5) // B
		}
		// Find changed bytes and send updates
		if err := sor.sendNeopixelUpdate(r, ls.oid, data); err != nil {
			return err
		}

	case "dotstar":
		// Dotstar: spi_send with APA102 protocol
		// Start frame (4 bytes 0x00), LED frames (4 bytes each), end frame (4 bytes 0xff)
		data := make([]byte, 4+ls.chainCount*4+4)
		// Start frame: 0x00 0x00 0x00 0x00
		// LED frames: [brightness=0xff][blue][green][red]
		for i, c := range ls.colors {
			idx := 4 + i*4
			data[idx] = 0xff // max brightness
			data[idx+1] = byte(c.B*255 + 0.5)
			data[idx+2] = byte(c.G*255 + 0.5)
			data[idx+3] = byte(c.R*255 + 0.5)
		}
		// End frame: 0xff 0xff 0xff 0xff
		endIdx := 4 + ls.chainCount*4
		data[endIdx] = 0xff
		data[endIdx+1] = 0xff
		data[endIdx+2] = 0xff
		data[endIdx+3] = 0xff

		// Only send if data changed
		if !bytesEqual(data, sor.prevDotstar) {
			cmd := fmt.Sprintf("spi_send oid=%d data=%s", ls.oid, formatBytes(data))
			if err := r.sendLine(cmd, r.cqMain, 0, 0); err != nil {
				return err
			}
			sor.prevDotstar = append([]byte{}, data...)
		}

	case "pca9533":
		// PCA9533: i2c_write to LED select register (0x05)
		// Uses rmap quantization: value*4 maps to [0, 2, 3, 1, 1]
		rmap := []byte{0, 2, 3, 1, 1}
		red := rmap[int(ls.colors[0].R*4)]
		green := rmap[int(ls.colors[0].G*4)]
		blue := rmap[int(ls.colors[0].B*4)]
		white := rmap[int(ls.colors[0].W*4)]
		ls0 := (white << 6) | (blue << 4) | (green << 2) | red

		// Only send if changed
		if ls0 != sor.prevPCA9533 {
			cmd := fmt.Sprintf("i2c_write oid=%d data=%s", ls.oid, formatBytes([]byte{0x05, ls0}))
			if err := r.sendLine(cmd, r.cqMain, 0, 0); err != nil {
				return err
			}
			sor.prevPCA9533 = ls0
		}

	case "pca9632":
		// PCA9632: i2c_write to PWM and LEDOUT registers
		// Color order is RGBW, registers 0x02-0x05 for PWM, 0x08 for LEDOUT
		led0 := byte(ls.colors[0].R*255 + 0.5) // PWM0 = R
		led1 := byte(ls.colors[0].G*255 + 0.5) // PWM1 = G
		led2 := byte(ls.colors[0].B*255 + 0.5) // PWM2 = B
		led3 := byte(ls.colors[0].W*255 + 0.5) // PWM3 = W

		// LEDOUT: set PWM mode (0x02) for non-zero channels
		var ledout byte
		if led0 != 0 {
			ledout |= 0x02 << 0
		}
		if led1 != 0 {
			ledout |= 0x02 << 2
		}
		if led2 != 0 {
			ledout |= 0x02 << 4
		}
		if led3 != 0 {
			ledout |= 0x02 << 6
		}

		newRegs := [5]byte{led0, led1, led2, led3, ledout}

		// Send only changed registers
		regAddrs := []byte{0x02, 0x03, 0x04, 0x05, 0x08}
		for i, val := range newRegs {
			if val != sor.prevPCA9632[i] {
				cmd := fmt.Sprintf("i2c_write oid=%d data=%s", ls.oid, formatBytes([]byte{regAddrs[i], val}))
				if err := r.sendLine(cmd, r.cqMain, 0, 0); err != nil {
					return err
				}
				sor.prevPCA9632[i] = val
			}
		}

	default:
		return fmt.Errorf("unsupported LED type: %s", ls.ledType)
	}

	return nil
}

// sendNeopixelUpdate sends neopixel_update commands for changed bytes.
func (sor *sensorOnlyRuntime) sendNeopixelUpdate(r *runtime, oid int, data []byte) error {
	oldData := sor.prevNeopixel
	if oldData == nil {
		oldData = make([]byte, len(data))
		for i := range oldData {
			oldData[i] = data[i] ^ 1 // Force all bytes to be different initially
		}
	}

	if bytesEqual(data, oldData) {
		return nil
	}

	// Find changed byte ranges and batch nearby changes
	type diff struct {
		pos   int
		count int
	}
	var diffs []diff
	for i := 0; i < len(data); i++ {
		if data[i] != oldData[i] {
			diffs = append(diffs, diff{i, 1})
		}
	}

	// Batch together changes that are close (within 5 bytes) and small (< 16 bytes)
	for i := len(diffs) - 2; i >= 0; i-- {
		pos := diffs[i].pos
		nextpos, nextcount := diffs[i+1].pos, diffs[i+1].count
		if pos+5 >= nextpos && nextcount < 16 {
			diffs[i].count = nextcount + (nextpos - pos)
			diffs = append(diffs[:i+1], diffs[i+2:]...)
		}
	}

	// Send updates
	for _, d := range diffs {
		cmd := fmt.Sprintf("neopixel_update oid=%d pos=%d data=%s", oid, d.pos, formatBytes(data[d.pos:d.pos+d.count]))
		if err := r.sendLine(cmd, r.cqMain, 0, 0); err != nil {
			return err
		}
	}

	sor.prevNeopixel = append([]byte{}, data...)
	return nil
}

// bytesEqual checks if two byte slices are equal.
func bytesEqual(a, b []byte) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// formatBytes formats a byte slice as b'...' Python-style string.
// Note: spaces (0x20) must be hex-escaped to avoid breaking command parsing.
func formatBytes(data []byte) string {
	var sb strings.Builder
	sb.WriteString("b'")
	for _, b := range data {
		// Printable ASCII excluding space, quote, and backslash
		if b > 32 && b < 127 && b != '\'' && b != '\\' {
			sb.WriteByte(b)
		} else {
			sb.WriteString(fmt.Sprintf("\\x%02x", b))
		}
	}
	sb.WriteString("'")
	return sb.String()
}
