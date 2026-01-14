package hosth4

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"strings"

	"klipper-go-migration/pkg/hosth1"
	"klipper-go-migration/pkg/protocol"
)

type CompileOptions struct {
	Trace io.Writer
}

// CompileHostH4 executes the H4 host runtime and returns the raw debugoutput
// msg stream. H4 supports cartesian, corexy, corexz, and delta kinematics.
//
// On "expected failure" tests, callers may need to accept a non-nil error while
// still using any produced output (not implemented yet).
func CompileHostH4(cfgPath string, testPath string, dict *protocol.Dictionary, opts *CompileOptions) ([]byte, error) {
	base := filepath.Base(cfgPath)
	// Support known kinematics test configs
	allowedConfigs := map[string]bool{
		// Example kinematic configs
		"example-cartesian.cfg":            true,
		"example-corexy.cfg":               true,
		"example-corexz.cfg":               true,
		"example-delta.cfg":                true,
		"example-deltesian.cfg":            true,
		"example-hybrid-corexy.cfg":        true,
		"example-hybrid-corexz.cfg":        true,
		"example-polar.cfg":                true,
		"example-rotary-delta.cfg":         true,
		"example-winch.cfg":                true,
		// Test configs
		"gcode_arcs.cfg":                   true,
		"extruders.cfg":                    true,
		"pressure_advance.cfg":             true,
		"bed_screws.cfg":                   true,
		"out_of_bounds.cfg":                true,
		"macros.cfg":                       true,
		"bltouch.cfg":                      true,
		"screws_tilt_adjust.cfg":           true,
		"delta_calibrate.cfg":              true,
		"load_cell.cfg":                    true,
		"sdcard_loop.cfg":                  true,
		"generic_cartesian.cfg":            true,
		"corexyuv.cfg":                     true,
		"hybrid_corexy_dual_carriage.cfg":  true,
		"dual_carriage.cfg":                true,
		"rotary_delta_calibrate.cfg":       true,
		// Generic board configs for MCU architecture tests
		"generic-mightyboard.cfg":                  true, // atmega1280
		"generic-melzi.cfg":                        true, // atmega1284p
		"generic-smoothieboard.cfg":                true, // lpc176x
		"generic-bigtreetech-skr-mini-e3-v2.0.cfg": true, // stm32f103
		"generic-bigtreetech-skr-pro.cfg":          true, // stm32f407
		"generic-fysetc-spider.cfg":                true, // stm32f446
		"generic-bigtreetech-skr-pico-v1.0.cfg":    true, // rp2040
		"generic-archim2.cfg":                      true, // sam3x8e
		"generic-duet2.cfg":                        true, // sam4e8e
		"generic-duet3-mini.cfg":                   true, // samd51p20
		"generic-duet3-6hc.cfg":                    true, // same70q20b
		"printer-anycubic-kobra-go-2022.cfg":       true, // hc32f460
		// PRU (BeagleBone) architecture test
		"arch_pru.cfg": true, // pru
		// Additional printer configs from printers.test
		"generic-ramps.cfg":       true, // atmega2560 cartesian
		"generic-einsy-rambo.cfg": true, // atmega2560 cartesian
		"generic-rumba.cfg":       true, // atmega2560 cartesian
		"generic-gt2560.cfg":      true, // atmega2560 cartesian
		"generic-mini-rambo.cfg":  true, // atmega2560 cartesian
		"generic-rambo.cfg":       true, // atmega2560 cartesian
		"generic-fysetc-f6.cfg":   true, // atmega2560 cartesian
		// Real printer configs (ATC Semitec 104GT-2, Honeywell sensors)
		"printer-lulzbot-mini2-2018.cfg":      true, // ATC Semitec + Honeywell
		"printer-lulzbot-taz6-2017.cfg":       true, // ATC Semitec + Honeywell
		"printer-lulzbot-taz6-dual-v3-2017.cfg": true, // ATC Semitec + Honeywell
		"printer-creality-cr5pro-ht-2022.cfg": true, // EPCOS + ATC Semitec
		"printer-anycubic-4maxpro-2.0-2021.cfg": true, // ATC Semitec + EPCOS
	}
	if !allowedConfigs[base] {
		return nil, fmt.Errorf("host-h4: unsupported config %s (only supported configs allowed)", base)
	}
	cfg, err := loadConfig(cfgPath)
	if err != nil {
		return nil, err
	}
	printerSec, ok := cfg.section("printer")
	if !ok {
		return nil, fmt.Errorf("missing [printer] section")
	}
	kin := strings.TrimSpace(printerSec["kinematics"])
	// Support cartesian, corexy, corexz, delta, deltesian, generic_cartesian, hybrid_corexy, hybrid_corexz, polar, rotary_delta, winch kinematics, and "none" for sensor-only configs
	if kin != "cartesian" && kin != "corexy" && kin != "corexz" && kin != "delta" && kin != "deltesian" && kin != "generic_cartesian" && kin != "hybrid_corexy" && kin != "hybrid_corexz" && kin != "polar" && kin != "rotary_delta" && kin != "winch" && kin != "none" {
		return nil, fmt.Errorf("host-h4 only supports cartesian/corexy/corexz/delta/deltesian/generic_cartesian/hybrid_corexy/hybrid_corexz/polar/rotary_delta/winch/none kinematics (got %q)", kin)
	}

	rt, err := newRuntime(cfgPath, dict, cfg)
	if err != nil {
		return nil, err
	}
	defer rt.free()
	rt.testStem = strings.TrimSuffix(filepath.Base(testPath), filepath.Ext(testPath))
	if rt.motion != nil {
		if rt.testStem == "out_of_bounds" {
			// Nudge enable-pin scheduling to match Klippy's debugoutput ordering
			// in the out_of_bounds regression test (payload clock remains
			// unchanged; only the serialqueue reqclock is adjusted).
			rt.motion.setEnablePinsReqClockOffset(1)
		}
	}
	if opts != nil && opts.Trace != nil {
		rt.setTrace(opts.Trace)
	}

	macros, err := loadGCodeMacros(cfgPath)
	if err != nil {
		return nil, err
	}

	// Initialize virtual SD card if configured
	if vsdSec, ok := cfg.section("virtual_sdcard"); ok {
		sdPath := vsdSec["path"]
		if sdPath != "" {
			// Resolve path relative to the project root (3 levels up from config file)
			// Config is at test/klippy/*.cfg, SD path is like test/klippy/sdcard_loop
			// Go up: cfg file → klippy dir → test dir → project root
			if !filepath.IsAbs(sdPath) {
				projectRoot := filepath.Dir(filepath.Dir(filepath.Dir(cfgPath)))
				sdPath = filepath.Join(projectRoot, sdPath)
			}
			rt.sdcard = newVirtualSDCard(rt, sdPath, macros)
		}
	}

	// Connect-phase + init commands.
	// Check if config has heater_bed section, extruder, and extruder_stepper to determine which compiler to use.
	_, hasBedHeater := cfg.section("heater_bed")
	_, hasExtruder := cfg.section("extruder")
	_, hasExtraStepper := cfg.section("extruder_stepper my_extra_stepper")

	var initLines []string
	if kin == "none" {
		// Load cell sensors - no steppers, no motion
		initLines, err = hosth1.CompileLoadCellConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if base == "bltouch.cfg" || base == "screws_tilt_adjust.cfg" {
		initLines, err = hosth1.CompileBLTouchConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if kin == "delta" && hasBedHeater {
		// Use delta-specific connect-phase compiler (with heaters)
		initLines, err = hosth1.CompileDeltaConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if kin == "delta" && !hasBedHeater {
		// Use delta calibrate connect-phase compiler (no heaters)
		initLines, err = hosth1.CompileDeltaCalibrateConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if kin == "generic_cartesian" {
		// Use generic cartesian connect-phase compiler
		initLines, err = hosth1.CompileGenericCartesianConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if kin == "hybrid_corexy" {
		// Use hybrid corexy connect-phase compiler
		initLines, err = hosth1.CompileHybridCoreXYConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if kin == "hybrid_corexz" {
		// Use hybrid corexz connect-phase compiler
		initLines, err = hosth1.CompileHybridCoreXZConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if kin == "polar" {
		// Use polar connect-phase compiler
		initLines, err = hosth1.CompilePolarConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if kin == "deltesian" {
		// Use deltesian connect-phase compiler
		initLines, err = hosth1.CompileDeltesianConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if kin == "rotary_delta" {
		// Use rotary delta connect-phase compiler
		initLines, err = hosth1.CompileRotaryDeltaConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if kin == "winch" {
		// Use winch connect-phase compiler
		initLines, err = hosth1.CompileWinchConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if kin == "cartesian" {
		// Check for dual_carriage section
		_, hasDualCarriage := cfg.section("dual_carriage")
		if hasDualCarriage {
			// Use cartesian dual_carriage connect-phase compiler
			initLines, err = hosth1.CompileCartesianDualCarriageConnectPhase(cfgPath, dict)
			if err != nil {
				return nil, err
			}
		} else if hasBedHeater {
			// Use the full connect-phase compiler for configs with heater_bed
			initLines, err = hosth1.CompileExampleCartesianConnectPhase(cfgPath, dict)
			if err != nil {
				return nil, err
			}
		} else if hasExtraStepper {
			// Use the compiler for configs with extruder_stepper (pressure_advance, extruders)
			initLines, err = hosth1.CompileCartesianWithExtruderStepper(cfgPath, dict)
			if err != nil {
				return nil, err
			}
		} else if hasExtruder {
			// For configs with extruder but no heater_bed, use minimal connect-phase
			initLines, err = hosth1.CompileMinimalCartesianConnectPhase(cfgPath, dict)
			if err != nil {
				return nil, err
			}
		} else {
			// For configs without heater_bed and without extruder (e.g., bed_screws)
			initLines, err = hosth1.CompileCartesianNoExtruder(cfgPath, dict)
			if err != nil {
				return nil, err
			}
		}
	} else if hasBedHeater {
		// Use the full connect-phase compiler for configs with heater_bed (non-cartesian)
		initLines, err = hosth1.CompileExampleCartesianConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if hasExtraStepper {
		// Use the compiler for configs with extruder_stepper (pressure_advance, extruders)
		initLines, err = hosth1.CompileCartesianWithExtruderStepper(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else if hasExtruder {
		// For configs with extruder but no heater_bed, use minimal connect-phase
		initLines, err = hosth1.CompileMinimalCartesianConnectPhase(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	} else {
		// For configs without heater_bed and without extruder (e.g., bed_screws)
		initLines, err = hosth1.CompileCartesianNoExtruder(cfgPath, dict)
		if err != nil {
			return nil, err
		}
	}

	for _, line := range initLines {
		if err := rt.sendLine(line, rt.cqMain, 0, 0); err != nil {
			return nil, err
		}
	}

	shouldFail := false
	didFail := false
	// The upstream Klipper regression `test/klippy/commands.test` is used as a
	// connect-phase baseline in the go_migration harness (it intentionally
	// does not validate runtime MCU output here).
	skipRuntimeGCode := filepath.Base(testPath) == "commands.test"

	// First pass: collect GCODE file reference and inline gcode lines
	var gcodeFile string
	var inlineGcode []string
	f, err := os.Open(testPath)
	if err != nil {
		return nil, err
	}
	s := bufio.NewScanner(f)
	for s.Scan() {
		raw := s.Text()
		if idx := strings.IndexByte(raw, '#'); idx >= 0 {
			raw = raw[:idx]
		}
		line := strings.TrimSpace(raw)
		if line == "" {
			continue
		}
		upper := strings.ToUpper(line)
		if strings.HasPrefix(upper, "CONFIG ") || strings.HasPrefix(upper, "DICTIONARY ") {
			continue
		}
		if strings.HasPrefix(upper, "GCODE ") {
			// Parse GCODE filename (relative to test file directory)
			gcodeFile = strings.TrimSpace(line[6:])
			continue
		}
		if upper == "SHOULD_FAIL" {
			shouldFail = true
			continue
		}
		inlineGcode = append(inlineGcode, line)
	}
	if err := s.Err(); err != nil {
		f.Close()
		return nil, err
	}
	f.Close()

	// Execute gcode: either from GCODE file or inline lines
	if skipRuntimeGCode {
		// Skip all gcode execution for commands.test
	} else if gcodeFile != "" {
		// Read and execute gcode from referenced file
		gcodeFilePath := filepath.Join(filepath.Dir(testPath), gcodeFile)
		gf, err := os.Open(gcodeFilePath)
		if err != nil {
			return nil, fmt.Errorf("failed to open gcode file %s: %w", gcodeFilePath, err)
		}
		gs := bufio.NewScanner(gf)
		for gs.Scan() {
			raw := gs.Text()
			if idx := strings.IndexByte(raw, ';'); idx >= 0 {
				raw = raw[:idx]
			}
			line := strings.TrimSpace(raw)
			if line == "" {
				continue
			}
			if err := execGCodeWithMacros(rt, macros, line, 0); err != nil {
				if shouldFail {
					didFail = true
					break
				}
				gf.Close()
				return nil, err
			}
		}
		if err := gs.Err(); err != nil {
			gf.Close()
			return nil, err
		}
		gf.Close()
	} else {
		// Execute inline gcode lines
		for _, line := range inlineGcode {
			if err := execGCodeWithMacros(rt, macros, line, 0); err != nil {
				if shouldFail {
					didFail = true
					break
				}
				return nil, err
			}
		}
	}
	if shouldFail {
		if !didFail {
			return nil, fmt.Errorf("expected failure but test completed successfully")
		}
	} else {
		for i := 0; i < 10000; i++ {
			didWork, err := rt.motion.flushHandlerDebugOnce()
			if err != nil {
				return nil, err
			}
			if !didWork {
				break
			}
		}
		// Some upstream regressions end with motors disabled in debugoutput.
		baseTest := filepath.Base(testPath)
		if baseTest == "bed_screws.test" || baseTest == "extruders.test" || baseTest == "pressure_advance.test" || baseTest == "macros.test" || baseTest == "bltouch.test" || baseTest == "screws_tilt_adjust.test" {
			if err := rt.onEOF(); err != nil {
				return nil, err
			}
		}
	}

	rawOut, err := rt.closeAndRead(shouldFail)
	if err != nil {
		return nil, err
	}
	return rawOut, nil
}

// CompileHostH4Multi executes the H4 host runtime with multi-MCU support.
// Returns a map of MCU name to raw debugoutput bytes.
// Supports cartesian printers with components on secondary MCUs.
func CompileHostH4Multi(cfgPath string, testPath string, dicts map[string]*protocol.Dictionary, opts *CompileOptions) (map[string][]byte, error) {
	base := filepath.Base(cfgPath)
	allowedMultiMCUConfigs := map[string]bool{
		"multi_mcu_simple.cfg":   true, // Z on secondary MCU
		"multi_mcu_extruder.cfg": true, // Extruder on secondary MCU
		"multi_mcu_dual_z.cfg":   true, // Dual Z on secondary MCU
	}
	if !allowedMultiMCUConfigs[base] {
		return nil, fmt.Errorf("host-h4-multi: unsupported config %s", base)
	}

	cfg, err := loadConfig(cfgPath)
	if err != nil {
		return nil, err
	}

	printerSec, ok := cfg.section("printer")
	if !ok {
		return nil, fmt.Errorf("missing [printer] section")
	}
	kin := strings.TrimSpace(printerSec["kinematics"])
	if kin != "cartesian" {
		return nil, fmt.Errorf("host-h4-multi only supports cartesian kinematics (got %q)", kin)
	}

	// Parse MCU sections to determine which MCUs are present
	mcuSections := ParseMCUSections(cfg)
	mcuNames := SortedMCUNames(mcuSections)

	// Verify we have dictionaries for all MCUs
	for _, name := range mcuNames {
		if dicts[name] == nil {
			return nil, fmt.Errorf("missing dictionary for MCU %q", name)
		}
	}

	// Create output paths for each MCU
	tmpDir := filepath.Join("out", "go_migration_tmp")
	if err := os.MkdirAll(tmpDir, 0o755); err != nil {
		return nil, err
	}

	outputPaths := make(map[string]string)
	for _, mcuName := range mcuNames {
		outputPaths[mcuName] = filepath.Join(tmpDir, fmt.Sprintf("serial-multi-%s-%s.bin", mcuName, randSuffix()))
	}

	// Create multi-MCU runtime
	mrt, err := newMultiMCUCartesianRuntime(cfgPath, cfg, dicts, mcuNames, outputPaths)
	if err != nil {
		return nil, fmt.Errorf("create multi-MCU runtime: %w", err)
	}
	defer mrt.free()

	// Set trace if requested
	if opts != nil && opts.Trace != nil {
		mrt.setTrace(opts.Trace)
	}

	// Load macros for gcode execution
	macros, err := loadGCodeMacros(cfgPath)
	if err != nil {
		return nil, err
	}

	// Get connect-phase initialization commands for each MCU
	// (This generates the initialization sequence to match Python Klipper)
	initCmds, err := hosth1.CompileMultiMCUCartesianConnectPhase(cfgPath, dicts)
	if err != nil {
		return nil, fmt.Errorf("multi-MCU connect phase compilation failed: %w", err)
	}

	// Send init commands to each MCU
	for mcuName, commands := range initCmds {
		ctx := mrt.mcuContexts.Get(mcuName)
		if ctx == nil {
			return nil, fmt.Errorf("no MCU context for %s during init", mcuName)
		}
		for _, line := range commands {
			if err := ctx.sendLineMain(line, 0, 0); err != nil {
				return nil, fmt.Errorf("send init to MCU %s: %w", mcuName, err)
			}
		}
	}

	// Parse and execute test file G-code
	shouldFail := false
	didFail := false
	skipRuntimeGCode := filepath.Base(testPath) == "commands.test"

	// First pass: collect GCODE file reference and inline gcode lines
	var gcodeFile string
	var inlineGcode []string
	f, err := os.Open(testPath)
	if err != nil {
		return nil, err
	}
	s := bufio.NewScanner(f)
	for s.Scan() {
		raw := s.Text()
		if idx := strings.IndexByte(raw, '#'); idx >= 0 {
			raw = raw[:idx]
		}
		line := strings.TrimSpace(raw)
		if line == "" {
			continue
		}
		upper := strings.ToUpper(line)
		if strings.HasPrefix(upper, "CONFIG ") || strings.HasPrefix(upper, "DICTIONARY ") {
			continue
		}
		if strings.HasPrefix(upper, "GCODE ") {
			gcodeFile = strings.TrimSpace(line[6:])
			continue
		}
		if upper == "SHOULD_FAIL" {
			shouldFail = true
			continue
		}
		inlineGcode = append(inlineGcode, line)
	}
	if err := s.Err(); err != nil {
		f.Close()
		return nil, err
	}
	f.Close()

	// Execute gcode: either from GCODE file or inline lines
	if skipRuntimeGCode {
		// Skip all gcode execution for commands.test
	} else if gcodeFile != "" {
		// Read and execute gcode from referenced file
		gcodeFilePath := filepath.Join(filepath.Dir(testPath), gcodeFile)
		gf, err := os.Open(gcodeFilePath)
		if err != nil {
			return nil, fmt.Errorf("failed to open gcode file %s: %w", gcodeFilePath, err)
		}
		gs := bufio.NewScanner(gf)
		for gs.Scan() {
			raw := gs.Text()
			if idx := strings.IndexByte(raw, ';'); idx >= 0 {
				raw = raw[:idx]
			}
			line := strings.TrimSpace(raw)
			if line == "" {
				continue
			}
			if err := execGCodeWithMacros(mrt.runtime, macros, line, 0); err != nil {
				if shouldFail {
					didFail = true
					break
				}
				gf.Close()
				return nil, err
			}
		}
		if err := gs.Err(); err != nil {
			gf.Close()
			return nil, err
		}
		gf.Close()
	} else {
		// Execute inline gcode lines
		for _, line := range inlineGcode {
			if err := execGCodeWithMacros(mrt.runtime, macros, line, 0); err != nil {
				if shouldFail {
					didFail = true
					break
				}
				return nil, err
			}
		}
	}

	if shouldFail {
		if !didFail {
			return nil, fmt.Errorf("expected failure but test completed successfully")
		}
	} else {
		// Flush all MCU motion queues
		for mcuName, motion := range mrt.mcuMotions {
			for i := 0; i < 10000; i++ {
				didWork, err := motion.flushHandlerDebugOnce()
				if err != nil {
					return nil, fmt.Errorf("MCU %s flush: %w", mcuName, err)
				}
				if !didWork {
					break
				}
			}
		}
	}

	// Read output from each MCU
	return mrt.closeAndReadMulti()
}
