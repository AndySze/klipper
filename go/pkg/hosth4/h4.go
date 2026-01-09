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
// msg stream. H4 focuses on cartesian kinematics (G28 homing + bounds checks).
//
// On "expected failure" tests, callers may need to accept a non-nil error while
// still using any produced output (not implemented yet).
func CompileHostH4(cfgPath string, testPath string, dict *protocol.Dictionary, opts *CompileOptions) ([]byte, error) {
    base := filepath.Base(cfgPath)
    // Support known cartesian kinematics test configs
    allowedConfigs := map[string]bool{
        "example-cartesian.cfg": true,
        "gcode_arcs.cfg":        true,
        "extruders.cfg":         true,
        "pressure_advance.cfg":  true,
        "bed_screws.cfg":        true,
        "out_of_bounds.cfg":     true,
    }
    if !allowedConfigs[base] {
        return nil, fmt.Errorf("host-h4: unsupported config %s (only cartesian configs supported)", base)
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
        return nil, fmt.Errorf("host-h4 only supports kinematics=cartesian (got %q)", kin)
    }

    rt, err := newRuntime(cfgPath, dict, cfg)
    if err != nil {
        return nil, err
    }
    defer rt.free()
    if opts != nil && opts.Trace != nil {
        rt.setTrace(opts.Trace)
    }

    // Connect-phase + init commands.
    // Check if config has heater_bed section, extruder, and extruder_stepper to determine which compiler to use.
    _, hasBedHeater := cfg.section("heater_bed")
    _, hasExtruder := cfg.section("extruder")
    _, hasExtraStepper := cfg.section("extruder_stepper my_extra_stepper")

    var initLines []string
    if hasBedHeater {
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

    for _, line := range initLines {
        if err := rt.sendLine(line, rt.cqMain, 0, 0); err != nil {
            return nil, err
        }
    }

    shouldFail := false
    didFail := false
    f, err := os.Open(testPath)
    if err != nil {
        return nil, err
    }
    defer f.Close()

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
        if strings.HasPrefix(upper, "CONFIG ") || strings.HasPrefix(upper, "DICTIONARY ") ||
            strings.HasPrefix(upper, "GCODE ") {
            continue
        }
        if upper == "SHOULD_FAIL" {
            shouldFail = true
            continue
        }
        cmd, err := parseGCodeLine(line)
        if err != nil {
            return nil, err
        }
        if err := rt.exec(cmd); err != nil {
            if shouldFail {
                didFail = true
                break
            }
            return nil, err
        }
    }
    if err := s.Err(); err != nil {
        return nil, err
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
        // Note: Python Klipper does not call motor_off at EOF in normal test execution
        // Motor-off is only called in specific scenarios (e.g., shutdown, config reload)
        // if err := rt.onEOF(); err != nil {
        //     return nil, err
        // }
    }

    rawOut, err := rt.closeAndRead()
    if err != nil {
        return nil, err
    }
    return rawOut, nil
}
