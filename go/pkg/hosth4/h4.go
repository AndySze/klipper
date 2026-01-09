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
    if base != "example-cartesian.cfg" && base != "gcode_arcs.cfg" {
        return nil, fmt.Errorf("host-h4 only supports example-cartesian.cfg or gcode_arcs.cfg (got %s)", base)
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

    // Connect-phase + init commands (already strict-validated by host-h1).
    initLines, err := hosth1.CompileExampleCartesianConnectPhase(cfgPath, dict)
    if err != nil {
        return nil, err
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
        if err := rt.onEOF(); err != nil {
            return nil, err
        }
    }

    rawOut, err := rt.closeAndRead()
    if err != nil {
        return nil, err
    }
    return rawOut, nil
}
