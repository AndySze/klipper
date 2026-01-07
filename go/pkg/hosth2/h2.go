package hosth2

import (
    "bufio"
    "fmt"
    "os"
    "path/filepath"
    "strings"

    "klipper-go-migration/pkg/hosth1"
    "klipper-go-migration/pkg/protocol"
)

type machine struct {
    // Virtual time in seconds (fileoutput mode).
    now float64
}

func (m *machine) exec(cmd *gcodeCommand) error {
    if cmd == nil {
        return nil
    }
    switch cmd.Name {
    case "G4":
        // Dwell. Klipper supports P(ms) and S(seconds).
        if p, ok, err := cmd.floatArg("P", nil); err != nil {
            return err
        } else if ok {
            m.now += p / 1000.0
            return nil
        }
        if s, ok, err := cmd.floatArg("S", nil); err != nil {
            return err
        } else if ok {
            m.now += s
            return nil
        }
        return fmt.Errorf("G4 missing P or S parameter")
    case "GET_POSITION", "M114", "STATUS", "HELP", "QUERY_ENDSTOPS", "M115", "M18":
        return nil
    case "G28", "SAVE_GCODE_STATE", "RESTORE_GCODE_STATE":
        // TODO(H2): implement stateful behavior; treat as no-op for now.
        return nil
    case "G92", "G91", "G1":
        // TODO(H2): implement gcode state + motion; treat as no-op for now.
        return nil
    case "SET_GCODE_OFFSET", "M206", "SET_VELOCITY_LIMIT", "M204", "SET_PRESSURE_ADVANCE":
        // TODO(H2): implement real behavior; keep as no-op during bring-up.
        return nil
    case "RESTART":
        // Signals end of test; stop execution loop at caller.
        return errRestart
    default:
        return fmt.Errorf("unsupported gcode %q (H2 bring-up): %q", cmd.Name, cmd.Raw)
    }
}

var errRestart = fmt.Errorf("restart")

// RunFromTestFile executes the gcode lines of a Klipper regression .test file.
// It is intentionally minimal for H2 bring-up.
func RunFromTestFile(testPath string) error {
    f, err := os.Open(testPath)
    if err != nil {
        return err
    }
    defer f.Close()

    m := &machine{}
    s := bufio.NewScanner(f)
    for s.Scan() {
        raw := s.Text()
        // Strip '#' comments used in .test files.
        if idx := strings.IndexByte(raw, '#'); idx >= 0 {
            raw = raw[:idx]
        }
        line := strings.TrimSpace(raw)
        if line == "" {
            continue
        }
        upper := strings.ToUpper(line)
        if strings.HasPrefix(upper, "CONFIG ") || strings.HasPrefix(upper, "DICTIONARY ") ||
            strings.HasPrefix(upper, "GCODE ") || upper == "SHOULD_FAIL" {
            continue
        }
        cmd, err := parseGCodeLine(line)
        if err != nil {
            return err
        }
        if err := m.exec(cmd); err != nil {
            if err == errRestart {
                return nil
            }
            return err
        }
    }
    if err := s.Err(); err != nil {
        return err
    }
    return nil
}

// CompileHostH2 compiles the connect-phase command stream and then executes
// the test file gcode. For now, the gcode runner does not emit additional MCU
// messages (it only advances internal time for commands like G4).
func CompileHostH2(cfgPath string, testPath string, dict *protocol.Dictionary) ([]string, error) {
    base := filepath.Base(cfgPath)
    switch base {
    case "linuxtest.cfg":
        lines, err := hosth1.CompileLinuxTestConnectPhase(cfgPath, dict)
        if err != nil {
            return nil, err
        }
        if err := RunFromTestFile(testPath); err != nil {
            return nil, err
        }
        return lines, nil
    case "example-cartesian.cfg":
        lines, err := hosth1.CompileExampleCartesianConnectPhase(cfgPath, dict)
        if err != nil {
            return nil, err
        }
        if err := RunFromTestFile(testPath); err != nil {
            return nil, err
        }
        return lines, nil
    default:
        return nil, fmt.Errorf("host-h2 unsupported config %q", base)
    }
}
