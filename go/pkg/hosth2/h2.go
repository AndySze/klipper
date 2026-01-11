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

// gcodeState holds the G-code coordinate system state for SAVE/RESTORE.
type gcodeState struct {
	name         string
	absoluteCoord bool
	absoluteExtrude bool
	position     [4]float64 // X, Y, Z, E
	offset       [4]float64 // G92 offset
	speed        float64
}

type machine struct {
	// Virtual time in seconds (fileoutput mode).
	now float64

	// G-code coordinate system state.
	absoluteCoord   bool       // G90 (true) / G91 (false)
	absoluteExtrude bool       // M82 (true) / M83 (false)
	position        [4]float64 // Current X, Y, Z, E position
	basePosition    [4]float64 // G92 offset (position = base + gcode)
	speed           float64    // Feedrate in mm/s
	speedFactor     float64    // Speed factor (default 1/60 for mm/min to mm/s)

	// Velocity limits (stored but not enforced in H2).
	velocity        float64
	accel           float64
	accelToDecel    float64
	squareCornerVel float64

	// Pressure advance (stored but not enforced in H2).
	pressureAdvance       float64
	smoothTime            float64

	// Saved state stack for SAVE_GCODE_STATE / RESTORE_GCODE_STATE.
	stateStack map[string]*gcodeState
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

	case "G90":
		// Absolute positioning mode.
		m.absoluteCoord = true
		return nil

	case "G91":
		// Relative positioning mode.
		m.absoluteCoord = false
		return nil

	case "M82":
		// Absolute extrusion mode.
		m.absoluteExtrude = true
		return nil

	case "M83":
		// Relative extrusion mode.
		m.absoluteExtrude = false
		return nil

	case "G92":
		// Set position (offset). Updates basePosition so that current gcode position becomes the given value.
		return m.execG92(cmd)

	case "G0", "G1":
		// Linear move (state tracking only in H2 - no actual MCU commands).
		return m.execG1(cmd)

	case "G28":
		// Homing - in H2, just reset position to 0.
		m.position = [4]float64{0, 0, 0, m.position[3]}
		m.basePosition = [4]float64{0, 0, 0, m.basePosition[3]}
		return nil

	case "SAVE_GCODE_STATE":
		return m.execSaveGcodeState(cmd)

	case "RESTORE_GCODE_STATE":
		return m.execRestoreGcodeState(cmd)

	case "SET_GCODE_OFFSET", "M206":
		return m.execSetGcodeOffset(cmd)

	case "SET_VELOCITY_LIMIT", "M204":
		return m.execSetVelocityLimit(cmd)

	case "SET_PRESSURE_ADVANCE":
		return m.execSetPressureAdvance(cmd)

	case "GET_POSITION", "M114", "STATUS", "HELP", "QUERY_ENDSTOPS", "M115", "M18", "M84":
		// Query/status commands - no-op in H2.
		return nil

	case "RESTART":
		// Signals end of test; stop execution loop at caller.
		return errRestart

	default:
		return fmt.Errorf("unsupported gcode %q (H2 bring-up): %q", cmd.Name, cmd.Raw)
	}
}

// execG92 handles G92 - set position offset.
func (m *machine) execG92(cmd *gcodeCommand) error {
	axes := []struct {
		key string
		idx int
	}{
		{"X", 0}, {"Y", 1}, {"Z", 2}, {"E", 3},
	}
	for _, ax := range axes {
		if v, ok, err := cmd.floatArg(ax.key, nil); err != nil {
			return err
		} else if ok {
			// G92 sets the offset so that current position becomes v.
			// basePosition[i] = position[i] - v
			m.basePosition[ax.idx] = m.position[ax.idx] - v
		}
	}
	return nil
}

// execG1 handles G0/G1 - linear move (state tracking only).
func (m *machine) execG1(cmd *gcodeCommand) error {
	axes := []struct {
		key      string
		idx      int
		absolute bool
	}{
		{"X", 0, m.absoluteCoord},
		{"Y", 1, m.absoluteCoord},
		{"Z", 2, m.absoluteCoord},
		{"E", 3, m.absoluteExtrude},
	}
	for _, ax := range axes {
		if v, ok, err := cmd.floatArg(ax.key, nil); err != nil {
			return err
		} else if ok {
			if ax.absolute {
				m.position[ax.idx] = v + m.basePosition[ax.idx]
			} else {
				m.position[ax.idx] += v
			}
		}
	}
	// Update feedrate if F is specified.
	if f, ok, err := cmd.floatArg("F", nil); err != nil {
		return err
	} else if ok && f > 0 {
		m.speed = f * m.speedFactor
	}
	return nil
}

// execSaveGcodeState handles SAVE_GCODE_STATE.
func (m *machine) execSaveGcodeState(cmd *gcodeCommand) error {
	name := "default"
	if n, ok := cmd.Args["NAME"]; ok && n != "" {
		name = n
	}
	if m.stateStack == nil {
		m.stateStack = make(map[string]*gcodeState)
	}
	m.stateStack[name] = &gcodeState{
		name:            name,
		absoluteCoord:   m.absoluteCoord,
		absoluteExtrude: m.absoluteExtrude,
		position:        m.position,
		offset:          m.basePosition,
		speed:           m.speed,
	}
	return nil
}

// execRestoreGcodeState handles RESTORE_GCODE_STATE.
func (m *machine) execRestoreGcodeState(cmd *gcodeCommand) error {
	name := "default"
	if n, ok := cmd.Args["NAME"]; ok && n != "" {
		name = n
	}
	if m.stateStack == nil {
		return nil // No saved states - treat as no-op like Python.
	}
	state, ok := m.stateStack[name]
	if !ok {
		return nil // State not found - treat as no-op.
	}
	m.absoluteCoord = state.absoluteCoord
	m.absoluteExtrude = state.absoluteExtrude
	m.position = state.position
	m.basePosition = state.offset
	m.speed = state.speed
	return nil
}

// execSetGcodeOffset handles SET_GCODE_OFFSET and M206.
func (m *machine) execSetGcodeOffset(cmd *gcodeCommand) error {
	// SET_GCODE_OFFSET [X=<pos>] [Y=<pos>] [Z=<pos>] [X_ADJUST=<delta>] ...
	axes := []struct {
		key    string
		adjust string
		idx    int
	}{
		{"X", "X_ADJUST", 0},
		{"Y", "Y_ADJUST", 1},
		{"Z", "Z_ADJUST", 2},
	}
	for _, ax := range axes {
		if v, ok, err := cmd.floatArg(ax.key, nil); err != nil {
			return err
		} else if ok {
			m.basePosition[ax.idx] = v
		}
		if v, ok, err := cmd.floatArg(ax.adjust, nil); err != nil {
			return err
		} else if ok {
			m.basePosition[ax.idx] += v
		}
	}
	return nil
}

// execSetVelocityLimit handles SET_VELOCITY_LIMIT and M204.
func (m *machine) execSetVelocityLimit(cmd *gcodeCommand) error {
	if v, ok, err := cmd.floatArg("VELOCITY", nil); err != nil {
		return err
	} else if ok {
		m.velocity = v
	}
	if v, ok, err := cmd.floatArg("ACCEL", nil); err != nil {
		return err
	} else if ok {
		m.accel = v
	}
	if v, ok, err := cmd.floatArg("ACCEL_TO_DECEL", nil); err != nil {
		return err
	} else if ok {
		m.accelToDecel = v
	}
	if v, ok, err := cmd.floatArg("SQUARE_CORNER_VELOCITY", nil); err != nil {
		return err
	} else if ok {
		m.squareCornerVel = v
	}
	return nil
}

// execSetPressureAdvance handles SET_PRESSURE_ADVANCE.
func (m *machine) execSetPressureAdvance(cmd *gcodeCommand) error {
	if v, ok, err := cmd.floatArg("ADVANCE", nil); err != nil {
		return err
	} else if ok {
		m.pressureAdvance = v
	}
	if v, ok, err := cmd.floatArg("SMOOTH_TIME", nil); err != nil {
		return err
	} else if ok {
		m.smoothTime = v
	}
	return nil
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

    m := &machine{
        absoluteCoord:   true,  // Default to absolute positioning (G90)
        absoluteExtrude: true,  // Default to absolute extrusion (M82)
        speedFactor:     1.0 / 60.0, // Convert mm/min to mm/s
        speed:           25.0,  // Default feedrate
        stateStack:      make(map[string]*gcodeState),
    }
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
