package hosth3

import (
    "bufio"
    "fmt"
    "os"
    "path/filepath"
    "strconv"
    "strings"

    "klipper-go-migration/pkg/chelper"
    "klipper-go-migration/pkg/protocol"
)

func CompileHostH3(cfgPath string, testPath string, dict *protocol.Dictionary) ([]byte, error) {
    if filepath.Base(cfgPath) != "manual_stepper.cfg" {
        return nil, fmt.Errorf("host-h3 only supports manual_stepper.cfg (got %s)", filepath.Base(cfgPath))
    }
    cfg, err := loadConfig(cfgPath)
    if err != nil {
        return nil, err
    }
    rt, err := newRuntime(cfgPath, dict, cfg)
    if err != nil {
        return nil, err
    }

    msCfgs, err := readManualSteppers(cfg)
    if err != nil {
        return nil, err
    }

    // Fixed OID layout matches the Python reference for manual_stepper.cfg.
    // - basic_stepper: stepper=0 enable=4
    // - homing_stepper: endstop=1 trsync=2 stepper=3 enable=5
    var basic manualStepperCfg
    var homing manualStepperCfg
    for _, m := range msCfgs {
        if strings.HasSuffix(m.name, "basic_stepper") {
            basic = m
        } else if strings.HasSuffix(m.name, "homing_stepper") {
            homing = m
        }
    }
    if basic.name == "" || homing.name == "" {
        return nil, fmt.Errorf("expected both basic_stepper and homing_stepper sections")
    }

    if homing.endstopPin == nil {
        return nil, fmt.Errorf("homing_stepper missing endstop_pin")
    }

    // Create enable-pin command queues.
    cqEnableBasic := chelper.NewCommandQueue()
    if cqEnableBasic == nil {
        return nil, fmt.Errorf("alloc enable command queue (basic) failed")
    }
    rt.cqEnPins[basic.name] = cqEnableBasic
    cqEnableHoming := chelper.NewCommandQueue()
    if cqEnableHoming == nil {
        return nil, fmt.Errorf("alloc enable command queue (homing) failed")
    }
    rt.cqEnPins[homing.name] = cqEnableHoming

    // Create steppers.
    basicStepsPerRotation := float64(basic.fullSteps * basic.microsteps)
    homingStepsPerRotation := float64(homing.fullSteps * homing.microsteps)
    basicStepDist := basic.rotationDistance / basicStepsPerRotation
    homingStepDist := homing.rotationDistance / homingStepsPerRotation

    stBasic, err := newStepper(rt.motion, rt.sq, "basic", 0, basicStepDist, basic.dirPin.invert, rt.queueStepID, rt.setDirID, rt.mcuFreq)
    if err != nil {
        return nil, err
    }
    stHoming, err := newStepper(rt.motion, rt.sq, "homing", 3, homingStepDist, homing.dirPin.invert, rt.queueStepID, rt.setDirID, rt.mcuFreq)
    if err != nil {
        stBasic.free()
        return nil, err
    }
    rt.steppers[basic.name] = stBasic
    rt.steppers[homing.name] = stHoming

    // Allocate manual-stepper trapqs and wire them into steppers.
    tqBasic, err := rt.motion.allocTrapQ()
    if err != nil {
        return nil, err
    }
    tqHoming, err := rt.motion.allocTrapQ()
    if err != nil {
        return nil, err
    }
    stBasic.setTrapQ(tqBasic)
    stHoming.setTrapQ(tqHoming)

    // Setup enable pins (digital_out commands are emitted at runtime).
    enBasic := &stepperEnablePin{
        out:         newDigitalOut(4, basic.enablePin.invert, rt.sq, cqEnableBasic, rt.formats, rt.mcuFreq),
        enableCount: 0,
        isDedicated: true,
    }
    enHoming := &stepperEnablePin{
        out:         newDigitalOut(5, homing.enablePin.invert, rt.sq, cqEnableHoming, rt.formats, rt.mcuFreq),
        enableCount: 0,
        isDedicated: true,
    }
    rt.stepperEnable.registerStepper(basic.name, stBasic, enBasic)
    rt.stepperEnable.registerStepper(homing.name, stHoming, enHoming)
    rt.forceMove.registerStepper(basic.name, stBasic)
    rt.forceMove.registerStepper(homing.name, stHoming)

    // Create manual stepper objects.
    msBasic := newManualStepper(basic.name, stBasic, rt.toolhead, rt.motion, tqBasic, basic.velocity, basic.accel)
    msHoming := newManualStepper(homing.name, stHoming, rt.toolhead, rt.motion, tqHoming, homing.velocity, homing.accel)
    rt.manuals["basic_stepper"] = msBasic
    rt.manuals["homing_stepper"] = msHoming

    // Send connect-phase commands.
    stepPulseTicks := uint64(int64(rt.mcuFreq * 0.000002))
    configCmds := []string{
        fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
            0, basic.stepPin.pin, basic.dirPin.pin, boolToInt(basic.stepPin.invert), stepPulseTicks),
        fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
            4, basic.enablePin.pin, boolToInt(basic.enablePin.invert), boolToInt(basic.enablePin.invert), 0),
        fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d",
            1, homing.endstopPin.pin, homing.endstopPin.pullup),
        fmt.Sprintf("config_trsync oid=%d", 2),
        fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
            3, homing.stepPin.pin, homing.dirPin.pin, boolToInt(homing.stepPin.invert), stepPulseTicks),
        fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
            5, homing.enablePin.pin, boolToInt(homing.enablePin.invert), boolToInt(homing.enablePin.invert), 0),
    }
    withAllocate := append([]string{"allocate_oids count=6"}, configCmds...)
    crc := computeFinalizeCRC(withAllocate)
    for _, line := range withAllocate {
        if err := rt.sendLine(line, rt.cqMain, 0, 0); err != nil {
            return nil, err
        }
    }
    if err := rt.sendLine(fmt.Sprintf("finalize_config crc=%d", crc), rt.cqMain, 0, 0); err != nil {
        return nil, err
    }

    gm := newGCodeMove(rt.toolhead)

    // Execute the test file.
    f, err := os.Open(testPath)
    if err != nil {
        return nil, err
    }
    defer f.Close()

    s := bufio.NewScanner(f)
    done := false
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
            strings.HasPrefix(upper, "GCODE ") || upper == "SHOULD_FAIL" {
            continue
        }

        name, args, err := parseCmd(line)
        if err != nil {
            return nil, err
        }
        switch name {
        case "MANUAL_STEPPER":
            stName := args["STEPPER"]
            ms := rt.manuals[stName]
            if ms == nil {
                return nil, fmt.Errorf("unknown manual stepper %q", stName)
            }
            if axis, ok := args["GCODE_AXIS"]; ok {
                icv := floatArg(args, "INSTANTANEOUS_CORNER_VELOCITY", 1.0)
                lv := floatArg(args, "LIMIT_VELOCITY", 999999.9)
                la := floatArg(args, "LIMIT_ACCEL", 999999.9)
                if err := ms.commandWithGcodeAxis(axis, icv, lv, la, rt.toolhead); err != nil {
                    return nil, err
                }
                gm.updateExtraAxes()
                continue
            }
            if en, ok := args["ENABLE"]; ok {
                v := en != "0"
                if err := ms.doEnable(v, rt.stepperEnable); err != nil {
                    return nil, err
                }
            }
            if sp, ok := args["SET_POSITION"]; ok {
                v, err := strconv.ParseFloat(sp, 64)
                if err != nil {
                    return nil, fmt.Errorf("bad SET_POSITION=%q", sp)
                }
                if err := ms.doSetPosition(v); err != nil {
                    return nil, err
                }
            }
            if mv, ok := args["MOVE"]; ok {
                pos, err := strconv.ParseFloat(mv, 64)
                if err != nil {
                    return nil, fmt.Errorf("bad MOVE=%q", mv)
                }
                speed := ms.velocity
                if v, ok := args["SPEED"]; ok {
                    speed, err = strconv.ParseFloat(v, 64)
                    if err != nil {
                        return nil, fmt.Errorf("bad SPEED=%q", v)
                    }
                }
                accel := ms.accel
                if v, ok := args["ACCEL"]; ok {
                    accel, err = strconv.ParseFloat(v, 64)
                    if err != nil {
                        return nil, fmt.Errorf("bad ACCEL=%q", v)
                    }
                }
                sync := true
                if v, ok := args["SYNC"]; ok && v == "0" {
                    sync = false
                }
                if err := ms.doMove(pos, speed, accel, sync); err != nil {
                    return nil, err
                }
            } else if v, ok := args["SYNC"]; ok && v != "0" {
                if err := ms.syncPrintTime(); err != nil {
                    return nil, err
                }
            }
        case "M84", "M18":
            if err := rt.stepperEnable.motorOff(); err != nil {
                return nil, err
            }
        case "STEPPER_BUZZ":
            stepperName := args["STEPPER"]
            if stepperName == "" {
                return nil, fmt.Errorf("STEPPER_BUZZ missing STEPPER")
            }
            if err := rt.forceMove.stepperBuzz(stepperName); err != nil {
                return nil, err
            }
        case "G28":
            if err := gm.cmdG28(); err != nil {
                return nil, err
            }
        case "G0", "G1":
            if err := gm.cmdG1(args); err != nil {
                return nil, err
            }
        case "GET_POSITION", "M114":
            if _, err := rt.toolhead.getLastMoveTime(); err != nil {
                return nil, err
            }
        case "RESTART":
            done = true
        default:
            return nil, fmt.Errorf("unsupported test command %q: %q", name, line)
        }
        if done {
            break
        }
    }
    if err := s.Err(); err != nil {
        return nil, err
    }

    // Klipper's fileinput mode requests an "exit" restart on EOF, which
    // triggers gcode:request_restart handlers (notably stepper motor off).
    if _, err := rt.toolhead.getLastMoveTime(); err != nil {
        return nil, err
    }
    if err := rt.stepperEnable.motorOff(); err != nil {
        return nil, err
    }
    if err := rt.toolhead.dwell(0.500); err != nil {
        return nil, err
    }
    rawOut, err := rt.closeAndRead()
    if err != nil {
        return nil, err
    }
    _ = stBasic
    _ = stHoming
    return rawOut, nil
}

func boolToInt(v bool) int {
    if v {
        return 1
    }
    return 0
}

func floatArg(args map[string]string, key string, def float64) float64 {
    raw, ok := args[key]
    if !ok || raw == "" {
        return def
    }
    v, err := strconv.ParseFloat(raw, 64)
    if err != nil {
        return def
    }
    return v
}

func parseCmd(line string) (string, map[string]string, error) {
    toks, err := splitFieldsQuoted(line)
    if err != nil {
        return "", nil, err
    }
    if len(toks) == 0 {
        return "", nil, nil
    }
    name := strings.ToUpper(toks[0])
    args := map[string]string{}
    for _, t := range toks[1:] {
        if strings.Contains(t, "=") {
            kv := strings.SplitN(t, "=", 2)
            k := strings.ToUpper(strings.TrimSpace(kv[0]))
            v := ""
            if len(kv) > 1 {
                v = strings.TrimSpace(kv[1])
            }
            v = trimQuotes(v)
            if k != "" {
                args[k] = v
            }
            continue
        }
        if len(t) >= 2 {
            k := strings.ToUpper(t[:1])
            v := strings.TrimSpace(t[1:])
            if k != "" {
                args[k] = trimQuotes(v)
            }
        }
    }
    return name, args, nil
}

func trimQuotes(v string) string {
    if len(v) >= 2 && ((v[0] == '"' && v[len(v)-1] == '"') || (v[0] == '\'' && v[len(v)-1] == '\'')) {
        return v[1 : len(v)-1]
    }
    return v
}

func splitFieldsQuoted(s string) ([]string, error) {
    var out []string
    var cur strings.Builder
    inQuote := byte(0)
    escaped := false
    for i := 0; i < len(s); i++ {
        c := s[i]
        if escaped {
            cur.WriteByte(c)
            escaped = false
            continue
        }
        if c == '\\' && inQuote != 0 {
            escaped = true
            continue
        }
        if inQuote != 0 {
            if c == inQuote {
                inQuote = 0
                continue
            }
            cur.WriteByte(c)
            continue
        }
        if c == '"' || c == '\'' {
            inQuote = c
            continue
        }
        if c == ' ' || c == '\t' || c == '\n' || c == '\r' {
            if cur.Len() > 0 {
                out = append(out, cur.String())
                cur.Reset()
            }
            continue
        }
        cur.WriteByte(c)
    }
    if inQuote != 0 {
        return nil, fmt.Errorf("unterminated quote in %q", s)
    }
    if escaped {
        return nil, fmt.Errorf("dangling escape in %q", s)
    }
    if cur.Len() > 0 {
        out = append(out, cur.String())
    }
    return out, nil
}
