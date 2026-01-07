package hosth3

import (
    "bufio"
    "fmt"
    "os"
    "path/filepath"
    "sort"
    "strconv"
    "strings"
)

type config struct {
    sections map[string]map[string]string
}

func loadConfig(path string) (*config, error) {
    c := &config{sections: map[string]map[string]string{}}
    abs, err := filepath.Abs(path)
    if err != nil {
        return nil, err
    }
    f, err := os.Open(abs)
    if err != nil {
        return nil, err
    }
    defer f.Close()
    var cur string
    s := bufio.NewScanner(f)
    for s.Scan() {
        line := strings.TrimSpace(s.Text())
        if line == "" {
            continue
        }
        if idx := strings.IndexByte(line, '#'); idx >= 0 {
            line = strings.TrimSpace(line[:idx])
            if line == "" {
                continue
            }
        }
        if strings.HasPrefix(line, "[") && strings.HasSuffix(line, "]") {
            cur = strings.TrimSpace(line[1 : len(line)-1])
            if cur == "" {
                return nil, fmt.Errorf("empty section header")
            }
            if _, ok := c.sections[cur]; !ok {
                c.sections[cur] = map[string]string{}
            }
            continue
        }
        if cur == "" {
            continue
        }
        kv := strings.SplitN(line, ":", 2)
        if len(kv) != 2 {
            kv = strings.SplitN(line, "=", 2)
        }
        if len(kv) != 2 {
            continue
        }
        k := strings.TrimSpace(kv[0])
        v := strings.TrimSpace(kv[1])
        if k == "" {
            continue
        }
        c.sections[cur][k] = v
    }
    if err := s.Err(); err != nil {
        return nil, err
    }
    return c, nil
}

func (c *config) section(name string) (map[string]string, bool) {
    sec, ok := c.sections[name]
    return sec, ok
}

func parseFloat(sec map[string]string, key string, def *float64) (float64, error) {
    raw := strings.TrimSpace(sec[key])
    if raw == "" {
        if def == nil {
            return 0, fmt.Errorf("missing %s", key)
        }
        return *def, nil
    }
    v, err := strconv.ParseFloat(raw, 64)
    if err != nil {
        return 0, fmt.Errorf("bad float %s=%q", key, raw)
    }
    return v, nil
}

func parseInt(sec map[string]string, key string, def *int) (int, error) {
    raw := strings.TrimSpace(sec[key])
    if raw == "" {
        if def == nil {
            return 0, fmt.Errorf("missing %s", key)
        }
        return *def, nil
    }
    v, err := strconv.Atoi(raw)
    if err != nil {
        return 0, fmt.Errorf("bad int %s=%q", key, raw)
    }
    return v, nil
}

type pin struct {
    pin    string
    invert bool
    pullup int
}

func parsePinDesc(desc string, canInvert bool, canPullup bool) (pin, error) {
    d := strings.TrimSpace(desc)
    p := pin{}
    if canPullup && (strings.HasPrefix(d, "^") || strings.HasPrefix(d, "~")) {
        p.pullup = 1
        if strings.HasPrefix(d, "~") {
            p.pullup = -1
        }
        d = strings.TrimSpace(d[1:])
    }
    if canInvert && strings.HasPrefix(d, "!") {
        p.invert = true
        d = strings.TrimSpace(d[1:])
    }
    if strings.Contains(d, ":") {
        parts := strings.SplitN(d, ":", 2)
        chip := strings.TrimSpace(parts[0])
        if chip != "mcu" {
            return pin{}, fmt.Errorf("unsupported pin chip %q in %q", chip, desc)
        }
        d = strings.TrimSpace(parts[1])
    }
    if strings.ContainsAny(d, "^~!:") || strings.Join(strings.Fields(d), "") != d {
        return pin{}, fmt.Errorf("invalid pin %q", desc)
    }
    p.pin = d
    return p, nil
}

func parsePin(sec map[string]string, key string, canInvert bool, canPullup bool) (pin, error) {
    raw := strings.TrimSpace(sec[key])
    if raw == "" {
        return pin{}, fmt.Errorf("missing %s", key)
    }
    return parsePinDesc(raw, canInvert, canPullup)
}

type manualStepperCfg struct {
    name             string
    stepPin          pin
    dirPin           pin
    enablePin        pin
    endstopPin       *pin
    microsteps       int
    fullSteps        int
    rotationDistance float64
    velocity         float64
    accel            float64
}

func readManualSteppers(cfg *config) ([]manualStepperCfg, error) {
    var names []string
    for sec := range cfg.sections {
        if strings.HasPrefix(sec, "manual_stepper ") {
            names = append(names, sec)
        }
    }
    sort.Strings(names)
    out := make([]manualStepperCfg, 0, len(names))
    for _, secName := range names {
        sec := cfg.sections[secName]
        stepPin, err := parsePin(sec, "step_pin", true, false)
        if err != nil {
            return nil, fmt.Errorf("[%s] %v", secName, err)
        }
        dirPin, err := parsePin(sec, "dir_pin", true, false)
        if err != nil {
            return nil, fmt.Errorf("[%s] %v", secName, err)
        }
        enablePin, err := parsePin(sec, "enable_pin", true, false)
        if err != nil {
            return nil, fmt.Errorf("[%s] %v", secName, err)
        }
        microsteps, err := parseInt(sec, "microsteps", nil)
        if err != nil {
            return nil, fmt.Errorf("[%s] %v", secName, err)
        }
        defFull := 200
        fullSteps, err := parseInt(sec, "full_steps_per_rotation", &defFull)
        if err != nil {
            return nil, fmt.Errorf("[%s] %v", secName, err)
        }
        rotationDistance, err := parseFloat(sec, "rotation_distance", nil)
        if err != nil {
            return nil, fmt.Errorf("[%s] %v", secName, err)
        }
        defVel := 5.0
        velocity, err := parseFloat(sec, "velocity", &defVel)
        if err != nil {
            return nil, fmt.Errorf("[%s] %v", secName, err)
        }
        defAccel := 0.0
        accel, err := parseFloat(sec, "accel", &defAccel)
        if err != nil {
            return nil, fmt.Errorf("[%s] %v", secName, err)
        }
        var endstop *pin
        if raw := strings.TrimSpace(sec["endstop_pin"]); raw != "" {
            p, err := parsePinDesc(raw, true, true)
            if err != nil {
                return nil, fmt.Errorf("[%s] bad endstop_pin: %v", secName, err)
            }
            endstop = &p
        }
        out = append(out, manualStepperCfg{
            name:             secName,
            stepPin:          stepPin,
            dirPin:           dirPin,
            enablePin:        enablePin,
            endstopPin:       endstop,
            microsteps:       microsteps,
            fullSteps:        fullSteps,
            rotationDistance: rotationDistance,
            velocity:         velocity,
            accel:            accel,
        })
    }
    if len(out) == 0 {
        return nil, fmt.Errorf("no [manual_stepper ...] sections found")
    }
    return out, nil
}

