package hosth2

import (
    "fmt"
    "regexp"
    "strconv"
    "strings"
)

type gcodeCommand struct {
    Name string
    Args map[string]string
    Raw  string
}

var (
    reParenComment = regexp.MustCompile(`\([^)]*\)`)
)

func parseGCodeLine(line string) (*gcodeCommand, error) {
    ln := strings.TrimSpace(line)
    if ln == "" {
        return nil, nil
    }
    if idx := strings.IndexByte(ln, ';'); idx >= 0 {
        ln = strings.TrimSpace(ln[:idx])
    }
    if ln == "" {
        return nil, nil
    }
    ln = strings.TrimSpace(reParenComment.ReplaceAllString(ln, " "))
    if ln == "" {
        return nil, nil
    }

    fields := strings.Fields(ln)
    if len(fields) == 0 {
        return nil, nil
    }
    name := strings.ToUpper(fields[0])
    args := map[string]string{}
    for _, f := range fields[1:] {
        if f == "" {
            continue
        }
        if strings.Contains(f, "=") {
            kv := strings.SplitN(f, "=", 2)
            k := strings.ToUpper(strings.TrimSpace(kv[0]))
            v := ""
            if len(kv) > 1 {
                v = strings.TrimSpace(kv[1])
            }
            if k != "" {
                args[k] = v
            }
            continue
        }
        // Letter-params like "P1000", "Z-5", "E0".
        if len(f) < 2 {
            continue
        }
        k := strings.ToUpper(f[:1])
        v := strings.TrimSpace(f[1:])
        if k != "" {
            args[k] = v
        }
    }
    return &gcodeCommand{Name: name, Args: args, Raw: line}, nil
}

func (c *gcodeCommand) floatArg(key string, def *float64) (float64, bool, error) {
    v, ok := c.Args[strings.ToUpper(key)]
    if !ok {
        if def == nil {
            return 0, false, nil
        }
        return *def, true, nil
    }
    if v == "" {
        return 0, true, fmt.Errorf("empty arg %s in %q", key, c.Raw)
    }
    f, err := strconv.ParseFloat(v, 64)
    if err != nil {
        return 0, true, fmt.Errorf("bad float %s=%q in %q", key, v, c.Raw)
    }
    return f, true, nil
}

