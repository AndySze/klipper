package hosth4

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

func floatArg(args map[string]string, key string, def float64) (float64, error) {
    raw, ok := args[strings.ToUpper(key)]
    if !ok {
        return def, nil
    }
    if raw == "" {
        return 0, fmt.Errorf("empty arg %s", key)
    }
    f, err := strconv.ParseFloat(raw, 64)
    if err != nil {
        return 0, fmt.Errorf("bad float %s=%q", key, raw)
    }
    return f, nil
}

