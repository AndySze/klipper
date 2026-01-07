package protocol

import (
    "fmt"
    "strconv"
    "strings"
)

// EncodeCommand parses a human-readable command line (e.g.
// "config_digital_out oid=1 pin=PA1 value=0") into raw bytes.
func EncodeCommand(cmdFormats map[string]*MessageFormat, line string) ([]byte, error) {
    fields := strings.Fields(strings.TrimSpace(line))
    if len(fields) == 0 {
        return nil, fmt.Errorf("empty command")
    }
    name := fields[0]
    m, ok := cmdFormats[name]
    if !ok {
        return nil, fmt.Errorf("unknown command %q", name)
    }
    args := map[string]string{}
    for _, f := range fields[1:] {
        kv := strings.SplitN(f, "=", 2)
        if len(kv) != 2 {
            return nil, fmt.Errorf("invalid arg %q", f)
        }
        args[kv[0]] = kv[1]
    }
    out := []byte{}
    EncodeUint32(&out, int32(m.ID))
    for idx, pname := range m.ParamNames {
        raw, ok := args[pname]
        if !ok {
            return nil, fmt.Errorf("missing arg %q", pname)
        }
        pt := m.ParamTypes[idx]
        switch t := pt.(type) {
        case ptEnum:
            val, ok := t.Values[raw]
            if !ok {
                return nil, fmt.Errorf("unknown enum %q for %s", raw, pname)
            }
            pt.EncodeInt(&out, int32(val))
        case ptInt:
            v, err := strconv.ParseInt(raw, 0, 32)
            if err != nil {
                return nil, fmt.Errorf("bad int %q for %s: %w", raw, pname, err)
            }
            pt.EncodeInt(&out, int32(v))
        case ptBuffer:
            b, err := parseBufferLiteral(raw)
            if err != nil {
                return nil, fmt.Errorf("bad buffer %q for %s: %w", raw, pname, err)
            }
            t.EncodeBytes(&out, b)
        default:
            v, err := strconv.ParseUint(raw, 0, 32)
            if err != nil {
                return nil, fmt.Errorf("bad int %q for %s: %w", raw, pname, err)
            }
            pt.EncodeInt(&out, int32(v))
        }
    }
    return out, nil
}

// DecodeCommand decodes bytes back to a human-readable line.
func DecodeCommand(cmdFormats map[string]*MessageFormat, data []byte) (string, error) {
    if len(data) == 0 {
        return "", fmt.Errorf("empty data")
    }
    msgid, pos := DecodeUint32(data, 0)
    var m *MessageFormat
    for _, v := range cmdFormats {
        if v.ID == int(msgid) {
            m = v
            break
        }
    }
    if m == nil {
        return "", fmt.Errorf("unknown msgid %d", msgid)
    }
    parts := []string{m.Name}
    for idx, pt := range m.ParamTypes {
        var sval string
        switch t := pt.(type) {
        case ptEnum:
            v, np := pt.DecodeInt(data, pos)
            pos = np
            if name, ok := t.Rev[int(v)]; ok {
                sval = name
            } else {
                sval = fmt.Sprintf("?%d", v)
            }
        case ptInt:
            v, np := pt.DecodeInt(data, pos)
            pos = np
            sval = fmt.Sprintf("%d", v)
        case ptBuffer:
            b, np := t.DecodeBytes(data, pos)
            pos = np
            sval = reprBytes(b)
        default: // ptUint/ptByte
            v, np := pt.DecodeInt(data, pos)
            pos = np
            sval = fmt.Sprintf("%d", uint32(v))
        }
        pname := m.ParamNames[idx]
        parts = append(parts, fmt.Sprintf("%s=%s", pname, sval))
    }
    return strings.Join(parts, " "), nil
}

// parseBufferLiteral accepts forms like b'1234' or b\"...\".
func parseBufferLiteral(raw string) ([]byte, error) {
    raw = strings.TrimSpace(raw)
    if len(raw) >= 3 && raw[0] == 'b' && (raw[1] == '\'' || raw[1] == '"') {
        quote := raw[1]
        if raw[len(raw)-1] != quote {
            return nil, fmt.Errorf("unterminated buffer literal")
        }
        body := raw[2 : len(raw)-1]
        return parsePythonBytesBody(body)
    }
    return nil, fmt.Errorf("unsupported buffer literal %q", raw)
}

func parsePythonBytesBody(body string) ([]byte, error) {
    out := make([]byte, 0, len(body))
    for i := 0; i < len(body); i++ {
        c := body[i]
        if c != '\\' {
            out = append(out, c)
            continue
        }
        if i+1 >= len(body) {
            return nil, fmt.Errorf("dangling escape")
        }
        i++
        esc := body[i]
        switch esc {
        case '\\', '\'', '"':
            out = append(out, esc)
        case 'n':
            out = append(out, '\n')
        case 'r':
            out = append(out, '\r')
        case 't':
            out = append(out, '\t')
        case 'a':
            out = append(out, '\a')
        case 'b':
            out = append(out, '\b')
        case 'f':
            out = append(out, '\f')
        case 'v':
            out = append(out, '\v')
        case 'x':
            if i+2 >= len(body) {
                return nil, fmt.Errorf("short \\x escape")
            }
            hi, ok1 := fromHex(body[i+1])
            lo, ok2 := fromHex(body[i+2])
            if !ok1 || !ok2 {
                return nil, fmt.Errorf("invalid \\x escape")
            }
            out = append(out, byte(hi<<4|lo))
            i += 2
        default:
            return nil, fmt.Errorf("unsupported escape: \\%c", esc)
        }
    }
    return out, nil
}

func fromHex(c byte) (byte, bool) {
    switch {
    case c >= '0' && c <= '9':
        return c - '0', true
    case c >= 'a' && c <= 'f':
        return c - 'a' + 10, true
    case c >= 'A' && c <= 'F':
        return c - 'A' + 10, true
    default:
        return 0, false
    }
}

func reprBytes(b []byte) string {
    // Mimic Python bytes repr (including Python's quote selection rules).
    //
    // Examples:
    //   repr(b"'")  == `b"'"`
    //   repr(b'"')  == `b'"'`
    //   repr(b"\n") == `b'\\n'`
    hasSingle := false
    hasDouble := false
    for _, c := range b {
        if c == '\'' {
            hasSingle = true
        } else if c == '"' {
            hasDouble = true
        }
    }
    quote := byte('\'')
    if hasSingle && !hasDouble {
        quote = '"'
    }
    var sb strings.Builder
    sb.WriteString("b")
    sb.WriteByte(quote)
    for _, c := range b {
        switch c {
        case '\n':
            sb.WriteString("\\n")
        case '\r':
            sb.WriteString("\\r")
        case '\t':
            sb.WriteString("\\t")
        case '\a':
            sb.WriteString("\\a")
        case '\b':
            sb.WriteString("\\b")
        case '\f':
            sb.WriteString("\\f")
        case '\v':
            sb.WriteString("\\v")
        case '\\':
            sb.WriteString("\\\\")
        default:
            if c == quote {
                sb.WriteByte('\\')
                sb.WriteByte(c)
                break
            }
            if c >= 0x20 && c <= 0x7e {
                sb.WriteByte(c)
            } else {
                sb.WriteString(fmt.Sprintf("\\x%02x", c))
            }
        }
    }
    sb.WriteByte(quote)
    return sb.String()
}
