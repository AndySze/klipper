package protocol

import (
    "encoding/json"
    "fmt"
    "os"
    "sort"
    "strconv"
    "strings"
)

// Dictionary models the subset of fields we need from *.dict.
type Dictionary struct {
    Commands     map[string]int            `json:"commands"`
    Responses    map[string]int            `json:"responses"`
    Output       map[string]int            `json:"output"`
    Enumerations map[string]map[string]int `json:"enumerations"`
    Config       map[string]any            `json:"config"`
}

// LoadDictionary reads a Klipper dict JSON file.
func LoadDictionary(path string) (*Dictionary, error) {
    b, err := os.ReadFile(path)
    if err != nil {
        return nil, err
    }
    var raw map[string]any
    if err := json.Unmarshal(b, &raw); err != nil {
        return nil, fmt.Errorf("parse dict: %w", err)
    }
    d := &Dictionary{
        Commands:     map[string]int{},
        Responses:    map[string]int{},
        Output:       map[string]int{},
        Enumerations: map[string]map[string]int{},
        Config:       map[string]any{},
    }
    for k, v := range raw {
        switch k {
        case "commands":
            if err := decodeMsgIDMap(d.Commands, v); err != nil {
                return nil, err
            }
        case "responses":
            if err := decodeMsgIDMap(d.Responses, v); err != nil {
                return nil, err
            }
        case "output":
            if err := decodeMsgIDMap(d.Output, v); err != nil {
                return nil, err
            }
        case "enumerations":
            if err := decodeEnums(d.Enumerations, v); err != nil {
                return nil, err
            }
        case "config":
            if err := decodeConfig(d.Config, v); err != nil {
                return nil, err
            }
        }
    }
    return d, nil
}

func decodeMsgIDMap(dst map[string]int, v any) error {
    if v == nil {
        return nil
    }
    m, ok := v.(map[string]any)
    if !ok {
        return fmt.Errorf("expected object for messages")
    }
    for k, vv := range m {
        switch tv := vv.(type) {
        case float64:
            dst[k] = int(tv)
        default:
            return fmt.Errorf("bad msg id type for %s: %T", k, vv)
        }
    }
    return nil
}

// decodeEnums supports both simple values and ranges like [start,count].
func decodeEnums(dst map[string]map[string]int, v any) error {
    if v == nil {
        return nil
    }
    m, ok := v.(map[string]any)
    if !ok {
        return fmt.Errorf("expected object for enumerations")
    }
    for enumName, vv := range m {
        sub := dst[enumName]
        if sub == nil {
            sub = map[string]int{}
            dst[enumName] = sub
        }
        m2, ok := vv.(map[string]any)
        if !ok {
            return fmt.Errorf("expected object for enumeration %s", enumName)
        }
        for key, val := range m2 {
            switch tv := val.(type) {
            case float64:
                sub[key] = int(tv)
            case []any:
                if len(tv) != 2 {
                    return fmt.Errorf("bad enum range %s=%v", key, tv)
                }
                start, ok1 := tv[0].(float64)
                count, ok2 := tv[1].(float64)
                if !ok1 || !ok2 {
                    return fmt.Errorf("bad enum range types for %s", key)
                }
                keyRoot := key
                for len(keyRoot) > 0 && keyRoot[len(keyRoot)-1] >= '0' && keyRoot[len(keyRoot)-1] <= '9' {
                    keyRoot = keyRoot[:len(keyRoot)-1]
                }
                startEnum := 0
                if len(keyRoot) != len(key) {
                    n, err := strconv.Atoi(key[len(keyRoot):])
                    if err != nil {
                        return fmt.Errorf("bad enum range suffix %s", key)
                    }
                    startEnum = n
                }
                for i := 0; i < int(count); i++ {
                    sub[fmt.Sprintf("%s%d", keyRoot, startEnum+i)] = int(start) + i
                }
            default:
                return fmt.Errorf("bad enum value for %s: %T", key, val)
            }
        }
    }
    return nil
}

func decodeConfig(dst map[string]any, v any) error {
    if v == nil {
        return nil
    }
    m, ok := v.(map[string]any)
    if !ok {
        return fmt.Errorf("expected object for config")
    }
    for k, vv := range m {
        dst[k] = vv
    }
    return nil
}

// MessageFormat mirrors the key parts of Klipper's MessageFormat.
type MessageFormat struct {
    Name        string
    Format      string
    ID          int
    ParamNames  []string
    ParamTypes  []paramType
    DebugFormat string
}

type paramType interface {
    EncodeInt(out *[]byte, v int32)
    DecodeInt(buf []byte, pos int) (int32, int)
    IsDynamicString() bool
}

type dynString interface {
    EncodeBytes(out *[]byte, data []byte)
    DecodeBytes(buf []byte, pos int) ([]byte, int)
}

type ptUint struct{}
type ptInt struct{}
type ptByte struct{}
type ptEnum struct {
    Name   string
    Values map[string]int
    Rev    map[int]string
}
type ptBuffer struct{}

func (ptUint) EncodeInt(out *[]byte, v int32) { EncodeUint32(out, v) }
func (ptUint) DecodeInt(buf []byte, pos int) (int32, int) {
    return DecodeUint32(buf, pos)
}
func (ptUint) IsDynamicString() bool { return false }

func (ptInt) EncodeInt(out *[]byte, v int32) { EncodeUint32(out, v) }
func (ptInt) DecodeInt(buf []byte, pos int) (int32, int) {
    return DecodeUint32(buf, pos)
}
func (ptInt) IsDynamicString() bool { return false }

func (ptByte) EncodeInt(out *[]byte, v int32)             { EncodeUint32(out, v) }
func (ptByte) DecodeInt(buf []byte, pos int) (int32, int) { return DecodeUint32(buf, pos) }
func (ptByte) IsDynamicString() bool                   { return false }

func (pt ptEnum) EncodeInt(out *[]byte, v int32) {
    // v stores the already-enumerated integer
    EncodeUint32(out, v)
}
func (pt ptEnum) DecodeInt(buf []byte, pos int) (int32, int) { return DecodeUint32(buf, pos) }
func (pt ptEnum) IsDynamicString() bool                   { return false }

// PT_buffer: 1-byte length + bytes (no trailing padding)
func (ptBuffer) EncodeInt(out *[]byte, v int32) {
    // not used
}
func (ptBuffer) DecodeInt(buf []byte, pos int) (int32, int) { return 0, pos }
func (ptBuffer) IsDynamicString() bool                      { return true }
func (ptBuffer) EncodeBytes(out *[]byte, data []byte) {
    *out = append(*out, byte(len(data)))
    *out = append(*out, data...)
}
func (ptBuffer) DecodeBytes(buf []byte, pos int) ([]byte, int) {
    if pos >= len(buf) {
        return nil, pos
    }
    l := int(buf[pos])
    pos++
    end := pos + l
    if end > len(buf) {
        end = len(buf)
    }
    return buf[pos:end], end
}

func parseFormat(msgformat string, enums map[string]map[string]int) (names []string, types []paramType) {
    parts := strings.Fields(msgformat)
    if len(parts) <= 1 {
        return
    }
    args := parts[1:]
    for _, arg := range args {
        pair := strings.SplitN(arg, "=", 2)
        if len(pair) != 2 {
            continue
        }
        name, fmtSpec := pair[0], pair[1]
        var pt paramType
        switch fmtSpec {
        case "%u", "%hu":
            pt = ptUint{}
        case "%i", "%hi":
            pt = ptInt{}
        case "%c":
            pt = ptByte{}
        case "%s", "%.*s", "%*s":
            pt = ptBuffer{}
        default:
            pt = ptUint{} // fallback
        }
        for enumName, enumMap := range enums {
            if name == enumName || strings.HasSuffix(name, "_"+enumName) {
                rev := make(map[int]string, len(enumMap))
                for k, v := range enumMap {
                    rev[v] = k
                }
                pt = ptEnum{Name: enumName, Values: enumMap, Rev: rev}
                break
            }
        }
        names = append(names, name)
        types = append(types, pt)
    }
    return
}

func convertDebugFormat(msgformat string, ptypes []paramType) string {
    parts := strings.Fields(msgformat)
    if len(parts) <= 1 {
        return msgformat
    }
    out := []string{parts[0]}
    args := parts[1:]
    for i, arg := range args {
        fmtSpec := "%d"
        if i < len(ptypes) {
            if _, ok := ptypes[i].(ptEnum); ok {
                fmtSpec = "%s"
            }
        }
        if strings.Contains(arg, "=") {
            kv := strings.SplitN(arg, "=", 2)
            out = append(out, fmt.Sprintf("%s=%s", kv[0], fmtSpec))
        } else {
            out = append(out, fmtSpec)
        }
    }
    return strings.Join(out, " ")
}

// Build command formats from the dict.
func (d *Dictionary) BuildCommandFormats() (map[string]*MessageFormat, error) {
    out := make(map[string]*MessageFormat, len(d.Commands))
    // Deterministic order for debugging.
    var keys []string
    for k := range d.Commands {
        keys = append(keys, k)
    }
    sort.Strings(keys)
    for _, fmtStr := range keys {
        id := d.Commands[fmtStr]
        names, types := parseFormat(fmtStr, d.Enumerations)
        m := &MessageFormat{
            Name:        strings.Fields(fmtStr)[0],
            Format:      fmtStr,
            ID:          id,
            ParamNames:  names,
            ParamTypes:  types,
            DebugFormat: convertDebugFormat(fmtStr, types),
        }
        out[m.Name] = m
    }
    return out, nil
}
