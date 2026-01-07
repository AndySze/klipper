package protocol

import (
    "fmt"
    "strings"
)

type messageKind int

const (
    messageKindUnknown messageKind = iota
    messageKindCommandOrResponse
    messageKindOutput
)

type messageDecoder struct {
    kind   messageKind
    msg    *MessageFormat
    output *OutputFormat
}

// OutputFormat mirrors the key parts of Klipper's OutputFormat in msgproto.py.
// It is used for decoding messages whose msgid is in the dict's "output" set.
type OutputFormat struct {
    ID         int
    MsgFormat  string
    ParamTypes []paramType
}

// BuildParsedumpDecoders builds a msgid->decoder map suitable for decoding the
// binary debugoutput files produced by Klippy tests (klippy/parsedump.py).
func (d *Dictionary) BuildParsedumpDecoders() (map[int]messageDecoder, error) {
    byID := map[int]messageDecoder{}

    for fmtStr, id := range d.Commands {
        msg := buildMessageFormat(fmtStr, id, d.Enumerations)
        byID[id] = messageDecoder{kind: messageKindCommandOrResponse, msg: msg}
    }
    for fmtStr, id := range d.Responses {
        msg := buildMessageFormat(fmtStr, id, d.Enumerations)
        byID[id] = messageDecoder{kind: messageKindCommandOrResponse, msg: msg}
    }
    for fmtStr, id := range d.Output {
        pt, err := parseOutputParamTypes(fmtStr)
        if err != nil {
            return nil, err
        }
        byID[id] = messageDecoder{
            kind:   messageKindOutput,
            output: &OutputFormat{ID: id, MsgFormat: fmtStr, ParamTypes: pt},
        }
    }
    return byID, nil
}

func buildMessageFormat(fmtStr string, id int, enums map[string]map[string]int) *MessageFormat {
    names, types := parseFormat(fmtStr, enums)
    return &MessageFormat{
        Name:        strings.Fields(fmtStr)[0],
        Format:      fmtStr,
        ID:          id,
        ParamNames:  names,
        ParamTypes:  types,
        DebugFormat: convertDebugFormat(fmtStr, types),
    }
}

// DecodeDebugOutput decodes a raw binary debugoutput stream (as written to
// _test_output files by klippy.py -o) into human-readable lines matching
// klippy/parsedump.py output (excluding the leading "seq:" line per packet).
func DecodeDebugOutput(d *Dictionary, data []byte) ([]string, error) {
    decoders, err := d.BuildParsedumpDecoders()
    if err != nil {
        return nil, err
    }
    return decodeDebugOutputWithDecoders(decoders, data)
}

func decodeDebugOutputWithDecoders(decoders map[int]messageDecoder, data []byte) ([]string, error) {
    var out []string
    buf := append([]byte{}, data...)
    for {
        l := checkPacket(buf)
        if l == 0 {
            break
        }
        if l < 0 {
            // Keep behavior aligned with parsedump.py: drop one byte and retry.
            if len(buf) <= 1 {
                break
            }
            buf = buf[1:]
            continue
        }
        frame := buf[:l]
        lines, err := dumpFrame(decoders, frame)
        if err != nil {
            return nil, err
        }
        out = append(out, lines...)
        buf = buf[l:]
    }
    return out, nil
}

func checkPacket(s []byte) int {
    if len(s) < MESSAGE_MIN {
        return 0
    }
    msglen := int(s[MESSAGE_POS_LEN])
    if msglen < MESSAGE_MIN || msglen > MESSAGE_MAX {
        return -1
    }
    msgseq := s[MESSAGE_POS_SEQ]
    if (msgseq & ^byte(MESSAGE_SEQ_MASK)) != byte(MESSAGE_DEST) {
        return -1
    }
    if len(s) < msglen {
        return 0
    }
    if s[msglen-MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC {
        return -1
    }
    crcHi, crcLo := CRC16CCITT(s[:msglen-MESSAGE_TRAILER_SIZE])
    if s[msglen-MESSAGE_TRAILER_CRC] != crcHi || s[msglen-MESSAGE_TRAILER_CRC+1] != crcLo {
        return -1
    }
    return msglen
}

func dumpFrame(decoders map[int]messageDecoder, frame []byte) ([]string, error) {
    // Mimic MessageParser.dump() output, but drop the leading "seq: .." line.
    pos := MESSAGE_HEADER_SIZE
    end := len(frame) - MESSAGE_TRAILER_SIZE
    if end < pos {
        return nil, fmt.Errorf("short frame")
    }
    var out []string
    for {
        if pos >= end {
            break
        }
        msgid, paramPos := DecodeUint32(frame, pos)
        id := int(msgid)
        dec, ok := decoders[id]
        if !ok {
            out = append(out, "#unknown "+reprBytes(frame))
            pos = end
            continue
        }
        switch dec.kind {
        case messageKindCommandOrResponse:
            line, next, err := decodeCommandOrResponse(dec.msg, frame, paramPos)
            if err != nil {
                return nil, err
            }
            out = append(out, line)
            pos = next
        case messageKindOutput:
            line, next, err := decodeOutput(dec.output, frame, paramPos)
            if err != nil {
                return nil, err
            }
            out = append(out, line)
            pos = next
        default:
            out = append(out, "#unknown "+reprBytes(frame))
            pos = end
        }
        if pos >= end {
            break
        }
    }
    return out, nil
}

func decodeCommandOrResponse(m *MessageFormat, frame []byte, pos int) (string, int, error) {
    parts := []string{m.Name}
    for idx, pt := range m.ParamTypes {
        pname := m.ParamNames[idx]
        switch t := pt.(type) {
        case ptEnum:
            v, np := pt.DecodeInt(frame, pos)
            pos = np
            if name, ok := t.Rev[int(v)]; ok {
                parts = append(parts, fmt.Sprintf("%s=%s", pname, name))
            } else {
                parts = append(parts, fmt.Sprintf("%s=?%d", pname, v))
            }
        case ptInt:
            v, np := pt.DecodeInt(frame, pos)
            pos = np
            parts = append(parts, fmt.Sprintf("%s=%d", pname, v))
        case ptBuffer:
            b, np := t.DecodeBytes(frame, pos)
            pos = np
            parts = append(parts, fmt.Sprintf("%s=%s", pname, reprBytes(b)))
        default: // ptUint/ptByte
            v, np := pt.DecodeInt(frame, pos)
            pos = np
            parts = append(parts, fmt.Sprintf("%s=%d", pname, uint32(v)))
        }
    }
    return strings.Join(parts, " "), pos, nil
}

func decodeOutput(of *OutputFormat, frame []byte, pos int) (string, int, error) {
    var values []string
    for _, pt := range of.ParamTypes {
        switch t := pt.(type) {
        case ptInt:
            v, np := pt.DecodeInt(frame, pos)
            pos = np
            values = append(values, fmt.Sprintf("%d", v))
        case ptBuffer:
            b, np := t.DecodeBytes(frame, pos)
            pos = np
            values = append(values, reprBytes(b))
        default:
            v, np := pt.DecodeInt(frame, pos)
            pos = np
            values = append(values, fmt.Sprintf("%d", uint32(v)))
        }
    }
    msg, err := formatOutputMsg(of.MsgFormat, values)
    if err != nil {
        return "", pos, err
    }
    return "#output " + msg, pos, nil
}

func parseOutputParamTypes(msgformat string) ([]paramType, error) {
    // Equivalent to msgproto.lookup_output_params().
    var pts []paramType
    for i := 0; i < len(msgformat); i++ {
        if msgformat[i] != '%' {
            continue
        }
        if i+1 < len(msgformat) && msgformat[i+1] == '%' {
            i++
            continue
        }
        tok, n := findTypeToken(msgformat[i:])
        if n == 0 {
            return nil, fmt.Errorf("invalid output format for %q", msgformat)
        }
        pt, ok := messageTypesByToken[tok]
        if !ok {
            return nil, fmt.Errorf("invalid output type %q in %q", tok, msgformat)
        }
        pts = append(pts, pt)
        i += n - 1
    }
    return pts, nil
}

var messageTypesByToken = map[string]paramType{
    "%u":    ptUint{},
    "%i":    ptInt{},
    "%hu":   ptUint{},
    "%hi":   ptInt{},
    "%c":    ptByte{},
    "%s":    ptBuffer{},
    "%.*s":  ptBuffer{},
    "%*s":   ptBuffer{},
}

func findTypeToken(s string) (string, int) {
    // s starts with '%' and we match the same token lengths as msgproto.py.
    if len(s) >= 4 {
        if _, ok := messageTypesByToken[s[:4]]; ok {
            return s[:4], 4
        }
    }
    if len(s) >= 3 {
        if _, ok := messageTypesByToken[s[:3]]; ok {
            return s[:3], 3
        }
    }
    if len(s) >= 2 {
        if _, ok := messageTypesByToken[s[:2]]; ok {
            return s[:2], 2
        }
    }
    return "", 0
}

func formatOutputMsg(msgformat string, values []string) (string, error) {
    // Convert msgformat to the same result as:
    //   convert_msg_format(msgformat) % tuple(values)
    // by scanning and substituting tokens in order.
    var out []byte
    valueIdx := 0
    for i := 0; i < len(msgformat); i++ {
        c := msgformat[i]
        if c != '%' {
            out = append(out, c)
            continue
        }
        if i+1 < len(msgformat) && msgformat[i+1] == '%' {
            out = append(out, '%')
            i++
            continue
        }
        _, n := findTypeToken(msgformat[i:])
        if n == 0 {
            return "", fmt.Errorf("invalid output format for %q", msgformat)
        }
        if valueIdx >= len(values) {
            return "", fmt.Errorf("not enough values for %q", msgformat)
        }
        out = append(out, []byte(values[valueIdx])...)
        valueIdx++
        i += n - 1
    }
    if valueIdx != len(values) {
        return "", fmt.Errorf("unused values for %q", msgformat)
    }
    return string(out), nil
}
