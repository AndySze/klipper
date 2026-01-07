package protocol

import (
    "fmt"
    "sort"
    "strconv"
    "strings"
)

// EncodeDebugOutputFromText encodes human-readable parsedump lines into a raw
// binary debugoutput stream (msgblocks) suitable for parsing by DecodeDebugOutput.
//
// It accepts the same per-message lines that appear in expected.txt, such as:
//   finalize_config crc=3915627893
//   #output hello 1 2 b'ABC'
func EncodeDebugOutputFromText(d *Dictionary, lines []string) ([]byte, error) {
    msgFormats, err := d.BuildMessageFormats()
    if err != nil {
        return nil, err
    }
    outFormats, err := d.BuildOutputFormats()
    if err != nil {
        return nil, err
    }

    seq := 0
    payload := []byte{}
    stream := []byte{}

    flush := func() {
        if len(payload) == 0 {
            return
        }
        stream = append(stream, EncodeMsgblock(seq, payload)...)
        seq = (seq + 1) & MESSAGE_SEQ_MASK
        payload = []byte{}
    }

    for _, raw := range lines {
        line := strings.TrimSpace(raw)
        if line == "" {
            continue
        }

        var msg []byte
        if strings.HasPrefix(line, "#output ") {
            msg, err = encodeOutputLine(outFormats, strings.TrimPrefix(line, "#output "))
            if err != nil {
                return nil, err
            }
        } else if strings.HasPrefix(line, "#unknown ") {
            return nil, fmt.Errorf("cannot encode unknown message: %s", line)
        } else if strings.HasPrefix(line, "#") {
            return nil, fmt.Errorf("unsupported parsedump line: %s", line)
        } else {
            msg, err = EncodeCommand(msgFormats, line)
            if err != nil {
                return nil, err
            }
        }

        if len(msg) > MESSAGE_PAYLOAD_MAX {
            return nil, fmt.Errorf("message too large for msgblock (%d bytes): %s", len(msg), line)
        }
        if len(payload)+len(msg) > MESSAGE_PAYLOAD_MAX {
            flush()
        }
        payload = append(payload, msg...)
    }
    flush()
    return stream, nil
}

// BuildMessageFormats returns formats for commands + responses (by name).
func (d *Dictionary) BuildMessageFormats() (map[string]*MessageFormat, error) {
    out := map[string]*MessageFormat{}
    for fmtStr, id := range d.Commands {
        m := buildMessageFormat(fmtStr, id, d.Enumerations)
        if _, ok := out[m.Name]; ok {
            return nil, fmt.Errorf("duplicate message name %q", m.Name)
        }
        out[m.Name] = m
    }
    for fmtStr, id := range d.Responses {
        m := buildMessageFormat(fmtStr, id, d.Enumerations)
        if _, ok := out[m.Name]; ok {
            return nil, fmt.Errorf("duplicate message name %q", m.Name)
        }
        out[m.Name] = m
    }
    return out, nil
}

type outputFormatEncoded struct {
    ID       int
    MsgFmt   string
    Segments []outputSegment
    Types    []paramType
}

type outputSegment struct {
    lit string
    tok string
}

func (d *Dictionary) BuildOutputFormats() ([]outputFormatEncoded, error) {
    var keys []string
    for k := range d.Output {
        keys = append(keys, k)
    }
    sort.Strings(keys)

    out := make([]outputFormatEncoded, 0, len(keys))
    for _, fmtStr := range keys {
        id := d.Output[fmtStr]
        pts, err := parseOutputParamTypes(fmtStr)
        if err != nil {
            return nil, err
        }
        segs, err := splitOutputFormat(fmtStr)
        if err != nil {
            return nil, err
        }
        out = append(out, outputFormatEncoded{
            ID:       id,
            MsgFmt:   fmtStr,
            Segments: segs,
            Types:    pts,
        })
    }
    return out, nil
}

func splitOutputFormat(msgformat string) ([]outputSegment, error) {
    var segs []outputSegment
    var lit strings.Builder

    flushLit := func() {
        if lit.Len() == 0 {
            return
        }
        segs = append(segs, outputSegment{lit: lit.String()})
        lit.Reset()
    }

    for i := 0; i < len(msgformat); i++ {
        c := msgformat[i]
        if c != '%' {
            lit.WriteByte(c)
            continue
        }
        if i+1 < len(msgformat) && msgformat[i+1] == '%' {
            lit.WriteByte('%')
            i++
            continue
        }
        tok, n := findTypeToken(msgformat[i:])
        if n == 0 {
            return nil, fmt.Errorf("invalid output format for %q", msgformat)
        }
        flushLit()
        segs = append(segs, outputSegment{tok: tok})
        i += n - 1
    }
    flushLit()
    return segs, nil
}

func encodeOutputLine(formats []outputFormatEncoded, msg string) ([]byte, error) {
    for _, of := range formats {
        values, ok := parseOutputValues(of.Segments, msg)
        if !ok {
            continue
        }
        if len(values) != len(of.Types) {
            return nil, fmt.Errorf("output parse mismatch for %q", of.MsgFmt)
        }
        out := []byte{}
        EncodeUint32(&out, int32(of.ID))
        for i, pt := range of.Types {
            raw := strings.TrimSpace(values[i])
            switch t := pt.(type) {
            case ptInt:
                v, err := strconv.ParseInt(raw, 0, 32)
                if err != nil {
                    return nil, fmt.Errorf("bad int %q: %w", raw, err)
                }
                pt.EncodeInt(&out, int32(v))
            case ptBuffer:
                b, err := parseBufferLiteral(raw)
                if err != nil {
                    return nil, fmt.Errorf("bad buffer %q: %w", raw, err)
                }
                t.EncodeBytes(&out, b)
            default: // ptUint/ptByte
                v, err := strconv.ParseUint(raw, 0, 32)
                if err != nil {
                    return nil, fmt.Errorf("bad uint %q: %w", raw, err)
                }
                pt.EncodeInt(&out, int32(v))
            }
        }
        return out, nil
    }
    return nil, fmt.Errorf("no output format matched: %q", msg)
}

func parseOutputValues(segs []outputSegment, msg string) ([]string, bool) {
    // Extract token values by matching literals in order.
    pos := 0
    var values []string
    for i := 0; i < len(segs); i++ {
        seg := segs[i]
        if seg.tok == "" {
            if !strings.HasPrefix(msg[pos:], seg.lit) {
                return nil, false
            }
            pos += len(seg.lit)
            continue
        }
        // Find the next literal to determine this token's end.
        nextLit := ""
        for j := i + 1; j < len(segs); j++ {
            if segs[j].tok == "" && segs[j].lit != "" {
                nextLit = segs[j].lit
                break
            }
            if segs[j].tok != "" {
                // Adjacent tokens are ambiguous; avoid guessing.
                return nil, false
            }
        }
        if nextLit == "" {
            values = append(values, msg[pos:])
            pos = len(msg)
            continue
        }
        idx := strings.Index(msg[pos:], nextLit)
        if idx < 0 {
            return nil, false
        }
        values = append(values, msg[pos:pos+idx])
        pos = pos + idx
    }
    if pos != len(msg) {
        return nil, false
    }
    return values, true
}
