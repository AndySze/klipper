package protocol

import "testing"

func TestDecodeDebugOutput_CommandAndOutput(t *testing.T) {
    d := &Dictionary{
        Commands: map[string]int{
            "finalize_config crc=%u": 9,
        },
        Output: map[string]int{
            "hello %u %c %*s %% done": 2,
        },
        Enumerations: map[string]map[string]int{},
    }

    cmdFormats, err := d.BuildCommandFormats()
    if err != nil {
        t.Fatalf("BuildCommandFormats: %v", err)
    }
    cmd, err := EncodeCommand(cmdFormats, "finalize_config crc=3915627893")
    if err != nil {
        t.Fatalf("EncodeCommand: %v", err)
    }

    outMsg := []byte{}
    EncodeUint32(&outMsg, 2)
    EncodeUint32(&outMsg, 123)
    EncodeUint32(&outMsg, 7)
    ptBuffer{}.EncodeBytes(&outMsg, []byte("AB"))

    payload := append([]byte{}, cmd...)
    payload = append(payload, outMsg...)

    stream := EncodeMsgblock(7, payload)
    lines, err := DecodeDebugOutput(d, stream)
    if err != nil {
        t.Fatalf("DecodeDebugOutput: %v", err)
    }
    if len(lines) != 2 {
        t.Fatalf("len(lines)=%d want 2: %v", len(lines), lines)
    }
    if lines[0] != "finalize_config crc=3915627893" {
        t.Fatalf("lines[0]=%q", lines[0])
    }
    if lines[1] != "#output hello 123 7 b'AB' % done" {
        t.Fatalf("lines[1]=%q", lines[1])
    }
}

