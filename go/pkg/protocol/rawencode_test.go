package protocol

import "testing"

func TestEncodeDebugOutputFromText_Roundtrip(t *testing.T) {
    d := &Dictionary{
        Commands: map[string]int{
            "finalize_config crc=%u": 9,
        },
        Output: map[string]int{
            "hello %u %c %*s %% done": 2,
        },
    }

    in := []string{
        "finalize_config crc=3915627893",
        "#output hello 123 7 b'AB' % done",
    }
    raw, err := EncodeDebugOutputFromText(d, in)
    if err != nil {
        t.Fatalf("EncodeDebugOutputFromText: %v", err)
    }
    got, err := DecodeDebugOutput(d, raw)
    if err != nil {
        t.Fatalf("DecodeDebugOutput: %v", err)
    }
    if len(got) != len(in) {
        t.Fatalf("len(got)=%d want %d: %v", len(got), len(in), got)
    }
    for i := range in {
        if got[i] != in[i] {
            t.Fatalf("line %d got %q want %q", i, got[i], in[i])
        }
    }
}

