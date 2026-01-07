package protocol

import "testing"

func TestVLQ_Roundtrip(t *testing.T) {
    vals := []int32{
        0, 1, 31, 32, 33, 95, 96, 97, 127, 128, 129,
        0x1fff, 0x2000, 0x2001,
        -1, -31, -32, -33, -4095, -4096, -4097,
        0x7fffffff, -0x80000000,
    }
    for _, v := range vals {
        out := []byte{}
        EncodeUint32(&out, v)
        got, pos := DecodeUint32(out, 0)
        if pos != len(out) {
            t.Fatalf("DecodeUint32 consumed %d/%d for %d", pos, len(out), v)
        }
        if got != v {
            t.Fatalf("roundtrip %d -> %v -> %d", v, out, got)
        }
    }
}

func TestVLQ_KnownEncodings(t *testing.T) {
    // Spot-check a few encodings (these match msgproto.PT_uint32.encode()).
    cases := []struct {
        v    int32
        want []byte
    }{
        {0, []byte{0x00}},
        {1, []byte{0x01}},
        {31, []byte{0x1f}},
        {32, []byte{0x20}},
        {96, []byte{0x80, 0x60}},
        {-1, []byte{0x7f}},
        {-32, []byte{0x60}},
    }
    for _, tc := range cases {
        out := []byte{}
        EncodeUint32(&out, tc.v)
        if len(out) != len(tc.want) {
            t.Fatalf("EncodeUint32(%d)=%v want %v", tc.v, out, tc.want)
        }
        for i := range out {
            if out[i] != tc.want[i] {
                t.Fatalf("EncodeUint32(%d)=%v want %v", tc.v, out, tc.want)
            }
        }
    }
}
