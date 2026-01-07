package protocol

// PTUint32 encodes/decodes integers using the same VLQ scheme as Klipper's
// PT_uint32 (msgproto.py). This is sufficient for the command IDs and
// parameters used in the regression tests we round-trip.
func EncodeUint32(out *[]byte, v int32) {
    if v >= 0xc000000 || v < -0x4000000 {
        *out = append(*out, byte((v>>28)&0x7f|0x80))
    }
    if v >= 0x180000 || v < -0x80000 {
        *out = append(*out, byte((v>>21)&0x7f|0x80))
    }
    if v >= 0x3000 || v < -0x1000 {
        *out = append(*out, byte((v>>14)&0x7f|0x80))
    }
    if v >= 0x60 || v < -0x20 {
        *out = append(*out, byte((v>>7)&0x7f|0x80))
    }
    *out = append(*out, byte(v&0x7f))
}

func DecodeUint32(buf []byte, pos int) (int32, int) {
    c := buf[pos]
    pos++
    v := int32(c & 0x7f)
    if (c & 0x60) == 0x60 {
        v |= -0x20
    }
    for (c & 0x80) != 0 {
        c = buf[pos]
        pos++
        v = (v << 7) | int32(c&0x7f)
    }
    return v, pos
}
