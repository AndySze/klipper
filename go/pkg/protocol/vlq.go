package protocol

// PTUint32 encodes/decodes integers using the same VLQ scheme as Klipper's
// PT_uint32 (msgproto.py). This is sufficient for the command IDs and
// parameters used in the regression tests we round-trip.
func EncodeUint32(out *[]byte, v int32) {
	// Match klippy/msgproto.py PT_uint32.encode() and klippy/chelper/msgblock.c
	// encode_int(): range checks use a signed int32 view, but bit shifts use
	// the underlying 32-bit value.
	uv := uint32(v)
	sv := int32(v)
	if sv >= 0xc000000 || sv < -0x4000000 {
		*out = append(*out, byte(((uv>>28)&0x7f)|0x80))
	}
	if sv >= 0x180000 || sv < -0x80000 {
		*out = append(*out, byte(((uv>>21)&0x7f)|0x80))
	}
	if sv >= 0x3000 || sv < -0x1000 {
		*out = append(*out, byte(((uv>>14)&0x7f)|0x80))
	}
	if sv >= 0x60 || sv < -0x20 {
		*out = append(*out, byte(((uv>>7)&0x7f)|0x80))
	}
	*out = append(*out, byte(uv&0x7f))
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
