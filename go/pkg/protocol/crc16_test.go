package protocol

import "testing"

func TestCRC16CCITT_KnownVector(t *testing.T) {
    // msgproto.py's crc16_ccitt bitwise algorithm.
    hi, lo := CRC16CCITT([]byte("123456789"))
    got := uint16(hi)<<8 | uint16(lo)
    const want uint16 = 0x6f91
    if got != want {
        t.Fatalf("CRC16CCITT('123456789')=%04x want %04x", got, want)
    }
}

func TestCRC16CCITT_Empty(t *testing.T) {
    hi, lo := CRC16CCITT(nil)
    got := uint16(hi)<<8 | uint16(lo)
    const want uint16 = 0xffff
    if got != want {
        t.Fatalf("CRC16CCITT(empty)=%04x want %04x", got, want)
    }
}
