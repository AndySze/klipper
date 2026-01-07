package protocol

// CRC16-CCITT (same bitwise algorithm used in msgproto.py)
func CRC16CCITT(buf []byte) (byte, byte) {
    var crc uint16 = 0xffff
    for _, b := range buf {
        data := uint16(b)
        data ^= crc & 0xff
        data ^= (data & 0x0f) << 4
        crc = (crc >> 8) ^ (data << 8) ^ (data << 3) ^ (data >> 4)
    }
    return byte(crc >> 8), byte(crc & 0xff)
}
