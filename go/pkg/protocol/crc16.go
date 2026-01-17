package protocol

// CRC16-CCITT - optimized algorithm matching Klipper's crc16_ccitt.c
// This is the same algorithm used by msgproto.py and the MCU firmware.
//
// C reference from src/generic/crc16_ccitt.c:
//
//	data ^= crc & 0xff;
//	data ^= data << 4;
//	crc = ((((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4)
//	       ^ ((uint16_t)data << 3));
func CRC16CCITT(buf []byte) (byte, byte) {
    var crc uint16 = 0xffff
    for _, b := range buf {
        data := b ^ uint8(crc&0xff)
        data ^= data << 4 // 8-bit overflow intentional
        crc = ((uint16(data) << 8) | (crc >> 8)) ^ uint16(data>>4) ^ (uint16(data) << 3)
    }
    return byte(crc >> 8), byte(crc & 0xff)
}
