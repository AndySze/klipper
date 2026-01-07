package protocol

import "testing"

func TestEncodeMsgblock_KnownCRC(t *testing.T) {
    // payload 0x01 0x02 with seq 7:
    // header: len=7, seq=0x17
    // crc is computed over header+payload (4 bytes)
    b := EncodeMsgblock(7, []byte{0x01, 0x02})
    if len(b) != 7 {
        t.Fatalf("len=%d want 7", len(b))
    }
    if b[0] != 7 {
        t.Fatalf("len byte=%02x want 07", b[0])
    }
    if b[1] != 0x17 {
        t.Fatalf("seq byte=%02x want 17", b[1])
    }
    if b[6] != MESSAGE_SYNC {
        t.Fatalf("sync=%02x want %02x", b[6], MESSAGE_SYNC)
    }
    hi, lo := CRC16CCITT(b[:4])
    if b[4] != hi || b[5] != lo {
        t.Fatalf("crc=%02x%02x want %02x%02x", b[4], b[5], hi, lo)
    }
}
