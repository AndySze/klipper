package protocol

// EncodeMsgblock wraps a command payload into a Klipper message block.
// This matches MessageParser.encode_msgblock in msgproto.py.
// Sets MESSAGE_DEST flag for normal communication.
func EncodeMsgblock(seq int, payload []byte) []byte {
    return encodeMsgblockInternal(seq, payload, true)
}

// EncodeMsgblockRaw wraps a command payload without setting MESSAGE_DEST.
// Used during identify handshake when communicating with unknown MCU.
func EncodeMsgblockRaw(seq int, payload []byte) []byte {
    return encodeMsgblockInternal(seq, payload, false)
}

func encodeMsgblockInternal(seq int, payload []byte, setDest bool) []byte {
    msglen := MESSAGE_MIN + len(payload)
    seq = seq & MESSAGE_SEQ_MASK
    if setDest {
        seq |= MESSAGE_DEST
    }
    out := []byte{byte(msglen), byte(seq)}
    out = append(out, payload...)
    crcHi, crcLo := CRC16CCITT(out)
    out = append(out, crcHi, crcLo, MESSAGE_SYNC)
    return out
}

const (
    MESSAGE_MIN          = 5
    MESSAGE_MAX          = 64
    MESSAGE_HEADER_SIZE  = 2
    MESSAGE_TRAILER_SIZE = 3
    MESSAGE_POS_LEN      = 0
    MESSAGE_POS_SEQ      = 1
    MESSAGE_TRAILER_CRC  = 3
    MESSAGE_TRAILER_SYNC = 1
    MESSAGE_PAYLOAD_MAX  = MESSAGE_MAX - MESSAGE_MIN
    MESSAGE_DEST         = 0x10
    MESSAGE_SYNC         = 0x7e
    MESSAGE_SEQ_MASK     = 0x0f
)
