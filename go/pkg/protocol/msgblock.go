package protocol

// EncodeMsgblock wraps a command payload into a Klipper message block.
// This matches MessageParser.encode_msgblock in msgproto.py.
func EncodeMsgblock(seq int, payload []byte) []byte {
    msglen := MESSAGE_MIN + len(payload)
    seq = (seq & MESSAGE_SEQ_MASK) | MESSAGE_DEST
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
    MESSAGE_DEST         = 0x10
    MESSAGE_SYNC         = 0x7e
    MESSAGE_SEQ_MASK     = 0x0f
)
