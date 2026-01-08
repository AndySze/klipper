package protocol

import "testing"

func TestEncodeDecodeCommand_Roundtrip(t *testing.T) {
    d := &Dictionary{
        Commands: map[string]int{
            "queue_step oid=%c interval=%u count=%c add=%i": 5,
            "finalize_config crc=%u":                       9,
            "config_ds18b20 oid=%c serial=%*s max_error_count=%c": 11,
            "set_digital_out pin=%u value=%c":                    13,
        },
        Enumerations: map[string]map[string]int{
            "pin": {"PA1": 10},
        },
    }
    fmts, err := d.BuildCommandFormats()
    if err != nil {
        t.Fatalf("BuildCommandFormats: %v", err)
    }

    cases := []string{
        "queue_step oid=8 interval=106555 count=2 add=-32572",
        "finalize_config crc=3915627893",
        "config_ds18b20 oid=0 serial=b'12345678' max_error_count=4",
        "set_digital_out pin=PA1 value=1",
    }
    for _, line := range cases {
        enc, err := EncodeCommand(fmts, line)
        if err != nil {
            t.Fatalf("EncodeCommand(%q): %v", line, err)
        }
        dec, err := DecodeCommand(fmts, enc)
        if err != nil {
            t.Fatalf("DecodeCommand(%q): %v", line, err)
        }
        if dec != line {
            t.Fatalf("roundtrip mismatch:\n  in:  %s\n  out: %s", line, dec)
        }
    }
}

func TestEncodeCommand_Unknown(t *testing.T) {
    d := &Dictionary{Commands: map[string]int{"foo a=%u": 1}}
    fmts, err := d.BuildCommandFormats()
    if err != nil {
        t.Fatalf("BuildCommandFormats: %v", err)
    }
    if _, err := EncodeCommand(fmts, "bar a=1"); err == nil {
        t.Fatalf("expected error for unknown command")
    }
}

func TestEncodeCommand_TruncatesUint32(t *testing.T) {
    d := &Dictionary{
        Commands: map[string]int{
            "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c": 1,
            "finalize_config crc=%u": 2,
        },
    }
    fmts, err := d.BuildCommandFormats()
    if err != nil {
        t.Fatalf("BuildCommandFormats: %v", err)
    }

    // Values can exceed 2^32-1; msgproto truncates to 32 bits when packing.
    enc, err := EncodeCommand(fmts, "trsync_start oid=1 report_clock=4950762637 report_ticks=1200000 expire_reason=4")
    if err != nil {
        t.Fatalf("EncodeCommand: %v", err)
    }
    dec, err := DecodeCommand(fmts, enc)
    if err != nil {
        t.Fatalf("DecodeCommand: %v", err)
    }
    want := "trsync_start oid=1 report_clock=655795341 report_ticks=1200000 expire_reason=4"
    if dec != want {
        t.Fatalf("got %q want %q", dec, want)
    }

    // Negative values on %u fields should also round-trip via 32-bit casting.
    enc, err = EncodeCommand(fmts, "finalize_config crc=-1")
    if err != nil {
        t.Fatalf("EncodeCommand(-1): %v", err)
    }
    dec, err = DecodeCommand(fmts, enc)
    if err != nil {
        t.Fatalf("DecodeCommand(-1): %v", err)
    }
    want = "finalize_config crc=4294967295"
    if dec != want {
        t.Fatalf("got %q want %q", dec, want)
    }
}

func TestDecodeCommand_UnknownEnumValue(t *testing.T) {
    d := &Dictionary{
        Commands: map[string]int{"set_pin pin=%u": 3},
        Enumerations: map[string]map[string]int{
            "pin": {"PA1": 10},
        },
    }
    fmts, err := d.BuildCommandFormats()
    if err != nil {
        t.Fatalf("BuildCommandFormats: %v", err)
    }

    // Manually encode "set_pin pin=<unknown>".
    data := []byte{}
    EncodeUint32(&data, 3)  // msgid
    EncodeUint32(&data, 11) // pin (unknown)

    got, err := DecodeCommand(fmts, data)
    if err != nil {
        t.Fatalf("DecodeCommand: %v", err)
    }
    want := "set_pin pin=?11"
    if got != want {
        t.Fatalf("got %q want %q", got, want)
    }
}

func TestReprBytes(t *testing.T) {
    got := reprBytes([]byte{0x00, 'A', 0x7f, '\n', '\'', '\\'})
    want := "b\"\\x00A\\x7f\\n'\\\\\""
    if got != want {
        t.Fatalf("reprBytes=%q want %q", got, want)
    }
}

func TestParseBufferLiteral(t *testing.T) {
    b, err := parseBufferLiteral("b\"1234\"")
    if err != nil {
        t.Fatalf("parseBufferLiteral: %v", err)
    }
    if string(b) != "1234" {
        t.Fatalf("got %q want %q", string(b), "1234")
    }
    b, err = parseBufferLiteral("b'\\x41\\n'")
    if err != nil {
        t.Fatalf("parseBufferLiteral escapes: %v", err)
    }
    if string(b) != "A\n" {
        t.Fatalf("got %q want %q", string(b), "A\\n")
    }
    b, err = parseBufferLiteral("313233")
    if err != nil {
        t.Fatalf("parseBufferLiteral hex: %v", err)
    }
    if string(b) != "123" {
        t.Fatalf("got %q want %q", string(b), "123")
    }
    if _, err := parseBufferLiteral("b\"unterminated"); err == nil {
        t.Fatalf("expected error for unterminated literal")
    }
    if _, err := parseBufferLiteral("\"nope\""); err == nil {
        t.Fatalf("expected error for non-bytes literal")
    }
}
