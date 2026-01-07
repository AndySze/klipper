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
