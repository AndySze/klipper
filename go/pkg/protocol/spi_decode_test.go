package protocol

import (
    "path/filepath"
    "os"
    "testing"
    "runtime"
)

func TestDecodeSpiSend(t *testing.T) {
    _, currentFile, _, _ := runtime.Caller(0)
    repoRoot := filepath.Join(filepath.Dir(currentFile), "..", "..", "..")
    
    dict, err := LoadDictionary(filepath.Join(repoRoot, "dict", "atmega2560.dict"))
    if err != nil {
        t.Fatalf("load dict: %v", err)
    }
    
    raw, err := os.ReadFile(filepath.Join(repoRoot, "test", "go_migration", "golden", "printers_einsy", "raw-host-h4-mcu.bin"))
    if err != nil {
        t.Fatalf("read raw: %v", err)
    }
    
    t.Logf("Raw file size: %d bytes", len(raw))
    
    // Check spi_send command ID
    for fmtStr, id := range dict.Commands {
        if len(fmtStr) >= 8 && fmtStr[:8] == "spi_send" {
            t.Logf("spi_send command format: %q id=%d", fmtStr, id)
        }
    }
    
    lines, err := DecodeDebugOutput(dict, raw)
    if err != nil {
        t.Fatalf("decode: %v", err)
    }
    
    t.Logf("Decoded %d lines", len(lines))
    
    spiCount := 0
    for _, ln := range lines {
        if len(ln) >= 8 && ln[:8] == "spi_send" {
            spiCount++
        }
    }
    t.Logf("spi_send lines: %d", spiCount)
    
    if spiCount == 0 {
        // Print first 30 lines
        for i, ln := range lines {
            if i >= 30 {
                break
            }
            t.Logf("line[%d]: %s", i, ln)
        }
        t.Errorf("expected spi_send commands but found none")
    }
}
