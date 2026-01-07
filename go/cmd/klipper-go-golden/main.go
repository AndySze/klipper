package main

import (
    "bufio"
    "errors"
    "flag"
    "fmt"
    "io"
    "os"
    "path/filepath"
    "regexp"
    "sort"
    "strings"

    "klipper-go-migration/pkg/protocol"
)

type mcuSection struct {
    name  string
    dict  string
    lines []string
}

var mcuHeaderRe = regexp.MustCompile(`^##\s+MCU:\s+(.+?)\s+\(dict:\s+(.+?)\)\s*$`)

func readSuite(path string) ([]string, error) {
    f, err := os.Open(path)
    if err != nil {
        return nil, err
    }
    defer f.Close()

    var out []string
    s := bufio.NewScanner(f)
    for s.Scan() {
        line := strings.TrimSpace(s.Text())
        if line == "" || strings.HasPrefix(line, "#") {
            continue
        }
        out = append(out, line)
    }
    if err := s.Err(); err != nil {
        return nil, err
    }
    if len(out) == 0 {
        return nil, fmt.Errorf("suite is empty: %s", path)
    }
    return out, nil
}

func parseExpectedSections(path string) ([]mcuSection, error) {
    b, err := os.ReadFile(path)
    if err != nil {
        return nil, err
    }
    text := strings.ReplaceAll(string(b), "\r\n", "\n")
    lines := strings.Split(text, "\n")

    seen := make(map[string]bool)
    var out []*mcuSection
    var cur *mcuSection
    for _, ln := range lines {
        m := mcuHeaderRe.FindStringSubmatch(strings.TrimRight(ln, "\r"))
        if m == nil {
            if cur != nil {
                cur.lines = append(cur.lines, strings.TrimRight(ln, "\r"))
            }
            continue
        }
        name := strings.TrimSpace(m[1])
        dict := strings.TrimSpace(m[2])
        if name == "" {
            return nil, fmt.Errorf("invalid MCU section header in %s", path)
        }
        if seen[name] {
            return nil, fmt.Errorf("duplicate MCU section %q in %s", name, path)
        }
        seen[name] = true
        cur = &mcuSection{name: name, dict: dict}
        out = append(out, cur)
    }
    if len(out) == 0 {
        return nil, fmt.Errorf("no MCU sections found in %s", path)
    }
    // Trim trailing blanks in each section
    final := make([]mcuSection, 0, len(out))
    for _, sec := range out {
        for len(sec.lines) > 0 && strings.TrimSpace(sec.lines[len(sec.lines)-1]) == "" {
            sec.lines = sec.lines[:len(sec.lines)-1]
        }
        final = append(final, *sec)
    }
    return final, nil
}

func writeActual(path string, srcTest string, mode string, sections []mcuSection) error {
    dir := filepath.Dir(path)
    if err := os.MkdirAll(dir, 0o755); err != nil {
        return err
    }

    f, err := os.Create(path)
    if err != nil {
        return err
    }
    defer f.Close()

    w := bufio.NewWriter(f)
    defer w.Flush()

    fmt.Fprintf(w, "# Source: %s\n", srcTest)
    fmt.Fprintf(w, "# Generated-by: go/cmd/klipper-go-golden\n")
    fmt.Fprintf(w, "# Mode: %s\n", mode)

    sort.Slice(sections, func(i, j int) bool {
        // Keep mcu first if present.
        if sections[i].name == "mcu" && sections[j].name != "mcu" {
            return true
        }
        if sections[j].name == "mcu" && sections[i].name != "mcu" {
            return false
        }
        return sections[i].name < sections[j].name
    })

    for _, sec := range sections {
        fmt.Fprintf(w, "\n## MCU: %s (dict: %s)\n\n", sec.name, sec.dict)
        switch mode {
        case "stub":
            // Keep the section empty on purpose; it's a placeholder contract.
        case "roundtrip", "copy-expected":
            for _, ln := range sec.lines {
                ln = strings.TrimSpace(ln)
                if ln == "" {
                    continue
                }
                fmt.Fprintf(w, "%s\n", ln)
            }
        default:
            return fmt.Errorf("unknown mode: %s", mode)
        }
    }
    return nil
}

func copyFile(dst, src string) error {
    in, err := os.Open(src)
    if err != nil {
        return err
    }
    defer in.Close()
    out, err := os.Create(dst)
    if err != nil {
        return err
    }
    defer out.Close()
    if _, err := io.Copy(out, in); err != nil {
        return err
    }
    return out.Close()
}

func main() {
    var (
        suite   = flag.String("suite", "../test/go_migration/suites/minimal.txt", "suite file")
        outdir  = flag.String("outdir", "../test/go_migration/golden", "golden directory")
        only    = flag.String("only", "", "only generate for a single test (path or stem)")
        mode    = flag.String("mode", "stub", "output mode: stub|copy-expected|roundtrip")
        dictdir = flag.String("dictdir", "../dict", "dictionary directory")
    )
    flag.Parse()

    tests, err := readSuite(*suite)
    if err != nil {
        fmt.Fprintf(os.Stderr, "ERROR: %v\n", err)
        os.Exit(2)
    }

    for _, testRel := range tests {
        stem := strings.TrimSuffix(filepath.Base(testRel), filepath.Ext(testRel))
        if *only != "" && *only != testRel && *only != stem {
            continue
        }

        caseDir := filepath.Join(*outdir, stem)
        expected := filepath.Join(caseDir, "expected.txt")
        actual := filepath.Join(caseDir, "actual.txt")

        if _, err := os.Stat(expected); err != nil {
            if errors.Is(err, os.ErrNotExist) {
                fmt.Fprintf(os.Stderr, "ERROR: missing expected: %s\n", expected)
                os.Exit(2)
            }
            fmt.Fprintf(os.Stderr, "ERROR: %v\n", err)
            os.Exit(2)
        }

        switch *mode {
        case "copy-expected":
            if err := copyFile(actual, expected); err != nil {
                fmt.Fprintf(os.Stderr, "ERROR: %v\n", err)
                os.Exit(2)
            }
        case "stub":
            sections, err := parseExpectedSections(expected)
            if err != nil {
                fmt.Fprintf(os.Stderr, "ERROR: %v\n", err)
                os.Exit(2)
            }
            if err := writeActual(actual, testRel, *mode, sections); err != nil {
                fmt.Fprintf(os.Stderr, "ERROR: %v\n", err)
                os.Exit(2)
            }
        case "roundtrip":
            sections, err := parseExpectedSections(expected)
            if err != nil {
                fmt.Fprintf(os.Stderr, "ERROR: %v\n", err)
                os.Exit(2)
            }
            for i := range sections {
                dictPath := filepath.Join(*dictdir, sections[i].dict)
                dict, err := protocol.LoadDictionary(dictPath)
                if err != nil {
                    fmt.Fprintf(os.Stderr, "ERROR: load dict %s: %v\n", dictPath, err)
                    os.Exit(2)
                }
                formats, err := dict.BuildCommandFormats()
                if err != nil {
                    fmt.Fprintf(os.Stderr, "ERROR: build formats: %v\n", err)
                    os.Exit(2)
                }
                var outLines []string
                for _, ln := range sections[i].lines {
                    ln = strings.TrimSpace(ln)
                    if ln == "" || strings.HasPrefix(ln, "#") {
                        continue
                    }
                    encoded, err := protocol.EncodeCommand(formats, ln)
                    if err != nil {
                        fmt.Fprintf(os.Stderr, "ERROR encode %s: %v\n", ln, err)
                        os.Exit(2)
                    }
                    decoded, err := protocol.DecodeCommand(formats, encoded)
                    if err != nil {
                        fmt.Fprintf(os.Stderr, "ERROR decode %s: %v\n", ln, err)
                        os.Exit(2)
                    }
                    outLines = append(outLines, decoded)
                }
                sections[i].lines = outLines
            }
            if err := writeActual(actual, testRel, *mode, sections); err != nil {
                fmt.Fprintf(os.Stderr, "ERROR: %v\n", err)
                os.Exit(2)
            }
        default:
            fmt.Fprintf(os.Stderr, "ERROR: unknown mode: %s\n", *mode)
            os.Exit(2)
        }
    }
}
