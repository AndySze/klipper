# Go Migration - Quick Reference
**Last Updated: January 11, 2026**

## Test Status Summary

| Test | Mode | Needs Normalization? | Fix Function |
|------|-------|---------------------|--------------|
| commands | host-h4 | ❌ No | None |
| out_of_bounds | host-h4 | ✅ Yes | fixHostH4OutOfBoundsOrdering |
| gcode_arcs | host-h4 | ✅ Yes | fixHostH4GcodeArcs |
| bed_screws | host-h4 | ✅ Yes | fixHostH4BedScrews |
| extruders | host-h4 | ❌ No | None |
| pressure_advance | host-h4 | ❌ No | fixHostH4PressureAdvanceOrdering (obsolete?) |
| manual_stepper | host-h3 | ❌ No | fixHostH3ManualStepper (maybe obsolete) |
| linuxtest | host-h1 | ❌ No | None |
| macros | host-h4 | ❌ No | None |
| bltouch | host-h4 | ❌ No | fixHostH4BLTouch (**CONFIRMED OBSOLETE**) |

## Quick Commands

### Generate Golden Output
```bash
./scripts/go_migration_golden.py gen --dictdir dict
```

### Generate Go Output (Single Test)
```bash
mkdir -p out/go-build-cache out/go-path
cd go && GOCACHE="$PWD/../out/go-build-cache" GOPATH="$PWD/../out/go-path" \
  go run ./cmd/klipper-go-golden -mode host-h4 -only <test> -dictdir ../dict
```

### Generate Go Output (Auto Mode - All Tests)
```bash
mkdir -p out/go-build-cache out/go-path
cd go && GOCACHE="$PWD/../out/go-build-cache" GOPATH="$PWD/../out/go-path" \
  go run ./cmd/klipper-go-golden -mode auto -dictdir ../dict
```

### Compare Results
```bash
./scripts/go_migration_golden.py compare --only <test> --mode strict
```

### Compare All Tests
```bash
./scripts/go_migration_golden.py compare --mode strict --fail-missing
```

## Key Files

### Main Application
- `go/cmd/klipper-go-golden/main.go` - Main golden generator
  - Auto mode: lines 1426-1448
  - Fix functions: lines 198, 241, 396, 417, 435, 1084

### New Normalize Package
- `go/pkg/normalize/normalize.go` - Core rule engine
- `go/pkg/normalize/rules.go` - Predefined test rules
- `go/pkg/normalize/README.md` - Full documentation
- `go/pkg/normalize/example_test.go` - Examples and tests

### Documentation
- `go/docs/session_summary_2026-01-11.md` - Previous session work
- `go/docs/completed_tasks_2026-01-11.md` - Detailed completion report
- `go/docs/quick_reference.md` - This file

## Auto Mode Mapping

All tests now have explicit mode assignments:
- Most tests → host-h4 (full motion planning)
- manual_stepper → host-h3 (connect-phase only)
- linuxtest → host-h1 (minimal host)

## Important Findings

### Confirmed Obsolete
- `fixHostH4BLTouch()` - Not needed, can be removed (~150 lines)

### Possibly Obsolete
- `fixHostH4PressureAdvanceOrdering()` - Test passes without fix
- `fixHostH3ManualStepper()` - Test passes, fix may be defensive

### Still Required
- `fixHostH4OutOfBoundsOrdering()` - Test fails without it
- `fixHostH4BedScrews()` - Test fails without it
- `fixHostH4GcodeArcs()` - Test fails without it (complex issues)

## Next Steps

### Immediate
1. Remove `fixHostH4BLTouch()` function and call site
2. Update documentation
3. Test after removal

### Short Term
1. Integrate normalize package for out_of_bounds and bed_screws
2. Complete gcode_ars rules
3. Investigate missing blocks in gcode_arcs output

### Long Term
1. Fix Go implementation to match Python output exactly
2. Remove all normalization
3. Expand test coverage

## Common Issues

### Compilation Errors
```bash
cd go && go build ./cmd/klipper-go-golden
```

### Test Failures
- Check test/klippy/<test>.test for config
- Check dict/ for required dictionary files
- Compare expected.txt vs actual.txt manually

### Debug Mode
Add `-trace` flag:
```bash
go run ./cmd/klipper-go-golden -mode host-h4 -only <test> -trace -dictdir ../dict
```

## Resources

- Upstream Klipper: https://github.com/KevinOConnor/klipper
- Phase 0 README: `test/go_migration/README.md`
- Normalize Package Docs: `go/pkg/normalize/README.md`
