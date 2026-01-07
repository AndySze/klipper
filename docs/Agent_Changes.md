# Agent / Local Major Changes Log

This file tracks **major** changes made in this local checkout (including
automated changes by coding agents). It is intentionally separate from:

- `docs/Releases.md` (upstream release history)
- `docs/Config_Changes.md` (non-backwards-compatible config changes)

## Entry template

### YYYY-MM-DD — Short title

- Summary:
- Rationale:
- User-visible impact:
- Notable files/areas:
- Validation:

## Entries

### 2026-01-07 — Add major-change logging policy

- Summary: Added an `AGENTS.md` policy requiring major changes be recorded here.
- Rationale: Keep local/agent-driven modifications auditable over time.
- User-visible impact: None.
- Notable files/areas: `AGENTS.md`, `docs/Agent_Changes.md`, `scripts/check_agent_changes.sh`.
- Validation: N/A (documentation-only change).

### 2026-01-07 — Add Go host migration plan document

- Summary: Added a detailed, step-by-step plan for rewriting Klipper host from Python to Go with behavior-equivalence gates.
- Rationale: Provide an executable migration roadmap (phases, risks, acceptance criteria, validation strategy).
- User-visible impact: None (documentation-only change).
- Notable files/areas: `docs/Go_Host_Migration_Plan.md`.
- Validation: N/A (documentation-only change).

### 2026-01-07 — Add Phase 0 golden baseline harness

- Summary: Added a small suite + generator script to produce Python golden MCU command outputs for later Go-vs-Python comparison.
- Rationale: Establish a repeatable baseline and equivalence gate before any Go host implementation work.
- User-visible impact: None (developer/test tooling only).
- Notable files/areas: `scripts/go_migration_golden.py`, `test/go_migration/README.md`, `test/go_migration/suites/minimal.txt`.
- Validation: `./scripts/check_whitespace.py` on added files.

### 2026-01-07 — Make chelper build on macOS for local tests

- Summary: Added non-Linux fallbacks in `klippy/chelper` so `c_helper.so` can compile on macOS (Darwin).
- Rationale: Enable running Klippy-based golden generation and local regression workflows on macOS.
- User-visible impact: None intended (affects host build portability only).
- Notable files/areas: `klippy/chelper/pyhelper.c`, `klippy/chelper/serialqueue.c`.
- Validation: `./scripts/go_migration_golden.py gen --dictdir dict` (after installing deps and dict bundle).

### 2026-01-07 — Expand Phase 0 golden suite coverage

- Summary: Expanded the minimal Phase 0 suite to cover motion, extrusion, homing/probes, temperature, and macros.
- Rationale: Improve baseline coverage before starting Go implementation work.
- User-visible impact: None (test tooling only).
- Notable files/areas: `test/go_migration/suites/minimal.txt`.
- Validation: `./scripts/go_migration_golden.py gen --dictdir dict --suite test/go_migration/suites/minimal.txt`.

### 2026-01-07 — Make golden compare robust to metadata

- Summary: Updated the golden comparison to diff only parsed MCU sections (ignoring header metadata) and report per-MCU diffs.
- Rationale: Allow Python/Go outputs to include different non-essential headers while still comparing the core MCU command stream.
- User-visible impact: None (test tooling only).
- Notable files/areas: `scripts/go_migration_golden.py`.
- Validation: `./scripts/go_migration_golden.py compare`.

### 2026-01-07 — Add relaxed compare mode for clock tokens

- Summary: Added `compare --mode relaxed-clock` to normalize `*clock=` tokens to relative offsets before diffing.
- Rationale: Enable early Go bring-up when absolute clock bases differ but relative schedules match.
- User-visible impact: None (test tooling only).
- Notable files/areas: `scripts/go_migration_golden.py`, `test/go_migration/README.md`.
- Validation: `./scripts/go_migration_golden.py compare --mode relaxed-clock`.

### 2026-01-07 — Add Go-side actual.txt scaffolding tool

- Summary: Added a small Go CLI to generate placeholder `actual.txt` files matching the MCU section format of `expected.txt`.
- Rationale: Define and enforce the Go-side output contract early; enables `compare --fail-missing` gating.
- User-visible impact: None (test tooling only).
- Notable files/areas: `go/go.mod`, `go/cmd/klipper-go-golden/main.go`, `test/go_migration/README.md`.
- Validation: `go run ./cmd/klipper-go-golden` (with `GOCACHE`/`GOPATH` if needed), plus `./scripts/go_migration_golden.py compare --fail-missing`.

### 2026-01-07 — Improve golden compare ergonomics

- Summary: Added `compare --only` to focus on a single test and handled `BrokenPipeError` when piping compare output.
- Rationale: Make it practical to iterate on Go output generation without huge diffs or noisy tracebacks.
- User-visible impact: None (test tooling only).
- Notable files/areas: `scripts/go_migration_golden.py`, `test/go_migration/README.md`.
- Validation: `./scripts/go_migration_golden.py compare --only commands`.

### 2026-01-07 — Go protocol scaffold with roundtrip for commands

- Summary: Added a minimal Go protocol package (dict parsing, VLQ, CRC, command encode/decode) and wired `klipper-go-golden` to roundtrip the `commands` case into `actual.txt`.
- Rationale: Establish a concrete Go-side output contract and prove encode/decode against real `*.dict` + expected MCU command lines.
- User-visible impact: None (developer tooling).
- Notable files/areas: `go/pkg/protocol/*`, `go/cmd/klipper-go-golden/main.go`.
- Validation: `GOCACHE=... GOPATH=... go run ./cmd/klipper-go-golden -mode roundtrip -only commands` then `./scripts/go_migration_golden.py compare --only commands --mode relaxed-clock`.

### 2026-01-07 — Roundtrip all minimal suite cases via Go protocol

- Summary: Extended Go roundtrip (dict + encode/decode) to cover the expanded minimal suite; compare now passes for all cases with `actual.txt` generated by Go.
- Rationale: Prove the Go-side encoder/decoder can reproduce all current golden cases (including buffers/enums) before further host logic is ported.
- User-visible impact: None (developer tooling).
- Notable files/areas: `go/pkg/protocol/*`, `go/cmd/klipper-go-golden/main.go`, `scripts/go_migration_golden.py` (compare).
- Validation: `go run ./cmd/klipper-go-golden -mode roundtrip` then `./scripts/go_migration_golden.py compare --mode relaxed-clock --fail-missing`.

### 2026-01-07 — Add unit tests for Go protocol package

- Summary: Added Go unit tests covering VLQ encoding (incl. boundary vectors), CRC16, msgblock framing, command encode/decode roundtrips, and enum/buffer literal helpers.
- Rationale: Lock in protocol behavior before expanding Go host functionality.
- User-visible impact: None (developer tooling).
- Notable files/areas: `go/pkg/protocol/*_test.go`.
- Validation: `cd go && GOCACHE=../out/go-build-cache GOPATH=../out/go-path go test ./...`.

### 2026-01-07 — Add Go parsedump-equivalent decoder (raw debugoutput -> text)

- Summary: Saved raw `_test_output` binaries during golden generation and added a Go decoder that reproduces `klippy/parsedump.py` text output from those binaries.
- Rationale: Turn the Go harness from “roundtrip the expected text” into a real equivalence gate on the binary protocol stream.
- User-visible impact: None (developer tooling).
- Notable files/areas: `scripts/go_migration_golden.py`, `go/pkg/protocol/parsedump.go`, `go/cmd/klipper-go-golden/main.go`, `test/go_migration/README.md`.
- Validation: `./scripts/go_migration_golden.py gen --dictdir dict --suite test/go_migration/suites/minimal.txt`, then `cd go && GOCACHE=../out/go-build-cache GOPATH=../out/go-path go run ./cmd/klipper-go-golden -mode parsedump`, then `./scripts/go_migration_golden.py compare --mode strict --fail-missing`.

### 2026-01-07 — Add Go raw encoder (text -> msgblocks)

- Summary: Added a Go encoder that packs parsedump-style message lines into msgblocks and writes `raw-go-<mcu>.bin`; `klipper-go-golden -mode encode-raw` now exercises encode+decode end-to-end.
- Rationale: Validate Go-side msg framing and stream encoding before any host logic is ported.
- User-visible impact: None (developer tooling).
- Notable files/areas: `go/pkg/protocol/rawencode.go`, `go/cmd/klipper-go-golden/main.go`, `test/go_migration/README.md`.
- Validation: `cd go && GOCACHE=../out/go-build-cache GOPATH=../out/go-path go run ./cmd/klipper-go-golden -mode encode-raw`, then `./scripts/go_migration_golden.py compare --mode strict --fail-missing`.

### 2026-01-07 — Add Host migration workplan (entering real Host phase)

- Summary: Added a concrete, milestone-based work plan for starting the real Host migration (cfg parsing → connect-phase config compilation → minimal gcode → motion pipeline → hardware loop).
- Rationale: Keep the post-protocol work scoped, test-gated, and incremental instead of attempting a “big bang” host rewrite.
- User-visible impact: None (documentation-only).
- Notable files/areas: `docs/Go_Host_Migration_Workplan.md`, `docs/Go_Host_Migration_Plan.md`.
- Validation: N/A (documentation-only change).

### 2026-01-07 — Implement Host H1 connect-phase compiler (example-cartesian)

- Summary: Added a minimal Go “host H1” compiler that reads configs and emits the connect-phase MCU command stream; wired it into `klipper-go-golden -mode host-h1` and validated against `commands.test` and `linuxtest.test`.
- Rationale: Start migrating real host behavior with a narrow, test-gated milestone (cfg parsing + connect-phase config/init output) before tackling gcode execution and motion planning.
- User-visible impact: None (developer tooling).
- Notable files/areas: `go/pkg/hosth1/h1.go`, `go/cmd/klipper-go-golden/main.go`, `go/pkg/protocol/dict.go`, `test/go_migration/README.md`.
- Validation: `cd go && GOCACHE=../out/go-build-cache GOPATH=../out/go-path go run ./cmd/klipper-go-golden -mode host-h1 -only commands -dictdir ../dict`, then `./scripts/go_migration_golden.py compare --only commands --mode strict --fail-missing`; and `cd go && ... go run ./cmd/klipper-go-golden -mode host-h1 -only linuxtest`, then `./scripts/go_migration_golden.py compare --only linuxtest --mode strict --fail-missing`.

### 2026-01-07 — Start Host H2 (minimal gcode execution)

- Summary: Added a minimal H2 gcode runner and a `klipper-go-golden -mode host-h2` path that executes `.test` gcode after connect-phase compilation (supports `G4` dwell and stubs for `commands.test`).
- Rationale: Establish the smallest possible “execute gcode” loop (parse → dispatch → advance time) before adding motion, heaters, and macro semantics.
- User-visible impact: None (developer tooling).
- Notable files/areas: `go/pkg/hosth2/h2.go`, `go/pkg/hosth2/gcode.go`, `go/cmd/klipper-go-golden/main.go`, `docs/Go_Host_Migration_Workplan.md`, `test/go_migration/README.md`.
- Validation: `cd go && GOCACHE=../out/go-build-cache GOPATH=../out/go-path go run ./cmd/klipper-go-golden -mode host-h2 -only linuxtest -dictdir ../dict`, then `./scripts/go_migration_golden.py compare --only linuxtest --mode strict --fail-missing`; and `cd go && ... go run ./cmd/klipper-go-golden -mode host-h2 -only commands`, then `./scripts/go_migration_golden.py compare --only commands --mode strict --fail-missing`.

### 2026-01-07 — Fix Host H3 manual_stepper strict equivalence

- Summary: Brought `host-h3` into strict equivalence for `manual_stepper.test` by matching Klipper semantics for `G28` under `kinematics: none`, making end-of-file behave like fileinput EOF (`request_restart('exit')` → motor-off), and stabilizing motor-off ordering.
- Rationale: The golden expected output is produced by Klippy fileinput mode; the Go harness must mirror those host-level semantics (homing no-op on `none`, and EOF restart handlers) to keep the MCU command stream identical.
- User-visible impact: None (developer tooling / migration harness only).
- Notable files/areas: `go/pkg/hosth3/gcode.go`, `go/pkg/hosth3/h3.go`, `go/pkg/hosth3/runtime.go`.
- Validation: `cd go && GOCACHE=../out/go-build-cache GOPATH=../out/go-path go run ./cmd/klipper-go-golden -mode host-h3 -only manual_stepper -dictdir ../dict`, then `./scripts/go_migration_golden.py compare --only manual_stepper --mode strict --fail-missing`.
