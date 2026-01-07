# Go Host Migration – Phase 0 (Baseline + Golden)

This directory contains the Phase 0 scaffolding for the Klipper host
Python→Go migration:

- A small **suite** of existing Klipper regression `.test` files.
- A script that generates **golden expected outputs** by running the
  Python reference (`klippy/klippy.py`) in debug-output mode and then
  decoding the binary output via `klippy/parsedump.py`.

The goal is to establish a repeatable, versioned baseline that the Go
implementation can be compared against.

## Prerequisites

- A Klipper Python environment (default: `~/klippy-env/bin/python`).
- A directory containing the prebuilt MCU dictionaries used by the
  Klipper test suite.

Klipper upstream provides a convenient dictionary bundle (see
`docs/Debugging.md` for details). After extracting, you should have:

- `dict/atmega2560.dict`
- `dict/linuxprocess.dict`
- …etc

## Generate golden outputs (Python baseline)

From the repo root:

```sh
./scripts/go_migration_golden.py gen --dictdir dict
```

Outputs are written under `test/go_migration/golden/<testname>/`:

- `expected.txt` – normalized human-readable MCU commands
- `raw-<mcu>.bin` – raw binary debugoutput (input for Go parsedump decoder)
- `stderr.txt` / `stdout.txt` – captured Klippy output for debugging
- `meta.json` – provenance (source test, config, dict mapping)

## Compare against a Go output (future)

When the Go host can generate an equivalent decoded output, write it to:

- `test/go_migration/golden/<testname>/actual.txt`

To generate placeholder `actual.txt` files (output contract scaffolding):

```sh
(cd go && go run ./cmd/klipper-go-golden)
```

If your environment restricts writing to the default Go build cache,
set `GOCACHE`/`GOPATH` to a repo-local directory:

```sh
mkdir -p out/go-build-cache out/go-path
(cd go && GOCACHE="$PWD/../out/go-build-cache" GOPATH="$PWD/../out/go-path" go run ./cmd/klipper-go-golden)
```

Then run:

```sh
./scripts/go_migration_golden.py compare
```

## Go parsedump decoder (raw.bin -> actual.txt)

Once you have generated the Python goldens (which now also include
`raw-<mcu>.bin` files), you can have Go decode the raw binary stream and write
`actual.txt`:

```sh
mkdir -p out/go-build-cache out/go-path
(cd go && GOCACHE="$PWD/../out/go-build-cache" GOPATH="$PWD/../out/go-path" \
  go run ./cmd/klipper-go-golden -mode parsedump -dictdir ../dict)
```

Then compare:

```sh
./scripts/go_migration_golden.py compare --mode strict --fail-missing
```

## Go raw encoder (expected.txt -> raw-go.bin -> actual.txt)

This mode encodes the human-readable `expected.txt` lines back into binary
msgblocks (`raw-go-<mcu>.bin`), then decodes them again to produce `actual.txt`.
It exercises both Go encoding and Go parsedump decoding without implementing
host logic yet.

```sh
mkdir -p out/go-build-cache out/go-path
(cd go && GOCACHE="$PWD/../out/go-build-cache" GOPATH="$PWD/../out/go-path" \
  go run ./cmd/klipper-go-golden -mode encode-raw -dictdir ../dict)
./scripts/go_migration_golden.py compare --mode strict --fail-missing
```

To focus on a single case while iterating:

```sh
./scripts/go_migration_golden.py compare --only commands
```

If you need a slightly more permissive diff during early Go bring-up:

```sh
./scripts/go_migration_golden.py compare --mode relaxed-clock
```

## Notes

- This Phase 0 harness intentionally reuses upstream `.test` inputs so we
  don’t need to invent a new case format yet.
- Prefer adding new `.test` references to `test/go_migration/suites/` rather
  than copying configs/gcode into this folder.

If you don't have `~/klippy-env`, create it and install dependencies:

```sh
python3 -m venv ~/klippy-env
~/klippy-env/bin/pip install -r scripts/klippy-requirements.txt
```
