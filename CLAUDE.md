# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Klipper is a 3D printer firmware that uses a split architecture:
- **Host software** (Klippy) runs on a general-purpose computer (Raspberry Pi, etc.) - handles high-level logic
- **Micro-controller firmware** runs on dedicated MCUs (AVR, STM32, LPC176x, RP2040, etc.) - handles real-time control

The host computes complex operations (motion planning, kinematics) and sends precise timing commands to micro-controllers, which execute stepper motor movements and handle hardware I/O in real-time.

## Build Commands

### Micro-controller firmware (C code)
```bash
# Configure for a specific board
make menuconfig

# Or use an existing config
cp test/configs/some-config .config
make olddefconfig

# Build (verbose output)
make V=1

# Clean build
make clean
```

Build outputs:
- AVR: `out/klipper.elf.hex`
- ARM: `out/klipper.bin`

The build system uses Kconfig for configuration management. Cross-compilation is supported for different architectures.

### Host software (Python)
```bash
# Create virtual environment (first time only)
virtualenv ~/klippy-env
~/klippy-env/bin/pip install -r scripts/klippy-requirements.txt

# Run Klippy
~/klippy-env/bin/python ./klippy/klippy.py ~/printer.cfg

# Batch mode (for testing/debugging)
~/klippy-env/bin/python ./klippy/klippy.py ~/printer.cfg -i test.gcode -o test.serial -v -d out/klipper.dict
~/klippy-env/bin/python ./klippy/parsedump.py out/klipper.dict test.serial > test.txt
```

### Go migration code (in development)
```bash
cd go

# Set up Go environment (required to avoid cache issues)
export GOCACHE=../out/go-build-cache
export GOPATH=../out/go-path

# Run tests
CGO_ENABLED=1 go test ./...

# Run golden test harness
go run ./cmd/klipper-go-golden -mode host-h4 -only out_of_bounds -dictdir ../dict
```

## Test Commands

```bash
# Whitespace check (must pass before commits)
./scripts/check_whitespace.sh

# Host software regression tests (requires data dictionaries)
./scripts/test_klippy.py -d dict/ test/klippy/*.test

# Run single test
./scripts/test_klippy.py -d dict/ test/klippy/commands.test

# Go migration golden tests
./scripts/go_migration_golden.py gen --dictdir dict --suite test/go_migration/suites/minimal.txt
./scripts/go_migration_golden.py compare --only out_of_bounds --mode strict --fail-missing
```

## Architecture

### Host Software (`klippy/`)

**Core execution flow:**
1. `klippy.py` - Main entry point, reads config, instantiates printer objects
2. `gcode.py` - Parses G-code commands, translates to printer object calls
3. `toolhead.py` - Motion planning and look-ahead queue
4. `kinematics/` - Converts cartesian moves to stepper positions
5. `mcu.py` - Microcontroller communication interface

**Key threads:**
- Main thread: Reactor pattern, handles G-code and high-level events
- Per-MCU read threads: Handle incoming MCU messages
- Per-MCU write threads (C): Queue messages to MCUs
- Per-stepper threads (C): Calculate step timing

**Motion pipeline:**
```
G1 command → gcode.py → ToolHead.move() → LookAheadQueue
  → kinematics check_move() → trapq_append()
  → steppersync background thread → itersolve (iterative solver)
  → stepcompress → queue_step commands to MCU
```

### Micro-controller Code (`src/`)

**Execution flow:**
1. Architecture-specific `main.c` calls `sched_main()`
2. Runs `DECL_INIT()` functions during initialization
3. Main loop runs `DECL_TASK()` functions repeatedly
4. Timer interrupts scheduled via `sched_add_timer()`
5. `DECL_COMMAND()` functions parse and execute host commands

**Key files:**
- `src/sched.c` - Scheduler and timer management
- `src/command.c` - Command dispatch and parsing
- `src/stepper.c` - Stepper motor pulse generation

**Architecture directories:**
- `src/avr/` - Atmel AVR (Arduino)
- `src/stm32/` - STM32 ARM
- `src/lpc176x/` - NXP LPC176x
- `src/rp2040/` - Raspberry Pi RP2040
- `src/linux/` - Linux process simulation
- `src/generic/` - Architecture-independent code

### Go Migration (`go/`)

**Active migration from Python host to Go.** Key packages:
- `go/pkg/protocol/` - MCU protocol (dict, encoding, decoding)
- `go/pkg/hosth1/` - Config parsing and connect-phase compilation
- `go/pkg/hosth2/` - Minimal G-code execution
- `go/pkg/hosth3/` - Manual stepper support
- `go/pkg/hosth4/` - Cartesian kinematics, homing, bounds checking
- `go/pkg/chelper/` - CGO bindings to `klippy/chelper` C code

**Progress tracking:** `docs/Go_Host_Migration_Workplan.md`

## Adding New Features

### Host modules (Python)
Create `klippy/extras/my_module.py`. Config section `[my_module]` auto-loads it. Use `klippy/extras/servo.py` as a reference.

Key points:
- Implement `load_config()` or `load_config_prefix()`
- Use `config.getprinter()` to get printer reference
- Register event handlers for `klippy:connect` and `klippy:ready`
- Don't store references to the `config` object past initialization
- Use floating-point constants (e.g., `1.` instead of `1`)

### Kinematics (Python + C)
1. Create `klippy/kinematics/my_kinematics.py`
2. Implement `check_move()`, `calc_position()`, `get_status()`, `home()`
3. Add C stepper position functions in `klippy/chelper/kin_*.c`
4. See existing kinematics as examples

### MCU ports (C)
See `Code_Overview.md` "Porting to a new micro-controller" section:
1. Add 3rd-party libraries to `lib/`
2. Create `src/<arch>/` with Kconfig and Makefile
3. Bring up serial communication first (most critical)
4. Add timer dispatch, GPIO, then peripherals
5. Create example config in `config/`

## Important Notes

- **Time systems:** Klipper uses system time, print time (synced to MCU), and MCU clock (integer ticks). See `Code_Overview.md` "Time" section.
- **Coordinate systems:** Multiple coordinate representations (MCU steps, stepper position, kinematic, toolhead, gcode). See `Code_Overview.md` "Coordinate Systems" section.
- **GPIO abstraction:** All GPIO operations go through architecture-specific wrappers.
- **Testing approach:** The project uses golden file testing - compare MCU command streams between Python reference and Go implementations.
- **Go CGO requirement:** The Go migration requires CGO_ENABLED=1 to bind to `klippy/chelper` C code for step timing.

## Code Style

- **C:** Linux kernel style
- **Python:** PEP 8 (mostly)
- No excessive debug code, commented-out code, or "todo" comments
- Changes should target ≥100 real-world users
- Each commit should address a single topic
- Use `Signed-off-by:` in commits

## Documentation Updates Required

For user-facing changes, update:
- `docs/G-Codes.md` - Commands and parameters
- `docs/Config_Reference.md` - Config parameters
- `docs/Status_Reference.md` - Status variables
- `docs/API_Server.md` - Webhooks
- `docs/Config_Changes.md` - Non-backwards-compatible changes

## Agent-Specific Guidelines

- Document major changes (>200 lines, multi-subsystem, user-visible) in `docs/Agent_Changes.md`
- Run `./scripts/check_whitespace.sh` before committing
- Avoid drive-by refactors - keep changes focused
- This is a checkout of upstream Klipper - maintain compatibility with upstream
