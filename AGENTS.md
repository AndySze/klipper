# Agent notes (local)

This repository is a checkout of the upstream Klipper project.
Keep changes focused, avoid drive-by refactors, and follow existing
conventions in the surrounding code.

## Build/lint/test commands

**Building firmware (C):**
```bash
# Configure (interactive menuconfig or use existing .config)
make menuconfig

# Build firmware
make

# Clean builds
make clean        # Remove out/ directory
make distclean    # Clean and remove .config
```

Output: `out/klipper.elf` (hex files on AVR), `out/klipper.bin` (ARM)

**Running tests:**
```bash
# Run single test
python3 scripts/test_klippy.py test/klippy/<testfile>.test

# Run all tests in a file
python3 scripts/test_klippy.py test/klippy/*.test

# Run tests with verbose output
python3 scripts/test_klippy.py -v test/klippy/<testfile>.test

# Keep temporary files for debugging
python3 scripts/test_klippy.py -k test/klippy/<testfile>.test
```

Tests use custom format (CONFIG, DICTIONARY, GCODE, SHOULD_FAIL directives),
not pytest.

**Whitespace/linting:**
```bash
# Check whitespace across all source files
./scripts/check_whitespace.sh

# Check specific files
python3 scripts/check_whitespace.py <files...>
```

## Code style guidelines

**File headers (all source files):**
```python
# Code for [brief description]
#
# Copyright (C) YEAR  Name <email@example.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
```

**Python (klippy/):**
- Imports: comma-separated on single line when possible: `import math, logging, chelper`
- Classes: CamelCase (e.g., `ExtruderStepper`, `PrinterExtruder`)
- Functions/methods: snake_case (e.g., `get_status`, `calc_position`)
- Private members: prefix with underscore (e.g., `_handle_connect`)
- Error handling: raise exceptions with descriptive messages using `logging.exception()`
- Config access: use `config.getint()`, `config.getfloat()`, `config.get()`
- No type hints (code predates widespread type hints)
- 4-space indentation (Python standard)

**C (src/):**
- Standard: `-std=gnu11`, 80-char line limit
- Functions: snake_case (most), some core infra uses CamelCase (e.g., `alloc_chunk`)
- Static functions for module-private scope
- Return type on separate line for function definitions
- No tabs (except Makefiles), 4-space indentation
- Includes grouped by type, use `//` comments for include descriptions

**Error handling patterns:**
- Python: `raise self.printer.command_error("message")` or `gcmd.error("message")`
- Use `logging.info()`, `logging.debug()`, `logging.exception()` appropriately
- C: `shutdown("message")` for fatal errors (macro calling sched_shutdown)

**Naming conventions:**
- Modules/files: snake_case (e.g., `toolhead.py`, `command.c`)
- Config sections: lowercase with underscores
- G-code commands: UPPER_CASE (e.g., `SET_PRESSURE_ADVANCE`)
- Macros/constants: UPPER_CASE (e.g., `MAX_SCHEDULE_TICKS`, `DECL_COMMAND`)
- Status variables: lowercase with underscores (e.g., `pressure_advance`)

**Whitespace rules (enforced):**
- No trailing spaces
- No tabs in source (Makefiles excepted)
- Single newline at EOF (no blank lines at end)
- Lines ≤ 80 characters for source code

## Repo map (common entry points)

- `klippy/`: Host-side Python code (main Klipper logic).
- `klippy/chelper/`: CFFI C helper code compiled for performance.
- `klippy/kinematics/`: Robot kinematics implementations.
- `klippy/extras/`: Extensible modules (sensors, tools).
- `src/`: Micro-controller firmware (C).
- `src/generic/`: Architecture-agnostic C code.
- `src/<arch>/`: Architecture-specific code (avr, stm32, etc.).
- `config/`: Example and reference configuration snippets.
- `docs/`: User and developer documentation.
- `scripts/`: Developer utilities and checks.
- `test/klippy/`: Test cases with `.test` extension.
- `lib/`: External 3rd-party libraries.

## Skills (Codex)

If the user names a skill (for example `$skill-installer`) or if the task
clearly matches a skill description, open that skill's `SKILL.md` and follow
its workflow for the current turn.

## Documenting major changes

When making a *major* change, add an entry to `docs/Agent_Changes.md`.

Treat a change as **major** if it is any of:

- User-visible behavior change (even if backwards compatible).
- Non-trivial refactor touching multiple subsystems/directories.
- Adds/removes/renames modules, commands, config, or protocols.
- Large patch (rule of thumb: ~200+ lines changed across the repo).

The entry should include: what changed, why, impact/risk, and how it was
validated (tests, build, manual check, etc.).
