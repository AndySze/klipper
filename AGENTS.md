# Agent notes (local)

This repository is a checkout of the upstream Klipper project.
Keep changes focused, avoid drive-by refactors, and follow existing
conventions in the surrounding code.

## Repo map (common entry points)

- `klippy/`: Host-side Python code (main Klipper logic).
- `src/`: Micro-controller firmware (C).
- `config/`: Example and reference configuration snippets.
- `docs/`: User and developer documentation.
- `scripts/`: Developer utilities and checks.
- `test/`: Test cases and fixtures.

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
