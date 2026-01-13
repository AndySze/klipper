#!/usr/bin/env python3
# Golden output harness for the Go host migration.
#
# This script drives the existing Klipper regression runner
# (scripts/test_klippy.py) and converts its binary debug output into a
# stable, human-readable form via klippy/parsedump.py.
#
# It is intended to provide a repeatable "Python baseline" for later
# Go-vs-Python equivalence testing.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
import difflib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


class Fatal(Exception):
    pass


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]

def _is_executable_file(path: str) -> bool:
    p = Path(path)
    return p.exists() and p.is_file() and os.access(str(p), os.X_OK)


def ensure_python(python: str) -> None:
    if _is_executable_file(python):
        return
    raise Fatal(
        "\n".join(
            [
                f"python executable not found: {python}",
                "",
                "Fix one of:",
                "- Create a venv at ~/klippy-env and install deps:",
                "    python3 -m venv ~/klippy-env",
                "    ~/klippy-env/bin/pip install -r",
                "        scripts/klippy-requirements.txt",
                "- Or pass an explicit interpreter:",
                "    ./scripts/go_migration_golden.py gen --python",
                "        /path/to/python",
            ]
        )
    )


def read_suite_file(path: Path) -> list[str]:
    items: list[str] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        items.append(line)
    if not items:
        raise Fatal(f"suite is empty: {path}")
    return items


_DICT_RE = re.compile(r"^DICTIONARY\s+(.+)$")
_CONFIG_RE = re.compile(r"^CONFIG\s+(\S+)\s*$")
_GCODE_RE = re.compile(r"^GCODE\s+(\S+)\s*$")
_MCU_HDR_RE = re.compile(r"^##\s+MCU:\s+(.+?)\s+\(dict:\s+(.+?)\)\s*$")
_CLOCK_TOKEN_RE = re.compile(r"\b([A-Za-z_]*clock)=(\d+)\b")


def parse_test_configs_and_dict(
    test_path: Path,
) -> tuple[list[str], dict[str, str]]:
    """Return (config_paths, dictionaries) where dictionaries maps mcu->dict.

    The main MCU is stored under key 'mcu'. Additional MCUs are stored under
    their MCU name. Multiple CONFIG entries are supported for multi-config
    tests.
    """
    config_values: list[str] = []
    dict_map: dict[str, str] = {}
    for raw in test_path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        m = _CONFIG_RE.match(line)
        if m:
            config_values.append(m.group(1))
            continue
        m = _DICT_RE.match(line)
        if m:
            parts = m.group(1).split()
            if not parts:
                continue
            dict_map["mcu"] = parts[0]
            for extra in parts[1:]:
                if "=" not in extra:
                    raise Fatal(
                        f"unsupported DICTIONARY token {extra!r} in {test_path}"
                    )
                mcu_name, fname = extra.split("=", 1)
                dict_map[mcu_name.strip()] = fname.strip()
    if not dict_map:
        raise Fatal(f"missing DICTIONARY in {test_path}")
    if not config_values:
        raise Fatal(f"missing CONFIG in {test_path}")
    return config_values, dict_map


def parse_test_file(test_path: Path) -> dict[str, object]:
    """Parse a Klipper regression .test file (supports multiple CONFIGs)."""
    config_rels, dict_map = parse_test_configs_and_dict(test_path)
    should_fail = False
    gcode_rel: str | None = None
    inline_gcode: list[str] = []

    for raw in test_path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if line.strip() == "SHOULD_FAIL":
            should_fail = True
            continue
        m = _GCODE_RE.match(line.strip())
        if m:
            gcode_rel = m.group(1)
            continue
        if line.strip().startswith(("CONFIG ", "DICTIONARY ")):
            continue
        inline_gcode.append(line.strip())

    if gcode_rel is not None and inline_gcode:
        raise Fatal(
            f"{test_path} specifies both GCODE file and inline commands"
        )

    return {
        "config_rels": config_rels,  # list of configs
        "dict_map": dict_map,
        "should_fail": should_fail,
        "gcode_rel": gcode_rel,
        "inline_gcode": inline_gcode,
    }

def parse_mcu_sections(text: str) -> dict[str, dict[str, object]]:
    """Parse expected/actual files into MCU sections.

    The canonical format is:

        ## MCU: <name> (dict: <dictfile>)
        <command line>
        <command line>
    """
    sections: dict[str, dict[str, object]] = {}
    cur_name: str | None = None
    cur_dict: str | None = None
    cur_lines: list[str] = []

    def flush() -> None:
        nonlocal cur_name, cur_dict, cur_lines
        if cur_name is None:
            return
        if cur_name in sections:
            raise Fatal(f"duplicate MCU section: {cur_name}")
        # Strip trailing blank lines
        while cur_lines and cur_lines[-1] == "":
            cur_lines.pop()
        sections[cur_name] = {
            "dict": cur_dict or "",
            "lines": cur_lines,
        }
        cur_name = None
        cur_dict = None
        cur_lines = []

    for raw in normalize_text(text).splitlines():
        m = _MCU_HDR_RE.match(raw)
        if m:
            flush()
            cur_name = m.group(1).strip()
            cur_dict = m.group(2).strip()
            continue
        if raw.startswith("#"):
            continue
        if cur_name is None:
            continue
        cur_lines.append(raw.rstrip())
    flush()
    return sections


def normalize_mcu_lines(lines: list[str], mode: str) -> list[str]:
    if mode == "strict":
        return lines
    if mode != "relaxed-clock":
        raise Fatal(f"unknown compare mode: {mode}")

    clock_values: list[int] = []
    for ln in lines:
        for _, v in _CLOCK_TOKEN_RE.findall(ln):
            clock_values.append(int(v))
    if not clock_values:
        return lines
    base = min(clock_values)

    def repl(m: re.Match[str]) -> str:
        key = m.group(1)
        val = int(m.group(2))
        return f"{key}=+{val - base}"

    return [_CLOCK_TOKEN_RE.sub(repl, ln) for ln in lines]


def normalize_text(text: str) -> str:
    lines = [
        ln.rstrip()
        for ln in text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
    ]
    while lines and lines[-1] == "":
        lines.pop()
    return "\n".join(lines) + "\n"


def run(cmd: list[str], cwd: Path) -> None:
    proc = subprocess.run(cmd, cwd=str(cwd))
    if proc.returncode != 0:
        raise Fatal(f"command failed ({proc.returncode}): {' '.join(cmd)}")


def run_capture(cmd: list[str], cwd: Path) -> str:
    proc = subprocess.run(
        cmd,
        cwd=str(cwd),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if proc.returncode != 0:
        raise Fatal(
            "command failed (%d): %s\n--- stderr ---\n%s"
            % (proc.returncode, " ".join(cmd), proc.stderr)
        )
    return proc.stdout


def output_file_mcu_name(fname: str) -> str:
    # _test_output -> main mcu
    # _test_output-foo -> foo
    prefix = "_test_output"
    if fname == prefix:
        return "mcu"
    if fname.startswith(prefix + "-"):
        return fname[len(prefix) + 1 :]
    return ""

def required_dict_files_from_tests(
    root: Path, test_paths: list[str]
) -> set[str]:
    required: set[str] = set()
    for test_rel in test_paths:
        test_path = (root / test_rel).resolve()
        spec = parse_test_file(test_path)
        dict_map = spec["dict_map"]
        required.update(dict_map.values())
    return required


def ensure_dict_bundle(dictdir: Path, required: set[str]) -> None:
    missing = sorted([d for d in required if not (dictdir / d).exists()])
    if not missing:
        return
    preview = "\n".join([f"- {d}" for d in missing[:25]])
    if len(missing) > 25:
        preview += f"\n- (+{len(missing) - 25} more)"
    raise Fatal(
        "\n".join(
            [
                f"dictionary bundle incomplete under: {dictdir}",
                "missing files:",
                preview,
                "",
                "You need the prebuilt *.dict files used by Klipper tests.",
                "See docs/Debugging.md ('Running the regression tests').",
            ]
        )
    )


def _gen_single_config(
    python: str,
    dictdir: Path,
    test_rel: str,
    test_path: Path,
    config_rel: str,
    dict_map: dict[str, str],
    should_fail: bool,
    gcode_rel: str | None,
    inline_gcode: list[str],
    case_dir: Path,
    keep_work: bool,
) -> None:
    """Generate golden output for a single config within a test file."""
    root = repo_root()
    work_dir = case_dir / "work"
    case_dir.mkdir(parents=True, exist_ok=True)
    work_dir.mkdir(parents=True, exist_ok=True)

    # Clean previous work outputs
    for p in work_dir.iterdir():
        if p.name.startswith("_test_output") or p.name in (
            "_test_.log",
            "_test_.gcode",
        ):
            p.unlink()
    # Clean previous preserved raw outputs
    for p in case_dir.glob("raw-*.bin"):
        p.unlink()

    config_path = (test_path.parent / config_rel).resolve()
    if not config_path.exists():
        raise Fatal(f"missing config file: {config_path}")

    if gcode_rel is not None:
        gcode_path = (test_path.parent / str(gcode_rel)).resolve()
        if not gcode_path.exists():
            raise Fatal(f"missing gcode file: {gcode_path}")
    else:
        gcode_path = (work_dir / "_test_.gcode").resolve()
        gcode_path.write_text(
            "\n".join(list(inline_gcode) + [""]),
            encoding="utf-8",
        )

    out_prefix = (work_dir / "_test_output").resolve()
    args = [
        python,
        "./klippy/klippy.py",
        str(config_path),
        "-i",
        str(gcode_path),
        "-o",
        str(out_prefix),
        "-v",
    ]
    main_dict = dict_map.get("mcu")
    if main_dict is None:
        raise Fatal(f"missing main mcu dict in {test_rel}")
    args += ["-d", str((dictdir / main_dict).resolve())]
    for mcu_name, dname in sorted(dict_map.items()):
        if mcu_name == "mcu":
            continue
        args += ["-d", f"{mcu_name}={str((dictdir / dname).resolve())}"]

    proc = subprocess.run(
        args,
        cwd=str(root),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    run_out = normalize_text(proc.stdout)
    run_err = normalize_text(proc.stderr)
    (work_dir / "run.stdout").write_text(run_out, encoding="utf-8")
    (work_dir / "run.stderr").write_text(run_err, encoding="utf-8")
    (case_dir / "stdout.txt").write_text(run_out, encoding="utf-8")
    (case_dir / "stderr.txt").write_text(run_err, encoding="utf-8")

    if should_fail and proc.returncode == 0:
        raise Fatal(f"{test_rel} expected failure but klippy returned 0")
    if not should_fail and proc.returncode != 0:
        raise Fatal(
            f"{test_rel} [{config_rel}] failed (rc={proc.returncode}); "
            f"see {work_dir}/run.stderr"
        )

    outputs = sorted(
        [p for p in work_dir.iterdir() if p.name.startswith("_test_output")]
    )
    if not outputs and not should_fail:
        raise Fatal(f"no _test_output files produced for {test_rel}")

    sections: list[dict[str, str]] = []
    for op in outputs:
        mcu_name = output_file_mcu_name(op.name)
        if not mcu_name:
            continue
        dict_fname = dict_map.get(mcu_name)
        if dict_fname is None:
            raise Fatal(
                "found output %s but no matching dict for mcu %r in %s"
                % (op.name, mcu_name, test_rel)
            )
        dict_path = (dictdir / dict_fname).resolve()
        if not dict_path.exists():
            raise Fatal(f"missing dict file: {dict_path}")

        # Preserve the raw binary output alongside the decoded text so that the
        # Go side can re-decode it without needing Python's parsedump.
        raw_out = case_dir / f"raw-{mcu_name}.bin"
        shutil.copyfile(str(op), str(raw_out))

        parsed = run_capture(
            [
                python,
                "klippy/parsedump.py",
                str(dict_path),
                str(op),
            ],
            cwd=root,
        )
        sections.append(
            {
                "mcu": mcu_name,
                "dict": dict_fname,
                "output_file": op.name,
                "text": normalize_text(parsed),
            }
        )

    # Write expected output
    expected_path = case_dir / "expected.txt"
    with expected_path.open("w", encoding="utf-8", newline="\n") as f:
        f.write(f"# Source: {test_rel}\n")
        f.write(f"# Config: {config_rel}\n")
        f.write(f"# Expected failure: {should_fail}\n")
        f.write(f"# Return code: {proc.returncode}\n")
        for sec in sorted(
            sections, key=lambda s: (s["mcu"] != "mcu", s["mcu"])
        ):
            f.write(f"\n## MCU: {sec['mcu']} (dict: {sec['dict']})\n\n")
            f.write(sec["text"])

    meta = {
        "source_test": test_rel,
        "config": config_rel,
        "dicts": dict_map,
        "should_fail": should_fail,
        "return_code": proc.returncode,
        "outputs": [
            {
                "mcu": s["mcu"],
                "dict": s["dict"],
                "file": s["output_file"],
                "raw": f"raw-{s['mcu']}.bin",
            }
            for s in sections
        ],
        "generator": {"script": "scripts/go_migration_golden.py"},
    }
    (case_dir / "meta.json").write_text(
        json.dumps(meta, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    if not keep_work:
        for p in work_dir.iterdir():
            p.unlink()
        work_dir.rmdir()


def gen_one(
    python: str,
    dictdir: Path,
    test_rel: str,
    outdir: Path,
    keep_work: bool,
) -> None:
    """Generate golden outputs for a test file (supports multiple CONFIGs)."""
    root = repo_root()
    test_path = (root / test_rel).resolve()
    if not test_path.exists():
        raise Fatal(f"missing test: {test_rel}")
    spec = parse_test_file(test_path)
    config_rels = list(spec["config_rels"])  # type: ignore[arg-type]
    dict_map = spec["dict_map"]  # type: ignore[assignment]
    should_fail = bool(spec["should_fail"])
    gcode_rel = spec["gcode_rel"]  # type: ignore[assignment]
    inline_gcode = list(spec["inline_gcode"])  # type: ignore[arg-type]

    is_multi = len(config_rels) > 1

    for config_rel in config_rels:
        # For multi-config tests, use nested directory structure:
        #   golden/<test_stem>/<config_stem>/
        # For single-config tests, use flat structure:
        #   golden/<test_stem>/
        if is_multi:
            config_stem = Path(config_rel).stem
            case_dir = outdir / test_path.stem / config_stem
        else:
            case_dir = outdir / test_path.stem

        _gen_single_config(
            python=python,
            dictdir=dictdir,
            test_rel=test_rel,
            test_path=test_path,
            config_rel=config_rel,
            dict_map=dict_map,
            should_fail=should_fail,
            gcode_rel=gcode_rel,
            inline_gcode=inline_gcode,
            case_dir=case_dir,
            keep_work=keep_work,
        )


def cmd_gen(args: argparse.Namespace) -> None:
    root = repo_root()
    suite_path = (root / args.suite).resolve()
    if Path(args.dictdir).is_absolute():
        dictdir = Path(args.dictdir)
    else:
        dictdir = (root / args.dictdir).resolve()
    outdir = (root / args.outdir).resolve()

    if not suite_path.exists():
        raise Fatal(f"missing suite file: {suite_path}")
    if not dictdir.exists():
        raise Fatal(f"missing dictdir: {dictdir}")

    ensure_python(args.python)
    tests = read_suite_file(suite_path)
    ensure_dict_bundle(dictdir, required_dict_files_from_tests(root, tests))
    for t in tests:
        gen_one(args.python, dictdir, t, outdir, keep_work=args.keep_work)


def _compare_case_dir(
    case_dir: Path,
    mode: str,
    fail_missing: bool,
) -> bool:
    """Compare expected vs actual for a single case directory.

    Returns True if comparison failed, False if passed or skipped.
    """
    expected = case_dir / "expected.txt"
    actual = case_dir / "actual.txt"
    if not expected.exists():
        raise Fatal(f"missing expected output: {expected}")
    if not actual.exists():
        if fail_missing:
            raise Fatal(f"missing actual output: {actual}")
        return False  # skipped
    exp_sections = parse_mcu_sections(expected.read_text(encoding="utf-8"))
    act_sections = parse_mcu_sections(actual.read_text(encoding="utf-8"))
    if exp_sections.keys() != act_sections.keys():
        sys.stdout.write(f"--- {expected}\n+++ {actual}\n")
        sys.stdout.write(
            "ERROR: MCU section mismatch\n"
            f"expected: {sorted(exp_sections.keys())}\n"
            f"actual:   {sorted(act_sections.keys())}\n"
        )
        return True
    any_diff = False
    for mcu_name in sorted(exp_sections.keys()):
        exp_lines = [
            ln + "\n"
            for ln in exp_sections[mcu_name]["lines"]  # type: ignore[index]
        ]
        act_lines = [
            ln + "\n"
            for ln in act_sections[mcu_name]["lines"]  # type: ignore[index]
        ]
        exp_norm = normalize_mcu_lines(
            [ln.rstrip("\n") for ln in exp_lines],
            mode,
        )
        act_norm = normalize_mcu_lines(
            [ln.rstrip("\n") for ln in act_lines],
            mode,
        )
        exp_lines = [ln + "\n" for ln in exp_norm]
        act_lines = [ln + "\n" for ln in act_norm]
        if exp_lines == act_lines:
            continue
        any_diff = True
        fromfile = f"{expected} (MCU {mcu_name})"
        tofile = f"{actual} (MCU {mcu_name})"
        diff = difflib.unified_diff(
            exp_lines,
            act_lines,
            fromfile=fromfile,
            tofile=tofile,
        )
        sys.stdout.writelines(diff)
    return any_diff


def cmd_compare(args: argparse.Namespace) -> None:
    root = repo_root()
    suite_path = (root / args.suite).resolve()
    outdir = (root / args.outdir).resolve()

    tests = read_suite_file(suite_path)
    any_failed = False
    for test_rel in tests:
        test_path = (root / test_rel).resolve()
        stem = Path(test_rel).stem
        if args.only and args.only != test_rel and args.only != stem:
            continue

        # Parse test file to determine if it's multi-config
        spec = parse_test_file(test_path)
        config_rels = list(spec["config_rels"])  # type: ignore[arg-type]
        is_multi = len(config_rels) > 1

        if is_multi:
            # Multi-config test: check each nested config directory
            for config_rel in config_rels:
                config_stem = Path(config_rel).stem
                case_dir = outdir / stem / config_stem
                if not case_dir.exists():
                    # Config not generated yet, skip
                    continue
                if _compare_case_dir(case_dir, args.mode, args.fail_missing):
                    any_failed = True
        else:
            # Single-config test: use flat directory structure
            case_dir = outdir / stem
            if _compare_case_dir(case_dir, args.mode, args.fail_missing):
                any_failed = True

    if any_failed:
        raise Fatal("compare failed (diffs above)")


def cmd_list(args: argparse.Namespace) -> None:
    root = repo_root()
    suite_path = (root / args.suite).resolve()
    tests = read_suite_file(suite_path)
    for t in tests:
        print(t)


def main() -> None:
    p = argparse.ArgumentParser(prog="go_migration_golden")
    sub = p.add_subparsers(dest="cmd", required=True)

    gen = sub.add_parser(
        "gen",
        help="generate golden expected outputs from .test files",
    )
    gen.add_argument(
        "--python",
        default=os.path.expanduser("~/klippy-env/bin/python"),
    )
    gen.add_argument("--dictdir", default="dict")
    gen.add_argument("--suite", default="test/go_migration/suites/minimal.txt")
    gen.add_argument("--outdir", default="test/go_migration/golden")
    gen.add_argument("--keep-work", action="store_true")
    gen.set_defaults(func=cmd_gen)

    cmp_ = sub.add_parser(
        "compare",
        help="compare golden expected vs actual outputs",
    )
    cmp_.add_argument("--suite", default="test/go_migration/suites/minimal.txt")
    cmp_.add_argument("--outdir", default="test/go_migration/golden")
    cmp_.add_argument("--fail-missing", action="store_true")
    cmp_.add_argument(
        "--only",
        default="",
        help="only compare a single test (path or stem)",
    )
    cmp_.add_argument(
        "--mode",
        choices=["strict", "relaxed-clock"],
        default="strict",
        help="comparison mode (default: strict)",
    )
    cmp_.set_defaults(func=cmd_compare)

    ls = sub.add_parser("list", help="list tests in the suite")
    ls.add_argument("--suite", default="test/go_migration/suites/minimal.txt")
    ls.set_defaults(func=cmd_list)

    args = p.parse_args()
    try:
        args.func(args)
    except BrokenPipeError:
        # Allow piping output to tools like `head` without a traceback.
        try:
            sys.stdout.close()
        except Exception:
            pass
        sys.exit(1)
    except Fatal as e:
        sys.stderr.write(f"ERROR: {e}\n")
        sys.exit(2)


if __name__ == "__main__":
    main()
