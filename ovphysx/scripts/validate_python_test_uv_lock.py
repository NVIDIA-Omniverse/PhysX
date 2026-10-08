#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Validate every checked-in uv.lock resolves ovstage from build-staged wheels.

Accidental `uv run` without find-links/--locked can rewrite ovstage to PyPI URLs
while keeping the same version pin. CI and unit tests enforce the local-wheel
registry format instead.

Each lock records the staged wheel directory as a path relative to its own project
directory, so the expected registry is derived per lock the way the CMake drivers
derive their find-links, rather than hardcoded once. Locks that are absent are
skipped: the public source drop ships python/uv.lock and tests/python_tests/uv.lock,
not tests/python_benchmarks/uv.lock. A lock that is present but malformed still fails.

@implements REQ-PACKAGING-TESTDEPS-001
@covers AC-5
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

OVPHYSX_DIR = Path(__file__).resolve().parents[1]

# Where scripts/fetch_deps.cmake stages the host platform's ovstage wheel.
OVSTAGE_WHEEL_DIR = OVPHYSX_DIR / "_build" / "target-deps" / "ovstage_wheel"

# The locks whose resolved graph records ovstage. Mirrors the targets the ovstage
# version bump stamps. A checked-in test keeps this list and the tree itself in
# agreement.
LOCK_PATHS = (
    OVPHYSX_DIR / "python" / "uv.lock",
    OVPHYSX_DIR / "tests" / "python_tests" / "uv.lock",
    OVPHYSX_DIR / "tests" / "python_benchmarks" / "uv.lock",
)


def lock_label(lock_path: Path) -> str:
    """The lock's path relative to ovphysx/, for messages."""
    return Path(os.path.relpath(lock_path, OVPHYSX_DIR)).as_posix()


def expected_registry(lock_path: Path) -> str:
    """The staged wheel directory as the lock's own project directory spells it.

    Purely lexical, like CMake's file(RELATIVE_PATH): _build/ need not exist. Two
    levels down (tests/python_tests, tests/python_benchmarks) this is
    ../../_build/target-deps/ovstage_wheel; one level down (python) it is
    ../_build/target-deps/ovstage_wheel.
    """
    return Path(os.path.relpath(OVSTAGE_WHEEL_DIR, lock_path.parent)).as_posix()


def validate_lock_text(text: str, registry: str, label: str = "uv.lock") -> None:
    match = re.search(
        r'\[\[package\]\]\s*\nname = "ovstage"\s*\nversion = "[^"]+"\s*\n'
        r'source = \{ registry = "([^"]+)" \}',
        text,
    )
    if not match:
        raise ValueError(f'{label}: missing ovstage [[package]] block with registry source')

    found = match.group(1)
    if found != registry:
        raise ValueError(
            f"{label}: ovstage must use registry "
            f"{registry!r}, got {found!r}. "
            f"Revert the lock (git checkout -- {label}). Regenerating it needs all three "
            f"supported platforms' ovstage wheels staged and UV_FIND_LINKS={registry} "
            "passed from that project directory; the build stages only the host wheel, "
            "and a plain `uv lock` recreates the registry rejected here."
        )

    block_start = match.start()
    next_pkg = text.find("[[package]]", block_start + 1)
    block = text[block_start : next_pkg if next_pkg != -1 else len(text)]

    if 'url = "https://' in block:
        raise ValueError(
            f"{label}: ovstage wheels must use local {{ path = ... }}, not PyPI urls"
        )
    if '{ path = "ovstage-' not in block:
        raise ValueError(f"{label}: ovstage must list local wheel paths")


def validate_lock_file(lock_path: Path) -> None:
    if not lock_path.is_file():
        raise FileNotFoundError(f"Missing uv.lock at {lock_path}")
    validate_lock_text(
        lock_path.read_text(encoding="utf-8"), expected_registry(lock_path), lock_label(lock_path)
    )


def validate_all_locks(lock_paths=LOCK_PATHS) -> list[str]:
    """Validate the locks that are present and return their labels.

    An absent lock is not a failure (the public source drop keeps two of the three),
    but a tree with none of them is, so a stripped or misconfigured checkout cannot
    pass vacuously.
    """
    checked = []
    for lock_path in lock_paths:
        if not lock_path.is_file():
            continue
        validate_lock_file(lock_path)
        checked.append(lock_label(lock_path))
    if not checked:
        raise FileNotFoundError(
            "no checked-in uv.lock recording ovstage found under %s (expected at least one of %s)"
            % (OVPHYSX_DIR, ", ".join(lock_label(p) for p in lock_paths))
        )
    return checked


def main() -> int:
    try:
        checked = validate_all_locks()
    except (OSError, ValueError) as exc:
        print(f"validate_python_test_uv_lock: {exc}", file=sys.stderr)
        return 1
    print("validate_python_test_uv_lock: OK (%s)" % ", ".join(checked))
    return 0


if __name__ == "__main__":
    sys.exit(main())
