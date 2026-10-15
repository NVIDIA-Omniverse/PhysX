#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-TESTDEPS-001
# @covers AC-1, AC-5
#
"""Require an HTTPS registry and a valid SHA-256 for each locked OVStage wheel."""

from __future__ import annotations

import re
import sys
from pathlib import Path

OVPHYSX_DIR = Path(__file__).resolve().parents[1]


def ovstage_locks(root: Path = OVPHYSX_DIR) -> list[Path]:
    """Find uv locks whose resolved graph records ovstage."""
    return sorted(
        p for p in root.rglob("uv.lock")
        if "_build" not in p.parts
        and '\nname = "ovstage"\n' in p.read_text(encoding="utf-8")
    )

_OVSTAGE_BLOCK = re.compile(
    r'\[\[package\]\]\s*\nname = "ovstage"\s*\nversion = "([^"]+)"\s*\n'
    r'source = \{ registry = "([^"]+)" \}',
)


def validate_lock_text(text: str, label: str = "uv.lock") -> None:
    match = _OVSTAGE_BLOCK.search(text)
    if not match:
        raise ValueError(f'{label}: missing ovstage [[package]] block with a registry source')

    registry = match.group(2)
    if not registry.startswith("https://"):
        raise ValueError(
            f"{label}: ovstage must resolve from an index URL, got {registry!r}. "
            "A relative registry carries no hashes. Regenerate with `uv lock`."
        )

    block_start = match.start()
    next_pkg = text.find("[[package]]", block_start + 1)
    block = text[block_start : next_pkg if next_pkg != -1 else len(text)]

    # Parse fields rather than anchoring on the closing brace: a PyPI-sourced wheel carries
    # `size` and `upload-time` after `hash`.
    wheels = []
    for entry in re.findall(r"\{([^{}]*)\}", block[block.find("wheels = ["):]):
        url = re.search(r'\burl = "([^"]+)"', entry)
        if not url:
            continue
        digest = re.search(r'\bhash = "([^"]+)"', entry)
        wheels.append((url.group(1), digest.group(1) if digest else None))
    if not wheels:
        raise ValueError(f"{label}: ovstage lists no wheel URLs")
    invalid = [url for url, digest in wheels if not re.fullmatch(r"sha256:[0-9a-f]{64}", digest or "")]
    if invalid:
        raise ValueError(
            f"{label}: ovstage wheels must carry a sha256 hash; missing or invalid for: "
            + ", ".join(invalid)
        )


def validate_lock_file(lock_path: Path) -> None:
    if not lock_path.is_file():
        raise FileNotFoundError(f"Missing uv.lock at {lock_path}")
    validate_lock_text(lock_path.read_text(encoding="utf-8"), str(lock_path))


def validate_all() -> None:
    locks = ovstage_locks()
    if not locks:
        raise ValueError("no uv.lock records ovstage; the discovery is probably broken")
    for lock_path in locks:
        validate_lock_file(lock_path)


def main() -> int:
    try:
        validate_all()
    except (OSError, ValueError) as exc:
        print(f"validate_python_test_uv_lock: {exc}", file=sys.stderr)
        return 1
    print("validate_python_test_uv_lock: ok (%d lock(s))" % len(ovstage_locks()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
