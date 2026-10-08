#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Fetch the IsaacLab ANYmal-C asset from NVIDIA's public S3 bucket into
tests/benchmarks/data/anymal/ so the Lab.anymal_* benchmarks activate.

Platform-agnostic. Uses only the stdlib, so packman's bundled Python or any
system python3 runs it.

Idempotent: each file is checked for expected size and skipped if matched.

Usage (from ovphysx/):
    python3 scripts/fetch_anymal_asset.py

Or via cmake (wrapper):
    cmake -P scripts/fetch_anymal_asset.cmake
"""
from __future__ import annotations

import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

# IsaacLab 5.1 ANYmal-C, the production asset IsaacLab uses for locomotion
# environments. Update version/variant here when IsaacLab moves.
ISAAC_VERSION = "5.1"
ANYMAL_VARIANT = "ANYmal-C"
S3_BASE = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    f"/Assets/Isaac/{ISAAC_VERSION}/Isaac/IsaacLab/Robots/ANYbotics/{ANYMAL_VARIANT}"
)

# (source URL suffix, local destination relative to DEST_DIR, expected size in bytes).
# Expected sizes are checked for idempotency and to catch truncated downloads.
ASSETS = [
    # Main articulation USD, renamed locally to anymal.usd so the wrapper
    # (anymal_envs.usda) can reference @./anymal.usd@ regardless of the
    # pinned upstream IsaacLab variant.
    ("anymal_c.usd", "anymal.usd", 33947),
    # Mesh references the main USD pulls in. Without this PhysX falls back
    # to simplified collision and the benchmark numbers are skewed.
    ("Props/instanceable_meshes.usd", "Props/instanceable_meshes.usd", 3580312),
]

CONNECT_TIMEOUT_S = 15.0
TOTAL_TIMEOUT_S = 300.0
MAX_RETRIES = 5
RETRY_DELAY_S = 2.0


def _ovphysx_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _fetch_one(src_url: str, dst: Path, expected_size: int) -> None:
    if dst.exists():
        actual = dst.stat().st_size
        if actual == expected_size:
            print(f"  [skip] {dst} already at {expected_size} bytes")
            return
        print(f"  [refetch] {dst} is {actual} bytes, expected {expected_size}")

    dst.parent.mkdir(parents=True, exist_ok=True)

    last_err: Exception | None = None
    for attempt in range(1, MAX_RETRIES + 1):
        print(f"  [fetch] {src_url} -> {dst} (attempt {attempt}/{MAX_RETRIES})")
        try:
            # The timeout arg only bounds the connect. The stdlib has no knob
            # for total transfer time, so the connect timeout and the small
            # package size (~3.5MB) are relied on instead.
            req = urllib.request.Request(src_url)
            with urllib.request.urlopen(req, timeout=CONNECT_TIMEOUT_S) as resp:
                # Stream to disk rather than buffering 3MB in memory.
                with dst.open("wb") as out:
                    chunk = resp.read(64 * 1024)
                    while chunk:
                        out.write(chunk)
                        chunk = resp.read(64 * 1024)
            break
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, OSError) as e:
            last_err = e
            if attempt < MAX_RETRIES:
                time.sleep(RETRY_DELAY_S)
            else:
                raise RuntimeError(f"download failed for {src_url}: {e}") from e

    if last_err is not None and not dst.exists():
        raise RuntimeError(f"download failed for {src_url}: {last_err}")

    got = dst.stat().st_size
    if got != expected_size:
        raise RuntimeError(f"{dst} is {got} bytes, expected {expected_size}")


def main() -> int:
    dest_dir = _ovphysx_root() / "tests" / "benchmarks" / "data" / "anymal"
    print(f"=== Fetching {ANYMAL_VARIANT} asset (Isaac {ISAAC_VERSION}) ===")
    for src_suffix, dst_rel, expected in ASSETS:
        _fetch_one(f"{S3_BASE}/{src_suffix}", dest_dir / dst_rel, expected)
    print("=== ANYmal asset fetch complete ===")
    print("Lab.anymal_* benchmarks will now activate on the next build/install/run.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
