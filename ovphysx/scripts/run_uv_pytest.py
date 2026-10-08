#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Run pytest under uv without mutating tests/python_tests/uv.lock.

Sets UV_LOCKED=1 and points UV_FIND_LINKS at the build-staged ovstage wheel
directory, spelled relative to the project directory uv runs in. That is the
spelling test_python_runtime.cmake passes, and the one the project's uv.lock
records. Use via tests/python_tests/run_pytest.sh|bat.
"""

# @implements REQ-PACKAGING-TESTDEPS-001
# @covers AC-2 AC-3

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def _usage() -> str:
    return (
        "usage: run_uv_pytest.py <test-project-dir> [pytest args...]\n"
        "example: run_uv_pytest.py tests/python_tests test_clone.py -k reset"
    )


def main() -> int:
    if len(sys.argv) < 2:
        print(_usage(), file=sys.stderr)
        return 2

    test_dir = Path(sys.argv[1]).resolve()
    if not (test_dir / "pyproject.toml").is_file():
        print(f"run_uv_pytest: not a uv project directory: {test_dir}", file=sys.stderr)
        return 2

    ovphysx_dir = test_dir.parents[1]
    wheel_dir = ovphysx_dir / "_build" / "target-deps" / "ovstage_wheel"
    if not any(wheel_dir.glob("ovstage-*.whl")):
        print(
            "run_uv_pytest: ovstage wheel not found in "
            f"{wheel_dir}. Run build.sh (or fetch_ovstage_release.py) first.",
            file=sys.stderr,
        )
        return 1

    # uv runs below with cwd=test_dir and accepts uv.lock as written only when the
    # find-links directory is spelled relative to that directory. An absolute path
    # makes it discard the lock and re-resolve, which has no solution against the
    # single staged wheel. Forward slashes on every platform, matching
    # file(RELATIVE_PATH) in the CMake drivers and the lock itself.
    find_links = os.path.relpath(wheel_dir, test_dir).replace(os.sep, "/")

    env = os.environ.copy()
    env["UV_LOCKED"] = "1"
    env["UV_FIND_LINKS"] = find_links

    cmd = ["uv", "run", "--locked", "pytest", *sys.argv[2:]]
    return subprocess.call(cmd, cwd=test_dir, env=env)


if __name__ == "__main__":
    sys.exit(main())
