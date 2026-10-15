#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-6

"""Run pytest under uv without mutating tests/python_tests/uv.lock.

Use via tests/python_tests/run_pytest.sh|bat.
"""

# @implements REQ-PACKAGING-TESTDEPS-001
# @covers AC-1

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

    env = os.environ.copy()
    env["UV_LOCKED"] = "1"

    build_dir = Path(__file__).resolve().parents[1] / (env.get("OVPHYSX_BUILD_DIR") or "_build")
    stamp = build_dir / "ovphysx_dev_schema.stamp"
    if stamp.is_file():
        provider = Path(stamp.read_text(encoding="utf-8").strip())
        complete = all((provider / "physx_usd_schemas" / module / "resources" / filename).is_file()
                       for module in ("PhysxSchema", "OmniUsdPhysicsDeformableSchema")
                       for filename in ("plugInfo.json", "generatedSchema.usda"))
        if (not provider.is_absolute() or not complete
                or len(list(provider.glob("physx_usd_schemas-*.dist-info/METADATA"))) != 1):
            print("Missing local devschema provider. Rebuild with --devschema; refusing to use published schemas instead.",
                  file=sys.stderr)
            return 2
        env["PHYSX_USD_SCHEMAS_ROOT"] = str(provider)
        env["PYTHONPATH"] = str(provider) + os.pathsep + env.get("PYTHONPATH", "")

    cmd = ["uv", "run", "--locked", "pytest", *sys.argv[2:]]
    return subprocess.call(cmd, cwd=test_dir, env=env)


if __name__ == "__main__":
    sys.exit(main())
