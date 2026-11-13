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
import re
import stat
import subprocess
import sys
from pathlib import Path


def _usage() -> str:
    return (
        "usage: run_uv_pytest.py <test-project-dir> [pytest args...]\n"
        "example: run_uv_pytest.py tests/python_tests test_clone.py -k reset"
    )


def _build_path(project: Path, value: str | Path) -> Path:
    """Keep managed test state beneath _build without traversing directory links."""
    if re.match(r"^[A-Za-z]:($|[^/\\])", str(value)):
        raise ValueError("Build paths must be inside ovphysx/_build; drive-relative paths are unsupported.")
    root = project / "_build"
    path = Path(os.path.abspath(project / value))
    if not path.is_relative_to(root):
        raise ValueError("Build paths must be inside ovphysx/_build.")
    components = [root]
    for part in path.relative_to(root).parts:
        components.append(components[-1] / part)
    for component in components:
        # lstat also sees dangling symlinks and Windows junctions without following them.
        try:
            status = component.lstat()
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(status.st_mode) or (
                getattr(status, "st_file_attributes", 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT):
            raise ValueError(f"Build paths under ovphysx/_build cannot use directory links: {component}")
    return path


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

    project = Path(__file__).resolve().parents[1]
    try:
        build_dir = _build_path(project, env.get("OVPHYSX_BUILD_DIR") or "_build")
        env["UV_PROJECT_ENVIRONMENT"] = str(_build_path(project, build_dir / "venv/python_tests"))
        env["UV_CACHE_DIR"] = str(_build_path(project, build_dir / ".uv_cache"))
        env["PYTHONPYCACHEPREFIX"] = str(_build_path(project, build_dir / "pycache"))
        pytest_cache = _build_path(project, build_dir / "pytest_cache/python_tests")
    except (ValueError, OSError) as error:
        print(f"run_uv_pytest: {error}", file=sys.stderr)
        return 2
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

    # The test project declares dependencies but has no package to install.
    result = subprocess.call(["uv", "sync", "--locked", "--no-install-project"], cwd=test_dir, env=env)
    if result:
        return result
    cmd = ["uv", "run", "--no-sync", "pytest", "-o", f"cache_dir={pytest_cache}", *sys.argv[2:]]
    return subprocess.call(cmd, cwd=test_dir, env=env)


if __name__ == "__main__":
    sys.exit(main())
