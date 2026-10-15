#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# physx-usd-schemas Wheel Build Script
#
# Stages wheel contents in _build/python_wheel_staging/ and builds a pure-Python
# wheel into _dist/.  No compiled binaries -- the wheel tag is py3-none-any.
#
# All Python modules and USD resources are taken from the schema build output
# (_build/schema/) so the wheel always reflects what was actually generated.
# Run './build.sh' (or 'build.bat') before running this script.
#
# Usage (from schemas/physx/):
#   python scripts/build_wheel.py
#   python scripts/build_wheel.py --clean   # remove staging + dist, don't build

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------

SCHEMA_ROOT      = Path(__file__).resolve().parent.parent
PYTHON_DIR       = SCHEMA_ROOT / "python"
BUILD_PYTHON_DIR = SCHEMA_ROOT / "_build" / "schema" / "lib" / "python"
BUILD_PLUGIN_DIR = SCHEMA_ROOT / "_build" / "schema" / "share" / "usd" / "plugins"
STAGING_DIR      = SCHEMA_ROOT / "_build" / "python_wheel_staging"
DIST_DIR         = SCHEMA_ROOT / "_dist"

# The wheel ships a single top-level package. The schema modules and helpers are
# staged as subpackages of it (physx_usd_schemas.PhysxSchema, ...); the package
# __init__ registers all schema plugins on import.
PKG_NAME = "physx_usd_schemas"
PKG_DIR  = STAGING_DIR / PKG_NAME

# Schema modules with USD resources (Python API + plugInfo.json + generatedSchema.usda).
# Names match the build output directories under BUILD_PYTHON_DIR / BUILD_PLUGIN_DIR.
SCHEMA_MODULES = [
    "PhysxSchema",
    "OmniUsdPhysicsDeformableSchema",
]

# Plain Python-only modules from the build output (no USD resources).
# units_db.py is generated at build time and is not present in the source tree.
PLAIN_BUILD_MODULES = [
    "PhysicsSchemaTools",
]

# Pure-Python files to copy from each schema module in the build output.
PYTHON_MODULE_FILES = ["__init__.py", "codeless_api.py", "_tokens.py"]

# USD schema data files to copy from the build plugin directory.
USD_RESOURCE_FILES = ["plugInfo.json", "generatedSchema.usda"]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _parse_pluginfo_json(text: str) -> dict[str, Any]:
    """Parse plugInfo.json, stripping # comment lines that usdGenSchema emits."""
    stripped = re.sub(r"^\s*#[^\n]*\n?", "", text, flags=re.MULTILINE)
    return json.loads(stripped)


def _validate_codeless_pluginfo(plug_info: dict[str, Any], module_name: str) -> None:
    """Fail fast if plugInfo.json breaks the codeless-resource contract.

    Catches regressions from schema regen (e.g. Type reverts to 'library') at
    packaging time rather than silently shipping a broken wheel.
    """
    plugins = plug_info.get("Plugins", [])
    if not plugins:
        raise SystemExit(f"[build_wheel] {module_name}: plugInfo.json has no Plugins entry.")
    for plugin in plugins:
        name = plugin.get("Name")
        if plugin.get("Type") != "resource":
            raise SystemExit(
                f"[build_wheel] {module_name}: plugin {name!r} has "
                f"Type={plugin.get('Type')!r}, expected 'resource'."
            )
        if plugin.get("Root") != "..":
            raise SystemExit(
                f"[build_wheel] {module_name}: plugin {name!r} has "
                f"Root={plugin.get('Root')!r}, expected '..'."
            )
        if plugin.get("ResourcePath") != "resources":
            raise SystemExit(
                f"[build_wheel] {module_name}: plugin {name!r} has "
                f"ResourcePath={plugin.get('ResourcePath')!r}, expected 'resources'."
            )
        if plugin.get("LibraryPath"):
            raise SystemExit(
                f"[build_wheel] {module_name}: plugin {name!r} has non-empty "
                f"LibraryPath={plugin.get('LibraryPath')!r}; codeless schemas must not "
                "reference a compiled library."
            )
        if not plugin.get("Info", {}).get("Types"):
            raise SystemExit(
                f"[build_wheel] {module_name}: plugin {name!r} has no Info.Types; "
                "refusing to package an empty codeless schema."
            )


def _check_build_output() -> None:
    """Abort early if the schema build hasn't been run."""
    if not BUILD_PYTHON_DIR.is_dir():
        raise SystemExit(
            f"[build_wheel] Build output not found: {BUILD_PYTHON_DIR}\n"
            "Run './build.sh' (or 'build.bat') before building the wheel."
        )


def read_version() -> str:
    version = (SCHEMA_ROOT / "VERSION").read_text().strip()
    # Match OVPhysX: prefer the parent build number over the current pipeline.
    build_number = (os.environ.get("PHYSX_SCHEMA_BUILD_NUMBER")
                    or os.environ.get("CI_PIPELINE_ID", "")).strip()
    if not build_number:
        return version
    if not re.fullmatch(r"[0-9]+", build_number):
        print(f"[build_wheel] Ignoring non-numeric build number {build_number!r}", file=sys.stderr)
        return version
    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version):
        return version
    return f"{version}.{build_number}"


def write_version_py(version: str, dst: Path) -> None:
    dst.write_text(
        "# Auto-generated by scripts/build_wheel.py\n"
        f'__version__ = "{version}"\n'
    )


def stage_schema_module(module: str) -> None:
    """Stage a schema module: Python API from build output + USD resources."""
    py_src = BUILD_PYTHON_DIR / module
    if not py_src.is_dir():
        raise SystemExit(
            f"[build_wheel] Build output missing: {py_src}\n"
            "Run './build.sh' (or 'build.bat') before building the wheel."
        )

    dst = PKG_DIR / module
    dst.mkdir(parents=True, exist_ok=True)

    for fname in PYTHON_MODULE_FILES:
        shutil.copy2(py_src / fname, dst / fname)

    # USD schema data -> <module>/resources/ so that
    # Plug.Registry().RegisterPlugins(str(dst / "resources")) works with Root=".."
    plugin_src = BUILD_PLUGIN_DIR / module / "resources"
    if not plugin_src.is_dir():
        raise SystemExit(
            f"[build_wheel] USD plugin resources missing: {plugin_src}\n"
            "Run './build.sh' (or 'build.bat') before building the wheel."
        )
    plug_info_src = plugin_src / "plugInfo.json"
    _validate_codeless_pluginfo(_parse_pluginfo_json(plug_info_src.read_text()), module)
    resources = dst / "resources"
    resources.mkdir(exist_ok=True)
    for fname in USD_RESOURCE_FILES:
        shutil.copy2(plugin_src / fname, resources / fname)


def stage_plain_build_module(module: str) -> None:
    """Stage a pure-Python module from the build output (no USD resources)."""
    src = BUILD_PYTHON_DIR / module
    if not src.is_dir():
        raise SystemExit(
            f"[build_wheel] Build output missing: {src}\n"
            "Run './build.sh' (or 'build.bat') before building the wheel."
        )
    dst = PKG_DIR / module
    dst.mkdir(parents=True, exist_ok=True)

    for py_file in src.glob("*.py"):
        shutil.copy2(py_file, dst / py_file.name)

    # Create __init__.py if the build didn't produce one (e.g. PhysicsSchemaTools).
    init = dst / "__init__.py"
    if not init.exists():
        init.write_text("# Auto-generated by scripts/build_wheel.py\n")


def build_wheel(staging: Path, dist: Path) -> None:
    """Build a pure-Python wheel from the staging tree into dist."""
    dist.mkdir(parents=True, exist_ok=True)
    for stale in dist.glob("physx_usd_schemas-*.whl"):
        stale.unlink()

    subprocess.run(
        [
            sys.executable, "-m", "pip", "wheel",
            "--no-deps",
            "--wheel-dir", str(dist),
            str(staging),
        ],
        check=True,
        cwd=staging,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description="Build physx-usd-schemas wheel.")
    parser.add_argument("--clean", action="store_true", help="Remove staging and dist, then exit.")
    args = parser.parse_args()

    if args.clean:
        for d in (STAGING_DIR, DIST_DIR):
            if d.exists():
                shutil.rmtree(d)
                print(f"Removed {d}")
        return 0

    _check_build_output()

    version = read_version()
    print(f"Building physx-usd-schemas {version}")
    print(f"  Build   : {BUILD_PYTHON_DIR}")
    print(f"  Staging : {STAGING_DIR}")
    print(f"  Dist    : {DIST_DIR}")

    # --- Clean staging ---------------------------------------------------------
    if STAGING_DIR.exists():
        shutil.rmtree(STAGING_DIR)
    STAGING_DIR.mkdir(parents=True)

    # --- Packaging metadata ----------------------------------------------------
    shutil.copy2(PYTHON_DIR / "pyproject.toml", STAGING_DIR / "pyproject.toml")
    readme = PYTHON_DIR / "README.md"
    if readme.exists():
        shutil.copy2(readme, STAGING_DIR / "README.md")

    # --- Top-level package (physx_usd_schemas) ---------------------------------
    # Everything below is staged as a subpackage of this directory, so the wheel
    # ships one importable package whose __init__ registers all schema plugins.
    helper_src = PYTHON_DIR / PKG_NAME
    shutil.copytree(helper_src, PKG_DIR)
    write_version_py(version, PKG_DIR / "_version.py")
    print(f"  Staged  {PKG_NAME}")

    # --- Schema modules (Python API + USD resources from build output) ---------
    # Staged as subpackages: physx_usd_schemas.PhysxSchema, etc.
    for module in SCHEMA_MODULES:
        stage_schema_module(module)
        print(f"  Staged  {PKG_NAME}.{module}")

    # --- Plain build modules (generated at build time, no USD resources) ------
    for module in PLAIN_BUILD_MODULES:
        stage_plain_build_module(module)
        print(f"  Staged  {PKG_NAME}.{module}")

    # --- Build -----------------------------------------------------------------
    print(f"\nRunning pip wheel ...")
    build_wheel(STAGING_DIR, DIST_DIR)

    wheels = sorted(DIST_DIR.glob("physx_usd_schemas-*.whl"))
    if wheels:
        print(f"\nBuilt: {wheels[-1]}")
    else:
        print(f"\nWheel written to {DIST_DIR}/")

    return 0


if __name__ == "__main__":
    sys.exit(main())
