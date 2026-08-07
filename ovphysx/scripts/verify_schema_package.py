#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""
Verify that a physics schema tree is a valid codeless schema package.

What problem this solves
------------------------
ovphysx depends on a "physics schema" package (`usd_ext_physics`). The schema
is codeless: it ships USD plugin data (`plugInfo.json` as `Type=resource` plus
`generatedSchema.usda`) and Python, but NO native library. Because the schema
carries no compiled code, it is platform- and USD-ABI-independent -- there is
nothing to link and no per-library USD ABI to get wrong.

This check guards against a regression to the old codefull layout (any native
artifact sneaking back in, or a `plugInfo.json` still declaring `Type=library`
with a `LibraryPath`), which would silently reintroduce the ABI hazard the
codeless migration removed.

What this script does
---------------------
Given a path to an unpacked schema package tree, it:

  1. Fails if any native binary artifact exists anywhere in the tree
     (.so/.so.*, .dll, .pyd, .dylib, .a, .lib) -- the package is
     platform-independent and must ship no compiled code.
  2. Finds the schema plugInfo.json files under <dir>/share/usd/plugins,
     parsing each with only the generated `#` header tolerated ('//' is not,
     since PXR rejects it at RegisterPlugins() time).
  3. Fails if any plugin declares a non-resource type or a LibraryPath, and
     requires at least one Type=resource plugin with Name/Info/Info.Types and a
     matching generatedSchema.usda.
  4. Exits 0 when the tree is a clean codeless schema, or 1 with a
     human-readable explanation.

Where this is called from
-------------------------
  - scripts/build.cmake -- right after the schema package is fetched from
    packman (and after a local --devschema build), so a bad package fails the
    build before a single source file is compiled.
  - scripts/package_deps.py -- right before the schema data is staged into
    _install/ or the wheel, so a direct install/wheel invocation (which may
    skip build.cmake) still refuses bad input.

Usage
-----
    python scripts/verify_schema_package.py \\
        --dir <path to usd_ext_physics/>

Exit status
-----------
    0  the package is a clean codeless schema
    1  a native lib is present, a plugInfo is not codeless, or data is missing
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# A codeless package is platform-independent and ships no compiled code, so any
# native binary artifact anywhere in the tree is a contract violation.
NATIVE_ARTIFACT_SUFFIXES = (".so", ".dll", ".pyd", ".dylib", ".a", ".lib")


def _is_native_artifact(name: str) -> bool:
    lower = name.lower()
    # Plain suffixes plus versioned ELF sonames (libfoo.so.1, libfoo.so.1.2).
    return lower.endswith(NATIVE_ARTIFACT_SUFFIXES) or ".so." in lower


def _native_artifacts(schema_dir: Path) -> list[Path]:
    """Return every native binary artifact anywhere in the package."""
    return sorted(
        p for p in schema_dir.rglob("*")
        if p.is_file() and not p.is_symlink() and _is_native_artifact(p.name)
    )


def _load_pluginfo(pluginfo: Path) -> dict:
    """Parse a USD plugInfo.json, tolerating only its '#' comment header.

    usdGenSchema emits a '#'-prefixed header, which PXR's plugInfo reader
    tolerates but Python's json does not, so strip '#' comment lines before
    decoding. '//' is deliberately NOT tolerated: PXR rejects it at
    RegisterPlugins() time ("Invalid value"), so a '//'-bearing file must fail
    here too rather than be normalized into something the runtime would refuse.
    """
    lines = pluginfo.read_text(encoding="utf-8").splitlines()
    stripped = [ln for ln in lines if not ln.lstrip().startswith("#")]
    return json.loads("\n".join(stripped))


def _pluginfo_result(pluginfo: Path) -> tuple[list[str], int]:
    """Return (errors, valid_resource_plugin_count) for one plugInfo.json.

    A plugin counts as valid only if it is a Type=resource plugin with the
    metadata USD needs to register schema types (Name, Info, Info.Types), no
    LibraryPath, and its resource data (generatedSchema.usda) present.
    """
    try:
        data = _load_pluginfo(pluginfo)
    except (OSError, json.JSONDecodeError) as exc:
        return [f"{pluginfo}: could not parse plugInfo.json: {exc}"], 0

    errors: list[str] = []
    valid = 0
    for plugin in data.get("Plugins", []):
        name = plugin.get("Name") or pluginfo.parent.name
        plugin_type = plugin.get("Type")
        if plugin_type != "resource":
            errors.append(
                f"{name}: plugInfo Type is '{plugin_type}', expected 'resource' "
                f"(codeless schema)."
            )
            continue
        problems: list[str] = []
        if plugin.get("LibraryPath"):
            problems.append(
                f"declares LibraryPath '{plugin['LibraryPath']}' (a codeless schema ships no library)"
            )
        missing = [k for k in ("Name", "Info") if not plugin.get(k)]
        if not (plugin.get("Info") or {}).get("Types"):
            missing.append("Info.Types")
        if missing:
            problems.append(f"missing required metadata: {', '.join(missing)}")
        # The resource data resolves to <ResourcePath> next to plugInfo.json.
        resources_dir = pluginfo.parent
        if plugin.get("ResourcePath", "resources") != resources_dir.name:
            problems.append(
                f"ResourcePath '{plugin.get('ResourcePath')}' does not match resources dir '{resources_dir.name}'"
            )
        if not (resources_dir / "generatedSchema.usda").is_file():
            problems.append(f"generatedSchema.usda missing in {resources_dir}")
        if problems:
            errors.append(f"{name}: " + "; ".join(problems) + ".")
        else:
            valid += 1
    return errors, valid


def verify_schema_package(schema_dir: Path) -> list[str]:
    """Verify the tree at `schema_dir` is a clean codeless schema package.

    Returns an empty list on success, or a list of human-readable error
    strings on failure.
    """
    errors: list[str] = []

    for artifact in _native_artifacts(schema_dir):
        errors.append(
            f"unexpected native artifact {artifact.relative_to(schema_dir)}; the schema is "
            f"codeless and platform-independent and must ship no compiled code."
        )

    pluginfos = sorted((schema_dir / "share" / "usd" / "plugins").rglob("plugInfo.json"))
    if not pluginfos:
        errors.append(
            f"no schema plugInfo.json found under {schema_dir}/share/usd/plugins; "
            f"this does not look like a schema package."
        )

    total_valid = 0
    for pluginfo in pluginfos:
        errs, valid = _pluginfo_result(pluginfo)
        errors.extend(errs)
        total_valid += valid

    if pluginfos and total_valid == 0:
        errors.append(
            "no valid resource schema plugin found; the package must contain at least one "
            "Type=resource plugin with Name/Info/Info.Types and a matching generatedSchema.usda."
        )

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify physics schema package is codeless.")
    parser.add_argument("--dir", type=Path, required=True,
                        help="Path to the schema package tree (e.g. _build/target-deps/usd_ext_physics)")
    args = parser.parse_args()

    if not args.dir.is_dir():
        print(f"[ERROR] Schema directory not found: {args.dir}", file=sys.stderr)
        return 1

    errors = verify_schema_package(args.dir)
    if errors:
        print(f"\n[FAIL] Schema package at {args.dir} is not a valid codeless schema:\n")
        for err in errors:
            print(f"  - {err}")
        print()
        return 1

    print(f"[PASS] Schema package at {args.dir} is a valid codeless schema")
    return 0


if __name__ == "__main__":
    sys.exit(main())
