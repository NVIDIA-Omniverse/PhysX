#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
"""Derive codeless PhysX USD schema artifacts from the staged runtime schema.

ovphysx packages its runtime USD schemas under
``_install/plugins/usd/<Module>/resources/``. Those ``plugInfo.json`` files are
not cleanly codeless -- some are ``Type=library`` and link ovphysx's namespaced
USD, others carry a build-time ``LibraryPath`` placeholder -- so a stock
``usd-core`` cannot register them.

This script reads the *packaged* runtime schema (whatever revision was selected:
the packman ``usd_ext_physics`` package by default, or a local build under
``--devschema``) and emits a normalized, codeless copy under
``<out-dir>/<Module>/resources/`` for external authoring/validation tooling
(see ``ovphysx.codeless_schema_paths()``).

Deriving from the packaged runtime guarantees the exposed codeless schemas match
the schema revision ovphysx actually ships, and automatically tracks new schema
modules instead of relying on a hand-maintained list (OMPE-86833).
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

# Reuse the tolerant USD plugInfo loader used for runtime packaging so parsing
# (comments / trailing commas) behaves identically to the registry merge step.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from package_deps import _load_json_like  # noqa: E402

# ovphysx-owned physics schema modules are identified by their USD plugin Name.
# Matching by prefix auto-includes new sibling modules (e.g. physxSchemaAddition)
# and excludes core USD schemas such as usdPhysics / usdGeom.
PHYSICS_NAME_PREFIXES = ("physx", "omniusdphysics")


def _is_physics_module(plug_info: dict) -> bool:
    for plugin in plug_info.get("Plugins", []):
        name = (plugin.get("Name") or "").lower()
        if name.startswith(PHYSICS_NAME_PREFIXES):
            return True
    return False


def _to_codeless(plug_info: dict) -> dict:
    """Normalize every plugin entry into a codeless USD resource plugin.

    A codeless schema is registered purely from ``generatedSchema.usda``; it must
    not reference a compiled library. We therefore force ``Type="resource"`` and
    drop ``LibraryPath`` (whether a real path or an unsubstituted placeholder),
    keeping ``Root``/``ResourcePath`` so the adjacent ``generatedSchema.usda``
    still resolves.
    """
    for plugin in plug_info.get("Plugins", []):
        plugin["Type"] = "resource"
        plugin.pop("LibraryPath", None)
        plugin["Root"] = plugin.get("Root", "..")
        plugin["ResourcePath"] = plugin.get("ResourcePath", "resources")
    return plug_info


def _validate_codeless(plug_info: dict, module_name: str) -> None:
    """Assert the plugin entries satisfy the codeless USD resource-plugin contract.

    Stock ``usd-core`` registration relies on these invariants. Checking them at
    packaging time makes a schema-regeneration or transform regression fail here,
    rather than surfacing later in external authoring/validation tools.
    """
    plugins = plug_info.get("Plugins", [])
    if not plugins:
        raise SystemExit(f"[export_codeless_schema] {module_name}: plugInfo has no Plugins.")
    for plugin in plugins:
        name = plugin.get("Name")
        if plugin.get("Type") != "resource":
            raise SystemExit(
                f"[export_codeless_schema] {module_name}: plugin {name!r} has "
                f"Type={plugin.get('Type')!r}, expected 'resource'."
            )
        if plugin.get("Root") != "..":
            raise SystemExit(
                f"[export_codeless_schema] {module_name}: plugin {name!r} has "
                f"Root={plugin.get('Root')!r}, expected '..'."
            )
        if plugin.get("ResourcePath") != "resources":
            raise SystemExit(
                f"[export_codeless_schema] {module_name}: plugin {name!r} has "
                f"ResourcePath={plugin.get('ResourcePath')!r}, expected 'resources'."
            )
        if plugin.get("LibraryPath"):
            raise SystemExit(
                f"[export_codeless_schema] {module_name}: plugin {name!r} still has a "
                f"non-empty LibraryPath ({plugin.get('LibraryPath')!r}); codeless "
                "schemas must not reference a compiled library."
            )
        if not plugin.get("Info", {}).get("Types"):
            raise SystemExit(
                f"[export_codeless_schema] {module_name}: plugin {name!r} has no "
                "Info.Types; refusing to export an empty codeless schema."
            )


def export_codeless_schemas(plugins_usd_dir: Path, out_dir: Path) -> list[str]:
    """Export codeless copies of every physics schema module under plugins_usd_dir.

    Returns the list of exported module directory names.
    """
    # Rebuild the output tree from scratch so the exposed set exactly matches the
    # currently packaged runtime (no stale modules linger from a previous run).
    if out_dir.exists():
        shutil.rmtree(out_dir)

    exported: list[str] = []
    for plug_info_path in sorted(plugins_usd_dir.glob("*/resources/plugInfo.json")):
        module_dir = plug_info_path.parent.parent
        generated = plug_info_path.parent / "generatedSchema.usda"

        data = _load_json_like(plug_info_path)
        if data is None:
            # The file exists (it was globbed) but did not parse. If the module
            # directory looks like a PhysX/physics schema module, silently dropping
            # it would ship an incomplete codeless schema set, so fail loudly. Core
            # USD modules we never expose (usdGeom, ar, ...) are simply skipped.
            if module_dir.name.lower().startswith(PHYSICS_NAME_PREFIXES):
                raise SystemExit(
                    f"[export_codeless_schema] {module_dir.name}: failed to parse "
                    f"{plug_info_path}; refusing to ship an incomplete codeless "
                    "schema set."
                )
            continue
        if not _is_physics_module(data):
            continue
        if not generated.is_file():
            raise SystemExit(
                f"[export_codeless_schema] {module_dir.name}: missing "
                f"generatedSchema.usda next to {plug_info_path}; cannot expose a "
                "codeless schema without it."
            )

        data = _to_codeless(data)
        _validate_codeless(data, module_dir.name)

        dst = out_dir / module_dir.name / "resources"
        dst.mkdir(parents=True, exist_ok=True)
        (dst / "plugInfo.json").write_text(json.dumps(data, indent=4) + "\n")
        shutil.copy2(generated, dst / "generatedSchema.usda")
        exported.append(module_dir.name)
        print(f"  [codeless] {module_dir.name} -> {dst}")

    return exported


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Export codeless PhysX USD schemas from the staged runtime."
    )
    parser.add_argument(
        "--plugins-usd-dir", type=Path, required=True,
        help="Staged runtime USD plugin directory (e.g. _install/plugins/usd).",
    )
    parser.add_argument(
        "--out-dir", type=Path, required=True,
        help="Output root for the codeless schemas (e.g. _install/schemas/physx).",
    )
    args = parser.parse_args()

    if not args.plugins_usd_dir.is_dir():
        print(f"[ERROR] plugins/usd directory not found: {args.plugins_usd_dir}", file=sys.stderr)
        return 1

    exported = export_codeless_schemas(args.plugins_usd_dir, args.out_dir)
    if not exported:
        print(
            f"[ERROR] No PhysX schema modules found under {args.plugins_usd_dir} "
            f"(expected USD plugin names starting with {PHYSICS_NAME_PREFIXES}).",
            file=sys.stderr,
        )
        return 1

    print(f"[OK] Exported {len(exported)} codeless schema module(s): {', '.join(exported)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
