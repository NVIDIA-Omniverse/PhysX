# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-1

"""Discovery of the codeless PhysX USD schemas that ovphysx ships.

ovphysx ships the PhysX USD schemas as *codeless* schema artifacts: a root
``plugInfo.json`` plus a ``plugInfo.json`` (``Type: resource``) and
``generatedSchema.usda`` per schema module, with no compiled library. ovphysx
does not load or register them and ships no USD runtime: the application owns
its USD runtime(s) and registers the schemas explicitly.

The codeless schemas ship in the wheel and SDK under a stable convention::

    <ovphysx>/schemas/physx/plugInfo.json
    <ovphysx>/schemas/physx/<module>/resources/{plugInfo.json,generatedSchema.usda}

Register them with ovstage before the first population call in the process::

    import ovphysx
    import ovstage
    ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])

Register them with a stock ``usd-core`` runtime via :func:`codeless_schema_root`
or :func:`codeless_schema_paths`::

    import ovphysx
    from pxr import Plug
    Plug.Registry().RegisterPlugins([str(p) for p in ovphysx.codeless_schema_paths()])

After registration the PhysX API schemas can be applied by their schema
identifier, e.g. ``prim.ApplyAPI("PhysxRigidBodyAPI")``. Codeless schemas carry
no compiled C++/Python helper classes (no ``PhysxSchema.PhysxRigidBodyAPI``
binding); use USD's generic schema API.

These helpers are pure-Python: importing or calling them never triggers
ovphysx native loading, so they are safe to use in a process that only authors
or validates USD with a stock ``usd-core`` and never starts the simulator.
"""

import os
from pathlib import Path

__all__ = ["codeless_schema_root", "codeless_schema_paths"]

# Stable, documented filesystem convention for the exposed codeless schemas.
# <ovphysx>/schemas/physx/<module>/resources/{plugInfo.json,generatedSchema.usda}
_SCHEMA_SUBDIR = ("schemas", "physx")


def _candidate_roots():
    """Yield candidate ``schemas/physx`` roots, most specific first."""
    # 0. OVPHYSX_LIB selects the native library the bindings load. The matching
    #    schemas are next to that library (copied runtime) or one level up
    #    (SDK layout), probed in the same order the native helper uses.
    override = os.environ.get("OVPHYSX_LIB")
    if override:
        override_path = Path(override).resolve()
        lib_dir = override_path if override_path.is_dir() else override_path.parent
        yield lib_dir.joinpath(*_SCHEMA_SUBDIR)
        yield lib_dir.parent.joinpath(*_SCHEMA_SUBDIR)
    pkg = Path(__file__).parent
    # 1. Installed wheel: the schemas tree is bundled next to this package.
    yield pkg.joinpath(*_SCHEMA_SUBDIR)
    # 2. Source / editable checkout after `cmake -P scripts/install.cmake`:
    #    python/ovphysx/../../ == the ovphysx project root, which holds
    #    the staged _install/schemas/physx tree.
    yield pkg.parent.parent.joinpath("_install", *_SCHEMA_SUBDIR)


def codeless_schema_root() -> Path:
    """Return the directory holding ovphysx's codeless PhysX USD schema packages.

    The returned directory follows the convention
    ``schemas/physx/<module>/resources/`` and contains one subdirectory per
    PhysX schema module ovphysx ships (e.g. ``PhysxSchema`` and
    ``OmniUsdPhysicsDeformableSchema``).

    Raises:
        FileNotFoundError: If no staged codeless schema tree can be found.
            Install the ovphysx wheel (``pip install ovphysx``) or run
            ``cmake -P scripts/install.cmake`` from a source checkout to stage
            the schemas.
    """
    for root in _candidate_roots():
        if (root / "plugInfo.json").is_file():
            return root
    searched = ", ".join(str(root) for root in _candidate_roots())
    raise FileNotFoundError(
        "ovphysx codeless USD schemas not found. Checked: "
        f"{searched}. Install the ovphysx wheel (pip install ovphysx) or run "
        "'cmake -P scripts/install.cmake' to stage them."
    )


def codeless_schema_paths() -> list[Path]:
    """Return the per-module codeless schema resource directories.

    Each returned path is a ``resources`` directory containing a USD
    ``plugInfo.json`` (``Type: resource``) and ``generatedSchema.usda``, ready
    to hand to ``pxr.Plug.Registry().RegisterPlugins()`` for use with a stock
    ``usd-core`` runtime.

    Raises:
        FileNotFoundError: If the staged schema tree is missing or contains no
            registrable schema packages. See :func:`codeless_schema_root`.
    """
    root = codeless_schema_root()
    paths = sorted(
        path
        for path in root.glob("*/resources")
        if (path / "plugInfo.json").is_file() and (path / "generatedSchema.usda").is_file()
    )
    if not paths:
        raise FileNotFoundError(
            f"No codeless schema packages found under {root} "
            "(expected <module>/resources/plugInfo.json). The ovphysx "
            "installation may be incomplete."
        )
    return paths
