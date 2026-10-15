# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# physx-usd-schemas top-level package.
#
# The codeless PhysX USD schema modules ship as subpackages of this package
# (physx_usd_schemas.PhysxSchema, physx_usd_schemas.OmniUsdPhysicsDeformableSchema,
# physx_usd_schemas.PhysicsSchemaTools). Importing this package registers all schema
# plugins with the active USD runtime, so a plain ``import physx_usd_schemas`` is
# enough to make the PhysX schema types known to USD. The schema Python APIs load on
# demand -- e.g. ``from physx_usd_schemas import PhysxSchema``. register_all() /
# schema_paths() remain available for explicit use.

from __future__ import annotations

from pathlib import Path

# _version.py is generated into the wheel by scripts/build_wheel.py.
from ._version import __version__

__all__ = [
    "PhysxSchema",
    "OmniUsdPhysicsDeformableSchema",
    "PhysicsSchemaTools",
    "schema_paths",
    "register_all",
    "__version__",
]

# Subpackages that ship USD plugin data (plugInfo.json + generatedSchema.usda under
# their resources/ directory).
_SCHEMA_MODULES = ("PhysxSchema", "OmniUsdPhysicsDeformableSchema")


def schema_paths() -> list[Path]:
    """Return the per-module resources/ directories for all bundled PhysX USD schemas.

    Each returned path is a ``resources/`` directory containing a USD
    ``plugInfo.json`` (``Type: resource``) and ``generatedSchema.usda``, ready to
    pass to ``pxr.Plug.Registry().RegisterPlugins()``.

    Example::

        from pxr import Plug
        import physx_usd_schemas
        Plug.Registry().RegisterPlugins([str(p) for p in physx_usd_schemas.schema_paths()])

    Raises:
        FileNotFoundError: If no schema resources are found. This usually means
            the wheel was not installed correctly (``pip install physx-usd-schemas``).
    """
    base = Path(__file__).parent
    paths: list[Path] = []
    for module_name in _SCHEMA_MODULES:
        resources = base / module_name / "resources"
        if (resources / "plugInfo.json").is_file():
            paths.append(resources)

    missing = [m for m in _SCHEMA_MODULES
               if not (base / m / "resources" / "plugInfo.json").is_file()]
    if missing:
        raise FileNotFoundError(
            f"physx-usd-schemas resources missing for: {', '.join(missing)}. "
            "Re-install the wheel: pip install physx-usd-schemas"
        )
    return paths


def register_all() -> None:
    """Register all bundled PhysX USD schemas with the active USD runtime.

    Registers ``PhysxSchema`` and ``OmniUsdPhysicsDeformableSchema`` via
    ``pxr.Plug.Registry().RegisterPlugins()``. This runs automatically on import;
    it is kept public for explicit callers and is safe to call again.  After it
    runs the PhysX API schemas can be applied by their schema identifier::

        import physx_usd_schemas
        from pxr import Usd

        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/World/Box", "Cube")
        prim.ApplyAPI("PhysxRigidBodyAPI")

    Raises:
        FileNotFoundError: If the schema resources are not installed.
        ImportError: If ``pxr`` (``usd-core``) is not available.
    """
    from pxr import Plug  # noqa: PLC0415 -- deferred so the ImportError names pxr, not this module

    paths = schema_paths()
    Plug.Registry().RegisterPlugins([str(p) for p in paths])


# Register all schemas on import so `import physx_usd_schemas` is sufficient to make
# the PhysX schema types known to the USD runtime (see module docstring).
register_all()
