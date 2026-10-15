# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-1 AC-2 AC-3 AC-7

"""Application-side discovery of the separately installed PhysX schema data.

Prefer the installed sample namespace so a copied sample needs no sibling
directory:

    from ovphysx.samples.python_samples.schema_support import physx_schema_paths

Copy this directory beside a sample only when adapting the recipe outside the
wheel. Package metadata lookup does not import physx_usd_schemas (whose import
registers with stock USD) or load any USD runtime.
"""

from importlib import metadata
from pathlib import Path

SCHEMA_MODULES = ("PhysxSchema", "OmniUsdPhysicsDeformableSchema")


def physx_schema_paths() -> list[Path]:
    """Validate the external provider and return its two registration directories."""
    try:
        package = metadata.distribution("physx-usd-schemas")
    except metadata.PackageNotFoundError as error:
        raise FileNotFoundError(
            "PhysX schemas are not installed. Install physx-usd-schemas "
            "from PyPI, then register its schemas before population."
        ) from error
    paths = [Path(package.locate_file(f"physx_usd_schemas/{module}/resources"))
             for module in SCHEMA_MODULES]
    for path in paths:
        for name in ("plugInfo.json", "generatedSchema.usda"):
            if not (path / name).is_file():
                raise FileNotFoundError(
                    f"Incomplete physx-usd-schemas: missing {path / name}. "
                    "Reinstall the schema package before population."
                )
    return paths
