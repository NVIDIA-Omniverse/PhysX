# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Locate the independently installed Newton schema without importing USD."""

import importlib.util
from pathlib import Path

__all__ = ["newton_schema_root", "find_newton_schema_root"]

# The Newton USD schema is a separate, application-installed package.
NEWTON_SCHEMA_PACKAGE = "newton_usd_schemas"
NEWTON_SCHEMA_PIP_NAME = "newton-usd-schemas"
NEWTON_SCHEMA_URL = "https://github.com/newton-physics/newton-usd-schemas"
NEWTON_SCHEMA_INSTALL_HINT = (
    f"Install it with 'pip install {NEWTON_SCHEMA_PIP_NAME}' or download it from "
    f"{NEWTON_SCHEMA_URL}, and register the directory holding its plugInfo.json with "
    "ovstage.population.register_usd_schemas() alongside the external PhysX schemas "
    "before the first population call in the process."
)

def find_newton_schema_root() -> Path | None:
    """Return the installed ``newton_usd_schemas`` package directory, or ``None``.

    The directory holds the schema's ``plugInfo.json`` and ``generatedSchema.usda``
    and is what ``ovstage.population.register_usd_schemas()`` takes. The package
    is located through ``importlib.util.find_spec`` and never imported: its import
    needs ``pxr`` and registers the schema with that stock USD runtime, not with
    the runtime ovstage populates through.
    """
    try:
        spec = importlib.util.find_spec(NEWTON_SCHEMA_PACKAGE)
    except (ImportError, ValueError):
        return None
    if spec is None or not spec.submodule_search_locations:
        return None
    for location in spec.submodule_search_locations:
        root = Path(location)
        if (root / "plugInfo.json").is_file():
            return root
    return None


def newton_schema_root() -> Path:
    """Return the directory of the installed Newton USD schema (``newton-usd-schemas``).

    Register it with ovstage alongside the external PhysX schemas, before the
    first population call in the process; the parser reads the schema's
    ``newton:*`` attributes as fallbacks for the PhysX spellings, and population
    drops them unless the schema is registered.

    Raises:
        FileNotFoundError: If the ``newton-usd-schemas`` package is not installed
            in this Python environment. The message carries the install hint.
    """
    root = find_newton_schema_root()
    if root is None:
        raise FileNotFoundError(
            f"The Newton USD schema ({NEWTON_SCHEMA_PIP_NAME}) is not installed in this "
            f"Python environment. {NEWTON_SCHEMA_INSTALL_HINT}"
        )
    return root
