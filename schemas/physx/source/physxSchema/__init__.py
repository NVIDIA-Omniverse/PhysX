# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Codeless PhysxSchema Python API. No compiled _physxSchema bindings: the schema
# classes (PhysxSchema.PhysxFooAPI) and the token table (PhysxSchema.Tokens) are
# pure-Python, generated from schema.usda by tools/gen_codeless_api.py / gen_tokens.py.
from pathlib import Path

from .codeless_api import *      # noqa: F401,F403  -- PhysxSchema.<Class> API
from ._tokens import Tokens      # noqa: F401        -- PhysxSchema.Tokens.<name>


def schema_path() -> Path:
    """Return the resources/ directory containing this schema's USD plugin data.

    Pass the result to ``pxr.Plug.Registry().RegisterPlugins()`` to make the
    PhysxSchema types available in a stock ``usd-core`` runtime.
    """
    return Path(__file__).parent / "resources"


def register() -> None:
    """Register PhysxSchema with the active USD runtime via Plug.Registry().

    Call this before the USD schema registry is first exercised; types registered
    afterwards remain unknown to code that already ran.

    Equivalent to::

        from pxr import Plug
        import PhysxSchema
        Plug.Registry().RegisterPlugins(str(PhysxSchema.schema_path()))
    """
    resources = schema_path()
    if not (resources / "plugInfo.json").is_file():
        raise FileNotFoundError(
            f"physxSchema plugInfo.json not found at {resources}. "
            "Install the wheel (pip install physx-usd-schemas) to use register()."
        )
    from pxr import Plug  # noqa: PLC0415 -- deferred to avoid hard pxr dependency
    Plug.Registry().RegisterPlugins(str(resources))
