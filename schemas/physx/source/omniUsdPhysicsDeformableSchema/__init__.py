# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Codeless OmniUsdPhysicsDeformableSchema Python API. No compiled bindings: the schema
# classes (OmniUsdPhysicsDeformableSchema.<Class>) and the token table
# (OmniUsdPhysicsDeformableSchema.Tokens) are pure-Python, generated from schema.usda by
# tools/gen_codeless_api.py / gen_tokens.py.
from pathlib import Path

from .codeless_api import *      # noqa: F401,F403  -- OmniUsdPhysicsDeformableSchema.<Class> API
from ._tokens import Tokens      # noqa: F401        -- OmniUsdPhysicsDeformableSchema.Tokens.<name>


def schema_path() -> Path:
    """Return the resources/ directory containing this schema's USD plugin data.

    Pass the result to ``pxr.Plug.Registry().RegisterPlugins()`` to make the
    OmniUsdPhysicsDeformableSchema types available in a stock ``usd-core`` runtime.
    """
    return Path(__file__).parent / "resources"


def register() -> None:
    """Register OmniUsdPhysicsDeformableSchema with the active USD runtime via Plug.Registry().

    Call this before the USD schema registry is first exercised; types registered
    afterwards remain unknown to code that already ran.

    Equivalent to::

        from pxr import Plug
        import OmniUsdPhysicsDeformableSchema
        Plug.Registry().RegisterPlugins(str(OmniUsdPhysicsDeformableSchema.schema_path()))
    """
    resources = schema_path()
    if not (resources / "plugInfo.json").is_file():
        raise FileNotFoundError(
            f"omniUsdPhysicsDeformableSchema plugInfo.json not found at {resources}. "
            "Install the wheel (pip install physx-usd-schemas) to use register()."
        )
    from pxr import Plug  # noqa: PLC0415 -- deferred to avoid hard pxr dependency
    Plug.Registry().RegisterPlugins(str(resources))
