# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Runtime patch that extends UsdPhysics.MeshCollisionAPI.physics:approximation
allowedTokens with PhysX-specific values (sdf, sphereFill) so generic USD UIs
(e.g. the Variant Editor) can offer them in their dropdowns.

We mutate via Usd.SchemaRegistry().FindAppliedAPIPrimDefinition because the
Variant Editor (and other generic USD consumers) read allowedTokens from the
registry's internal schematics layer; mutating an on-disk SdfLayer instead has
no effect on those readers.

To avoid being the trigger that prematurely freezes the schema registry before
other extensions (omni.rtx, omni.playback, ...) have registered their own
plugins, omni.usd.schema.physx defers this call to omni.kit.app's first update
tick. By then every auto-load extension's Python module has been imported and
its Plug.Registry().RegisterPlugins(...) calls have run.
"""

from typing import List, Optional

import carb
from pxr import Sdf, Usd

# Tokens PhysX adds on top of UsdPhysics.MeshCollisionAPI's stock allowedTokens.
_APPROXIMATION_EXTRA_TOKENS = ("sdf", "sphereFill")

# Subset of _APPROXIMATION_EXTRA_TOKENS that this module actually appended at
# patch time. None means not patched. Tracking only what we added lets unpatch
# coexist with other parties that may have authored their own tokens onto the
# same spec.
_added_tokens: Optional[List[str]] = None


def _get_approximation_attr_spec() -> Optional[Sdf.AttributeSpec]:
    prim_def = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition("PhysicsMeshCollisionAPI")
    if prim_def is None:
        return None
    return prim_def.GetSchemaAttributeSpec("physics:approximation")


def patch_physics_approximation_allowed_tokens() -> None:
    global _added_tokens
    if _added_tokens is not None:
        return

    attr_spec = _get_approximation_attr_spec()
    if attr_spec is None:
        carb.log_warn(
            "Could not find UsdPhysics.MeshCollisionAPI.physics:approximation attribute "
            "spec; skipping allowedTokens extension."
        )
        return

    try:
        current = list(attr_spec.allowedTokens)
        to_add = [t for t in _APPROXIMATION_EXTRA_TOKENS if t not in current]
        if to_add:
            attr_spec.allowedTokens = current + to_add
        _added_tokens = to_add
    except Exception as e:
        carb.log_warn(f"Failed to extend physics:approximation allowedTokens: {e}")


def unpatch_physics_approximation_allowed_tokens() -> None:
    global _added_tokens
    if _added_tokens is None:
        return

    added = _added_tokens
    _added_tokens = None
    if not added:
        return

    attr_spec = _get_approximation_attr_spec()
    if attr_spec is None:
        return

    try:
        current = list(attr_spec.allowedTokens)
        attr_spec.allowedTokens = [t for t in current if t not in added]
    except Exception as e:
        carb.log_warn(
            f"Failed to remove physics:approximation allowedTokens extension: {e}"
        )
