# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MIGRATION-001
# @covers AC-2 AC-3 AC-4
"""Explicit migration entrypoints; no Kit imports or automatic runtime discovery."""
import importlib
from types import MappingProxyType


class MigrationError(RuntimeError):
    """The legacy call needs an explicit owner, data contract or host adapter."""


# These are the existing core authoring implementations. Their documented Python
# signatures, stage-unit conventions and schema registration policy apply.
HELPERS=MappingProxyType({
    **{f'physicsUtils.{name}':f'ovphysx.utils.shapes:{name}' for name in (
        'add_box','add_collider_box','add_rigid_box','add_cube','add_collider_cube','add_rigid_cube',
        'add_sphere','add_collider_sphere','add_rigid_sphere','add_capsule','add_collider_capsule',
        'add_rigid_capsule','add_cylinder','add_collider_cylinder','add_rigid_cylinder',
        'add_cone','add_collider_cone','add_rigid_cone','add_xform','add_rigid_xform')},
    **{f'physicsUtils.{name}':f'ovphysx.utils.authoring:{name}' for name in ('add_mass','add_density','add_force_torque')},
    **{f'physicsUtils.{name}':f'ovphysx.utils.filtering:{name}' for name in (
        'add_collision_to_collision_group','remove_collision_from_collision_group','is_in_collision_group')},
})


def authoring_helper(legacy_name):
    """Resolve a supported shared helper without creating or attaching physics.

    This is an explicit migration aid, not an omni.physx monkey patch. Helpers
    use their core signatures and customer USD. For revision checks and undo,
    invoke in a preflighted EditSession batch or use the higher-level USD helpers.
    """
    target=HELPERS.get(legacy_name)
    if target is None:
        raise MigrationError(f'{legacy_name}: no signature-compatible adapter; use the explicit '
            'ovphysx_companion.usd helper and a host command returning EditPatch (docs/migration.md)')
    module,name=target.split(':')
    return getattr(importlib.import_module(module),name)
