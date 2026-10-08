# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for the physics output read API (ADR-0007): PhysX.read over an
ovstage-attached scene. The read is ovstage-native, so the scene is attached
through the ovstage backend (load_usd_with_ovstage).
"""

# @implements REQ-PYTHON-READ-001
# @covers AC-1 AC-3 AC-5
# @maps_to TEST-PYTHON-READ-001

import warp as wp
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import data_path, load_usd_with_ovstage


def _col_width(t) -> int:
    """Trailing component width of a read column, device-agnostic."""
    return int(t.shape[-1]) if t.ndim >= 2 else 1


def _col_rows(t) -> int:
    """Row count (number of prims) of a read column, device-agnostic."""
    return int(t.shape[0])


def _is_read_column(t) -> bool:
    """Every Python output-read column uses the public Warp frontend."""
    return isinstance(t, wp.array)


def test_rigid_body_output_read(physx_sdk):
    """Query rigid bodies and read position + orientation as faithful column groups.

    The `physx_sdk` fixture is GPU-mode, so its Warp arrays are device-resident.
    The same public type and trailing-component shape are returned on a CPU sim.
    """
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    with physx_sdk.read(
        SimObjectType.RIGID_BODY, ["position", "orientation"], scope=ObjectScope.ALL
    ) as result:
        groups = result.groups

        # At least a position (vec3) and an orientation (quat) column. Native
        # DLPack lanes are exposed uniformly as Warp's trailing dimension.
        assert len(groups) >= 2
        pos_groups = [g for g in groups if g.tensors and _col_width(g.tensors[0]) == 3]
        ori_groups = [g for g in groups if g.tensors and _col_width(g.tensors[0]) == 4]
        assert pos_groups, "expected a position (vec3) column"
        assert ori_groups, "expected an orientation (quat) column"

        g = pos_groups[0]
        assert g.object_type == SimObjectType.RIGID_BODY
        assert not g.is_array and not g.is_delete
        # A standalone-body fixed column: one tensor stacked over its prims.
        assert len(g.tensors) == 1
        data = g.tensors[0]
        assert _is_read_column(data)
        assert data.device.is_cuda
        assert _col_rows(data) > 0 and _col_width(data) == 3
        assert g.prim_count == _col_rows(data)
        # Interned identifiers come back as non-zero handles (valid within the block).
        assert g.attribute != 0
        assert g.prim_list != 0
        # A fixed column has no element-axis scatter.
        assert g.index_map is None


def test_read_empty_when_no_match(physx_sdk):
    """A type with no objects in the scene yields an empty result, not an error."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    # The falling-boxes scene has no particle sets.
    with physx_sdk.read(SimObjectType.PARTICLE_SET, ["points"]) as result:
        assert result.groups == []


def test_read_tokens_accepts_attribute_token_from_read_group(physx_sdk):
    """An emitted attribute token can be fed directly into the public token overload."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    with physx_sdk.read(
        SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL
    ) as named_result:
        attribute_token = next(group for group in named_result.groups if group.tensors).attribute

    with physx_sdk.read_tokens(
        SimObjectType.RIGID_BODY, [attribute_token], scope=ObjectScope.ALL
    ) as token_result:
        token_group = next(
            group
            for group in token_result.groups
            if group.attribute == attribute_token and group.tensors
        )
        assert isinstance(token_group.tensors[0], wp.array)
