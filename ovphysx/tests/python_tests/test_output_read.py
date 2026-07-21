# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Tests for the physics output read API (ADR-0007): PhysX.read over an
ovstage-attached scene. The read is ovstage-native, so the scene is attached
through the ovstage backend (load_usd_with_ovstage).
"""

import numpy as np
from ovphysx.types import SimObjectType, ObjectScope
from test_utils import data_path, load_usd_with_ovstage


def test_rigid_body_output_read(physx_sdk):
    """Query rigid bodies and read position + orientation as faithful column groups."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    with physx_sdk.read(
        SimObjectType.RIGID_BODY, ["position", "orientation"], ObjectScope.ALL
    ) as result:
        groups = result.groups

        # At least a position (vec3) and an orientation (quat) column. Tuple width
        # is the trailing NumPy dim (faithful: ovstage carries it in dtype.lanes).
        assert len(groups) >= 2
        pos_groups = [g for g in groups if g.tensors and g.tensors[0].shape[-1] == 3]
        ori_groups = [g for g in groups if g.tensors and g.tensors[0].shape[-1] == 4]
        assert pos_groups, "expected a position (vec3) column"
        assert ori_groups, "expected an orientation (quat) column"

        g = pos_groups[0]
        assert g.object_type == SimObjectType.RIGID_BODY
        assert not g.is_array and not g.is_delete
        # A standalone-body fixed column: one tensor stacked over its prims.
        assert len(g.tensors) == 1
        data = g.tensors[0]
        assert isinstance(data, np.ndarray)
        assert data.ndim == 2 and data.shape[0] > 0 and data.shape[1] == 3
        assert g.prim_count == data.shape[0]
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
