# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# ADR-0008: PhysX.read returns Warp arrays that preserve the native CUDA allocation on a GPU sim.
# The `physx_sdk` fixture (conftest.py) sets /physics/suppressReadback, so the sim here is GPU-mode.

# @implements REQ-PYTHON-READ-001
# @covers AC-1 AC-2
# @maps_to TEST-PYTHON-READ-001

import warp as wp
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import data_path, load_usd_with_ovstage


def test_rigid_body_read_returns_cuda_warp_arrays(physx_sdk):
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    with physx_sdk.read(SimObjectType.RIGID_BODY, ["position", "linearVelocity"], scope=ObjectScope.ALL) as result:
        groups = result.groups
        assert groups, "expected at least one rigid-body read group"
        for g in groups:
            assert g.tensors, "read group carried no tensors"
            for t in g.tensors:
                assert isinstance(t, wp.array), f"expected warp.array, got {type(t).__name__}"
                # A GPU sim's read column must live on the GPU, not the host.
                assert t.device.is_cuda, (
                    f"expected a CUDA Warp array, got {t.device}. "
                    "PhysX.read must preserve the GPU allocation."
                )
            # Count comes from group metadata, independent of where the tensor lives.
            assert g.prim_count >= 1


def test_device_column_expands_lanes_to_a_trailing_dimension(physx_sdk):
    """A vec3 column arrives from native as ndim-1 `[N]` with the tuple width in
    `dtype.lanes`. The frontend must expand those lanes to the public `[N, 3]` shape."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    with physx_sdk.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
        groups = result.groups
        assert groups, "expected at least one rigid-body read group"
        for g in groups:
            for t in g.tensors:
                assert isinstance(t, wp.array) and t.device.is_cuda
                assert len(t.shape) == 2, f"expected a 2-D [N, 3] view of a vec3 column, got {t.shape}"
                assert t.shape[1] == 3, f"position is vec3; got trailing extent {t.shape[1]}"
