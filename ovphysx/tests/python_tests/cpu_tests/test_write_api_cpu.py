# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""CPU placement and aliasing coverage for ``PhysX.write``."""

# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001
# @implements REQ-INPUT-DEVICE-001
# @covers AC-1 AC-4 AC-5
# @maps_to TEST-INPUT-DEVICE-001

import numpy as np
import warp as wp
from ovphysx.types import SimObjectType
from test_utils import data_path, load_usd_with_ovstage


def _read_positions(sdk):
    with sdk.read(SimObjectType.RIGID_BODY, ["position"]) as result:
        assert result.groups
        return np.concatenate([g.tensors[0].numpy().reshape(g.prim_count, 3) for g in result.groups])


def test_cpu_write_returns_warp_aliases_and_round_trips(physx_sdk_cpu):
    load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk_cpu.step_sync(1.0 / 60.0)

    before = _read_positions(physx_sdk_cpu)
    target = np.arange(before.size, dtype=np.float32).reshape(before.shape) + 200.0

    with physx_sdk_cpu.write(SimObjectType.RIGID_BODY, "position") as session:
        assert session.groups
        row = 0
        for group in session.groups:
            tensor = group.tensors[0]
            assert isinstance(tensor, wp.array)
            assert tensor.size > 0, "the alias assertion applies to mapped non-empty storage"
            assert tensor.device.is_cpu
            tensor.assign(np.ascontiguousarray(target[row : row + group.prim_count]))
            session.commit(group)
            row += group.prim_count
        assert row == target.shape[0]

    np.testing.assert_allclose(_read_positions(physx_sdk_cpu), target, rtol=0, atol=1e-3)
