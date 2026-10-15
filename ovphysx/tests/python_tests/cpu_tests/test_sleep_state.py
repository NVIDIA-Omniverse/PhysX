# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Sleeping-state reads and retained Python snapshots on a CPU scene."""

# @implements REQ-CAPI-SLEEP-001
# @covers AC-2
# @maps_to TEST-CAPI-SLEEP-001

import numpy as np
import warp as wp

from ovphysx.types import SimObjectType
from test_utils import data_path, load_usd_with_ovstage


def _write_wake(physx_sdk, value):
    with physx_sdk.write_sync(SimObjectType.RIGID_BODY, "wake") as write:
        assert write.groups
        for group in write.groups:
            tensor = group.tensors[0]
            tensor.assign(np.full(tensor.shape, value, dtype=np.uint8))
            write.commit_sync(group)


def test_sleeping_state_changes_leave_retained_python_snapshot_unchanged(physx_sdk):
    """Fresh reads follow sleep/wake while borrowed arrays retain the previous state."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.step_sync(1.0 / 60.0)
    _write_wake(physx_sdk, 0)

    with physx_sdk.read_sync(SimObjectType.RIGID_BODY, ["isSleeping"]) as result:
        assert len(result.groups) == 1
        group = result.groups[0]
        asleep = group.tensors[0]
        assert asleep.dtype == wp.uint8 and not asleep.device.is_cuda
        assert group.prim_count > 0
        assert np.all(asleep.numpy() == 1)
        token = group.attribute

    _write_wake(physx_sdk, 1)
    with physx_sdk.read_tokens_sync(SimObjectType.RIGID_BODY, [token]) as result:
        assert len(result.groups) == 1
        awake = result.groups[0].tensors[0]
        assert awake.dtype == wp.uint8 and not awake.device.is_cuda
        assert np.all(awake.numpy() == 0)
    assert np.all(asleep.numpy() == 1)
    physx_sdk.step_sync(1.0 / 60.0)
    assert np.all(asleep.numpy() == 1)
