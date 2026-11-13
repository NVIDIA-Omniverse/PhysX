# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""CPU placement and aliasing coverage for ``PhysX.write_sync``."""

# @implements REQ-CAPI-WRITE-001
# @covers AC-9 AC-16
# @maps_to TEST-CAPI-WRITE-001
# @maps_to TEST-CAPI-WRITE-002
# @implements REQ-INPUT-DEVICE-001
# @covers AC-1 AC-4 AC-5
# @maps_to TEST-INPUT-DEVICE-001

import numpy as np
import pytest
import warp as wp
from ovphysx.types import SimObjectType
from test_utils import data_path, load_usd_with_ovstage


def _read_positions(sdk):
    with sdk.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        assert result.groups
        return np.concatenate([g.tensors[0].numpy().reshape(g.prim_count, 3) for g in result.groups])


def test_cpu_write_returns_warp_aliases_and_round_trips(physx_sdk_cpu):
    load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk_cpu.step_sync(1.0 / 60.0)

    before = _read_positions(physx_sdk_cpu)
    target = np.arange(before.size, dtype=np.float32).reshape(before.shape) + 200.0

    with physx_sdk_cpu.write_sync(SimObjectType.RIGID_BODY, "position") as session:
        assert session.groups
        row = 0
        for group in session.groups:
            tensor = group.tensors[0]
            assert isinstance(tensor, wp.array)
            assert tensor.size > 0, "the alias assertion applies to mapped non-empty storage"
            assert tensor.device.is_cpu
            tensor.assign(np.ascontiguousarray(target[row : row + group.prim_count]))
            session.commit_sync(group)
            row += group.prim_count
        assert row == target.shape[0]

    np.testing.assert_allclose(_read_positions(physx_sdk_cpu), target, rtol=0, atol=1e-3)


def test_cpu_masked_commit_writes_only_selected_rows(physx_sdk_cpu):
    """``commit(mask=...)`` writes the rows a bool mask selects and leaves the others as they were (ADR-0052)."""
    load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk_cpu.step_sync(1.0 / 60.0)
    before = _read_positions(physx_sdk_cpu)

    selected = 0
    with physx_sdk_cpu.write_sync(SimObjectType.RIGID_BODY, "position") as session:
        for group in session.groups:
            group.tensors[0].assign(np.full((group.prim_count, 3), 300.0, dtype=np.float32))
            mask = np.zeros(group.prim_count, dtype=np.bool_)
            mask[::2] = True
            selected += int(mask.sum())
            session.commit_sync(group, mask=mask)

    after = _read_positions(physx_sdk_cpu)
    changed = np.all(after == 300.0, axis=1)
    assert int(changed.sum()) == selected
    np.testing.assert_allclose(after[~changed], before[~changed], rtol=0, atol=0)


def test_cpu_malformed_mask_leaves_group_committable(physx_sdk_cpu):
    """A mask whose length does not match the group is refused at commit, and the same group then commits."""
    load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk_cpu.step_sync(1.0 / 60.0)

    with physx_sdk_cpu.write_sync(SimObjectType.RIGID_BODY, "position") as session:
        assert len(session.groups) == 1
        group = session.groups[0]
        group.tensors[0].assign(np.full((group.prim_count, 3), 300.0, dtype=np.float32))
        too_long = np.ones(group.prim_count + 1, dtype=np.bool_)
        with pytest.raises(RuntimeError, match="row mask is rejected"):
            session.commit(group, mask=too_long)
        session.commit_sync(group, mask=np.ones(group.prim_count, dtype=np.bool_))

    assert np.all(_read_positions(physx_sdk_cpu) == 300.0)
