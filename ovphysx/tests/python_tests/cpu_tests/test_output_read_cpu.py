# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# PhysX.read exposes one public Warp-array contract on CPU and CUDA. CPU arrays
# alias the read session's native storage just like CUDA arrays. Their lease keeps
# that storage valid if the array outlives the ReadResult context.

# @implements REQ-PYTHON-READ-001
# @covers AC-1 AC-3
# @maps_to TEST-PYTHON-READ-001

import gc
import threading

import numpy as np
import warp as wp
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import data_path, load_usd_with_ovstage


def _load_and_step(sdk, scene, n_steps=15):
    load_usd_with_ovstage(sdk, data_path(scene))
    for _ in range(n_steps):
        sdk.step(1.0 / 60.0)
    sdk.wait_all()


def test_cpu_rigid_read_returns_cpu_warp_arrays(physx_sdk_cpu):
    _load_and_step(physx_sdk_cpu, "boxes_falling_on_groundplane.usda", n_steps=10)
    with physx_sdk_cpu.read(SimObjectType.RIGID_BODY, ["position", "linearVelocity"], scope=ObjectScope.ALL) as result:
        assert result.groups, "expected at least one rigid-body read group"
        checked = 0
        for g in result.groups:
            for t in g.tensors:
                assert isinstance(t, wp.array), f"CPU column must be a Warp array, got {type(t).__name__}"
                assert t.device.is_cpu, f"expected CPU residence, got {t.device}"
                assert np.all(np.isfinite(t.numpy()))
                checked += 1
        assert checked >= 1


def test_cpu_joint_velocity_read_returns_cpu_warp_arrays(physx_sdk_cpu):
    _load_and_step(physx_sdk_cpu, "mixed_base_articulations.usda", n_steps=20)
    with physx_sdk_cpu.read(SimObjectType.ARTICULATION_JOINT, ["jointVelocity"], scope=ObjectScope.ALL) as result:
        assert result.groups, "expected at least one articulation-joint read group"
        dof_rows = 0
        for g in result.groups:
            for t in g.tensors:
                assert isinstance(t, wp.array), f"CPU joint column must be a Warp array, got {type(t).__name__}"
                assert t.device.is_cpu
                assert np.all(np.isfinite(t.numpy()))
                dof_rows += int(t.size)
        assert dof_rows >= 1, "expected at least one unlocked joint DOF velocity"


def test_cpu_read_column_is_safe_to_keep_past_the_block(physx_sdk_cpu):
    """A borrowed CPU Warp array keeps its read-session storage alive after close."""
    _load_and_step(physx_sdk_cpu, "boxes_falling_on_groundplane.usda", n_steps=10)
    with physx_sdk_cpu.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
        assert result.groups and result.groups[0].tensors
        live = result.groups[0].tensors[0]
        assert isinstance(live, wp.array) and live.device.is_cpu
        snapshot = live.numpy().copy()

    assert np.all(np.isfinite(live.numpy()))
    np.testing.assert_array_equal(live.numpy(), snapshot)


def test_downstream_numpy_view_keeps_cpu_read_session_alive(physx_sdk_cpu, monkeypatch):
    """A downstream DLPack consumer retains the Warp source session."""
    _load_and_step(physx_sdk_cpu, "boxes_falling_on_groundplane.usda", n_steps=5)
    calls = []
    original = physx_sdk_cpu._lib.ovphysx_release_read

    def release_read(*args):
        calls.append(threading.get_ident())
        return original(*args)

    monkeypatch.setattr(physx_sdk_cpu._lib, "ovphysx_release_read", release_read)
    owner_thread = threading.get_ident()

    with physx_sdk_cpu.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
        source = result.groups[0].tensors[0]
        expected = source.numpy().copy()
        consumer = np.from_dlpack(source)

    del result
    del source
    gc.collect()
    assert calls == []
    np.testing.assert_array_equal(consumer, expected)

    del consumer
    gc.collect()
    assert calls == []
    physx_sdk_cpu.step_sync(1.0 / 60.0)
    assert calls == [owner_thread]
