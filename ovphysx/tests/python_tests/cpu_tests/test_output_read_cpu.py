# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# PhysX.read_sync exposes one public Warp-array contract on CPU and CUDA. CPU arrays
# alias the read session's native storage just like CUDA arrays. Their lease keeps
# that storage valid if the array outlives the ReadResult context.

# @implements REQ-PYTHON-READ-001
# @covers AC-1 AC-3
# @maps_to TEST-PYTHON-READ-001

import gc
import threading
import warnings

import numpy as np
import ovstage
import warp as wp
from ovphysx.types import ObjectScope, SimObjectType, TensorType
from test_utils import attach_usd_with_ovstage, data_path, load_usd_with_ovstage


def _load_and_step(sdk, scene, n_steps=15):
    load_usd_with_ovstage(sdk, data_path(scene))
    for _ in range(n_steps):
        sdk.step(1.0 / 60.0)
    sdk.wait_all()


def test_cpu_rigid_read_returns_cpu_warp_arrays(physx_sdk_cpu):
    _load_and_step(physx_sdk_cpu, "boxes_falling_on_groundplane.usda", n_steps=10)
    with physx_sdk_cpu.read_sync(SimObjectType.RIGID_BODY, ["position", "linearVelocity"], scope=ObjectScope.ALL) as result:
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
    with physx_sdk_cpu.read_sync(SimObjectType.ARTICULATION_JOINT, ["jointVelocity"], scope=ObjectScope.ALL) as result:
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
    with physx_sdk_cpu.read_sync(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
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

    with physx_sdk_cpu.read_sync(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
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


# ---------------------------------------------------------------------------
# Prim-set selection on CPU (ADR-0025 / REQ-READ-SELECT-001): the GPU mirror lives in
# test_output_read.py.
# ---------------------------------------------------------------------------

_CUBE_PATTERN = "/World/Cube*"
# The rigid bodies _CUBE_PATTERN names in boxes_falling_on_groundplane.usda.
_CUBE_PATHS = sorted(f"/World/Cube{i}" for i in range(1, 12))


def _handles_for(stage, paths):
    """Native ovx_primpath_t handles for `paths` -- the explicit prim_paths selection takes handles, not
    strings (a caller gets these from a read's prim_list or interns them through ovstage)."""
    with ovstage.PathDictionary(stage) as dictionary:
        return [dictionary.intern_path(p) for p in paths]


def _cpu_selected_read_paths(sdk, stage, *, prim_paths=None, pattern=None):
    out = []
    with sdk.read_sync(SimObjectType.RIGID_BODY, ["position"], prim_paths=prim_paths, pattern=pattern) as result:
        with ovstage.PathDictionary(stage) as dictionary:
            for group in result.groups:
                handles = dictionary.get_paths(group.prim_list)
                out.extend(
                    dictionary.path_to_string(h)
                    for h in handles[group.prim_offset : group.prim_offset + group.prim_count]
                )
    return sorted(set(out))


def test_cpu_read_pattern_matches_tensor_binding(physx_sdk_cpu):
    """On CPU, a pattern-narrowed read selects exactly the prims the glob names in the fixture, the same
    prims a pattern-bound tensor binding selects."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()

    assert _cpu_selected_read_paths(physx_sdk_cpu, stage, pattern=_CUBE_PATTERN) == _CUBE_PATHS

    # The deprecated pattern binding resolves the same prims; this comparison retires with the binding.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        binding = physx_sdk_cpu.create_tensor_binding(pattern=_CUBE_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
        try:
            assert sorted(binding.prim_paths) == _CUBE_PATHS
        finally:
            binding.destroy()


def test_cpu_read_prim_paths_subset(physx_sdk_cpu):
    """On CPU, an explicit prim_paths list reads exactly those prims."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()

    everything = _cpu_selected_read_paths(physx_sdk_cpu, stage)
    assert len(everything) >= 2
    wanted = everything[:2]
    assert _cpu_selected_read_paths(physx_sdk_cpu, stage, prim_paths=_handles_for(stage, wanted)) == wanted


def test_cpu_read_selection_empty_match(physx_sdk_cpu):
    """On CPU, a selection that matches no prim is a valid empty read."""
    stage = attach_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()
    # A native handle interned for a path not in the scene: valid handle, no live object, matches nothing.
    with physx_sdk_cpu.read_sync(
        SimObjectType.RIGID_BODY, ["position"], prim_paths=_handles_for(stage, ["/World/DoesNotExist"])
    ) as result:
        assert result.groups == []


def _cpu_selected_read_values(sdk, stage, attribute, *, prim_paths=None, pattern=None):
    """{prim path -> value row (np.ndarray)} for a (possibly selected) CPU read."""
    out = {}
    with sdk.read_sync(SimObjectType.RIGID_BODY, [attribute], prim_paths=prim_paths, pattern=pattern) as result:
        with ovstage.PathDictionary(stage) as dictionary:
            for group in result.groups:
                handles = dictionary.get_paths(group.prim_list)
                vals = np.asarray(group.tensors[0].numpy()).reshape(group.prim_count, -1)
                for r in range(group.prim_count):
                    path = dictionary.path_to_string(handles[group.prim_offset + r])
                    out[path] = vals[r].copy()
    return out


def test_cpu_read_selection_value_parity(physx_sdk_cpu):
    """On CPU, a selected read returns the same VALUES as the whole read for the selected prims.

    Guards against a mis-index after selection compaction that would emit the right prims with the wrong
    values -- which the path/count cases cannot see.
    """
    stage = attach_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(10):
        physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()

    whole = _cpu_selected_read_values(physx_sdk_cpu, stage, "position")
    assert len(whole) >= 3
    wanted = sorted(whole)[:2]

    selected = _cpu_selected_read_values(physx_sdk_cpu, stage, "position", prim_paths=_handles_for(stage, wanted))
    assert sorted(selected) == wanted
    for path in wanted:
        np.testing.assert_allclose(selected[path], whole[path], rtol=1e-5, atol=1e-5)


def test_physx_pointer_is_host_uint64(physx_sdk_cpu):
    """A path-bound actor and the PxPhysics singleton read as host uint64."""
    _load_and_step(physx_sdk_cpu, "boxes_falling_on_groundplane.usda", n_steps=1)
    with physx_sdk_cpu.read_sync(SimObjectType.RIGID_ACTOR, ["physxPointer"], scope=ObjectScope.ALL) as result:
        assert result.groups
        for group in result.groups:
            column = group.tensors[0]
            assert isinstance(column, wp.array)
            assert column.dtype == wp.uint64
            assert column.device.is_cpu
            values = np.asarray(column.numpy()).reshape(-1)
            assert values.size >= 1
            assert np.all(values != 0)
    with physx_sdk_cpu.read_sync(SimObjectType.PHYSICS, ["physxPointer"], scope=ObjectScope.ALL) as result:
        assert len(result.groups) == 1
        column = result.groups[0].tensors[0]
        assert column.dtype == wp.uint64
        assert int(np.asarray(column.numpy()).reshape(-1)[0]) != 0
