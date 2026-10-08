# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# DEPRECATED (tensor-binding-deprecation): the deliberate pre-clone binding IS the NVBug 6428316
# trigger, so this regression retires with the binding. The clone + session read/write verification
# it guards is exercised by the session tests that stay.

"""Regression coverage for NVBug 6428316."""

import warnings

import numpy as np
import pytest
from ovphysx.types import ObjectScope, SimObjectType
from ovphysx.types import TensorType as TT
from test_utils import CudaArray, data_path, load_usd_with_ovstage


def _to_host(column):
    """A read column as host NumPy, whether it came back as NumPy (CPU) or a Warp array (GPU)."""
    return column if isinstance(column, np.ndarray) else column.numpy()


def _fill(t, block):
    """Write `block` into a write column in place (host NumPy or device Warp)."""
    if isinstance(t, np.ndarray):
        t.reshape(block.shape)[:] = block
        return
    t.assign(np.ascontiguousarray(block).reshape(t.shape))


def _read_positions(physx):
    """Every rigid body's position as one [N, 3] host array, in prim order."""
    with physx.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
        return np.concatenate([_to_host(g.tensors[0]).reshape(g.prim_count, -1) for g in result.groups])


def test_preclone_binding_velocity_reaches_all_envs(physx_sdk):
    """DirectGPU binding before clone must not leave stale GPU sim data."""
    N = 32
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    # The bug trigger is a LIVE pre-clone tensor binding during clone() (NVBug 6428316), so the
    # binding is created here and held across clone(). Dropping the return would let it be
    # collected first. The verification below uses the session read/write API.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        preclone_binding = physx_sdk.create_tensor_binding(
            pattern="/World/envs/*/table", tensor_type=TT.RIGID_BODY_POSE
        )

    g = int(np.ceil(np.sqrt(N)))
    physx_sdk.clone(
        source_path="/World/envs/env0",
        target_paths=[f"/World/envs/env{i}" for i in range(1, N)],
        anchor_transforms=[
            ((i % g) * 3.0, 0.0, (i // g) * 3.0, 0, 0, 0, 1)
            for i in range(1, N)
        ],
    )
    physx_sdk.wait_all()
    physx_sdk.warmup()

    # After cloning basic_simulation's single table into N envs, the tables are the scene's only
    # rigid bodies, so the whole-set session read/write reaches exactly them.
    before = _read_positions(physx_sdk)
    assert before.shape[0] == N, f"expected {N} bodies, got {before.shape[0]}"

    # Drive every body: linear velocity x = 3.
    groups_written = 0
    with physx_sdk.write(SimObjectType.RIGID_BODY, "linearVelocity") as w:
        for grp in w.groups:
            _fill(grp.tensors[0], np.tile(np.array([3, 0, 0], np.float32), (grp.prim_count, 1)))
            w.commit(grp)
            groups_written += 1
    assert groups_written > 0, "write session produced no groups; the velocity drive never happened"
    physx_sdk.wait_all()

    for _ in range(60):
        physx_sdk.step(1 / 240.0)
    physx_sdk.wait_all()
    after = _read_positions(physx_sdk)

    moved = int(np.sum(np.abs(after[:, 0] - before[:, 0]) > 0.3))
    assert moved == N, f"expected all {N} environments to move, got {moved}/{N}"

    # clone() invalidated the pre-clone binding. destroy() still releases the handle cleanly and
    # emits no runtime warning.
    preclone_binding.destroy()


def test_clone_invalidates_retained_gpu_contact_binding(physx_sdk):
    """A contact binding retained across clone must reject cleanly."""
    load_usd_with_ovstage(
        physx_sdk, data_path("boxes_falling_on_groundplane.usda")
    )
    physx_sdk.wait_all()

    cb = physx_sdk.create_contact_binding(
        sensor_patterns=["/World/Cube1"],
        max_contact_data_count=64,
    )
    assert cb.sensor_count == 1

    targets = ["/World/CloneA", "/World/CloneB", "/World/CloneC"]
    physx_sdk.clone("/World/Cube1", targets)
    physx_sdk.wait_all()
    physx_sdk.warmup()

    stale_forces = CudaArray((cb.sensor_count, 3), dtype=np.float32)
    with pytest.raises(RuntimeError):
        cb.read_net_forces(stale_forces.dltensor)

    cb = physx_sdk.create_contact_binding(
        sensor_patterns=["/World/Cube1"] + targets,
        max_contact_data_count=64,
    )
    assert cb.sensor_count == 1 + len(targets)

    forces = CudaArray((cb.sensor_count, 3), dtype=np.float32)
    cb.read_net_forces(forces.dltensor)
