# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Regression coverage for NVBug 6428316."""

import numpy as np
import pytest
from ovphysx.types import TensorType as TT
from test_utils import CudaArray, data_path, load_usd_with_ovstage


def test_preclone_binding_velocity_reaches_all_envs(physx_sdk):
    """DirectGPU binding before clone must not leave stale GPU sim data."""
    N = 32
    load_usd_with_ovstage(physx_sdk, data_path("basic_simulation.usda"))
    physx_sdk.wait_all()

    # Deliberately create a binding before clone (the bug trigger).
    physx_sdk.create_tensor_binding(
        pattern="/World/envs/*/table", tensor_type=TT.RIGID_BODY_POSE
    )

    g = int(np.ceil(np.sqrt(N)))
    physx_sdk.clone(
        source_path="/World/envs/env0",
        target_paths=[f"/World/envs/env{i}" for i in range(1, N)],
        parent_transforms=[
            ((i % g) * 3.0, 0.0, (i // g) * 3.0, 0, 0, 0, 1)
            for i in range(1, N)
        ],
    )
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    vel = physx_sdk.create_tensor_binding(
        pattern="/World/envs/*/table", tensor_type=TT.RIGID_BODY_VELOCITY
    )
    pose = physx_sdk.create_tensor_binding(
        pattern="/World/envs/*/table", tensor_type=TT.RIGID_BODY_POSE
    )
    assert vel.count == N, f"expected {N} velocity rows, got {vel.count}"
    assert pose.count == N, f"expected {N} pose rows, got {pose.count}"

    vel.write(np.tile(np.array([3, 0, 0, 0, 0, 0], np.float32), (N, 1)))
    physx_sdk.wait_all()

    before = np.zeros(tuple(pose.shape), np.float32)
    pose.read(before)
    for _ in range(60):
        physx_sdk.step(1 / 240.0)
    physx_sdk.wait_all()
    after = np.zeros(tuple(pose.shape), np.float32)
    pose.read(after)

    moved = int(np.sum(np.abs(after[:, 0] - before[:, 0]) > 0.3))
    assert moved == N, (
        f"expected all {N} environments to move, got {moved}/{N}"
    )


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
    physx_sdk.warmup_gpu()

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
