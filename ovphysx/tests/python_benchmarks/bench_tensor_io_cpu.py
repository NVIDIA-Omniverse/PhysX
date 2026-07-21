# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""CPU-side tensor binding read/write throughput via DLPack <-> numpy."""

import numpy as np
import pytest

from ovphysx.types import TensorType
from conftest import attach_usd_with_ovstage, cpu_only


@pytest.fixture
def pose_binding(physx, data_dir, bench_device):
    cpu_only(bench_device)
    attach_usd_with_ovstage(physx, data_dir / "basic_simulation.usda", "ovphysx-bench-cpu-pose")
    physx.wait_all()
    binding = physx.create_tensor_binding(
        pattern="/World/envs/env0/*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    yield binding
    binding.destroy()


def test_pose_create_cpu(benchmark, physx, data_dir, bench_device):
    cpu_only(bench_device)
    attach_usd_with_ovstage(physx, data_dir / "basic_simulation.usda", "ovphysx-bench-cpu-create")
    physx.wait_all()

    def create_and_destroy():
        binding = physx.create_tensor_binding(
            pattern="/World/envs/env0/*",
            tensor_type=TensorType.RIGID_BODY_POSE,
        )
        _ = binding.shape
        binding.destroy()

    benchmark(create_and_destroy)


def test_pose_read_cpu(benchmark, pose_binding):
    arr = np.zeros(pose_binding.shape, dtype=np.float32)

    def read_once():
        pose_binding.read(arr)

    benchmark(read_once)


def test_pose_write_cpu(benchmark, pose_binding):
    # Prime the buffer with the binding's current state. Without this, the
    # write benchmark pushes (0,0,0,0) quaternions every iteration; PhysX
    # rejects with "PxRigidDynamic::setGlobalPose: pose is not valid",
    # biasing the benchmark toward rejection-path cost and spamming logs.
    # See TensorBindings.cpp:131-138 for the C++ equivalent priming step.
    arr = np.zeros(pose_binding.shape, dtype=np.float32)
    pose_binding.read(arr)

    def write_once():
        pose_binding.write(arr)

    benchmark(write_once)
