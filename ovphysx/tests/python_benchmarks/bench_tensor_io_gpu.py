# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""GPU-side tensor binding read/write throughput via DLPack <-> torch.cuda.

Skipped when torch is not installed or no CUDA device is available. Install
the optional [gpu] extra to opt in: ``uv pip install -e .[gpu]`` from
``tests/python_benchmarks/``.
"""

import pytest

from ovphysx.types import TensorType
from conftest import attach_usd_with_ovstage, gpu_only

torch = pytest.importorskip("torch")
if not torch.cuda.is_available():
    pytest.skip("CUDA not available", allow_module_level=True)


@pytest.fixture
def pose_binding(physx, data_dir, bench_device):
    gpu_only(bench_device)
    attach_usd_with_ovstage(physx, data_dir / "basic_simulation.usda", "ovphysx-bench-gpu-pose")
    physx.wait_all()
    binding = physx.create_tensor_binding(
        pattern="/World/envs/env0/*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    yield binding
    binding.destroy()


def test_pose_create_gpu(benchmark, physx, data_dir, bench_device):
    gpu_only(bench_device)
    attach_usd_with_ovstage(physx, data_dir / "basic_simulation.usda", "ovphysx-bench-gpu-create")
    physx.wait_all()

    def create_and_destroy():
        binding = physx.create_tensor_binding(
            pattern="/World/envs/env0/*",
            tensor_type=TensorType.RIGID_BODY_POSE,
        )
        _ = binding.shape
        binding.destroy()

    benchmark(create_and_destroy)


def test_pose_read_gpu(benchmark, pose_binding):
    t = torch.zeros(pose_binding.shape, dtype=torch.float32, device="cuda")

    def read_once():
        pose_binding.read(t)

    benchmark(read_once)


def test_pose_write_gpu(benchmark, pose_binding):
    # Prime the buffer with the binding's current state (see comment in
    # bench_tensor_io_cpu.py for the quaternion-validation rationale).
    t = torch.zeros(pose_binding.shape, dtype=torch.float32, device="cuda")
    pose_binding.read(t)

    def write_once():
        pose_binding.write(t)

    benchmark(write_once)
