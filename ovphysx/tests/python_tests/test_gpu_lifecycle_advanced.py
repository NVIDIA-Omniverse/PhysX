# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-BINDING-DEVICE-001
# @covers AC-2
# @maps_to TEST-PYTHON-BINDING-DEVICE-001
# @implements REQ-INPUT-CORE-001
# @covers AC-4
# @maps_to TEST-INPUT-CORE-001
# @implements REQ-CAPI-WRITE-001
# @covers AC-1 AC-5a AC-9
# @maps_to TEST-CAPI-WRITE-001
# PARTIALLY DEPRECATED (tensor-binding-deprecation): test_prim_paths_gpu_binding and test_prim_paths_gpu_zero_count_binding exercise the binding's own prim_paths / count accessors, which retire with the binding and have no session-read equivalent. The warmup-read and GPU-without-DirectGPU tests here use PhysX.read, and the ContactBinding tests are a separate API that stays.

"""GPU-mode lifecycle and state-management tests.

Covers gaps not addressed by test_tensor_bindings_api_gpu.py or the GPU session
conftest:
  - warmup explicit timing / idempotency / after-reset behaviour
  - prim_paths on GPU bindings
  - ContactBinding GPU properties and unfiltered error paths
  - step_n_sync n=0/-1 boundary on GPU
  - get_contact_report on GPU
  - GPU mode WITHOUT DirectGPU (no suppressReadback) via a separate fixture
"""

import os
import subprocess
import sys
import textwrap

import numpy as np
import pytest
from ovphysx.types import TensorType
from test_utils import data_path, load_usd_with_ovstage, read_rigid_body_poses

_RB_PATTERN = "/World/Cube*"
_ARTI_PATTERN = "/World/articulation*"
_SENSOR_PAT = "/World/Cube*"
_FILTER_PAT = "/World/GroundPlane"


def _load_rb(sdk, warmup=True, n_steps=3):
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    if warmup:
        sdk.warmup()
    for _ in range(n_steps):
        sdk.step_sync(1.0 / 60.0)


def _load_artic(sdk, n_steps=3):
    load_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
    sdk.wait_all()
    sdk.warmup()
    for _ in range(n_steps):
        sdk.step_sync(1.0 / 60.0)


# ---------------------------------------------------------------------------
# warmup behaviour on GPU
# ---------------------------------------------------------------------------


def test_warmup_explicit_then_first_read(physx_sdk):
    """Explicit warmup before first tensor read must succeed and not error."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup()

    # A read session after explicit warmup must open and yield the rigid-body poses.
    assert read_rigid_body_poses(physx_sdk).shape[0] > 0, "expected rigid-body poses after warmup"


def test_warmup_multiple_calls_idempotent(physx_sdk):
    """warmup() called three times must not raise and state must be consistent."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    physx_sdk.warmup()
    physx_sdk.warmup()
    physx_sdk.warmup()

    # State must still be usable: a read session opens and yields the rigid-body poses.
    assert read_rigid_body_poses(physx_sdk).shape[0] > 0, "expected rigid-body poses after warmup"


def test_warmup_after_reset_triggers_again(physx_sdk):
    """After reset+reload, an explicit warmup must succeed (GPU state re-initializes)."""
    _load_rb(physx_sdk, warmup=True)

    physx_sdk.reset_stage()
    physx_sdk.wait_all()

    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    physx_sdk.warmup()

    # After reset + reload + warmup, a read session must open and yield the rigid-body poses.
    assert read_rigid_body_poses(physx_sdk).shape[0] > 0, "expected rigid-body poses after warmup"


# ---------------------------------------------------------------------------
# prim_paths on GPU binding
# ---------------------------------------------------------------------------


def test_prim_paths_gpu_binding(physx_sdk):
    """prim_paths on a RIGID_BODY_POSE GPU binding must return a list of USD paths."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims")
        paths = binding.prim_paths
        assert isinstance(paths, list)
        assert len(paths) == binding.count
        for p in paths:
            assert isinstance(p, str) and p.startswith("/")
    finally:
        binding.destroy()


def test_prim_paths_gpu_zero_count_binding(physx_sdk):
    """A binding matching no prims on GPU must have prim_paths == []."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(
        pattern="/World/NonExistentPrimXYZ*",
        tensor_type=TensorType.RIGID_BODY_POSE,
    )
    try:
        assert binding.count == 0
        assert binding.prim_paths == []
    finally:
        binding.destroy()


# ---------------------------------------------------------------------------
# ContactBinding GPU properties
# ---------------------------------------------------------------------------


def test_contact_binding_max_count_property_gpu(physx_sdk):
    """max_contact_data_count must equal the capacity passed at creation (GPU)."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    CAPACITY = 256
    cb = physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PAT],
        filter_patterns=[_FILTER_PAT],
        filters_per_sensor=1,
        max_contact_data_count=CAPACITY,
    )
    physx_sdk.warmup()
    try:
        assert cb.max_contact_data_count == CAPACITY
    finally:
        cb.destroy()


def test_contact_binding_sensor_paths_gpu(physx_sdk):
    """sensor_paths must be a list of strings with length == sensor_count (GPU)."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    cb = physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PAT],
        filter_patterns=[_FILTER_PAT],
        filters_per_sensor=1,
        max_contact_data_count=128,
    )
    physx_sdk.warmup()
    try:
        paths = cb.sensor_paths
        assert isinstance(paths, list)
        assert len(paths) == cb.sensor_count
        for p in paths:
            assert isinstance(p, str)
    finally:
        cb.destroy()


def test_contact_binding_unfiltered_read_normal_force_matrix_raises_gpu(physx_sdk):
    """read_normal_force_matrix on an unfiltered GPU binding must raise."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    cb = physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PAT],
        max_contact_data_count=128,
    )
    physx_sdk.warmup()
    physx_sdk.step_sync(1.0 / 60.0)
    try:
        buf = np.zeros((cb.sensor_count, cb.filter_count, 3), dtype=np.float32)
        with pytest.raises((RuntimeError, ValueError)):
            cb.read_normal_force_matrix(buf)
    finally:
        cb.destroy()


# ---------------------------------------------------------------------------
# step_n_sync boundary on GPU
# ---------------------------------------------------------------------------


def test_step_n_sync_zero_raises_gpu(physx_sdk):
    """step_n_sync(n=0) must raise RuntimeError on GPU instance."""
    _load_rb(physx_sdk)
    with pytest.raises(RuntimeError):
        physx_sdk.step_n_sync(n=0, dt=1.0 / 60.0)


def test_step_n_sync_negative_raises_gpu(physx_sdk):
    """step_n_sync(n=-1) must raise RuntimeError on GPU instance."""
    _load_rb(physx_sdk)
    with pytest.raises(RuntimeError):
        physx_sdk.step_n_sync(n=-1, dt=1.0 / 60.0)


# ---------------------------------------------------------------------------
# get_contact_report on GPU
# ---------------------------------------------------------------------------


def test_get_contact_report_gpu(physx_sdk):
    """get_contact_report on GPU after a step must return a dict with
    num_headers >= 0 and accessible struct fields."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup()
    for _ in range(20):  # let boxes fall and generate contacts
        physx_sdk.step_sync(1.0 / 60.0)

    report = physx_sdk.get_contact_report(include_friction_anchors=False)
    assert isinstance(report, dict)
    assert "num_headers" in report
    assert isinstance(report["num_headers"], int)
    assert report["num_headers"] >= 0
    assert "num_points" in report
    assert isinstance(report["num_points"], int)

    if report["num_headers"] > 0:
        h = report["headers"][0]
        assert hasattr(h, "actor0")
        assert hasattr(h, "actor1")
        assert hasattr(h, "numContactData")


# ---------------------------------------------------------------------------
# GPU mode WITHOUT DirectGPU (no suppressReadback)
# Runs in a subprocess to avoid polluting the shared GPU session.
# ---------------------------------------------------------------------------


def test_gpu_mode_without_directgpu():
    """GPU instance WITHOUT suppressReadback must support session read and write.

    GPU without DirectGPU is the default. This subprocess verifies PhysX.read
    and the disableGravity simulation effect through PhysX.write.
    """
    _tests_dir = os.path.dirname(os.path.abspath(__file__))
    _data_dir = os.path.join(_tests_dir, "..", "data")
    script = textwrap.dedent(f"""
        import sys, os
        sys.path.insert(0, {repr(os.path.dirname(os.path.abspath(__file__)))})
        import numpy as np
        from ovphysx import PhysX
        from ovphysx.dlpack import DLDeviceType
        from ovphysx.types import TensorType
        from ovphysx.types import ObjectScope, SimObjectType
        from test_utils import load_usd_with_ovstage

        usd_path = os.path.join({repr(_data_dir)}, "boxes_falling_on_groundplane_gpu.usda")

        # GPU WITHOUT suppressReadback (default after 0.4.1)
        physx = PhysX()
        load_usd_with_ovstage(physx, usd_path)
        physx.wait_all()
        physx.warmup()
        physx.step_sync(1.0 / 60.0)

        binding = physx.create_tensor_binding(
            pattern="/World/Cube*",
            tensor_type=TensorType.RIGID_BODY_POSE,
        )
        assert binding.native_device.device_type.value == DLDeviceType.kDLCPU
        assert binding.native_device.device_id == 0
        buf = np.zeros(binding.shape, dtype=np.float32)
        binding.read(buf)  # must succeed without DirectGPU
        assert buf.shape[1] == 7
        binding.destroy()
        with physx.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
            # The rigid-body pose column must read back without DirectGPU. Check every partition's
            # column, not just groups[0] -- the first-group-only shape is the one that keeps regressing.
            assert result.groups, "read returned no groups"
            for g in result.groups:
                assert g.tensors and g.tensors[0].shape[1] == 3

        def write_uniform(attribute, value, dtype):
            with physx.write(SimObjectType.RIGID_BODY, attribute) as session:
                assert session.groups
                for group in session.groups:
                    tensor = group.tensors[0]
                    tensor.assign(np.full(tensor.shape, value, dtype=dtype))
                    session.commit(group)

        def read_positions():
            with physx.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
                assert result.groups
                return np.concatenate([group.tensors[0].numpy() for group in result.groups])

        def read_velocities():
            with physx.read(SimObjectType.RIGID_BODY, ["linearVelocity"], scope=ObjectScope.ALL) as result:
                assert result.groups
                return np.concatenate([group.tensors[0].numpy() for group in result.groups])

        def write_alternating_disable_gravity():
            offset = 0
            with physx.write(SimObjectType.RIGID_BODY, "disableGravity") as session:
                assert session.groups
                for group in session.groups:
                    tensor = group.tensors[0]
                    values = np.zeros(tensor.shape, dtype=np.uint8)
                    values.reshape(-1)[(offset % 2)::2] = 1
                    tensor.assign(values)
                    session.commit(group)
                    offset += values.size

        before_velocity = read_velocities()
        write_alternating_disable_gravity()
        physx.step_sync(1.0 / 60.0)
        after_velocity = read_velocities()
        np.testing.assert_allclose(after_velocity[::2, 2], before_velocity[::2, 2], rtol=0, atol=2e-2)
        assert np.all(after_velocity[1::2, 2] < before_velocity[1::2, 2] - 0.1)
        held = read_positions()

        write_uniform("disableGravity", 0, np.uint8)
        for _ in range(20):
            physx.step_sync(1.0 / 60.0)
        released = read_positions()
        assert np.all(released[:, 2] < held[:, 2] - 0.05)
        print("GPU_NO_DIRECTGPU_OK")
    """)

    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=120)
    if result.returncode != 0:
        # GPU not available is acceptable in headless environments
        combined = result.stdout + result.stderr
        if any(kw in combined for kw in ["GPU_NOT_AVAILABLE", "CUDA", "No CUDA"]):
            pytest.skip("GPU not available in this environment")
        pytest.fail(f"GPU-without-DirectGPU test failed:\n" f"STDOUT: {result.stdout}\nSTDERR: {result.stderr}")
    assert "GPU_NO_DIRECTGPU_OK" in result.stdout


def test_prestep_write_applies_without_directgpu():
    """GPU with readback: a pre-step session write commits and is applied (AC-5a)."""
    _tests_dir = os.path.dirname(os.path.abspath(__file__))
    _data_dir = os.path.join(_tests_dir, "..", "data")
    script = textwrap.dedent(f"""
        import os, sys
        sys.path.insert(0, {repr(os.path.dirname(os.path.abspath(__file__)))})
        import numpy as np
        import warp as wp
        from ovphysx import PhysX
        from ovphysx.types import SimObjectType
        from test_utils import load_usd_with_ovstage

        usd_path = os.path.join({repr(_data_dir)}, "boxes_falling_on_groundplane.usda")
        # GPU-with-readback tensors stay on host; do not assert tensor.device.is_cuda.
        # Fail before PhysX() if this process has no CUDA, so a CPU fallback cannot
        # print GPU_PRESTEP_WRITE_OK. Parent skip matches "No CUDA devices found".
        if wp.get_cuda_device_count() == 0:
            raise RuntimeError("No CUDA devices found")
        physx = PhysX()
        assert not PhysX.get_cpu_mode(), "process is hard CPU-only; this case is GPU-with-readback"
        load_usd_with_ovstage(physx, usd_path)
        physx.wait_all()
        with physx.write(SimObjectType.RIGID_BODY, "position") as w:
            assert w.groups, "expected writable groups before the first step"
            float_off = 0
            chunks = []
            for g in w.groups:
                n = g.prim_count
                block = np.arange(n * 3, dtype=np.float32).reshape(n, 3) + 100.0 + float_off
                g.tensors[0].assign(np.ascontiguousarray(block).reshape(g.tensors[0].shape))
                cuda = next((t for t in g.tensors if t.size and t.device.is_cuda), None)
                if cuda is None:
                    w.commit(g)
                else:
                    stream = wp.get_stream(cuda.device)
                    w.commit(g, cuda_stream=int(stream.cuda_stream or 1))
                chunks.append(block)
                float_off += n * 3
            target = np.concatenate(chunks)
        with physx.read(SimObjectType.RIGID_BODY, ["position"]) as r:
            assert r.groups, "GPU-with-readback can read before the first step"
            got = np.concatenate([g.tensors[0].numpy().reshape(g.prim_count, -1) for g in r.groups])
        assert got.shape == target.shape
        assert np.allclose(got, target, rtol=0, atol=1e-3), got
        print("GPU_PRESTEP_WRITE_OK")
    """)

    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=120)
    if result.returncode != 0:
        combined = result.stdout + result.stderr
        # Bare "CUDA" would skip Warp/commit regressions that mention CUDA in the traceback.
        if any(
            kw in combined
            for kw in [
                "GPU_NOT_AVAILABLE",
                "CUDA driver not available",
                "No CUDA devices found",
            ]
        ):
            pytest.skip("GPU not available in this environment")
        pytest.fail(f"GPU-with-readback pre-step write failed:\nSTDOUT: {result.stdout}\nSTDERR: {result.stderr}")
    assert "GPU_PRESTEP_WRITE_OK" in result.stdout
