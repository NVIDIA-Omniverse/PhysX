# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""GPU-mode lifecycle and state-management tests.

Covers gaps not addressed by test_tensor_bindings_api_gpu.py or the GPU session
conftest:
  - warmup_gpu explicit timing / idempotency / after-reset behaviour
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
from test_utils import data_path
from test_utils import load_usd_with_ovstage

_RB_PATTERN = "/World/Cube*"
_ARTI_PATTERN = "/World/articulation*"
_SENSOR_PAT = "/World/Cube*"
_FILTER_PAT = "/World/GroundPlane"


def _load_rb(sdk, warmup=True, n_steps=3):
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    if warmup:
        sdk.warmup_gpu()
    for _ in range(n_steps):
        sdk.step_sync(1.0 / 60.0)


def _load_artic(sdk, n_steps=3):
    load_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
    sdk.wait_all()
    sdk.warmup_gpu()
    for _ in range(n_steps):
        sdk.step_sync(1.0 / 60.0)


# ---------------------------------------------------------------------------
# warmup_gpu behaviour on GPU
# ---------------------------------------------------------------------------


def test_warmup_gpu_explicit_then_first_read(physx_sdk):
    """Explicit warmup_gpu before first tensor read must succeed and not error."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    physx_sdk.warmup_gpu()

    # Tensor read after explicit warmup should not trigger another warmup step
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
    try:
        buf = np.zeros(binding.shape, dtype=np.float32)
        binding.read(buf)  # must not raise
    finally:
        binding.destroy()


def test_warmup_gpu_multiple_calls_idempotent(physx_sdk):
    """warmup_gpu() called three times must not raise and state must be consistent."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    physx_sdk.warmup_gpu()
    physx_sdk.warmup_gpu()
    physx_sdk.warmup_gpu()

    # State must still be usable
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
    try:
        buf = np.zeros(binding.shape, dtype=np.float32)
        binding.read(buf)
    finally:
        binding.destroy()


def test_warmup_gpu_after_reset_triggers_again(physx_sdk):
    """After reset+reload, an explicit warmup_gpu must succeed (GPU state re-initializes)."""
    _load_rb(physx_sdk, warmup=True)

    physx_sdk.reset_stage()
    physx_sdk.wait_all()

    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    physx_sdk.warmup_gpu()

    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
    try:
        buf = np.zeros(binding.shape, dtype=np.float32)
        binding.read(buf)
    finally:
        binding.destroy()


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
    physx_sdk.warmup_gpu()
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
    physx_sdk.warmup_gpu()
    try:
        paths = cb.sensor_paths
        assert isinstance(paths, list)
        assert len(paths) == cb.sensor_count
        for p in paths:
            assert isinstance(p, str)
    finally:
        cb.destroy()


def test_contact_binding_unfiltered_read_force_matrix_raises_gpu(physx_sdk):
    """read_force_matrix on an unfiltered GPU binding must raise."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()
    cb = physx_sdk.create_contact_binding(
        sensor_patterns=[_SENSOR_PAT],
        max_contact_data_count=128,
    )
    physx_sdk.warmup_gpu()
    physx_sdk.step_sync(1.0 / 60.0)
    try:
        buf = np.zeros((cb.sensor_count, cb.filter_count, 3), dtype=np.float32)
        with pytest.raises((RuntimeError, ValueError)):
            cb.read_force_matrix(buf)
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
    physx_sdk.warmup_gpu()
    for _ in range(20):  # let boxes fall and generate contacts
        physx_sdk.step_sync(1.0 / 60.0)

    report = physx_sdk.get_contact_report(include_friction_anchors=False)
    assert isinstance(report, dict)
    assert "num_headers" in report
    assert isinstance(report["num_headers"], int)
    assert report["num_headers"] >= 0
    assert "num_points" in report
    assert isinstance(report["num_points"], int)

    # If there are contacts, verify struct field access
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
    """GPU instance WITHOUT suppressReadback must support basic tensor read/write.

    The default since 0.4.1: GPU without DirectGPU. This subprocess verifies
    that tensor bindings still work in the non-DirectGPU GPU path.
    """
    _tests_dir = os.path.dirname(os.path.abspath(__file__))
    _data_dir = os.path.join(_tests_dir, "..", "data")
    script = textwrap.dedent(f"""
        import sys, os
        sys.path.insert(0, {repr(os.path.dirname(os.path.abspath(__file__)))})
        from ovphysx import PhysX
        from ovphysx.types import TensorType
        from test_utils import load_usd_with_ovstage
        import numpy as np

        usd_path = os.path.join({repr(_data_dir)}, "boxes_falling_on_groundplane.usda")

        # GPU WITHOUT suppressReadback (default after 0.4.1)
        physx = PhysX()
        load_usd_with_ovstage(physx, usd_path)
        physx.wait_all()
        physx.warmup_gpu()
        physx.step_sync(1.0 / 60.0)

        binding = physx.create_tensor_binding(
            pattern="/World/Cube*",
            tensor_type=TensorType.RIGID_BODY_POSE,
        )
        buf = np.zeros(binding.shape, dtype=np.float32)
        binding.read(buf)  # must succeed without DirectGPU
        assert buf.shape[1] == 7
        binding.destroy()
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
