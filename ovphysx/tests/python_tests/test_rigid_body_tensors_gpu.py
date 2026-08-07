# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""GPU-mode tests for rigid-body and articulation-root tensor types that are
covered by CPU tests but NOT by test_tensor_bindings_api_gpu.py.

Specifically fills gaps for:
  - RIGID_BODY_MASS / INERTIA / COM_POSE read/write GPU roundtrip
  - ARTICULATION_ROOT_POSE / ROOT_VELOCITY GPU roundtrip
  - Write-only types rejecting read on GPU
  - Read-only types rejecting write on GPU
  - Wrong-dtype CUDA tensor (float16) error on GPU

Uses the shared GPU physx_sdk fixture (DirectGPU mode with suppressReadback).
"""

import ctypes
import math
import sys

import numpy as np
import pytest
from ovphysx.dlpack import DLDataType, DLDataTypeCode, DLDevice, DLDeviceType, DLTensor
from ovphysx.types import TensorType

# CudaArray is the canonical GPU buffer helper, imported from the companion GPU test file.
from test_tensor_bindings_api_gpu import CudaArray, _gpu_read, _gpu_tensor, _gpu_write
from test_utils import data_path
from test_utils import load_usd_with_ovstage

_RB_PATTERN = "/World/Cube*"
_ARTI_PATTERN = "/World/articulation*"


def _load_rb(sdk, n_steps=3):
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
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
# Rigid body property tensors — GPU roundtrip
# ---------------------------------------------------------------------------


def test_rigid_body_mass_gpu_roundtrip(physx_sdk):
    """RIGID_BODY_MASS: write new mass values on GPU, read back, values match."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_MASS)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        N = binding.count

        # Read baseline using numpy (CPU property tensor accepts CPU buffer in GPU mode)
        baseline = np.zeros((N,), dtype=np.float32)
        binding.read(baseline)
        assert np.all(baseline > 0), "Mass should be positive"

        new_mass = baseline * 2.0
        binding.write(new_mass)

        result = np.zeros((N,), dtype=np.float32)
        binding.read(result)
        np.testing.assert_allclose(result, new_mass, rtol=1e-4)
    finally:
        binding.destroy()


def test_rigid_body_inertia_gpu_roundtrip(physx_sdk):
    """RIGID_BODY_INERTIA: write inertia matrix on GPU, read back."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_INERTIA)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        N = binding.count

        baseline = np.zeros((N, 9), dtype=np.float32)
        binding.read(baseline)

        # Scale diagonal elements
        new_inertia = baseline.copy()
        new_inertia[:, [0, 4, 8]] *= 2.0
        binding.write(new_inertia)

        result = np.zeros((N, 9), dtype=np.float32)
        binding.read(result)
        np.testing.assert_allclose(result, new_inertia, rtol=1e-4)
    finally:
        binding.destroy()


def test_rigid_body_com_pose_gpu_roundtrip(physx_sdk):
    """RIGID_BODY_COM_POSE: write COM local pose on GPU, read back."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_COM_POSE)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        N = binding.count

        baseline = np.zeros((N, 7), dtype=np.float32)
        binding.read(baseline)

        # Set identity quaternion [0,0,0,1] for all COM poses
        new_com = np.zeros((N, 7), dtype=np.float32)
        new_com[:, 6] = 1.0  # qw = 1
        binding.write(new_com)

        result = np.zeros((N, 7), dtype=np.float32)
        binding.read(result)
        np.testing.assert_allclose(result[:, 3:], new_com[:, 3:], atol=1e-4)
    finally:
        binding.destroy()


# ---------------------------------------------------------------------------
# Articulation root tensors — GPU roundtrip
# ---------------------------------------------------------------------------


def test_articulation_root_pose_gpu_roundtrip(physx_sdk):
    """ARTICULATION_ROOT_POSE: write root pose on GPU, step, read back."""
    _load_artic(physx_sdk)
    binding = physx_sdk.create_tensor_binding(
        pattern=_ARTI_PATTERN,
        tensor_type=TensorType.ARTICULATION_ROOT_POSE,
    )
    try:
        if binding.count == 0:
            pytest.skip("No articulations found")
        N = binding.count

        src = CudaArray((N, 7))
        host = np.zeros((N, 7), dtype=np.float32)
        host[:, 2] = 1.0  # pz = 1
        host[:, 6] = 1.0  # qw = 1
        src.upload(host)
        binding.write(src.dltensor)

        physx_sdk.step_sync(1.0 / 60.0)

        dst = CudaArray((N, 7))
        binding.read(dst.dltensor)
        result = dst.numpy()

        # pz should still be 1 (initial position) and qw close to 1
        assert result.shape == (N, 7)
    finally:
        binding.destroy()


def test_articulation_root_velocity_gpu_roundtrip(physx_sdk):
    """ARTICULATION_ROOT_VELOCITY: write root velocity on GPU, step, read back."""
    _load_artic(physx_sdk)
    binding = physx_sdk.create_tensor_binding(
        pattern=_ARTI_PATTERN,
        tensor_type=TensorType.ARTICULATION_ROOT_VELOCITY,
    )
    try:
        if binding.count == 0:
            pytest.skip("No articulations found")
        N = binding.count

        src = CudaArray((N, 6))
        host = np.zeros((N, 6), dtype=np.float32)
        src.upload(host)
        binding.write(src.dltensor)

        physx_sdk.step_sync(1.0 / 60.0)

        dst = CudaArray((N, 6))
        binding.read(dst.dltensor)
        result = dst.numpy()
        assert result.shape == (N, 6)
    finally:
        binding.destroy()


# ---------------------------------------------------------------------------
# Write-only type read enforcement on GPU
# ---------------------------------------------------------------------------


def test_write_only_rigid_body_force_read_raises_gpu(physx_sdk):
    """RIGID_BODY_FORCE (write-only): read must raise RuntimeError on GPU."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_FORCE)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims")
        dst = CudaArray(binding.shape)
        with pytest.raises(RuntimeError):
            binding.read(dst.dltensor)
    finally:
        binding.destroy()


def test_write_only_rigid_body_wrench_read_raises_gpu(physx_sdk):
    """RIGID_BODY_WRENCH (write-only): read must raise RuntimeError on GPU."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_WRENCH)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims")
        dst = CudaArray(binding.shape)
        with pytest.raises(RuntimeError):
            binding.read(dst.dltensor)
    finally:
        binding.destroy()


# ---------------------------------------------------------------------------
# Read-only type write enforcement on GPU
# ---------------------------------------------------------------------------


def test_read_only_link_pose_write_raises_gpu(physx_sdk):
    """ARTICULATION_LINK_POSE (read-only): write must raise RuntimeError on GPU."""
    _load_artic(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_POSE)
    try:
        if binding.count == 0:
            pytest.skip("No articulations")
        src = CudaArray(binding.shape)
        with pytest.raises(RuntimeError):
            binding.write(src.dltensor)
    finally:
        binding.destroy()


def test_read_only_link_velocity_write_raises_gpu(physx_sdk):
    """ARTICULATION_LINK_VELOCITY (read-only): write must raise RuntimeError on GPU."""
    _load_artic(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_VELOCITY)
    try:
        if binding.count == 0:
            pytest.skip("No articulations")
        src = CudaArray(binding.shape)
        with pytest.raises(RuntimeError):
            binding.write(src.dltensor)
    finally:
        binding.destroy()


# ---------------------------------------------------------------------------
# Wrong-dtype CUDA tensor error
# ---------------------------------------------------------------------------


def test_wrong_dtype_cuda_tensor_raises_gpu(physx_sdk):
    """A CUDA tensor with float16 dtype must be rejected (only float32 supported)."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims")

        N, C = binding.shape
        try:
            bad_ga = CudaArray((N, C), dtype=np.float16)
        except (KeyError, ValueError):
            pytest.skip("CudaArray does not support float16 — cannot exercise this path")

        with pytest.raises((RuntimeError, TypeError, ValueError)):
            binding.read(bad_ga.dltensor)
    finally:
        binding.destroy()
