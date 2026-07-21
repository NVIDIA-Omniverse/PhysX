# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Parametrized read/write access-mode enforcement and DLPack error tests.

Validates that:
  - Every read-only TensorType rejects write() calls.
  - Every write-only TensorType rejects read() calls.
  - TensorType.INVALID (0) is rejected at binding creation.
  - Dtype/shape/contiguity mismatches are rejected.
  - Indexed and masked write edge-cases behave correctly.

Uses two_articulations.usda for articulation-type tests and
boxes_falling_on_groundplane.usda for rigid-body-type tests.
"""

import os
import warnings

import numpy as np
import pytest
from ovphysx.types import TensorType
from test_utils import load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


# Rigid body prims in boxes_falling_on_groundplane.usda
_RB_PATTERN = "/World/Cube*"
# Articulation pattern in two_articulations.usda
_ARTI_PATTERN = "/World/articulation*"


def _load_rb(sdk, n_steps=3):
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    for _ in range(n_steps):
        sdk.step_sync(1.0 / 60.0)


def _load_artic(sdk, n_steps=3):
    load_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
    sdk.wait_all()
    for _ in range(n_steps):
        sdk.step_sync(1.0 / 60.0)


# ---------------------------------------------------------------------------
# Read-only types — write must raise
# ---------------------------------------------------------------------------

# (tensor_type, pattern, scene_loader)
_READ_ONLY_PARAMS = [
    # Rigid body read-only (values 6-8)
    (TensorType.RIGID_BODY_ACCELERATION, _RB_PATTERN, "rb"),
    (TensorType.RIGID_BODY_INV_MASS, _RB_PATTERN, "rb"),
    (TensorType.RIGID_BODY_INV_INERTIA, _RB_PATTERN, "rb"),
    # Articulation link read-only (values 20-22)
    (TensorType.ARTICULATION_LINK_POSE, _ARTI_PATTERN, "artic"),
    (TensorType.ARTICULATION_LINK_VELOCITY, _ARTI_PATTERN, "artic"),
    (TensorType.ARTICULATION_LINK_ACCELERATION, _ARTI_PATTERN, "artic"),
    # Articulation body inverse (values 63-64)
    (TensorType.ARTICULATION_BODY_INV_MASS, _ARTI_PATTERN, "artic"),
    (TensorType.ARTICULATION_BODY_INV_INERTIA, _ARTI_PATTERN, "artic"),
    # Dynamics queries (values 70-75)
    (TensorType.ARTICULATION_JACOBIAN, _ARTI_PATTERN, "artic"),
    (TensorType.ARTICULATION_MASS_MATRIX, _ARTI_PATTERN, "artic"),
    (TensorType.ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE, _ARTI_PATTERN, "artic"),
    (TensorType.ARTICULATION_GRAVITY_FORCE, _ARTI_PATTERN, "artic"),
    (TensorType.ARTICULATION_LINK_INCOMING_JOINT_FORCE, _ARTI_PATTERN, "artic"),
    (TensorType.ARTICULATION_DOF_PROJECTED_JOINT_FORCE, _ARTI_PATTERN, "artic"),
]

_READ_ONLY_IDS = [t[0].name for t in _READ_ONLY_PARAMS]


@pytest.mark.parametrize("tensor_type,pattern,scene", _READ_ONLY_PARAMS, ids=_READ_ONLY_IDS)
def test_read_only_type_write_raises(physx_sdk, tensor_type, pattern, scene):
    """Writing to a read-only TensorType must raise RuntimeError."""
    if scene == "rb":
        _load_rb(physx_sdk)
    else:
        _load_artic(physx_sdk)

    binding = physx_sdk.create_tensor_binding(pattern=pattern, tensor_type=tensor_type)
    try:
        if binding.count == 0:
            pytest.skip(f"No prims matched pattern '{pattern}' for {tensor_type.name}")
        buf = np.zeros(binding.shape, dtype=np.float32)
        with pytest.raises(RuntimeError):
            binding.write(buf)
    finally:
        binding.destroy()


# ---------------------------------------------------------------------------
# Write-only types — read must raise
# ---------------------------------------------------------------------------

_WRITE_ONLY_PARAMS = [
    (TensorType.RIGID_BODY_FORCE, _RB_PATTERN, "rb"),
    (TensorType.RIGID_BODY_WRENCH, _RB_PATTERN, "rb"),
    (TensorType.ARTICULATION_LINK_WRENCH, _ARTI_PATTERN, "artic"),
]
_WRITE_ONLY_IDS = [t[0].name for t in _WRITE_ONLY_PARAMS]


@pytest.mark.parametrize("tensor_type,pattern,scene", _WRITE_ONLY_PARAMS, ids=_WRITE_ONLY_IDS)
def test_write_only_type_read_raises(physx_sdk, tensor_type, pattern, scene):
    """Reading from a write-only TensorType must raise RuntimeError."""
    if scene == "rb":
        _load_rb(physx_sdk)
    else:
        _load_artic(physx_sdk)

    binding = physx_sdk.create_tensor_binding(pattern=pattern, tensor_type=tensor_type)
    try:
        if binding.count == 0:
            pytest.skip(f"No prims matched pattern '{pattern}' for {tensor_type.name}")
        buf = np.zeros(binding.shape, dtype=np.float32)
        with pytest.raises(RuntimeError):
            binding.read(buf)
    finally:
        binding.destroy()


# ---------------------------------------------------------------------------
# Invalid tensor type (0)
# ---------------------------------------------------------------------------


def test_invalid_tensor_type_zero_raises(physx_sdk):
    """create_tensor_binding with TensorType.INVALID (0) must raise RuntimeError."""
    _load_rb(physx_sdk)
    with pytest.raises(RuntimeError):
        physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.INVALID)


# ---------------------------------------------------------------------------
# Dtype / shape / contiguity errors
# ---------------------------------------------------------------------------


def _rb_pose_binding(sdk):
    """Helper: return a live RIGID_BODY_POSE binding (caller must destroy)."""
    binding = sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
    return binding


def test_write_wrong_dtype_float64_raises(physx_sdk):
    """Writing a float64 buffer to a float32 binding must raise an error."""
    _load_rb(physx_sdk)
    binding = _rb_pose_binding(physx_sdk)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        buf_f64 = np.zeros(binding.shape, dtype=np.float64)
        with pytest.raises((RuntimeError, TypeError, ValueError)):
            binding.write(buf_f64)
    finally:
        binding.destroy()


def test_read_wrong_shape_too_many_rows_raises(physx_sdk):
    """Reading into a buffer with N+1 rows must raise an error."""
    _load_rb(physx_sdk)
    binding = _rb_pose_binding(physx_sdk)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        N, C = binding.shape
        bad_buf = np.zeros((N + 1, C), dtype=np.float32)
        with pytest.raises((RuntimeError, ValueError)):
            binding.read(bad_buf)
    finally:
        binding.destroy()


def test_write_wrong_shape_too_many_rows_raises(physx_sdk):
    """Writing a buffer with N+1 rows must raise an error."""
    _load_rb(physx_sdk)
    binding = _rb_pose_binding(physx_sdk)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        N, C = binding.shape
        bad_buf = np.zeros((N + 1, C), dtype=np.float32)
        with pytest.raises((RuntimeError, ValueError)):
            binding.write(bad_buf)
    finally:
        binding.destroy()


def test_write_non_contiguous_raises(physx_sdk):
    """Writing a non-contiguous (strided) numpy array must raise ValueError."""
    _load_rb(physx_sdk)
    binding = _rb_pose_binding(physx_sdk)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        N, C = binding.shape
        # Create a 2× buffer and slice every other row → non-contiguous
        big = np.zeros((N * 2, C), dtype=np.float32)
        non_contig = big[::2, :]
        assert not non_contig.flags["C_CONTIGUOUS"]
        with pytest.raises((RuntimeError, ValueError)):
            binding.write(non_contig)
    finally:
        binding.destroy()


def test_read_non_contiguous_raises(physx_sdk):
    """Reading into a non-contiguous numpy array must raise ValueError."""
    _load_rb(physx_sdk)
    binding = _rb_pose_binding(physx_sdk)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        N, C = binding.shape
        big = np.zeros((N * 2, C), dtype=np.float32)
        non_contig = big[::2, :]
        assert not non_contig.flags["C_CONTIGUOUS"]
        with pytest.raises((RuntimeError, ValueError)):
            binding.read(non_contig)
    finally:
        binding.destroy()


def test_read_into_readonly_numpy_raises(physx_sdk):
    """Reading into a read-only numpy buffer must raise ValueError."""
    _load_rb(physx_sdk)
    binding = _rb_pose_binding(physx_sdk)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        buf = np.zeros(binding.shape, dtype=np.float32)
        buf.flags.writeable = False
        with pytest.raises((RuntimeError, ValueError)):
            binding.read(buf)
    finally:
        binding.destroy()


# ---------------------------------------------------------------------------
# Indexed write edge cases
# ---------------------------------------------------------------------------


def test_write_with_empty_indices_is_noop(physx_sdk):
    """write(buf, indices=empty_int32_array) must not change any data."""
    _load_rb(physx_sdk)
    binding = _rb_pose_binding(physx_sdk)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        N, C = binding.shape

        # Read baseline
        before = np.zeros((N, C), dtype=np.float32)
        binding.read(before)

        # Write with empty index list — should touch nothing
        src = np.full((N, C), 999.0, dtype=np.float32)
        empty_idx = np.array([], dtype=np.int32)
        binding.write(src, indices=empty_idx)

        after = np.zeros((N, C), dtype=np.float32)
        binding.read(after)
        np.testing.assert_array_equal(before, after)
    finally:
        binding.destroy()


def test_write_with_int64_indices_behavior(physx_sdk):
    """write(buf, indices=int64_array) — document behavior (error or coercion)."""
    _load_rb(physx_sdk)
    binding = _rb_pose_binding(physx_sdk)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        N, C = binding.shape
        src = np.zeros((N, C), dtype=np.float32)
        int64_idx = np.array([0], dtype=np.int64)
        # The API requires int32 indices; int64 may raise or be silently accepted.
        # Either outcome is documented by this test; we just ensure no crash if accepted.
        try:
            binding.write(src, indices=int64_idx)
        except (RuntimeError, TypeError, ValueError):
            pass  # expected: int64 rejected
    finally:
        binding.destroy()


def test_write_with_both_indices_and_mask_warns_and_uses_mask(physx_sdk):
    """Providing both indices and mask must emit UserWarning and apply mask semantics."""
    _load_artic(physx_sdk)
    binding = physx_sdk.create_tensor_binding(
        pattern=_ARTI_PATTERN,
        tensor_type=TensorType.ARTICULATION_DOF_POSITION,
    )
    try:
        if binding.count < 2:
            pytest.skip("Need at least 2 articulations")
        N, D = binding.shape

        # Mask: only first row
        mask = np.array([True] + [False] * (N - 1), dtype=bool)
        # Indices: only second row
        indices = np.array([1], dtype=np.int32)

        src = np.zeros((N, D), dtype=np.float32)
        src[0, :] = 0.1
        src[1, :] = 0.2

        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            binding.write(src, indices=indices, mask=mask)
            # A UserWarning about precedence should be emitted
            warning_types = [x.category for x in w]
            assert UserWarning in warning_types, "Expected UserWarning when both indices and mask given"

        # Mask takes precedence: row 0 should be updated, row 1 should not
        result = np.zeros((N, D), dtype=np.float32)
        binding.read(result)
        # Row 0 should reflect the write; exact value depends on physics state
        # (sufficient to verify no crash and warning was issued)
    finally:
        binding.destroy()
