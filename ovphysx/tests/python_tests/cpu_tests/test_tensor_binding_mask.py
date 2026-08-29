# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Tests for masked and indexed tensor binding writes (CPU mode).

These are normal pytest tests — no subprocess tricks needed because this entire
directory runs in its own pytest invocation with CPU-mode PhysX (see conftest.py
and test_python_runtime.cmake).
"""

import os

import numpy as np
from ovphysx.types import TensorType
from test_utils import destroy_ovstage_test_attachments, load_usd_with_ovstage

# Resolve test data directory relative to this file (../../data/)
_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_tensor_binding_write_mask_alternating(physx_sdk_cpu):
    """Masked write: only rows where mask=True are updated."""
    sdk = physx_sdk_cpu

    usd_path = os.path.join(_TEST_DIR, "data", "two_articulations.usda")
    op = load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_op(op)

    binding = sdk.create_tensor_binding(
        "/World/articulation*",
        tensor_type=TensorType.ARTICULATION_DOF_POSITION_TARGET,
    )
    try:
        assert binding.shape[0] == 2, f"Expected N=2, got {binding.shape[0]}"
        assert binding.shape[1] > 0

        # Read initial values
        initial = np.zeros(binding.shape, dtype=np.float32)
        binding.read(initial)

        # Write with mask=[True, False] — only row 0 should update
        src = np.full(binding.shape, 0.5, dtype=np.float32)
        mask = np.array([True, False], dtype=np.bool_)
        binding.write(src, mask=mask)

        after = np.zeros(binding.shape, dtype=np.float32)
        binding.read(after)

        assert np.allclose(after[0], 0.5), f"Row 0 should be 0.5, got {after[0]}"
        assert np.allclose(after[1], initial[1]), f"Row 1 should be unchanged, got {after[1]}"
    finally:
        binding.destroy()
        destroy_ovstage_test_attachments(sdk)
        sdk.reset_stage()
        sdk.wait_all()


def test_tensor_binding_write_mask_wrench_cpu(physx_sdk_cpu):
    """Masked wrench write: exercises AoS->SoA conversion under mask (CPU path).

    Wrenches are write-only (applied forces), so we can't read back -- just verify
    the call succeeds and the sim consumes the forces without error.
    """
    sdk = physx_sdk_cpu

    usd_path = os.path.join(_TEST_DIR, "data", "boxes_falling_on_groundplane.usda")
    op = load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_op(op)

    binding = sdk.create_tensor_binding(
        "/World/Cube*",
        tensor_type=TensorType.RIGID_BODY_WRENCH,
    )
    try:
        N = binding.shape[0]
        assert N >= 2, f"Expected at least 2 rigid bodies, got {N}"
        assert binding.shape[1] == 9

        dt = 1.0 / 60.0

        # Step once so the sim is warmed up
        op = sdk.step(dt)
        sdk.wait_op(op)

        # Apply wrench to first body only via mask
        src = np.zeros((N, 9), dtype=np.float32)
        src[0, :3] = [0.0, 100.0, 0.0]  # upward force on body 0
        mask = np.zeros(N, dtype=np.bool_)
        mask[0] = True
        binding.write(src, mask=mask)

        # Step to consume the forces -- should not crash or error
        op = sdk.step(dt)
        sdk.wait_op(op)

        # All-true mask
        mask_all = np.ones(N, dtype=np.bool_)
        binding.write(src, mask=mask_all)
        op = sdk.step(dt)
        sdk.wait_op(op)

        # All-false mask (no-op)
        mask_none = np.zeros(N, dtype=np.bool_)
        binding.write(src, mask=mask_none)
        op = sdk.step(dt)
        sdk.wait_op(op)
    finally:
        binding.destroy()
        destroy_ovstage_test_attachments(sdk)
        sdk.reset_stage()
        sdk.wait_all()


def test_tensor_binding_write_indices_full_tensor_semantics(physx_sdk_cpu):
    """Regression: indexed write requires full [N,...] src tensor."""
    sdk = physx_sdk_cpu

    usd_path = os.path.join(_TEST_DIR, "data", "two_articulations.usda")
    op = load_usd_with_ovstage(sdk, usd_path)
    sdk.wait_op(op)

    binding = sdk.create_tensor_binding(
        "/World/articulation*",
        tensor_type=TensorType.ARTICULATION_DOF_POSITION_TARGET,
    )
    try:
        assert binding.shape[0] == 2, f"Expected N=2, got {binding.shape[0]}"
        assert binding.shape[1] > 0

        # Initialize to known values
        init = np.full(binding.shape, 1.0, dtype=np.float32)
        binding.write(init)

        # Full src with distinct per-row values, indices=[0] updates only row 0
        src = np.empty(binding.shape, dtype=np.float32)
        src[0, :] = 0.25
        src[1, :] = 0.99
        indices = np.array([0], dtype=np.int32)
        binding.write(src, indices=indices)

        after = np.zeros(binding.shape, dtype=np.float32)
        binding.read(after)

        assert np.allclose(after[0], 0.25), f"Row 0 should be 0.25, got {after[0]}"
        assert np.allclose(after[1], 1.0), f"Row 1 should still be 1.0, got {after[1]}"
    finally:
        binding.destroy()
        destroy_ovstage_test_attachments(sdk)
        sdk.reset_stage()
        sdk.wait_all()
