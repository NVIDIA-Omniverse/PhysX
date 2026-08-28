# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Tests for TensorBinding.prim_paths property.

Complements the existing prim_paths coverage in test_tensor_bindings_api.py
(TestRigidBodyProperties) by adding:
  - Path correctness and ordering across multiple RB tensor types
  - Zero-count binding behaviour
  - Articulation binding returns root paths
  - Error after destroy
"""

import os

import pytest
from ovphysx.types import TensorType
from test_utils import load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


_RB_PATTERN = "/World/Cube*"
_ARTI_PATTERN = "/World/articulation*"


def _load_rb(sdk):
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()


def _load_artic(sdk):
    load_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
    sdk.wait_all()


# ---------------------------------------------------------------------------
# Correctness
# ---------------------------------------------------------------------------


def test_prim_paths_on_rigid_body_pose_binding(physx_sdk):
    """prim_paths for a RIGID_BODY_POSE binding returns a list of USD path strings."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
    try:
        if binding.count == 0:
            pytest.skip("No rigid body prims found")
        paths = binding.prim_paths
        assert isinstance(paths, list)
        assert len(paths) == binding.count
        for p in paths:
            assert isinstance(p, str)
            assert p.startswith("/"), f"Expected absolute USD path, got '{p}'"
    finally:
        binding.destroy()


def test_prim_paths_on_rigid_body_velocity_binding_matches_pose(physx_sdk):
    """prim_paths for RIGID_BODY_VELOCITY returns the same paths in the same order
    as a RIGID_BODY_POSE binding with the same pattern."""
    _load_rb(physx_sdk)
    pose_b = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
    vel_b = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_VELOCITY)
    try:
        if pose_b.count == 0:
            pytest.skip("No rigid body prims found")
        assert (
            pose_b.prim_paths == vel_b.prim_paths
        ), "prim_paths should be identical for the same pattern regardless of tensor type"
    finally:
        pose_b.destroy()
        vel_b.destroy()


def test_prim_paths_length_equals_count(physx_sdk):
    """len(binding.prim_paths) must equal binding.count."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_MASS)
    try:
        assert len(binding.prim_paths) == binding.count
    finally:
        binding.destroy()


# ---------------------------------------------------------------------------
# Zero-count binding
# ---------------------------------------------------------------------------


def test_prim_paths_zero_count_binding_returns_empty_list(physx_sdk):
    """A binding that matches no prims must return [] for prim_paths."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern="/World/NoSuchPrim*", tensor_type=TensorType.RIGID_BODY_POSE)
    try:
        assert binding.count == 0
        assert binding.prim_paths == []
    finally:
        binding.destroy()


# ---------------------------------------------------------------------------
# Articulation binding: returns one root path per articulation row
# ---------------------------------------------------------------------------


def test_prim_paths_on_articulation_binding_returns_root_paths(physx_sdk):
    """prim_paths on an ARTICULATION_* binding returns one root prim path per row.

    Per api.py: "Articulation bindings return one root prim path per
    articulation row. For per-articulation link names, use :attr:`body_names`."
    """
    _load_artic(physx_sdk)
    binding = physx_sdk.create_tensor_binding(
        pattern=_ARTI_PATTERN,
        tensor_type=TensorType.ARTICULATION_DOF_POSITION,
    )
    try:
        paths = binding.prim_paths
        assert isinstance(paths, list)
        assert len(paths) == binding.count
        for p in paths:
            assert isinstance(p, str)
            assert p.startswith("/World/articulation"), (
                f"Expected articulation root path under /World/articulation*, got '{p}'"
            )
    finally:
        binding.destroy()


# ---------------------------------------------------------------------------
# Error after destroy
# ---------------------------------------------------------------------------


def test_prim_paths_after_destroy_raises(physx_sdk):
    """Accessing prim_paths on a destroyed binding must raise RuntimeError."""
    _load_rb(physx_sdk)
    binding = physx_sdk.create_tensor_binding(pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
    binding.destroy()
    with pytest.raises(RuntimeError):
        _ = binding.prim_paths
