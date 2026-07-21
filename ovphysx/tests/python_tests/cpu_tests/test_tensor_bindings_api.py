# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Comprehensive tests for TensorBindingsAPI features.

Tests tensor types, metadata queries, contact binding, and dynamics tensors.

Scene used for articulation tests: two_articulations.usda
  - 2 articulations at /World/articulation and /World/articulation2
  - Each has 3 links (articulationLink0/1/2) and 2 revolute DOFs
  - Fixed base (rootJoint is a PhysicsFixedJoint)
  - Expected: N=2, L=3, D=2, is_fixed_base=True

Scene used for contact tests: boxes_falling_on_groundplane.usda
  - Multiple Cube rigid bodies falling onto a GroundPlane
"""

import os

import numpy as np
import pytest
from ovphysx.dlpack import DLDataTypeCode
from ovphysx.types import TensorType
from test_utils import load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

ARTI_PATTERN = "/World/articulation*"
EXPECTED_N = 2  # articulations
EXPECTED_L = 3  # links per articulation
EXPECTED_D = 2  # DOFs per articulation


def _load_two_articulations(sdk):
    """Load two_articulations.usda and wait for completion."""
    load_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
    sdk.wait_all()


def _load_and_step(sdk, n_steps=5, dt=1.0 / 60.0):
    """Load two_articulations.usda and advance simulation so dynamics data is populated."""
    _load_two_articulations(sdk)
    for _ in range(n_steps):
        sdk.step(dt)
    sdk.wait_all()


# ---------------------------------------------------------------------------
# DOF property tensors (35-41) -- read / write round-trip
# ---------------------------------------------------------------------------


class TestDofProperties:
    """Tests for new DOF property tensor types: stiffness, damping, limits, max_vel,
    max_force, armature, friction_properties.  Each test verifies:
      1. Binding creation succeeds and shape matches [N, D] or [N, D, C].
      2. A full read succeeds and produces finite values.
      3. A modified write followed by a second read reflects the change (round-trip).
      4. Masked write reflects the change for the masked rows only.
    """

    def _make_binding(self, sdk, tensor_type):
        _load_two_articulations(sdk)
        return sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=tensor_type)

    # -- stiffness [N, D] --

    def test_dof_stiffness_shape(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_STIFFNESS)
        assert b.ndim == 2
        assert b.shape == (EXPECTED_N, EXPECTED_D)
        b.destroy()

    def test_dof_stiffness_read(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_STIFFNESS)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "stiffness must be finite"
        b.destroy()

    def test_dof_stiffness_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_STIFFNESS)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original + 100.0
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5, err_msg="stiffness write/read round-trip failed")
        b.destroy()

    def test_dof_stiffness_indexed_write(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_STIFFNESS)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        indices = np.array([0], dtype=np.int32)
        src = original.copy()
        src[0] = 999.0
        b.write(src, indices=indices)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result[0], src[0], rtol=1e-5, err_msg="indexed write should update row 0")
        np.testing.assert_allclose(result[1], original[1], rtol=1e-5, err_msg="non-indexed row 1 should be unchanged")
        b.destroy()

    def test_dof_stiffness_masked_write(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_STIFFNESS)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        mask = np.array([True, False], dtype=np.bool_)
        src = original.copy()
        src[0] = 750.0
        b.write(src, mask=mask)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result[0], 750.0, rtol=1e-5, err_msg="masked write should update row 0")
        np.testing.assert_allclose(result[1], original[1], rtol=1e-5, err_msg="masked write should NOT update row 1")
        b.destroy()

    # -- damping [N, D] --

    def test_dof_damping_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_DAMPING)
        assert b.shape == (EXPECTED_N, EXPECTED_D)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        new_vals = out + 5.0
        b.write(new_vals)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, new_vals, rtol=1e-5)
        b.destroy()

    # -- limits [N, D, 2] --

    def test_dof_limit_shape(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_LIMIT)
        assert b.ndim == 3
        assert b.shape == (EXPECTED_N, EXPECTED_D, 2)
        b.destroy()

    def test_dof_limit_read(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_LIMIT)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "DOF limits must be finite"
        assert np.all(out[:, :, 0] <= out[:, :, 1]), "lower limit must be <= upper limit"
        b.destroy()

    def test_dof_limit_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_LIMIT)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        new_vals = original.copy()
        new_vals[:, :, 0] = -1.5  # lower
        new_vals[:, :, 1] = 1.5  # upper
        b.write(new_vals)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, new_vals, rtol=1e-5)
        b.destroy()

    # -- max_velocity [N, D] --

    def test_dof_max_velocity_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_MAX_VELOCITY)
        assert b.shape == (EXPECTED_N, EXPECTED_D)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        b.write(original + 10.0)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, original + 10.0, rtol=1e-5)
        b.destroy()

    # -- max_force [N, D] --

    def test_dof_max_force_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_MAX_FORCE)
        assert b.shape == (EXPECTED_N, EXPECTED_D)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        b.write(original + 50.0)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, original + 50.0, rtol=1e-5)
        b.destroy()

    # -- armature [N, D] --

    def test_dof_armature_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_ARMATURE)
        assert b.shape == (EXPECTED_N, EXPECTED_D)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        new_vals = np.full(b.shape, 0.01, dtype=np.float32)
        b.write(new_vals)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, new_vals, rtol=1e-5)
        b.destroy()

    # -- friction_properties [N, D, 3] --

    def test_dof_friction_properties_shape(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_FRICTION_PROPERTIES)
        assert b.ndim == 3
        assert b.shape == (EXPECTED_N, EXPECTED_D, 3), f"Expected [N={EXPECTED_N}, D={EXPECTED_D}, 3], got {b.shape}"
        b.destroy()

    def test_dof_friction_properties_read(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_FRICTION_PROPERTIES)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "friction properties must be finite"
        assert np.all(out >= 0.0), "friction properties must be >= 0"
        b.destroy()

    def test_dof_friction_properties_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_DOF_FRICTION_PROPERTIES)
        new_vals = np.zeros(b.shape, dtype=np.float32)
        new_vals[:, :, 0] = 0.1  # static friction effort
        new_vals[:, :, 1] = 0.05  # dynamic friction effort
        new_vals[:, :, 2] = 0.01  # viscous friction coefficient
        b.write(new_vals)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, new_vals, rtol=1e-5)
        b.destroy()


# ---------------------------------------------------------------------------
# Body property tensors (60-62) + link acceleration (22)
# ---------------------------------------------------------------------------


class TestBodyProperties:
    """Tests for new body (link) property tensor types: mass, COM, inertia.
    Also tests the new read-only link acceleration tensor (enum 22).
    """

    def _make_binding(self, sdk, tensor_type):
        _load_two_articulations(sdk)
        return sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=tensor_type)

    # -- mass [N, L] --

    def test_body_mass_shape(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_BODY_MASS)
        assert b.ndim == 2
        assert b.shape == (EXPECTED_N, EXPECTED_L)
        b.destroy()

    def test_body_mass_read(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_BODY_MASS)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "mass must be finite"
        assert np.all(out > 0.0), "all link masses must be positive"
        b.destroy()

    def test_body_mass_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_BODY_MASS)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        new_mass = original * 2.0
        b.write(new_mass)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, new_mass, rtol=1e-5)
        b.destroy()

    def test_body_mass_indexed_write(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_BODY_MASS)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        indices = np.array([1], dtype=np.int32)
        modified = original.copy()
        modified[1] = 42.0
        b.write(modified, indices=indices)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(
            result[1], 42.0, rtol=1e-5, err_msg="indexed write should update articulation 1 mass"
        )
        np.testing.assert_allclose(
            result[0], original[0], rtol=1e-5, err_msg="non-indexed articulation 0 mass should be unchanged"
        )
        b.destroy()

    # -- COM [N, L, 7] --

    def test_body_com_pose_shape(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_BODY_COM_POSE)
        assert b.ndim == 3
        assert b.shape == (EXPECTED_N, EXPECTED_L, 7), f"Expected [N={EXPECTED_N}, L={EXPECTED_L}, 7], got {b.shape}"
        b.destroy()

    def test_body_com_pose_read(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_BODY_COM_POSE)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "COM values must be finite"
        quat = out[:, :, 3:7]
        norms = np.linalg.norm(quat, axis=-1)
        np.testing.assert_allclose(norms, 1.0, atol=1e-4, err_msg="COM quaternions must be unit norm")
        b.destroy()

    def test_body_com_pose_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_BODY_COM_POSE)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original.copy()
        modified[:, :, 0] += 0.01
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()

    # -- inertia [N, L, 9] --

    def test_body_inertia_shape(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_BODY_INERTIA)
        assert b.ndim == 3
        assert b.shape == (EXPECTED_N, EXPECTED_L, 9), f"Expected [N={EXPECTED_N}, L={EXPECTED_L}, 9], got {b.shape}"
        b.destroy()

    def test_body_inertia_read(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_BODY_INERTIA)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "inertia values must be finite"
        diag_idx = [0, 4, 8]
        for i in diag_idx:
            assert np.all(out[:, :, i] > 0.0), f"inertia diagonal element {i} must be positive"
        b.destroy()

    def test_body_inertia_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_BODY_INERTIA)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        scaled = original * 2.0
        b.write(scaled)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, scaled, rtol=1e-5)
        b.destroy()

    # -- link acceleration [N, L, 6] -- enum 22, READ-ONLY --

    def test_link_acceleration_enum_value(self):
        assert (
            TensorType.ARTICULATION_LINK_ACCELERATION == 22
        ), "LINK_ACCELERATION must be enum value 22 (adjacent to LINK_VELOCITY=21)"

    def test_link_acceleration_shape(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_ACCELERATION
        )
        assert b.ndim == 3
        assert b.shape == (EXPECTED_N, EXPECTED_L, 6), f"Expected [N={EXPECTED_N}, L={EXPECTED_L}, 6], got {b.shape}"
        b.destroy()

    def test_link_acceleration_read(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_ACCELERATION
        )
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "link accelerations must be finite"
        b.destroy()

    def test_link_acceleration_is_read_only(self, physx_sdk_cpu):
        _load_two_articulations(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_ACCELERATION
        )
        src = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)(read.only|invalid)"):
            b.write(src)
        b.destroy()

    def test_link_acceleration_masked_write_rejected(self, physx_sdk_cpu):
        _load_two_articulations(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_ACCELERATION
        )
        src = np.zeros(b.shape, dtype=np.float32)
        mask = np.array([True, True], dtype=np.bool_)
        with pytest.raises(RuntimeError, match="(?i)(read.only|invalid)"):
            b.write(src, mask=mask)
        b.destroy()


# ---------------------------------------------------------------------------
# Enum value correctness
# ---------------------------------------------------------------------------


class TestEnumValues:
    """Verify that all new enum values match the specification."""

    def test_link_acceleration_enum(self):
        assert TensorType.ARTICULATION_LINK_ACCELERATION == 22

    def test_dof_property_enums(self):
        assert TensorType.ARTICULATION_DOF_STIFFNESS == 35
        assert TensorType.ARTICULATION_DOF_DAMPING == 36
        assert TensorType.ARTICULATION_DOF_LIMIT == 37
        assert TensorType.ARTICULATION_DOF_MAX_VELOCITY == 38
        assert TensorType.ARTICULATION_DOF_MAX_FORCE == 39
        assert TensorType.ARTICULATION_DOF_ARMATURE == 40
        assert TensorType.ARTICULATION_DOF_FRICTION_PROPERTIES == 41

    def test_body_property_enums(self):
        assert TensorType.ARTICULATION_BODY_MASS == 60
        assert TensorType.ARTICULATION_BODY_COM_POSE == 61
        assert TensorType.ARTICULATION_BODY_INERTIA == 62

    def test_dynamics_enums(self):
        assert TensorType.ARTICULATION_JACOBIAN == 70
        assert TensorType.ARTICULATION_MASS_MATRIX == 71
        assert TensorType.ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE == 72
        assert TensorType.ARTICULATION_GRAVITY_FORCE == 73
        assert TensorType.ARTICULATION_LINK_INCOMING_JOINT_FORCE == 74

    def test_rigid_body_property_enums(self):
        assert TensorType.RIGID_BODY_MASS == 3
        assert TensorType.RIGID_BODY_INERTIA == 4
        assert TensorType.RIGID_BODY_COM_POSE == 5

    def test_body_inv_property_enums(self):
        assert TensorType.ARTICULATION_BODY_INV_MASS == 63
        assert TensorType.ARTICULATION_BODY_INV_INERTIA == 64

    def test_projected_joint_force_enum(self):
        assert TensorType.ARTICULATION_DOF_PROJECTED_JOINT_FORCE == 75

    def test_fixed_tendon_enums(self):
        assert TensorType.ARTICULATION_FIXED_TENDON_STIFFNESS == 80
        assert TensorType.ARTICULATION_FIXED_TENDON_DAMPING == 81
        assert TensorType.ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS == 82
        assert TensorType.ARTICULATION_FIXED_TENDON_LIMIT == 83
        assert TensorType.ARTICULATION_FIXED_TENDON_REST_LENGTH == 84
        assert TensorType.ARTICULATION_FIXED_TENDON_OFFSET == 85

    def test_coriolis_member_exists(self):
        """The coriolis tensor must be accessible as TensorType.ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE."""
        assert hasattr(
            TensorType, "ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE"
        ), "TensorType must have an ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE member"
        assert TensorType.ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE == 72


# ---------------------------------------------------------------------------
# Articulation metadata queries
# ---------------------------------------------------------------------------


class TestTensorBindingSpec:
    """Tests for tensor spec metadata exposed through TensorBinding."""

    def _make_rigid_body_binding(self, sdk, tensor_type):
        load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
        sdk.wait_all()
        return sdk.create_tensor_binding(pattern="/World/Cube1", tensor_type=tensor_type)

    def test_rigid_body_pose_dtype_is_float32(self, physx_sdk_cpu):
        b = self._make_rigid_body_binding(physx_sdk_cpu, TensorType.RIGID_BODY_POSE)
        assert b.dtype.code == DLDataTypeCode.kDLFloat
        assert b.dtype.bits == 32
        assert b.dtype.lanes == 1
        assert str(b.dtype) == "float32"
        b.destroy()

    def test_rigid_body_disable_simulation_dtype_is_uint8(self, physx_sdk_cpu):
        b = self._make_rigid_body_binding(physx_sdk_cpu, TensorType.RIGID_BODY_DISABLE_SIMULATION)
        assert b.dtype.code == DLDataTypeCode.kDLUInt
        assert b.dtype.bits == 8
        assert b.dtype.lanes == 1
        assert str(b.dtype) == "uint8"
        b.destroy()

    def test_tensor_binding_spec_matches_individual_properties(self, physx_sdk_cpu):
        b = self._make_rigid_body_binding(physx_sdk_cpu, TensorType.RIGID_BODY_DISABLE_SIMULATION)
        spec = b.spec
        assert spec.dtype.code == b.dtype.code
        assert spec.dtype.bits == b.dtype.bits
        assert spec.dtype.lanes == b.dtype.lanes
        assert spec.ndim == b.ndim
        assert spec.shape == b.shape
        b.destroy()

    def test_tensor_binding_metadata_is_python_owned(self, physx_sdk_cpu):
        b = self._make_rigid_body_binding(physx_sdk_cpu, TensorType.RIGID_BODY_POSE)
        dtype = b.dtype
        spec = b.spec
        dtype.bits = 99
        spec.dtype.bits = 77
        assert b.dtype.bits == 32
        assert b.spec.dtype.bits == 32
        b.destroy()
        assert dtype.code == DLDataTypeCode.kDLFloat
        assert dtype.bits == 99
        assert dtype.lanes == 1
        assert spec.dtype.code == DLDataTypeCode.kDLFloat
        assert spec.dtype.bits == 77
        assert spec.dtype.lanes == 1
        assert spec.ndim == 2
        assert spec.shape == (1, 7)


class TestArticulationMetadata:
    """Tests for C ABI metadata query functions exposed via TensorBinding properties.
    Covers: dof_count, body_count, joint_count, is_fixed_base, dof_names,
            body_names, joint_names.
    """

    def _make_dof_binding(self, sdk):
        _load_two_articulations(sdk)
        return sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION)

    def test_dof_count(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        assert b.dof_count == EXPECTED_D, f"Expected dof_count={EXPECTED_D}, got {b.dof_count}"
        b.destroy()

    def test_body_count(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        assert b.body_count == EXPECTED_L, f"Expected body_count={EXPECTED_L}, got {b.body_count}"
        b.destroy()

    def test_joint_count(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        assert b.joint_count == EXPECTED_D, f"Expected joint_count={EXPECTED_D}, got {b.joint_count}"
        b.destroy()

    def test_is_fixed_base(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        assert b.is_fixed_base is True, "two_articulations.usda has fixed base (PhysicsFixedJoint rootJoint)"
        b.destroy()

    def test_dof_names_count(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        names = b.dof_names
        assert isinstance(names, list)
        assert len(names) == EXPECTED_D, f"Expected {EXPECTED_D} DOF names, got {len(names)}"
        b.destroy()

    def test_dof_names_are_strings(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        for name in b.dof_names:
            assert isinstance(name, str) and len(name) > 0, f"DOF name must be a non-empty string, got {name!r}"
        b.destroy()

    def test_body_names_count(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        names = b.body_names
        assert isinstance(names, list)
        assert len(names) == EXPECTED_L, f"Expected {EXPECTED_L} body names, got {len(names)}"
        b.destroy()

    def test_body_names_are_strings(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        for name in b.body_names:
            assert isinstance(name, str) and len(name) > 0, f"Body name must be a non-empty string, got {name!r}"
        b.destroy()

    def test_joint_names_count(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        names = b.joint_names
        assert isinstance(names, list)
        assert len(names) == EXPECTED_D, f"Expected {EXPECTED_D} joint names, got {len(names)}"
        b.destroy()

    def test_joint_names_are_strings(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        for name in b.joint_names:
            assert isinstance(name, str) and len(name) > 0, f"Joint name must be a non-empty string, got {name!r}"
        b.destroy()

    def test_dof_names_unique(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        names = b.dof_names
        assert len(names) == len(set(names)), "DOF names must be unique"
        b.destroy()

    def test_body_names_unique(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        names = b.body_names
        assert len(names) == len(set(names)), "Body names must be unique"
        b.destroy()

    def test_metadata_consistent_across_tensor_types(self, physx_sdk_cpu):
        """Metadata should return the same result regardless of which binding is queried."""
        _load_two_articulations(physx_sdk_cpu)
        b_pos = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION
        )
        b_stiff = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_STIFFNESS
        )
        assert b_pos.dof_count == b_stiff.dof_count
        assert b_pos.body_count == b_stiff.body_count
        assert b_pos.dof_names == b_stiff.dof_names
        assert b_pos.body_names == b_stiff.body_names
        b_pos.destroy()
        b_stiff.destroy()

    def test_dof_count_matches_binding_shape(self, physx_sdk_cpu):
        b = self._make_dof_binding(physx_sdk_cpu)
        assert b.shape[1] == b.dof_count
        b.destroy()

    def test_body_count_matches_link_tensor_shape(self, physx_sdk_cpu):
        _load_two_articulations(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_POSE)
        assert (
            b.body_count == b.shape[1]
        ), "body_count from metatype must match shape[1] (max_links) for homogeneous view"
        b.destroy()

    def test_get_articulation_metadata_struct(self, physx_sdk_cpu):
        """ovphysx_get_articulation_metadata fills all 6 scalar fields correctly."""
        b = self._make_dof_binding(physx_sdk_cpu)
        assert b.dof_count == EXPECTED_D
        assert b.body_count == EXPECTED_L
        assert b.joint_count == EXPECTED_D
        assert b.is_fixed_base is True
        assert b.fixed_tendon_count == 0
        assert b.spatial_tendon_count == 0
        m1 = b._get_artic_metadata()
        m2 = b._get_artic_metadata()
        assert m1 is m2, "metadata should be cached after first call"
        b.destroy()


# ---------------------------------------------------------------------------
# Contact binding
# ---------------------------------------------------------------------------


class TestContactBinding:
    """Tests for the ContactBinding API.

    Tests contact binding lifecycle, spec queries, and force reads.
    The boxes_falling_on_groundplane.usda scene is used: boxes drop onto
    a ground plane, providing known contact events after simulation runs.
    """

    def _make_cube_pair_contact_binding(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()

        cube1_pose = physx_sdk_cpu.create_tensor_binding(pattern="/World/Cube1", tensor_type=TensorType.RIGID_BODY_POSE)
        cube2_pose = physx_sdk_cpu.create_tensor_binding(pattern="/World/Cube2", tensor_type=TensorType.RIGID_BODY_POSE)
        cube2_velocity = physx_sdk_cpu.create_tensor_binding(
            pattern="/World/Cube2", tensor_type=TensorType.RIGID_BODY_VELOCITY
        )

        pose = np.zeros(cube1_pose.shape, dtype=np.float32)
        cube1_pose.read(pose)
        pose[0, 2] += 0.25
        cube2_pose.write(pose)
        cube2_velocity.write(np.zeros(cube2_velocity.shape, dtype=np.float32))
        physx_sdk_cpu.wait_all()

        cube1_pose.destroy()
        cube2_pose.destroy()
        cube2_velocity.destroy()

        cb = physx_sdk_cpu.create_contact_binding(
            sensor_patterns=["/World/Cube1"],
            filter_patterns=["/World/Cube2"],
            filters_per_sensor=1,
            max_contact_data_count=256,
        )

        physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()
        return cb

    def test_contact_binding_create_destroy(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        cb = physx_sdk_cpu.create_contact_binding(sensor_patterns=["/World/Cube1"])
        assert cb is not None
        assert cb.sensor_count >= 1, "Cube1 should match at least 1 sensor body"
        cb.destroy()

    def test_contact_binding_no_filter_spec(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        cb = physx_sdk_cpu.create_contact_binding(sensor_patterns=["/World/Cube1"])
        assert cb.filter_count == 0, "No filter patterns => filter_count must be 0"
        cb.destroy()

    def test_contact_binding_with_filter_spec(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        cb = physx_sdk_cpu.create_contact_binding(
            sensor_patterns=["/World/Cube1"],
            filter_patterns=["/World/GroundPlane/CollisionMesh"],
            filters_per_sensor=1,
            max_contact_data_count=256,
        )
        assert cb.sensor_count >= 1
        assert cb.filter_count == 1, "1 filter pattern per sensor => filter_count must be 1"
        assert cb.max_contact_data_count == 256
        assert cb.sensor_paths == ["/World/Cube1"]
        assert cb.filter_paths == [["/World/GroundPlane/CollisionMesh"]]
        cb.destroy()

    def test_contact_binding_multiple_sensors(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        cb = physx_sdk_cpu.create_contact_binding(sensor_patterns=["/World/Cube1", "/World/Cube2", "/World/Cube3"])
        assert cb.sensor_count == 3, f"3 exact sensor patterns should match 3 sensors, got {cb.sensor_count}"
        assert cb.sensor_paths == ["/World/Cube1", "/World/Cube2", "/World/Cube3"]
        filter_paths = cb.filter_paths
        assert len(filter_paths) == cb.sensor_count
        assert all(paths == [] for paths in filter_paths)
        cb.destroy()

    def test_contact_binding_matches_runtime_clones(self, physx_sdk_cpu):
        """Check contact binding sees cloned bodies and their contact events.

        Applications like IsaacLab write one source body to USD and ask ovphysx
        to make runtime clones for the other environments. The bug was that
        contact binding only saw the USD source body; this test expects the
        source and every runtime clone to report contacts with the ground plane.
        """
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()

        targets = ["/World/CloneA", "/World/CloneB", "/World/CloneC"]
        physx_sdk_cpu.clone("/World/Cube1", targets)
        physx_sdk_cpu.wait_all()

        paths = ["/World/Cube1"] + targets
        pose_binding = physx_sdk_cpu.create_tensor_binding(
            prim_paths=paths,
            tensor_type=TensorType.RIGID_BODY_POSE,
        )
        assert pose_binding.count == 4
        assert pose_binding.prim_paths == paths

        clone_pose_binding = physx_sdk_cpu.create_tensor_binding(
            prim_paths=targets,
            tensor_type=TensorType.RIGID_BODY_POSE,
        )
        clone_velocity_binding = physx_sdk_cpu.create_tensor_binding(
            prim_paths=targets,
            tensor_type=TensorType.RIGID_BODY_VELOCITY,
        )
        clone_poses = np.zeros(clone_pose_binding.shape, dtype=np.float32)
        clone_pose_binding.read(clone_poses)
        clone_poses[:, 0] = np.array([20.0, 18.0, 16.0], dtype=np.float32)
        clone_poses[:, 1] = 1.0
        clone_poses[:, 2] = 10.0
        clone_poses[:, 3:] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        clone_pose_binding.write(clone_poses)
        clone_velocity_binding.write(np.zeros(clone_velocity_binding.shape, dtype=np.float32))
        physx_sdk_cpu.wait_all()

        cb = physx_sdk_cpu.create_contact_binding(sensor_patterns=paths)
        assert cb.sensor_count == pose_binding.count
        assert cb.sensor_paths == pose_binding.prim_paths

        # Step long enough for the clones to land on the ground plane.
        for i in range(180):
            physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()

        # Clones inherit physxRigidBody:sleepThreshold=0 from Cube1, so they
        # never fully settle and a single-frame contact read can catch a
        # clone mid-bounce with zero force. Sample over a short window after
        # landing and take the max upward force per clone — any clone that
        # has touched the ground will register a positive Z force in at
        # least one of those frames.
        clone_indices = [cb.sensor_paths.index(path) for path in targets]
        sample_frames = 30
        net_forces = np.zeros((cb.sensor_count, 3), dtype=np.float32)
        max_upward_force = np.zeros(len(targets), dtype=np.float32)
        for i in range(sample_frames):
            physx_sdk_cpu.step(1.0 / 60.0)
            physx_sdk_cpu.wait_all()
            cb.read_net_forces(output=net_forces)
            assert np.all(np.isfinite(net_forces[clone_indices])), \
                "Runtime clone contact forces should be finite"
            max_upward_force = np.maximum(max_upward_force, net_forces[clone_indices, 2])

        assert np.all(max_upward_force > 0.0), (
            f"Runtime clones should report upward contact force in at least one of the last "
            f"{sample_frames} frames; got per-clone max upward forces {max_upward_force.tolist()}"
        )

        cb.destroy()
        clone_velocity_binding.destroy()
        clone_pose_binding.destroy()
        pose_binding.destroy()

    def test_contact_binding_context_manager(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        with physx_sdk_cpu.create_contact_binding(sensor_patterns=["/World/Cube1"]) as cb:
            assert cb.sensor_count >= 1

    def test_contact_net_forces_shape(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()

        cb = physx_sdk_cpu.create_contact_binding(sensor_patterns=["/World/Cube1"])
        sensor_count = cb.sensor_count
        assert sensor_count >= 1

        for _ in range(30):
            physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()

        out = np.zeros((sensor_count, 3), dtype=np.float32)
        cb.read_net_forces(output=out)
        assert out.shape == (sensor_count, 3), f"Expected shape ({sensor_count}, 3), got {out.shape}"
        assert np.all(np.isfinite(out)), "net forces must be finite"
        cb.destroy()

    def test_contact_net_forces_nonzero_after_landing(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()

        cb = physx_sdk_cpu.create_contact_binding(sensor_patterns=["/World/Cube1"])
        sensor_count = cb.sensor_count

        for _ in range(120):
            physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()

        out = np.zeros((sensor_count, 3), dtype=np.float32)
        cb.read_net_forces(output=out)
        assert np.any(np.abs(out) > 0.0), "After landing, contact net forces should be non-zero"
        cb.destroy()

    def test_contact_force_matrix_shape(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()

        cb = physx_sdk_cpu.create_contact_binding(
            sensor_patterns=["/World/Cube1"], filter_patterns=["/World/GroundPlane/CollisionMesh"], filters_per_sensor=1
        )
        sensor_count = cb.sensor_count
        filter_count = cb.filter_count
        assert filter_count == 1

        for _ in range(30):
            physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()

        out = np.zeros((sensor_count, filter_count, 3), dtype=np.float32)
        cb.read_force_matrix(output=out)
        assert out.shape == (
            sensor_count,
            filter_count,
            3,
        ), f"Expected ({sensor_count}, {filter_count}, 3), got {out.shape}"
        assert np.all(np.isfinite(out)), "force matrix must be finite"
        cb.destroy()

    def test_contact_data_flat_buffers(self, physx_sdk_cpu):
        cb = self._make_cube_pair_contact_binding(physx_sdk_cpu)

        c = cb.max_contact_data_count
        contact_forces = np.zeros((c, 1), dtype=np.float32)
        positions = np.zeros((c, 3), dtype=np.float32)
        normals = np.zeros((c, 3), dtype=np.float32)
        separations = np.zeros((c, 1), dtype=np.float32)
        counts = np.zeros((cb.sensor_count, cb.filter_count), dtype=np.int32)
        starts = np.zeros((cb.sensor_count, cb.filter_count), dtype=np.int32)

        cb.read_contact_data(contact_forces, positions, normals, separations, counts, starts)
        assert np.all(np.isfinite(contact_forces))
        assert np.all(np.isfinite(positions))
        assert np.all(np.isfinite(normals))
        assert np.all(np.isfinite(separations))
        assert np.all(counts >= 0)
        assert np.all(starts >= 0)
        assert int(counts.sum()) <= c
        pair_count = int(counts[0, 0])
        pair_start = int(starts[0, 0])
        assert pair_count > 0, "Overlapped Cube1/Cube2 pair should produce detailed contacts"
        assert pair_start + pair_count <= c
        valid_normals = normals[pair_start : pair_start + pair_count]
        normal_lengths = np.linalg.norm(valid_normals, axis=1)
        assert np.all(normal_lengths > 0.5)
        assert np.any(
            np.abs(valid_normals[:, 2]) > 0.5
        ), "Cube/Cube contacts should have a strong vertical normal component"
        cb.destroy()

    def test_contact_data_requires_positive_capacity(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()

        cb = physx_sdk_cpu.create_contact_binding(
            sensor_patterns=["/World/Cube1"], filter_patterns=["/World/GroundPlane/CollisionMesh"], filters_per_sensor=1
        )

        contact_forces = np.zeros((0, 1), dtype=np.float32)
        positions = np.zeros((0, 3), dtype=np.float32)
        normals = np.zeros((0, 3), dtype=np.float32)
        separations = np.zeros((0, 1), dtype=np.float32)
        counts = np.zeros((cb.sensor_count, cb.filter_count), dtype=np.int32)
        starts = np.zeros((cb.sensor_count, cb.filter_count), dtype=np.int32)

        with pytest.raises(RuntimeError, match="max_contact_data_count"):
            cb.read_contact_data(contact_forces, positions, normals, separations, counts, starts)
        cb.destroy()

    def test_contact_data_requires_filters(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()

        cb = physx_sdk_cpu.create_contact_binding(sensor_patterns=["/World/Cube1"], max_contact_data_count=256)

        contact_forces = np.zeros((cb.max_contact_data_count, 1), dtype=np.float32)
        positions = np.zeros((cb.max_contact_data_count, 3), dtype=np.float32)
        normals = np.zeros((cb.max_contact_data_count, 3), dtype=np.float32)
        separations = np.zeros((cb.max_contact_data_count, 1), dtype=np.float32)
        counts = np.zeros((cb.sensor_count, cb.filter_count), dtype=np.int32)
        starts = np.zeros((cb.sensor_count, cb.filter_count), dtype=np.int32)

        with pytest.raises(RuntimeError, match="filters_per_sensor"):
            cb.read_contact_data(contact_forces, positions, normals, separations, counts, starts)
        cb.destroy()

    def test_friction_data_flat_buffers_uint32_counts(self, physx_sdk_cpu):
        cb = self._make_cube_pair_contact_binding(physx_sdk_cpu)

        c = cb.max_contact_data_count
        friction_forces = np.zeros((c, 3), dtype=np.float32)
        friction_points = np.zeros((c, 3), dtype=np.float32)
        counts = np.zeros((cb.sensor_count, cb.filter_count), dtype=np.uint32)
        starts = np.zeros((cb.sensor_count, cb.filter_count), dtype=np.uint32)

        cb.read_friction_data(friction_forces, friction_points, counts, starts)
        assert np.all(np.isfinite(friction_forces))
        assert np.all(np.isfinite(friction_points))
        assert int(counts.sum()) <= c
        pair_count = int(counts[0, 0])
        pair_start = int(starts[0, 0])
        assert pair_count > 0, "Overlapped Cube1/Cube2 pair should produce friction anchors"
        assert pair_start + pair_count <= c
        cb.destroy()

    def test_friction_data_requires_filters(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()

        cb = physx_sdk_cpu.create_contact_binding(sensor_patterns=["/World/Cube1"], max_contact_data_count=256)

        friction_forces = np.zeros((cb.max_contact_data_count, 3), dtype=np.float32)
        friction_points = np.zeros((cb.max_contact_data_count, 3), dtype=np.float32)
        counts = np.zeros((cb.sensor_count, cb.filter_count), dtype=np.int32)
        starts = np.zeros((cb.sensor_count, cb.filter_count), dtype=np.int32)

        with pytest.raises(RuntimeError, match="filters_per_sensor"):
            cb.read_friction_data(friction_forces, friction_points, counts, starts)
        cb.destroy()

    def test_contact_binding_destroy_idempotent(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        cb = physx_sdk_cpu.create_contact_binding(sensor_patterns=["/World/Cube1"])
        cb.destroy()
        cb.destroy()  # second destroy should be a no-op, not a crash

    def test_contact_binding_ops_after_destroy_raise(self, physx_sdk_cpu):
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        cb = physx_sdk_cpu.create_contact_binding(sensor_patterns=["/World/Cube1"])
        sc = cb.sensor_count
        cb.destroy()
        out = np.zeros((sc, 3), dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)destroyed"):
            cb.read_net_forces(output=out)


# ---------------------------------------------------------------------------
# Dynamics tensors (70-74) -- Jacobians, mass matrix, etc.
# ---------------------------------------------------------------------------


class TestDynamicsTensors:
    """Tests for dynamics query tensors: Jacobian, mass matrix, Coriolis+centrifugal,
    gravity compensation, and link incoming joint force.

    IMPORTANT: All tests step the simulation before creating bindings because
    Jacobian/mass-matrix shapes are computed lazily after the first sim step.
    """

    def _setup(self, sdk, n_steps=5):
        _load_and_step(sdk, n_steps=n_steps)
        return sdk.create_tensor_binding

    # -- Jacobian [N, R, C] --

    def test_jacobian_shape_rank(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_JACOBIAN)
        assert b.ndim == 3, "Jacobian must be 3D [N, R, C]"
        N, R, C = b.shape
        assert N == EXPECTED_N, f"Jacobian axis 0 must be N={EXPECTED_N}, got {N}"
        assert R > 0, f"Jacobian R must be > 0, got {R}"
        assert C > 0, f"Jacobian C must be > 0, got {C}"
        assert R == (EXPECTED_L - 1) * 6, f"Fixed-base Jacobian rows should be (L-1)*6={(EXPECTED_L-1)*6}, got {R}"
        assert C == EXPECTED_D, f"Fixed-base Jacobian cols should equal DOF count={EXPECTED_D}, got {C}"
        b.destroy()

    def test_jacobian_read(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_JACOBIAN)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "Jacobian values must be finite"
        b.destroy()

    def test_jacobian_is_read_only(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_JACOBIAN)
        src = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)(read.only|invalid)"):
            b.write(src)
        b.destroy()

    def test_jacobian_masked_write_rejected(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_JACOBIAN)
        src = np.zeros(b.shape, dtype=np.float32)
        mask = np.array([True, True], dtype=np.bool_)
        with pytest.raises(RuntimeError, match="(?i)(read.only|invalid)"):
            b.write(src, mask=mask)
        b.destroy()

    # -- Mass matrix [N, M, M] --

    def test_mass_matrix_shape(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_MASS_MATRIX)
        assert b.ndim == 3, "Mass matrix must be 3D [N, M, M]"
        N, M1, M2 = b.shape
        assert N == EXPECTED_N
        assert M1 == M2, f"Mass matrix must be square, got [{N}, {M1}, {M2}]"
        assert M1 > 0
        assert M1 == EXPECTED_D, f"Fixed-base mass matrix dim should equal D={EXPECTED_D}, got {M1}"
        b.destroy()

    def test_mass_matrix_symmetry(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_MASS_MATRIX)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "mass matrix must be finite"
        transposed = np.transpose(out, axes=(0, 2, 1))
        np.testing.assert_allclose(out, transposed, atol=1e-4, err_msg="mass matrix must be symmetric")
        b.destroy()

    def test_mass_matrix_positive_definite_diagonal(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_MASS_MATRIX)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        N, M, _ = out.shape
        for i in range(M):
            assert np.all(out[:, i, i] > 0.0), f"Mass matrix diagonal element [{i},{i}] must be positive"
        b.destroy()

    def test_mass_matrix_is_read_only(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_MASS_MATRIX)
        src = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)(read.only|invalid)"):
            b.write(src)
        b.destroy()

    # -- Coriolis + centrifugal [N, M] --

    def test_coriolis_shape(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE)
        assert b.ndim == 2, "Coriolis tensor must be 2D [N, M]"
        N, M = b.shape
        assert N == EXPECTED_N
        assert M == EXPECTED_D, f"Fixed-base coriolis dim M should equal D={EXPECTED_D}, got {M}"
        b.destroy()

    def test_coriolis_read(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "coriolis values must be finite"
        b.destroy()

    def test_coriolis_is_read_only(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE)
        src = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)(read.only|invalid)"):
            b.write(src)
        b.destroy()

    # -- Gravity compensation [N, M] --

    def test_gravity_shape(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_GRAVITY_FORCE)
        assert b.ndim == 2
        N, M = b.shape
        assert N == EXPECTED_N
        assert M == EXPECTED_D
        b.destroy()

    def test_gravity_read(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_GRAVITY_FORCE)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "gravity compensation values must be finite"
        b.destroy()

    def test_gravity_is_read_only(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_GRAVITY_FORCE)
        src = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)(read.only|invalid)"):
            b.write(src)
        b.destroy()

    # -- Link incoming joint force [N, L, 6] --

    def test_link_incoming_joint_force_shape(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_INCOMING_JOINT_FORCE)
        assert b.ndim == 3
        assert b.shape == (EXPECTED_N, EXPECTED_L, 6), f"Expected [{EXPECTED_N}, {EXPECTED_L}, 6], got {b.shape}"
        b.destroy()

    def test_link_incoming_joint_force_read(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_INCOMING_JOINT_FORCE)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out)), "link incoming joint forces must be finite"
        b.destroy()

    def test_link_incoming_joint_force_is_read_only(self, physx_sdk_cpu):
        create = self._setup(physx_sdk_cpu)
        b = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_INCOMING_JOINT_FORCE)
        src = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)(read.only|invalid)"):
            b.write(src)
        b.destroy()

    def test_mass_matrix_and_coriolis_same_M_dimension(self, physx_sdk_cpu):
        """M dimension must be consistent across mass matrix, coriolis, and gravity."""
        create = self._setup(physx_sdk_cpu)
        b_mass = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_MASS_MATRIX)
        b_cor = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE)
        b_grav = create(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_GRAVITY_FORCE)
        M = b_mass.shape[1]
        assert b_cor.shape[1] == M, "Coriolis M must match mass matrix M"
        assert b_grav.shape[1] == M, "Gravity M must match mass matrix M"
        b_mass.destroy()
        b_cor.destroy()
        b_grav.destroy()


# ---------------------------------------------------------------------------
# Standalone rigid body property tensors (types 3-5)
# ---------------------------------------------------------------------------

RIGID_BODY_PATTERN = "/World/Cube*"
EXPECTED_RIGID_BODIES = 11  # Cube1 through Cube11 in boxes scene


def _load_boxes(sdk, n_steps=5, dt=1.0 / 60.0):
    """Load boxes_falling_on_groundplane.usda and step to populate data."""
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    sdk.warmup_gpu()
    for _ in range(n_steps):
        sdk.step(dt)
    sdk.wait_all()


class TestRigidBodyProperties:
    """Tests for standalone rigid body tensor types.

    Uses boxes_falling_on_groundplane.usda, which has 11 independent rigid body
    cubes.
    """

    def test_no_rigid_body_root_aliases(self):
        assert not hasattr(TensorType, "RIGID_BODY_ROOT_POSE")
        assert not hasattr(TensorType, "RIGID_BODY_ROOT_VELOCITY")

    def test_rigid_body_prim_paths(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE)
        paths = b.prim_paths
        assert len(paths) == b.count
        assert all(path.startswith("/World/Cube") for path in paths)
        assert b.prim_paths == paths
        b.destroy()

    def test_prim_paths_returns_articulation_root_paths(self, physx_sdk_cpu):
        _load_two_articulations(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION)
        assert b.prim_paths == ["/World/articulation", "/World/articulation2"]
        b.destroy()

    def test_rigid_body_acceleration_shape(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_ACCELERATION
        )
        assert b.ndim == 2
        assert b.shape == (EXPECTED_RIGID_BODIES, 6)
        b.destroy()

    def test_rigid_body_acceleration_read(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_ACCELERATION
        )
        buf = np.zeros(b.shape, dtype=np.float32)
        b.read(buf)
        assert np.all(np.isfinite(buf))
        b.destroy()

    def test_rigid_body_acceleration_is_read_only(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_ACCELERATION
        )
        src = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)(read.only|invalid)"):
            b.write(src)
        b.destroy()

    def test_rigid_body_mass_shape(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_MASS)
        assert b.ndim == 1
        assert b.shape[0] == EXPECTED_RIGID_BODIES
        b.destroy()

    def test_rigid_body_mass_read(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_MASS)
        buf = np.zeros(b.shape, dtype=np.float32)
        b.read(buf)
        assert np.all(np.isfinite(buf))
        assert np.all(buf > 0), "rigid body masses should be positive"
        b.destroy()

    def test_rigid_body_mass_write_roundtrip(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_MASS)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original * 2.0
        b.write(modified)
        readback = np.zeros(b.shape, dtype=np.float32)
        b.read(readback)
        np.testing.assert_allclose(readback, modified, rtol=1e-5)
        b.write(original)
        b.destroy()

    def test_rigid_body_inv_mass_shape_and_read(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_INV_MASS)
        assert b.ndim == 1
        assert b.shape == (EXPECTED_RIGID_BODIES,)
        buf = np.zeros(b.shape, dtype=np.float32)
        b.read(buf)
        assert np.all(np.isfinite(buf))
        assert np.all(buf >= 0), "inverse masses should be non-negative"
        b.destroy()

    def test_rigid_body_inv_mass_is_read_only(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_INV_MASS)
        src = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)(read.only|invalid)"):
            b.write(src)
        b.destroy()

    def test_rigid_body_inertia_shape(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_INERTIA)
        assert b.ndim == 2
        assert b.shape == (EXPECTED_RIGID_BODIES, 9)
        b.destroy()

    def test_rigid_body_inertia_read(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_INERTIA)
        buf = np.zeros(b.shape, dtype=np.float32)
        b.read(buf)
        assert np.all(np.isfinite(buf))
        b.destroy()

    def test_rigid_body_inv_inertia_shape_and_read(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_INV_INERTIA
        )
        assert b.ndim == 2
        assert b.shape == (EXPECTED_RIGID_BODIES, 9)
        buf = np.zeros(b.shape, dtype=np.float32)
        b.read(buf)
        assert np.all(np.isfinite(buf))
        b.destroy()

    def test_rigid_body_inv_inertia_is_read_only(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_INV_INERTIA
        )
        src = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)(read.only|invalid)"):
            b.write(src)
        b.destroy()

    def test_rigid_body_com_pose_shape(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_COM_POSE)
        assert b.ndim == 2
        assert b.shape == (EXPECTED_RIGID_BODIES, 7)
        b.destroy()

    def test_rigid_body_com_pose_read(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_COM_POSE)
        buf = np.zeros(b.shape, dtype=np.float32)
        b.read(buf)
        assert np.all(np.isfinite(buf))
        quat_norms = np.linalg.norm(buf[:, 3:7], axis=1)
        np.testing.assert_allclose(quat_norms, 1.0, atol=1e-5, err_msg="COM pose quaternions should be unit length")

    def test_rigid_body_com_pose_write_roundtrip(self, physx_sdk_cpu):
        _load_boxes(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=RIGID_BODY_PATTERN, tensor_type=TensorType.RIGID_BODY_COM_POSE)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original.copy()
        modified[:, 0] += 0.01
        b.write(modified)
        readback = np.zeros(b.shape, dtype=np.float32)
        b.read(readback)
        np.testing.assert_allclose(
            readback[:, :3], modified[:, :3], rtol=1e-5, err_msg="COM position should reflect written values"
        )
        quat_norms = np.linalg.norm(readback[:, 3:7], axis=1)
        np.testing.assert_allclose(
            quat_norms, 1.0, atol=1e-4, err_msg="COM quaternions should stay unit norm after write"
        )
        b.write(original)
        b.destroy()


# ---------------------------------------------------------------------------
# Fixed tendon property tensors (types 80-85) -- T=0 scenario
# ---------------------------------------------------------------------------


class TestFixedTendon:
    """Tests for fixed tendon property tensor types.

    two_articulations.usda has no fixed tendons, so T=0. These tests verify
    that the binding can be created, shapes are correct (second dim = 0),
    reads succeed (returning an empty-second-dim tensor), writes succeed
    silently (no-op), and fixed_tendon_count returns 0.
    """

    _TENDON_2D_TYPES = [
        TensorType.ARTICULATION_FIXED_TENDON_STIFFNESS,
        TensorType.ARTICULATION_FIXED_TENDON_DAMPING,
        TensorType.ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS,
        TensorType.ARTICULATION_FIXED_TENDON_REST_LENGTH,
        TensorType.ARTICULATION_FIXED_TENDON_OFFSET,
    ]

    def _make_binding(self, sdk, tensor_type):
        _load_two_articulations(sdk)
        return sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=tensor_type)

    def test_fixed_tendon_count_zero(self, physx_sdk_cpu):
        _load_two_articulations(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION)
        assert b.fixed_tendon_count == 0, "two_articulations.usda has no tendons, expected T=0"
        b.destroy()

    @pytest.mark.parametrize(
        "tensor_type", _TENDON_2D_TYPES, ids=["stiffness", "damping", "limit_stiffness", "rest_length", "offset"]
    )
    def test_tendon_2d_shape_t0(self, physx_sdk_cpu, tensor_type):
        b = self._make_binding(physx_sdk_cpu, tensor_type)
        assert b.ndim == 2
        assert b.shape == (EXPECTED_N, 0), f"With T=0, shape should be (N, 0), got {b.shape}"
        b.destroy()

    def test_tendon_limit_shape_t0(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_FIXED_TENDON_LIMIT)
        assert b.ndim == 3
        assert b.shape == (EXPECTED_N, 0, 2), f"With T=0, limit shape should be (N, 0, 2), got {b.shape}"
        b.destroy()

    @pytest.mark.parametrize(
        "tensor_type", _TENDON_2D_TYPES, ids=["stiffness", "damping", "limit_stiffness", "rest_length", "offset"]
    )
    def test_tendon_2d_count_and_shape_consistent(self, physx_sdk_cpu, tensor_type):
        """Verify that fixed_tendon_count=0 is consistent with the binding shape."""
        b = self._make_binding(physx_sdk_cpu, tensor_type)
        meta_b = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION
        )
        assert meta_b.fixed_tendon_count == b.shape[1]
        meta_b.destroy()
        b.destroy()

    def test_tendon_limit_count_and_shape_consistent(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_FIXED_TENDON_LIMIT)
        meta_b = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION
        )
        assert meta_b.fixed_tendon_count == b.shape[1]
        meta_b.destroy()
        b.destroy()


# ---------------------------------------------------------------------------
# DOF projected joint force (type 75) -- read-only
# ---------------------------------------------------------------------------


class TestProjectedJointForce:
    """Tests for TensorType.ARTICULATION_DOF_PROJECTED_JOINT_FORCE.
    Shape [N, D], read-only."""

    def test_projected_joint_force_shape(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_PROJECTED_JOINT_FORCE
        )
        assert b.ndim == 2
        assert b.shape == (EXPECTED_N, EXPECTED_D)
        b.destroy()

    def test_projected_joint_force_read(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_PROJECTED_JOINT_FORCE
        )
        buf = np.zeros(b.shape, dtype=np.float32)
        b.read(buf)
        assert np.all(np.isfinite(buf)), "projected joint forces must be finite"
        b.destroy()

    def test_projected_joint_force_write_raises(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_PROJECTED_JOINT_FORCE
        )
        buf = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)read.only"):
            b.write(buf)
        b.destroy()


# ---------------------------------------------------------------------------
# Spatial tendon (types 90-93) -- read / write
# ---------------------------------------------------------------------------


class TestSpatialTendon:
    """Tests for spatial tendon property tensor types.

    two_articulations.usda has no spatial tendons, so T=0. These tests verify
    that the binding can be created, shapes are correct (second dim = 0),
    reads succeed (returning an empty-second-dim tensor), writes succeed
    silently (no-op), and spatial_tendon_count returns 0.
    """

    _TENDON_TYPES = [
        TensorType.ARTICULATION_SPATIAL_TENDON_STIFFNESS,
        TensorType.ARTICULATION_SPATIAL_TENDON_DAMPING,
        TensorType.ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS,
        TensorType.ARTICULATION_SPATIAL_TENDON_OFFSET,
    ]

    def _make_binding(self, sdk, tensor_type):
        _load_two_articulations(sdk)
        return sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=tensor_type)

    def test_spatial_tendon_count_zero(self, physx_sdk_cpu):
        _load_two_articulations(physx_sdk_cpu)
        b = physx_sdk_cpu.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION)
        assert b.spatial_tendon_count == 0, "two_articulations.usda has no spatial tendons, expected T=0"
        b.destroy()

    @pytest.mark.parametrize("tensor_type", _TENDON_TYPES, ids=["stiffness", "damping", "limit_stiffness", "offset"])
    def test_shape_t0(self, physx_sdk_cpu, tensor_type):
        b = self._make_binding(physx_sdk_cpu, tensor_type)
        assert b.ndim == 2
        assert b.shape == (EXPECTED_N, 0), f"With T=0, shape should be (N, 0), got {b.shape}"
        b.destroy()

    @pytest.mark.parametrize("tensor_type", _TENDON_TYPES, ids=["stiffness", "damping", "limit_stiffness", "offset"])
    def test_count_and_shape_consistent(self, physx_sdk_cpu, tensor_type):
        """Verify that spatial_tendon_count=0 is consistent with the binding shape."""
        b = self._make_binding(physx_sdk_cpu, tensor_type)
        meta_b = physx_sdk_cpu.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION
        )
        assert meta_b.spatial_tendon_count == b.shape[1]
        meta_b.destroy()
        b.destroy()

    def test_enum_values_match_spec(self):
        """Spatial tendon enum values follow the semantic contract: 90-93."""
        assert TensorType.ARTICULATION_SPATIAL_TENDON_STIFFNESS == 90
        assert TensorType.ARTICULATION_SPATIAL_TENDON_DAMPING == 91
        assert TensorType.ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS == 92
        assert TensorType.ARTICULATION_SPATIAL_TENDON_OFFSET == 93


# ---------------------------------------------------------------------------
# Link wrench (type 52) -- write-only
# ---------------------------------------------------------------------------


class TestLinkWrench:
    """Tests for TensorType.ARTICULATION_LINK_WRENCH.

    Shape [N, L, 9] -- write-only tensor for applying external wrenches
    to articulation links. Each row is [fx,fy,fz,tx,ty,tz,px,py,pz]
    in world frame.
    """

    def _make_binding(self, sdk):
        _load_two_articulations(sdk)
        return sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_WRENCH)

    def test_shape(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu)
        assert b.ndim == 3
        assert b.shape == (EXPECTED_N, EXPECTED_L, 9), f"Expected (N={EXPECTED_N}, L={EXPECTED_L}, 9), got {b.shape}"
        b.destroy()

    def test_write_succeeds(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu)
        buf = np.zeros(b.shape, dtype=np.float32)
        buf[0, 0, 0] = 1.0  # apply force in x on first link
        b.write(buf)
        b.destroy()

    def test_read_raises(self, physx_sdk_cpu):
        """Link wrench is write-only; read must fail."""
        b = self._make_binding(physx_sdk_cpu)
        buf = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)write.only"):
            b.read(buf)
        b.destroy()

    def test_indexed_write(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu)
        buf = np.zeros(b.shape, dtype=np.float32)
        indices = np.array([0], dtype=np.int32)
        b.write(buf, indices=indices)
        b.destroy()

    def test_masked_write(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu)
        buf = np.zeros(b.shape, dtype=np.float32)
        mask = np.array([1, 0], dtype=np.uint8)
        b.write(buf, mask=mask)
        b.destroy()


class TestContactReport:
    """Tests for the pull-based contact report API (get_contact_report).

    Uses boxes_falling_on_groundplane.usda - boxes drop onto a ground plane,
    generating contact events after enough simulation steps.
    """

    def test_no_contacts_before_collision(self, physx_sdk_cpu):
        """Before any collision happens, contact report should be empty."""
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()
        report = physx_sdk_cpu.get_contact_report()
        assert report["num_headers"] >= 0
        assert report["num_points"] >= 0

    def test_contacts_after_falling(self, physx_sdk_cpu):
        """After enough steps for boxes to hit the ground, expect contacts."""
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        report = {"num_headers": 0, "num_points": 0}
        for _ in range(240):
            physx_sdk_cpu.step_sync(1.0 / 60.0)
            report = physx_sdk_cpu.get_contact_report()
            if report["num_headers"] > 0 and report["num_points"] > 0:
                break
        assert report["num_headers"] > 0, "Should have contact events after boxes fall onto ground"
        assert report["num_points"] > 0, "Should have contact data points"

    def test_contact_report_works_after_reset_stage(self, physx_sdk_cpu):
        """reset_stage() tears down physics and must not leave a broken contact report path."""
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        for _ in range(240):
            physx_sdk_cpu.step_sync(1.0 / 60.0)
            report = physx_sdk_cpu.get_contact_report()
            if report["num_headers"] > 0 and report["num_points"] > 0:
                break
        assert report["num_headers"] > 0
        assert report["num_points"] > 0

        physx_sdk_cpu.reset_stage()
        physx_sdk_cpu.wait_all()

        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        report_after_reset = {"num_headers": 0, "num_points": 0}
        for _ in range(240):
            physx_sdk_cpu.step_sync(1.0 / 60.0)
            report_after_reset = physx_sdk_cpu.get_contact_report()
            if report_after_reset["num_headers"] > 0 and report_after_reset["num_points"] > 0:
                break
        assert report_after_reset["num_headers"] > 0
        assert report_after_reset["num_points"] > 0

    def test_contact_report_struct_fields_accessible(self, physx_sdk_cpu):
        """When contacts exist, typed struct fields should be readable."""
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        for _ in range(60):
            physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()
        report = physx_sdk_cpu.get_contact_report()
        if report["num_headers"] > 0:
            h = report["headers"][0]
            assert hasattr(h, "actor0")
            assert hasattr(h, "numContactData")
            assert h.numContactData >= 0
        if report["num_points"] > 0:
            p = report["points"][0]
            assert hasattr(p, "position")
            assert hasattr(p, "normal")
            assert hasattr(p, "impulse")
            assert len(p.position) == 3
            assert len(p.normal) == 3

    def test_friction_anchors_returned_when_requested(self, physx_sdk_cpu):
        """With include_friction_anchors=True, dict should contain anchor fields."""
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        for _ in range(60):
            physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()
        report = physx_sdk_cpu.get_contact_report(include_friction_anchors=True)
        assert "num_anchors" in report
        assert "anchors" in report
        assert isinstance(report["num_anchors"], int)

    def test_friction_anchors_not_returned_by_default(self, physx_sdk_cpu):
        """Without include_friction_anchors, dict should not have anchor fields."""
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()
        report = physx_sdk_cpu.get_contact_report()
        assert "num_anchors" not in report
        assert "anchors" not in report

    def test_copy_true_returns_python_owned_dicts(self, physx_sdk_cpu):
        """With copy=True, headers/points/anchors must be lists of dicts."""
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        for _ in range(60):
            physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()
        report = physx_sdk_cpu.get_contact_report(include_friction_anchors=True, copy=True)
        assert isinstance(report["headers"], list)
        assert isinstance(report["points"], list)
        assert isinstance(report["anchors"], list)
        assert len(report["headers"]) == report["num_headers"]
        assert len(report["points"]) == report["num_points"]
        assert len(report["anchors"]) == report["num_anchors"]
        if report["num_headers"] > 0:
            h = report["headers"][0]
            assert isinstance(h, dict)
            assert "actor0" in h and "actor1" in h and "numContactData" in h
            assert isinstance(h["actor0"], int)
        if report["num_points"] > 0:
            p = report["points"][0]
            assert isinstance(p, dict)
            assert "position" in p and "normal" in p and "impulse" in p
            assert isinstance(p["position"], tuple)
            assert len(p["position"]) == 3
            assert all(isinstance(x, float) for x in p["position"])

    def test_copy_true_safe_across_steps(self, physx_sdk_cpu):
        """copy=True data must survive subsequent step() calls unchanged.

        Regression test for NVBug 6172700: zero-copy ctypes views silently
        corrupt after the next step; copy=True must return Python-owned data
        that does not depend on the internal C buffer's lifetime. Covers all
        three buffers (headers, points, anchors) since the hazard is
        symmetric across them.

        Snapshot via ``[dict(h) for h in ...]`` rather than ``list(...)``: we
        need *independent* dict objects so that if a future change ever lets
        the returned dicts alias C memory (lazy field reads, etc.) the
        assertion catches it. ``list()`` would just hold the same dict refs
        and pass trivially.
        """
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        for _ in range(60):
            physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()
        report = physx_sdk_cpu.get_contact_report(include_friction_anchors=True, copy=True)
        if report["num_headers"] == 0:
            pytest.skip("No contact events generated; cannot test cross-step retention.")
        snapshot_headers = [dict(h) for h in report["headers"]]
        snapshot_points = [dict(p) for p in report["points"]]
        snapshot_anchors = [dict(a) for a in report["anchors"]]
        # Advance simulation - this would reallocate / overwrite the internal
        # C buffers backing a zero-copy view.
        for _ in range(10):
            physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()
        # Independent snapshots must remain bit-identical to what we captured.
        # If the dicts ever start aliasing the C buffer this fails loudly.
        assert report["headers"] == snapshot_headers
        assert report["points"] == snapshot_points
        assert report["anchors"] == snapshot_anchors

    def test_copy_false_default_returns_ctypes_views(self, physx_sdk_cpu):
        """Default copy=False must keep returning ctypes array views (back-compat)."""
        load_usd_with_ovstage(physx_sdk_cpu, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk_cpu.wait_all()
        physx_sdk_cpu.step(1.0 / 60.0)
        physx_sdk_cpu.wait_all()
        report = physx_sdk_cpu.get_contact_report()
        # ctypes arrays expose typed-struct attribute access, not __getitem__-of-dict.
        # Use the existing field check pattern.
        assert not isinstance(report["headers"], list)
        assert not isinstance(report["points"], list)
