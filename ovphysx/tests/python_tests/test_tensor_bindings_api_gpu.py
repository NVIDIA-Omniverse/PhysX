# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""GPU-mode tests for TensorBindingsAPI features.

Covers DOF properties, body properties, shape properties, dynamics tensors,
link wrench, fixed tendons, and spatial tendons in GPU mode with actual
simulation stepping where needed.

GPU-state tensors (dynamics, wrench, tendons) require kDLCUDA buffers.
We use a minimal CudaArray helper that allocates device memory via the
CUDA driver API (cuMemAlloc/cuMemFree) and exposes a __dlpack__-compatible
DLTensor. No torch/cupy dependency.

CPU-property tensors (DOF stiffness/damping/limits, body mass/COM/inertia,
shape material/contact/rest offsets) accept numpy even in GPU mode.
"""

import ctypes
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
from ovphysx.dlpack import DLDataType, DLDataTypeCode, DLDevice, DLDeviceType, DLTensor
from ovphysx.types import TensorType
from test_utils import NP_TO_DL_DTYPE, CudaArray, data_path, get_cuda_driver
from test_utils import load_usd_with_ovstage


def _require_cuda_driver():
    try:
        get_cuda_driver()
    except OSError as exc:
        pytest.fail(f"CUDA driver not available for GPU test coverage: {exc}")


def _gpu_tensor(shape):
    """Create a zero-initialized CUDA float32 tensor."""
    _require_cuda_driver()
    return CudaArray(shape)


def _gpu_read(binding) -> np.ndarray:
    """Read GPU binding into a new CudaArray, return as numpy."""
    ga = _gpu_tensor(binding.shape)
    binding.read(ga.dltensor)
    return ga.numpy()


def _gpu_write(binding, host_array, **kwargs):
    """Upload host_array to GPU and write into binding."""
    ga = _gpu_tensor(binding.shape)
    ga.upload(host_array)
    binding.write(ga.dltensor, **kwargs)


def _gpu_write_with_indices(binding, host_array, indices_np):
    """Upload host_array and indices to GPU, then call indexed write."""
    ga = _gpu_tensor(binding.shape)
    ga.upload(host_array)
    gi = CudaArray(indices_np.shape, dtype=np.int32)
    gi.upload(indices_np)
    binding.write(ga.dltensor, indices=gi.dltensor)


def _gpu_write_with_mask(binding, host_array, mask_np):
    """Upload host_array and mask to GPU, then call masked write."""
    ga = _gpu_tensor(binding.shape)
    ga.upload(host_array)
    gm = CudaArray(mask_np.shape, dtype=np.uint8)
    gm.upload(mask_np)
    binding.write(ga.dltensor, mask=gm.dltensor)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

ARTI_PATTERN = "/World/articulation*"
DT = 1.0 / 60.0


def _load_and_step(sdk, scene="two_articulations.usda", n_steps=5):
    load_usd_with_ovstage(sdk, data_path(scene))
    sdk.wait_all()
    sdk.warmup_gpu()
    for _ in range(n_steps):
        sdk.step(DT)
    sdk.wait_all()


def _make_cube_pair_contact_binding(sdk):
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    sdk.warmup_gpu()

    cube1_pose = sdk.create_tensor_binding(pattern="/World/Cube1", tensor_type=TensorType.RIGID_BODY_POSE)
    cube2_pose = sdk.create_tensor_binding(pattern="/World/Cube2", tensor_type=TensorType.RIGID_BODY_POSE)
    cube2_velocity = sdk.create_tensor_binding(pattern="/World/Cube2", tensor_type=TensorType.RIGID_BODY_VELOCITY)

    pose = _gpu_read(cube1_pose)
    pose[0, 2] += 0.25
    _gpu_write(cube2_pose, pose)
    _gpu_write(cube2_velocity, np.zeros(cube2_velocity.shape, dtype=np.float32))
    sdk.wait_all()

    cube1_pose.destroy()
    cube2_pose.destroy()
    cube2_velocity.destroy()

    cb = sdk.create_contact_binding(
        sensor_patterns=["/World/Cube1"],
        filter_patterns=["/World/Cube2"],
        filters_per_sensor=1,
        max_contact_data_count=256,
    )

    sdk.step(DT)
    sdk.wait_all()
    return cb


def _make_usd_with_authored_dof_max_velocity(tmp_path, max_velocity_rad=5.0):
    src = Path(data_path("two_articulations.usda"))
    text = src.read_text()
    text = text.replace(
        'apiSchemas = ["PhysicsDriveAPI:angular"]', 'apiSchemas = ["PhysicsDriveAPI:angular", "PhysxJointAPI"]'
    )
    max_velocity_deg = max_velocity_rad * 180.0 / math.pi
    text = text.replace(
        'uniform token physics:axis = "Y"',
        f"float physxJoint:maxJointVelocity = {max_velocity_deg:.9f}\n" '            uniform token physics:axis = "Y"',
    )
    dst = tmp_path / "two_articulations_authored_max_velocity.usda"
    dst.write_text(text)
    return dst


def _cpu_read(binding) -> np.ndarray:
    """Read CPU-property binding into numpy (works for CPU-side props in GPU mode)."""
    buf = np.zeros(binding.shape, dtype=np.float32)
    binding.read(buf)
    return buf


# ---------------------------------------------------------------------------
# Articulation kinematic update
# ---------------------------------------------------------------------------


class TestArticulationKinematicUpdateGpu:

    def test_first_fk_update_auto_warmups_gpu(self, physx_sdk):
        """First GPU FK refresh should initialize DirectGPU buffers itself."""
        from ovphysx._bindings import _lib, ovphysx_log_fn

        load_usd_with_ovstage(physx_sdk, data_path("two_articulations.usda"))
        physx_sdk.wait_all()

        records = []

        @ovphysx_log_fn
        def collector(level, message, user_data):
            text = message.decode("utf-8", errors="replace") if message else ""
            records.append((level, text))

        result = _lib.ovphysx_register_log_callback(collector, None)
        assert result.status == 0, "Failed to register native log callback"

        try:
            physx_sdk.update_articulations_kinematic()
        finally:
            _lib.ovphysx_unregister_log_callback(collector, None)

        direct_gpu_errors = [
            text
            for _, text in records
            if "computeArticulationData" in text or "DirectGPU API has not been initialized" in text
        ]
        assert not direct_gpu_errors, "\n".join(direct_gpu_errors)

    def test_dof_position_write_updates_link_pose_after_fk(self, physx_sdk):
        """DOF writes followed by explicit FK refresh link poses without stepping."""
        _load_and_step(physx_sdk, n_steps=1)

        dof_b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION)
        link_pose_b = physx_sdk.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_POSE
        )

        before = _gpu_read(link_pose_b)
        dof_pos = _gpu_read(dof_b)
        dof_pos[:, :] = 0.5
        _gpu_write(dof_b, dof_pos)

        physx_sdk.update_articulations_kinematic()
        after = _gpu_read(link_pose_b)

        assert not np.allclose(
            after[:, :, :3], before[:, :, :3], atol=1e-4
        ), "Link positions must reflect the written DOF positions after FK update"

        dof_b.destroy()
        link_pose_b.destroy()


# ---------------------------------------------------------------------------
# Standalone rigid-body GPU state tensors
# ---------------------------------------------------------------------------


class TestRigidBodyStateGpu:

    def test_acceleration_readable_on_gpu(self, physx_sdk):
        _load_and_step(physx_sdk, scene="boxes_falling_on_groundplane.usda")
        b = physx_sdk.create_tensor_binding(pattern="/World/Cube*", tensor_type=TensorType.RIGID_BODY_ACCELERATION)
        assert b.ndim == 2 and b.shape[1] == 6
        buf = _gpu_read(b)
        assert np.all(np.isfinite(buf))
        b.destroy()


# ---------------------------------------------------------------------------
# Shape properties (100-112) -- CPU-property tensors
# ---------------------------------------------------------------------------


class TestShapePropertiesGpu:

    _RIGID_SHAPE_TYPES = [
        (TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION, "material"),
        (TensorType.RIGID_BODY_CONTACT_OFFSET, "contact_offset"),
        (TensorType.RIGID_BODY_REST_OFFSET, "rest_offset"),
    ]

    _ARTICULATION_SHAPE_TYPES = [
        (TensorType.ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION, "material"),
        (TensorType.ARTICULATION_CONTACT_OFFSET, "contact_offset"),
        (TensorType.ARTICULATION_REST_OFFSET, "rest_offset"),
    ]

    @staticmethod
    def _modify_shape_property(original: np.ndarray, tensor_type: TensorType) -> np.ndarray:
        modified = original.copy()
        if tensor_type in (
            TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION,
            TensorType.ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION,
        ):
            modified[:, :, 0] = 0.7
            modified[:, :, 1] = 0.5
            modified[:, :, 2] = 0.3
        elif tensor_type in (
            TensorType.RIGID_BODY_CONTACT_OFFSET,
            TensorType.ARTICULATION_CONTACT_OFFSET,
        ):
            modified += 0.01
        else:
            modified += 0.005
        return modified

    @pytest.mark.parametrize("tensor_type,name", _RIGID_SHAPE_TYPES, ids=[n for _, n in _RIGID_SHAPE_TYPES])
    def test_rigid_shape_property_cpu_roundtrip_in_gpu_mode(self, physx_sdk, tensor_type, name):
        _load_and_step(physx_sdk, scene="simple_physics_scene.usda")
        b = physx_sdk.create_tensor_binding(pattern="/World/Cube*", tensor_type=tensor_type)
        original = _cpu_read(b)
        assert np.all(np.isfinite(original)), f"{name} values must be finite"
        modified = self._modify_shape_property(original, tensor_type)
        b.write(modified)
        readback = _cpu_read(b)
        np.testing.assert_allclose(readback, modified, atol=1e-5, err_msg=f"Rigid-body {name} roundtrip failed")
        b.destroy()

    @pytest.mark.parametrize(
        "tensor_type,name", _ARTICULATION_SHAPE_TYPES, ids=[n for _, n in _ARTICULATION_SHAPE_TYPES]
    )
    def test_articulation_shape_property_cpu_roundtrip_in_gpu_mode(self, physx_sdk, tensor_type, name):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=tensor_type)
        original = _cpu_read(b)
        assert np.all(np.isfinite(original)), f"{name} values must be finite"
        modified = self._modify_shape_property(original, tensor_type)
        b.write(modified)
        readback = _cpu_read(b)
        np.testing.assert_allclose(readback, modified, atol=1e-5, err_msg=f"Articulation {name} roundtrip failed")
        b.destroy()

    def test_shape_property_cross_device_read_to_cuda_buffer(self, physx_sdk):
        # Shape-property tensors are CPU-only on the runtime side. Reading into
        # a CUDA dst uses cross-device staging (CPU staging buffer + memcpyHtoD).
        _load_and_step(physx_sdk, scene="simple_physics_scene.usda")
        b = physx_sdk.create_tensor_binding(
            pattern="/World/Cube*", tensor_type=TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION
        )
        ga = _gpu_tensor(b.shape)
        b.read(ga.dltensor)
        b.destroy()


# ---------------------------------------------------------------------------
# Contact bindings -- GPU-state detailed contact/friction buffers
# ---------------------------------------------------------------------------


class TestContactBindingGpu:

    def test_cuda_output_without_directgpu_mentions_opt_in(self):
        _require_cuda_driver()
        script = r"""
import numpy as np

from ovphysx import PhysX

from test_utils import CudaArray, data_path, load_usd_with_ovstage

physx = PhysX()
load_usd_with_ovstage(physx, data_path("boxes_falling_on_groundplane.usda"))
physx.wait_all()

cb = physx.create_contact_binding(
    sensor_patterns=["/World/Cube1"],
    filter_patterns=["/World/Cube2"],
    filters_per_sensor=1,
    max_contact_data_count=64,
)
physx.step(1.0 / 60.0)
physx.wait_all()
assert cb.sensor_count > 0, "Expected ContactBinding to resolve at least one sensor"
dst = CudaArray((cb.sensor_count, 3), dtype=np.float32)

try:
    cb.read_net_forces(dst.dltensor)
except RuntimeError as exc:
    msg = str(exc)
    assert "DirectGPU" in msg, msg
    assert "suppressReadback" in msg, msg
else:
    raise AssertionError("Expected ContactBinding CUDA read to fail without DirectGPU opt-in")
"""
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=Path(__file__).parent,
            text=True,
            capture_output=True,
            timeout=120,
        )
        assert result.returncode == 0, result.stdout + result.stderr

    def test_contact_data_flat_buffers_on_gpu(self, physx_sdk):
        cb = _make_cube_pair_contact_binding(physx_sdk)
        c = cb.max_contact_data_count
        contact_forces = CudaArray((c, 1), dtype=np.float32)
        positions = CudaArray((c, 3), dtype=np.float32)
        normals = CudaArray((c, 3), dtype=np.float32)
        separations = CudaArray((c, 1), dtype=np.float32)
        counts = CudaArray((cb.sensor_count, cb.filter_count), dtype=np.int32)
        starts = CudaArray((cb.sensor_count, cb.filter_count), dtype=np.int32)

        cb.read_contact_data(
            contact_forces.dltensor,
            positions.dltensor,
            normals.dltensor,
            separations.dltensor,
            counts.dltensor,
            starts.dltensor,
        )

        contact_forces_np = contact_forces.numpy()
        positions_np = positions.numpy()
        normals_np = normals.numpy()
        separations_np = separations.numpy()
        counts_np = counts.numpy()
        starts_np = starts.numpy()

        assert np.all(np.isfinite(contact_forces_np))
        assert np.all(np.isfinite(positions_np))
        assert np.all(np.isfinite(normals_np))
        assert np.all(np.isfinite(separations_np))
        assert np.all(counts_np >= 0)
        assert np.all(starts_np >= 0)
        assert int(counts_np.sum()) <= c
        pair_count = int(counts_np[0, 0])
        pair_start = int(starts_np[0, 0])
        assert pair_count > 0, "Overlapped Cube1/Cube2 pair should produce detailed contacts"
        assert pair_start + pair_count <= c
        normal_lengths = np.linalg.norm(normals_np[pair_start : pair_start + pair_count], axis=1)
        assert np.all(normal_lengths > 0.5)
        cb.destroy()

    def test_friction_data_flat_buffers_on_gpu(self, physx_sdk):
        cb = _make_cube_pair_contact_binding(physx_sdk)
        c = cb.max_contact_data_count
        friction_forces = CudaArray((c, 3), dtype=np.float32)
        friction_points = CudaArray((c, 3), dtype=np.float32)
        counts = CudaArray((cb.sensor_count, cb.filter_count), dtype=np.int32)
        starts = CudaArray((cb.sensor_count, cb.filter_count), dtype=np.int32)

        cb.read_friction_data(friction_forces.dltensor, friction_points.dltensor, counts.dltensor, starts.dltensor)

        friction_forces_np = friction_forces.numpy()
        friction_points_np = friction_points.numpy()
        counts_np = counts.numpy()
        starts_np = starts.numpy()

        assert np.all(np.isfinite(friction_forces_np))
        assert np.all(np.isfinite(friction_points_np))
        assert np.all(counts_np >= 0)
        assert np.all(starts_np >= 0)
        assert int(counts_np.sum()) <= c
        pair_count = int(counts_np[0, 0])
        pair_start = int(starts_np[0, 0])
        assert pair_count > 0, "Overlapped Cube1/Cube2 pair should produce friction anchors"
        assert pair_start + pair_count <= c
        cb.destroy()


# ---------------------------------------------------------------------------
# DOF properties (35-41) -- CPU-property tensors, numpy works in GPU mode
# ---------------------------------------------------------------------------


class TestDofPropertiesGpu:

    _DOF_PROP_2D = [
        (TensorType.ARTICULATION_DOF_STIFFNESS, "stiffness"),
        (TensorType.ARTICULATION_DOF_DAMPING, "damping"),
        (TensorType.ARTICULATION_DOF_MAX_VELOCITY, "max_velocity"),
        (TensorType.ARTICULATION_DOF_MAX_FORCE, "max_force"),
        (TensorType.ARTICULATION_DOF_ARMATURE, "armature"),
    ]

    @pytest.mark.parametrize("tensor_type,name", _DOF_PROP_2D, ids=[n for _, n in _DOF_PROP_2D])
    def test_read_finite(self, physx_sdk, tensor_type, name):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=tensor_type)
        buf = _cpu_read(b)
        assert np.all(np.isfinite(buf)), f"{name} must be finite after stepping"
        b.destroy()

    @pytest.mark.parametrize("tensor_type,name", _DOF_PROP_2D, ids=[n for _, n in _DOF_PROP_2D])
    def test_write_roundtrip(self, physx_sdk, tensor_type, name):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=tensor_type)
        original = _cpu_read(b)
        modified = original + 1.0
        b.write(modified)
        readback = _cpu_read(b)
        np.testing.assert_allclose(readback, modified, atol=1e-5, err_msg=f"{name} roundtrip failed")
        b.destroy()

    def test_dof_limit_roundtrip(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_LIMIT)
        assert b.ndim == 3
        assert b.shape[2] == 2
        original = _cpu_read(b)
        modified = original.copy()
        modified[:, :, 0] = -99.0
        modified[:, :, 1] = 99.0
        b.write(modified)
        readback = _cpu_read(b)
        np.testing.assert_allclose(readback, modified, atol=1e-5)
        b.destroy()

    def test_max_velocity_reads_physx_joint_usd_authoring(self, physx_sdk, tmp_path):
        usd_path = _make_usd_with_authored_dof_max_velocity(tmp_path, max_velocity_rad=5.0)
        load_usd_with_ovstage(physx_sdk, str(usd_path))
        physx_sdk.wait_all()
        physx_sdk.warmup_gpu()
        physx_sdk.wait_all()

        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_MAX_VELOCITY)
        values = _cpu_read(b)
        np.testing.assert_allclose(values, np.full(b.shape, 5.0, dtype=np.float32), atol=1e-5)
        b.destroy()

    def test_max_velocity_indexed_write_roundtrip(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_MAX_VELOCITY)
        original = _cpu_read(b)
        assert original.shape[0] >= 2

        modified = original.copy()
        modified[0, :] = 123.0
        b.write(modified, indices=np.array([0], dtype=np.int32))

        readback = _cpu_read(b)
        np.testing.assert_allclose(readback[0, :], modified[0, :], atol=1e-5)
        np.testing.assert_allclose(readback[1:, :], original[1:, :], atol=1e-5)
        b.destroy()

    def test_friction_properties_column_isolation(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_FRICTION_PROPERTIES
        )
        assert b.shape[2] == 3
        original = _cpu_read(b)
        modified = original.copy()
        modified[:, :, 0] = 42.0
        b.write(modified)
        readback = _cpu_read(b)
        np.testing.assert_allclose(readback[:, :, 0], 42.0, atol=1e-5)
        np.testing.assert_allclose(readback[:, :, 1], original[:, :, 1], atol=1e-5)
        np.testing.assert_allclose(readback[:, :, 2], original[:, :, 2], atol=1e-5)
        b.destroy()

    def test_masked_write(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_STIFFNESS)
        original = _cpu_read(b)
        modified = original.copy()
        modified[:] = 999.0
        mask = np.array([1, 0], dtype=np.uint8)
        b.write(modified, mask=mask)
        readback = _cpu_read(b)
        np.testing.assert_allclose(readback[0], 999.0, atol=1e-5, err_msg="Masked row 0 should be updated")
        np.testing.assert_allclose(readback[1], original[1], atol=1e-5, err_msg="Unmasked row 1 should be unchanged")
        b.destroy()


# ---------------------------------------------------------------------------
# Body properties (60-62) -- CPU-property tensors
# ---------------------------------------------------------------------------


class TestBodyPropertiesGpu:

    def test_body_mass_roundtrip(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_BODY_MASS)
        original = _cpu_read(b)
        assert np.all(original > 0), "masses should be positive"
        modified = original * 2.0
        b.write(modified)
        readback = _cpu_read(b)
        np.testing.assert_allclose(readback, modified, atol=1e-5)
        b.destroy()

    def test_body_com_pose_roundtrip(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_BODY_COM_POSE)
        assert b.ndim == 3 and b.shape[2] == 7
        original = _cpu_read(b)
        modified = original.copy()
        modified[:, :, 0] += 0.01
        b.write(modified)
        readback = _cpu_read(b)
        np.testing.assert_allclose(readback, modified, atol=1e-5)
        b.destroy()

    def test_body_inertia_roundtrip(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_BODY_INERTIA)
        assert b.ndim == 3 and b.shape[2] == 9
        original = _cpu_read(b)
        modified = original.copy()
        modified[:, :, 0] *= 1.5
        b.write(modified)
        readback = _cpu_read(b)
        np.testing.assert_allclose(readback, modified, atol=1e-5)
        b.destroy()

    def test_body_mass_indexed_write(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_BODY_MASS)
        original = _cpu_read(b)
        modified = original.copy()
        modified[:] = 123.0
        indices = np.array([0], dtype=np.int32)
        b.write(modified, indices=indices)
        readback = _cpu_read(b)
        np.testing.assert_allclose(readback[0], 123.0, atol=1e-5)
        np.testing.assert_allclose(readback[1], original[1], atol=1e-5)
        b.destroy()


# ---------------------------------------------------------------------------
# Body inverse mass/inertia (63-64) -- CPU-property, read-only
# ---------------------------------------------------------------------------


class TestBodyInverseGpu:

    def test_inv_mass_shape_and_values(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_BODY_INV_MASS)
        buf = _cpu_read(b)
        assert np.all(np.isfinite(buf))
        b.destroy()

    def test_inv_inertia_shape(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_BODY_INV_INERTIA)
        assert b.ndim == 3 and b.shape[2] == 9
        buf = _cpu_read(b)
        assert np.all(np.isfinite(buf))
        b.destroy()

    def test_inv_mass_consistency_with_mass(self, physx_sdk):
        """Verify inv_mass * mass ~= 1.0 (catches garbage return values)."""
        _load_and_step(physx_sdk)
        mass_b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_BODY_MASS)
        inv_b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_BODY_INV_MASS)
        mass = _cpu_read(mass_b)
        inv_mass = _cpu_read(inv_b)
        product = mass * inv_mass
        np.testing.assert_allclose(product, 1.0, atol=1e-4, err_msg="inv_mass * mass should be ~1.0")
        mass_b.destroy()
        inv_b.destroy()

    def test_inv_mass_write_rejected(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_BODY_INV_MASS)
        buf = np.zeros(b.shape, dtype=np.float32)
        with pytest.raises(RuntimeError, match="(?i)read.only"):
            b.write(buf)
        b.destroy()


# ---------------------------------------------------------------------------
# Dynamics tensors (70-75) -- GPU-state, read-only, requires CUDA buffers
# ---------------------------------------------------------------------------


class TestDynamicsTensorsGpu:

    def test_jacobian_readable(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_JACOBIAN)
        assert b.ndim == 3
        buf = _gpu_read(b)
        assert np.all(np.isfinite(buf))
        b.destroy()

    def test_mass_matrix_symmetric(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_MASS_MATRIX)
        assert b.ndim == 3
        assert b.shape[1] == b.shape[2], "mass matrix must be square"
        buf = _gpu_read(b)
        assert np.all(np.isfinite(buf))
        for i in range(buf.shape[0]):
            np.testing.assert_allclose(
                buf[i], buf[i].T, atol=1e-4, err_msg=f"Mass matrix for articulation {i} not symmetric"
            )
        b.destroy()

    def test_gravity_force_nonzero(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_GRAVITY_FORCE)
        buf = _gpu_read(b)
        assert np.all(np.isfinite(buf))
        assert not np.allclose(buf, 0.0), "gravity forces should be non-zero with gravity enabled"
        b.destroy()

    def test_coriolis_readable(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE
        )
        buf = _gpu_read(b)
        assert np.all(np.isfinite(buf))
        b.destroy()

    def test_projected_joint_force_readable(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_PROJECTED_JOINT_FORCE
        )
        buf = _gpu_read(b)
        assert np.all(np.isfinite(buf))
        b.destroy()

    def test_link_incoming_joint_force_readable(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_INCOMING_JOINT_FORCE
        )
        assert b.ndim == 3 and b.shape[2] == 6
        buf = _gpu_read(b)
        assert np.all(np.isfinite(buf))
        b.destroy()

    _DYNAMICS_TYPES = [
        TensorType.ARTICULATION_JACOBIAN,
        TensorType.ARTICULATION_MASS_MATRIX,
        TensorType.ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE,
        TensorType.ARTICULATION_GRAVITY_FORCE,
        TensorType.ARTICULATION_LINK_INCOMING_JOINT_FORCE,
        TensorType.ARTICULATION_DOF_PROJECTED_JOINT_FORCE,
    ]

    @pytest.mark.parametrize(
        "tensor_type",
        _DYNAMICS_TYPES,
        ids=["jacobian", "mass_matrix", "coriolis", "gravity", "link_incoming_joint_force", "projected_joint_force"],
    )
    def test_dynamics_write_rejected(self, physx_sdk, tensor_type):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=tensor_type)
        ga = _gpu_tensor(b.shape)
        with pytest.raises(RuntimeError, match="(?i)read.only"):
            b.write(ga.dltensor)
        b.destroy()


# ---------------------------------------------------------------------------
# Link wrench (52) -- GPU write-only + effect on simulation
# ---------------------------------------------------------------------------


class TestLinkWrenchGpu:

    def test_shape(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_WRENCH)
        assert b.ndim == 3 and b.shape[2] == 9
        b.destroy()

    def test_write_and_effect(self, physx_sdk):
        """Apply a wrench via GPU tensor and verify DOF positions change."""
        _load_and_step(physx_sdk, n_steps=1)
        pos_b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION)
        pos_before = _gpu_read(pos_b)

        wrench_b = physx_sdk.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_WRENCH
        )
        wrench_host = np.zeros(wrench_b.shape, dtype=np.float32)
        wrench_host[0, 1, 0] = 1000.0
        _gpu_write(wrench_b, wrench_host)

        for _ in range(5):
            physx_sdk.step(DT)
        physx_sdk.wait_all()

        pos_after = _gpu_read(pos_b)
        assert not np.allclose(pos_after, pos_before, atol=1e-6), "Wrench should have caused DOF position change"
        pos_b.destroy()
        wrench_b.destroy()

    def test_read_rejected(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_WRENCH)
        ga = _gpu_tensor(b.shape)
        with pytest.raises(RuntimeError, match="(?i)write.only"):
            b.read(ga.dltensor)
        b.destroy()

    def test_indexed_write(self, physx_sdk):
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_LINK_WRENCH)
        host = np.zeros(b.shape, dtype=np.float32)
        indices = np.array([0], dtype=np.int32)
        _gpu_write_with_indices(b, host, indices)
        b.destroy()


# ---------------------------------------------------------------------------
# Fixed tendons (80-85) -- GPU roundtrip with T>0 scene
# ---------------------------------------------------------------------------


class TestFixedTendonGpu:

    _PATTERN = "/FixedTendonTest"

    def _load(self, sdk):
        load_usd_with_ovstage(sdk, data_path("FixedTendonTest.usda"))
        sdk.wait_all()
        sdk.warmup_gpu()
        for _ in range(3):
            sdk.step(DT)
        sdk.wait_all()

    def test_fixed_tendon_count_positive(self, physx_sdk):
        self._load(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=self._PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION)
        assert b.fixed_tendon_count > 0, "FixedTendonTest.usda should have at least one fixed tendon"
        b.destroy()

    _FIXED_2D_TYPES = [
        (TensorType.ARTICULATION_FIXED_TENDON_STIFFNESS, "stiffness"),
        (TensorType.ARTICULATION_FIXED_TENDON_DAMPING, "damping"),
        (TensorType.ARTICULATION_FIXED_TENDON_LIMIT_STIFFNESS, "limit_stiffness"),
        (TensorType.ARTICULATION_FIXED_TENDON_REST_LENGTH, "rest_length"),
        (TensorType.ARTICULATION_FIXED_TENDON_OFFSET, "offset"),
    ]

    @pytest.mark.parametrize("tensor_type,name", _FIXED_2D_TYPES, ids=[n for _, n in _FIXED_2D_TYPES])
    def test_read_finite(self, physx_sdk, tensor_type, name):
        self._load(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=self._PATTERN, tensor_type=tensor_type)
        assert b.shape[1] > 0, f"Expected T>0 for {name}"
        buf = _gpu_read(b)
        assert np.all(np.isfinite(buf)), f"Fixed tendon {name} must be finite"
        b.destroy()

    def test_stiffness_write_roundtrip(self, physx_sdk):
        self._load(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            pattern=self._PATTERN, tensor_type=TensorType.ARTICULATION_FIXED_TENDON_STIFFNESS
        )
        original = _gpu_read(b)
        modified = original + 5.0
        _gpu_write(b, modified)
        readback = _gpu_read(b)
        np.testing.assert_allclose(readback, modified, atol=1e-4)
        b.destroy()

    def test_masked_write(self, physx_sdk):
        self._load(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            pattern=self._PATTERN, tensor_type=TensorType.ARTICULATION_FIXED_TENDON_DAMPING
        )
        N = b.shape[0]
        original = _gpu_read(b)
        modified = original + 77.0
        mask = np.ones(N, dtype=np.uint8)
        _gpu_write_with_mask(b, modified, mask)
        readback = _gpu_read(b)
        np.testing.assert_allclose(readback, modified, atol=1e-4)
        b.destroy()

    def test_limit_shape_3d(self, physx_sdk):
        self._load(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            pattern=self._PATTERN, tensor_type=TensorType.ARTICULATION_FIXED_TENDON_LIMIT
        )
        assert b.ndim == 3 and b.shape[2] == 2, "Limit shape must be [N, T, 2]"
        buf = _gpu_read(b)
        assert np.all(np.isfinite(buf))
        b.destroy()


# ---------------------------------------------------------------------------
# Spatial tendons (90-93) -- GPU roundtrip with T>0 scene
# ---------------------------------------------------------------------------


class TestSpatialTendonGpu:

    _PATTERN = "/SpatialTendonTest"

    def _load(self, sdk, scene="SpatialTendonTest.usda"):
        load_usd_with_ovstage(sdk, data_path(scene))
        sdk.wait_all()
        sdk.warmup_gpu()
        for _ in range(3):
            sdk.step(DT)
        sdk.wait_all()

    def test_spatial_tendon_count_positive(self, physx_sdk):
        self._load(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=self._PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION)
        assert b.spatial_tendon_count > 0, "SpatialTendonTest.usda should have at least one spatial tendon"
        b.destroy()

    _SPATIAL_TYPES = [
        (TensorType.ARTICULATION_SPATIAL_TENDON_STIFFNESS, "stiffness"),
        (TensorType.ARTICULATION_SPATIAL_TENDON_DAMPING, "damping"),
        (TensorType.ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS, "limit_stiffness"),
        (TensorType.ARTICULATION_SPATIAL_TENDON_OFFSET, "offset"),
    ]

    @pytest.mark.parametrize("tensor_type,name", _SPATIAL_TYPES, ids=[n for _, n in _SPATIAL_TYPES])
    def test_read_finite(self, physx_sdk, tensor_type, name):
        self._load(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=self._PATTERN, tensor_type=tensor_type)
        assert b.shape[1] > 0, f"Expected T>0 for spatial {name}"
        buf = _gpu_read(b)
        assert np.all(np.isfinite(buf)), f"Spatial tendon {name} must be finite"
        b.destroy()

    def test_damping_write_roundtrip(self, physx_sdk):
        self._load(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            pattern=self._PATTERN, tensor_type=TensorType.ARTICULATION_SPATIAL_TENDON_DAMPING
        )
        original = _gpu_read(b)
        modified = original + 10.0
        _gpu_write(b, modified)
        readback = _gpu_read(b)
        np.testing.assert_allclose(readback, modified, atol=1e-4)
        b.destroy()

    def test_masked_write(self, physx_sdk):
        self._load(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            pattern=self._PATTERN, tensor_type=TensorType.ARTICULATION_SPATIAL_TENDON_STIFFNESS
        )
        N = b.shape[0]
        original = _gpu_read(b)
        modified = original + 42.0
        mask = np.ones(N, dtype=np.uint8)
        _gpu_write_with_mask(b, modified, mask)
        readback = _gpu_read(b)
        np.testing.assert_allclose(readback, modified, atol=1e-4)
        b.destroy()

    def test_multiple_spatial_tendons_count(self, physx_sdk):
        self._load(physx_sdk, scene="MultipleSpatialTendonsTest.usda")
        b = physx_sdk.create_tensor_binding(pattern=self._PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION)
        assert (
            b.spatial_tendon_count == 2
        ), f"MultipleSpatialTendonsTest.usda has 2 spatial tendons, got {b.spatial_tendon_count}"
        b.destroy()

    def test_enum_values(self):
        assert TensorType.ARTICULATION_SPATIAL_TENDON_STIFFNESS == 90
        assert TensorType.ARTICULATION_SPATIAL_TENDON_DAMPING == 91
        assert TensorType.ARTICULATION_SPATIAL_TENDON_LIMIT_STIFFNESS == 92
        assert TensorType.ARTICULATION_SPATIAL_TENDON_OFFSET == 93


# ---------------------------------------------------------------------------
# Articulation metadata and dynamics -- GPU
# ---------------------------------------------------------------------------


class TestArticulationMetadataAndDynamicsGpu:

    def test_metadata(self, physx_sdk):
        """Verify all 6 metadata fields via direct TensorBinding (GPU simulation mode)."""
        _load_and_step(physx_sdk)
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION)
        assert b.dof_count == 2
        assert b.body_count == 3
        assert b.joint_count == 2
        assert b.is_fixed_base is True
        assert b.fixed_tendon_count == 0
        assert b.spatial_tendon_count == 0
        b.destroy()

    def test_metadata_readable_before_warmup(self, physx_sdk):
        """Articulation metadata is CPU-side topology data; it must be readable before
        warmup_gpu() and without any simulation steps.  If this regresses it means
        someone accidentally gated metadata behind the GPU readback path."""
        load_usd_with_ovstage(physx_sdk, data_path("two_articulations.usda"))
        physx_sdk.wait_all()
        # Deliberately NO warmup_gpu(), NO step() -- GPU buffers are uninitialised
        b = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_POSITION)
        assert b.dof_count == 2, "dof_count must be available before warmup"
        assert b.body_count == 3, "body_count must be available before warmup"
        assert b.joint_count == 2, "joint_count must be available before warmup"
        assert b.is_fixed_base is True, "is_fixed_base must be available before warmup"
        assert b.fixed_tendon_count == 0, "fixed_tendon_count must be available before warmup"
        assert b.spatial_tendon_count == 0, "spatial_tendon_count must be available before warmup"
        # Cache must be populated (second call returns same object, no second C call)
        m1 = b._get_artic_metadata()
        m2 = b._get_artic_metadata()
        assert m1 is m2, "metadata should be cached after first call"
        b.destroy()

    def test_dynamics_binding_shapes(self, physx_sdk):
        """Verify dynamics tensor shapes via direct TensorBinding creation."""
        _load_and_step(physx_sdk)

        jac = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_JACOBIAN)
        assert jac.ndim == 3

        mm = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_MASS_MATRIX)
        assert mm.ndim == 3 and mm.shape[1] == mm.shape[2]

        grav = physx_sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_GRAVITY_FORCE)
        assert grav.ndim == 2

        cor = physx_sdk.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_CORIOLIS_AND_CENTRIFUGAL_FORCE
        )
        assert cor.ndim == 2

        pjf = physx_sdk.create_tensor_binding(
            pattern=ARTI_PATTERN, tensor_type=TensorType.ARTICULATION_DOF_PROJECTED_JOINT_FORCE
        )
        assert pjf.ndim == 2

        for b in (jac, mm, grav, cor, pjf):
            b.destroy()


# ---------------------------------------------------------------------------
# Cross-device staging copy must respect DLTensor.byte_offset
# ---------------------------------------------------------------------------
#
# Regression: the cross-device staging path used to copy from/to the
# DLTensor's raw `data` pointer, ignoring `byte_offset`. Sliced views (where
# the caller exposes a contiguous tail of a larger allocation by setting
# byte_offset != 0) silently corrupted other rows or read sentinel data.
# `dlToTensorDesc` already applies `data + byte_offset` to the TensorDesc;
# the staging memcpy on both sides must use that same adjusted pointer.


class TestCrossDeviceStagingByteOffset:

    @staticmethod
    def _build_offset_dltensor(host_buf: np.ndarray, byte_offset: int, shape: tuple, dtype=np.float32) -> DLTensor:
        """Build a CPU DLTensor pointing at ``host_buf`` with ``byte_offset`` baked in.

        ``host_buf`` is the underlying numpy allocation; the returned DLTensor
        keeps a reference to ``host_buf`` via the shape array's lifetime in
        the test scope. The caller must keep ``host_buf`` alive until the
        write/read completes.
        """
        np_dtype = np.dtype(dtype)
        dl_code, dl_bits = NP_TO_DL_DTYPE[np_dtype]
        shape_arr = (ctypes.c_int64 * len(shape))(*shape)

        dl = DLTensor()
        dl.data = ctypes.c_void_p(host_buf.ctypes.data)
        dl.device = DLDevice()
        dl.device.device_type = DLDeviceType(DLDeviceType.kDLCPU)
        dl.device.device_id = 0
        dl.ndim = len(shape)
        dl.dtype = DLDataType()
        dl.dtype.code = DLDataTypeCode(dl_code)
        dl.dtype.bits = dl_bits
        dl.dtype.lanes = 1
        dl.shape = ctypes.cast(shape_arr, ctypes.POINTER(ctypes.c_int64))
        dl.strides = None
        dl.byte_offset = byte_offset
        # Keep the shape array alive for the duration of the write/read by
        # attaching it to the DLTensor object as an attribute.
        dl._shape_arr = shape_arr  # noqa: SLF001 -- intentional lifetime pin
        dl._host_buf = host_buf  # noqa: SLF001 -- intentional lifetime pin
        return dl

    def test_cross_device_write_respects_byte_offset(self, physx_sdk):
        """Cross-device write must read from data + byte_offset, not from base.

        Construct a CPU host buffer with a sentinel row at index 0 and the
        intended values at index 1; pass a DLTensor with byte_offset pointing
        at row 1 to a GPU rigid-body pose binding. Without the fix, the
        staging memcpy reads the sentinel row, the binding ends up holding
        sentinel poses, and the next read of the binding returns sentinels.
        """
        _load_and_step(physx_sdk, scene="simple_physics_scene.usda")
        b = physx_sdk.create_tensor_binding(pattern="/World/Cube*", tensor_type=TensorType.RIGID_BODY_POSE)

        # Pose layout: (N, 7) float32 = (px, py, pz, qx, qy, qz, qw).
        n = b.shape[0]
        cols = b.shape[1]
        assert cols == 7, "RIGID_BODY_POSE expected (N, 7) layout"
        row_bytes = cols * 4

        # Two rows worth of data: row 0 = sentinel (must NOT be written),
        # row 1 = the intended poses (must be the values that land in the binding).
        intended = np.zeros((n, cols), dtype=np.float32)
        intended[:, 2] = 5.25  # z = 5.25
        intended[:, 6] = 1.0  # qw = 1 (identity rotation)
        sentinel = np.full((n, cols), -999.0, dtype=np.float32)
        combined = np.concatenate([sentinel, intended], axis=0).astype(np.float32, copy=False)
        # Sanity: combined must be C-contiguous for byte_offset slicing to mean "row 1".
        assert combined.flags["C_CONTIGUOUS"]

        dl = self._build_offset_dltensor(combined, byte_offset=n * row_bytes, shape=(n, cols))
        b.write(dl)

        readback = _gpu_read(b)
        np.testing.assert_allclose(
            readback,
            intended,
            atol=1e-5,
            err_msg=("Cross-device write ignored byte_offset; binding holds " "sentinel data from the wrong slice."),
        )
        b.destroy()

    def test_cross_device_read_respects_byte_offset(self, physx_sdk):
        """Cross-device read must write into data + byte_offset, not into base.

        Construct a CPU host buffer that has a guard row at index 0; pass
        a DLTensor with byte_offset pointing at row 1 as the read destination
        for a GPU rigid-body pose binding. Without the fix, memcpyDtoH writes
        into row 0 and clobbers the guard, while row 1 stays uninitialized.
        """
        _load_and_step(physx_sdk, scene="simple_physics_scene.usda")
        b = physx_sdk.create_tensor_binding(pattern="/World/Cube*", tensor_type=TensorType.RIGID_BODY_POSE)

        n = b.shape[0]
        cols = b.shape[1]
        assert cols == 7
        row_bytes = cols * 4

        # Row 0 is a guard pattern that must survive untouched; row 1 is the
        # destination where the binding's pose data must land.
        GUARD = np.float32(-77.0)
        host = np.full((2 * n, cols), GUARD, dtype=np.float32)
        # Pre-fill row 1 with a different value so we can detect "didn't write".
        host[n:] = np.float32(42.0)

        dl = self._build_offset_dltensor(host, byte_offset=n * row_bytes, shape=(n, cols))
        b.read(dl)

        # Row 0 (guard slice) must be byte-for-byte unchanged.
        np.testing.assert_array_equal(
            host[:n],
            np.full((n, cols), GUARD, dtype=np.float32),
            err_msg=(
                "Cross-device read clobbered the pre-byte_offset region; "
                "byte_offset was ignored on the destination side."
            ),
        )
        # Row 1 must contain the binding's poses (= what _gpu_read returns).
        gpu_truth = _gpu_read(b)
        np.testing.assert_allclose(
            host[n:],
            gpu_truth,
            atol=1e-5,
            err_msg=(
                "Cross-device read did not write into the byte_offset slice; "
                "the destination pointer was used without byte_offset applied."
            ),
        )
        b.destroy()
