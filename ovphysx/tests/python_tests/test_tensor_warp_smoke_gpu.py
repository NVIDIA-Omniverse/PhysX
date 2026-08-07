# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Smoke tests: ovphysx tensor Python API with warp (``wp.array``) buffers, GPU mode.

These confirm the DLPack interop between warp arrays and the tensor-binding
read/write path works end to end.
"""

import numpy as np
import pytest

wp = pytest.importorskip("warp")
from ovphysx.types import TensorType  # noqa: E402
from test_utils import data_path, load_usd_with_ovstage  # noqa: E402

wp.init()

_DEVICE = "cuda:0"

_RB_PATTERN = "/World/Cube*"  # rigid bodies in boxes_falling_on_groundplane.usda
_ARTI_PATTERN = "/World/articulation*"  # articulations in two_articulations.usda
_GROUND_PATTERN = "/World/GroundPlane"  # contact filter for the raw-contact test


def _load_boxes(sdk):
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    sdk.warmup_gpu()


def _load_articulations(sdk):
    load_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
    sdk.wait_all()
    sdk.warmup_gpu()


# ---------------------------------------------------------------------------
# Plain float32, 2D -- read / full write / indexed write / masked write
# ---------------------------------------------------------------------------


class TestWarpGpuRigidBodyPose:
    """RIGID_BODY_POSE [N, 7] float32 via wp.float32 arrays."""

    def _binding(self, sdk):
        _load_boxes(sdk)
        return sdk.create_tensor_binding(
            raise_if_empty=True, pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE
        )

    def test_read(self, physx_sdk):
        b = self._binding(physx_sdk)
        buf = wp.zeros(b.shape, dtype=wp.float32, device=_DEVICE)
        b.read(buf)
        host = buf.numpy()
        assert host.shape == tuple(b.shape)
        assert np.all(np.isfinite(host))
        b.destroy()

    def test_full_write_roundtrip(self, physx_sdk):
        b = self._binding(physx_sdk)
        orig = wp.zeros(b.shape, dtype=wp.float32, device=_DEVICE)
        b.read(orig)
        modified_np = orig.numpy().copy()
        modified_np[..., 0] += 1.0  # shift world-x
        b.write(wp.array(modified_np, dtype=wp.float32, device=_DEVICE))
        result = wp.zeros(b.shape, dtype=wp.float32, device=_DEVICE)
        b.read(result)
        assert np.allclose(result.numpy()[..., 0], modified_np[..., 0], atol=1e-4)
        b.destroy()

    def test_indexed_write_int32(self, physx_sdk):
        """Indexed write driven by a warp int32 index array -- verifies selectivity:
        only the indexed row is written, every other row is left untouched."""
        b = self._binding(physx_sdk)
        n = b.shape[0]
        assert n >= 2, "need >=2 bodies to verify indexed-write selectivity"
        orig = wp.zeros(b.shape, dtype=wp.float32, device=_DEVICE)
        b.read(orig)
        orig_np = orig.numpy().copy()
        modified_np = orig_np.copy()
        modified_np[:, 0] += 2.0  # bump world-x on EVERY row in the source buffer
        idx = 0
        indices = wp.array(np.array([idx], dtype=np.int32), dtype=wp.int32, device=_DEVICE)
        b.write(wp.array(modified_np, dtype=wp.float32, device=_DEVICE), indices=indices)
        result = wp.zeros(b.shape, dtype=wp.float32, device=_DEVICE)
        b.read(result)
        res_np = result.numpy()
        # the indexed row got the new value...
        assert np.isclose(res_np[idx, 0], modified_np[idx, 0], atol=1e-4)
        # ...and every non-indexed row is unchanged (would fail if the write ignored indices)
        others = [r for r in range(n) if r != idx]
        assert np.allclose(res_np[others, 0], orig_np[others, 0], atol=1e-4)
        b.destroy()

    def test_masked_write_uint8(self, physx_sdk):
        """Masked write driven by a warp uint8 mask array -- verifies selectivity:
        only the masked row is written, every other row is left untouched."""
        b = self._binding(physx_sdk)
        n = b.shape[0]
        assert n >= 2, "need >=2 bodies to verify masked-write selectivity"
        orig = wp.zeros(b.shape, dtype=wp.float32, device=_DEVICE)
        b.read(orig)
        orig_np = orig.numpy().copy()
        modified_np = orig_np.copy()
        modified_np[:, 0] += 3.0  # bump world-x on EVERY row in the source buffer
        mask_np = np.zeros(n, dtype=np.uint8)
        mask_np[0] = 1  # mask only row 0
        b.write(
            wp.array(modified_np, dtype=wp.float32, device=_DEVICE),
            mask=wp.array(mask_np, dtype=wp.uint8, device=_DEVICE),
        )
        result = wp.zeros(b.shape, dtype=wp.float32, device=_DEVICE)
        b.read(result)
        res_np = result.numpy()
        # the masked row got the new value...
        assert np.isclose(res_np[0, 0], modified_np[0, 0], atol=1e-4)
        # ...and every unmasked row is unchanged (would fail if the write ignored the mask)
        assert np.allclose(res_np[1:, 0], orig_np[1:, 0], atol=1e-4)
        b.destroy()


# ---------------------------------------------------------------------------
# uint8 1D
# ---------------------------------------------------------------------------


class TestWarpGpuUint8:
    """RIGID_BODY_DISABLE_SIMULATION [N] uint8 via wp.uint8 arrays."""

    def test_disable_simulation_roundtrip(self, physx_sdk):
        _load_boxes(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            raise_if_empty=True,
            pattern=_RB_PATTERN,
            tensor_type=TensorType.RIGID_BODY_DISABLE_SIMULATION,
        )
        buf = wp.zeros(b.shape, dtype=wp.uint8, device=_DEVICE)
        b.read(buf)
        assert buf.numpy().shape == tuple(b.shape)
        ones = np.ones(b.shape, dtype=np.uint8)
        b.write(wp.array(ones, dtype=wp.uint8, device=_DEVICE))
        result = wp.zeros(b.shape, dtype=wp.uint8, device=_DEVICE)
        b.read(result)
        assert np.array_equal(result.numpy(), ones)
        b.destroy()

    def test_disable_gravity_roundtrip(self, physx_sdk):
        _load_boxes(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            raise_if_empty=True,
            pattern=_RB_PATTERN,
            tensor_type=TensorType.RIGID_BODY_DISABLE_GRAVITY,
        )
        buf = wp.zeros(b.shape, dtype=wp.uint8, device=_DEVICE)
        b.read(buf)
        assert buf.numpy().shape == tuple(b.shape)
        ones = np.ones(b.shape, dtype=np.uint8)
        b.write(wp.array(ones, dtype=wp.uint8, device=_DEVICE))
        result = wp.zeros(b.shape, dtype=wp.uint8, device=_DEVICE)
        b.read(result)
        assert np.array_equal(result.numpy(), ones)
        b.destroy()


# ---------------------------------------------------------------------------
# Rank-3 ([N, S, 3]) WRITE round-trip via a writable 3D binding
# ---------------------------------------------------------------------------


class TestWarpGpuShapeProperties3D:
    """RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION [N, S, 3] float32 via wp.float32 arrays."""

    def test_shape_friction_roundtrip_3d(self, physx_sdk):
        load_usd_with_ovstage(physx_sdk, data_path("simple_physics_scene.usda"))
        physx_sdk.wait_all()
        b = physx_sdk.create_tensor_binding(
            raise_if_empty=True, pattern=_RB_PATTERN,
            tensor_type=TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION,
        )
        assert len(b.shape) == 3 and b.shape[1] >= 1  # [N, S, 3]
        orig = wp.zeros(b.shape, dtype=wp.float32, device=_DEVICE)
        b.read(orig)
        modified_np = orig.numpy().copy()
        modified_np[:, :, 0] = 0.7  # static_friction
        modified_np[:, :, 1] = 0.5  # dynamic_friction
        modified_np[:, :, 2] = 0.3  # restitution
        b.write(wp.array(modified_np, dtype=wp.float32, device=_DEVICE))
        result = wp.zeros(b.shape, dtype=wp.float32, device=_DEVICE)
        b.read(result)
        assert np.allclose(result.numpy(), modified_np, atol=1e-4)
        b.destroy()


# ---------------------------------------------------------------------------
# Structured warp types (wp.transform / wp.spatial_vector / wp.vec3 / wp.mat33)
# ---------------------------------------------------------------------------


class TestWarpGpuStructuredTypes:
    """Warp vector/matrix dtypes a user is likely to reach for. Their DLPack
    export expands the inner dimension, so they line up with the matching
    binding shape -- except wp.mat33 (see the inertia contract test)."""

    def test_wp_transform_pose(self, physx_sdk):
        """wp.transform <-> RIGID_BODY_POSE [N, 7]: verifies the COMPONENT MAPPING,
        not just the shape -- a wp.transform's (translation, quaternion) must land in
        the pose's (px,py,pz, qx,qy,qz,qw) slots. Writes a known transform and reads
        it back as both plain float32 and wp.transform, checking the exact values."""
        _load_boxes(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            raise_if_empty=True, pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_POSE
        )
        n = b.shape[0]
        assert tuple(b.shape) == (n, 7)
        # known pose: translation (1,2,3); 90deg about +z -> quat (x,y,z,w) = (0,0,s,s)
        s = 2.0 ** -0.5
        expected = np.tile(np.array([1.0, 2.0, 3.0, 0.0, 0.0, s, s], dtype=np.float32), (n, 1))
        # WRITE through a wp.transform-typed buffer (translation + quaternion)...
        b.write(wp.array(expected.copy(), dtype=wp.transform, device=_DEVICE))
        # ...READ back as plain float32 [N, 7] -- components must match the pose layout
        out_f32 = wp.zeros((n, 7), dtype=wp.float32, device=_DEVICE)
        b.read(out_f32)
        assert np.allclose(out_f32.numpy(), expected, atol=1e-4), \
            "wp.transform did not map to pose (px,py,pz, qx,qy,qz,qw)"
        # ...and READ back into a wp.transform buffer yields the same data
        out_tf = wp.zeros(n, dtype=wp.transform, device=_DEVICE)
        b.read(out_tf)
        assert np.allclose(out_tf.numpy().reshape(n, 7), expected, atol=1e-4)
        b.destroy()

    def test_wp_spatial_vector_velocity(self, physx_sdk):
        """wp.spatial_vector <-> RIGID_BODY_VELOCITY [N, 6]: verifies the data maps by
        STORAGE ORDER. Writes a known velocity in the binding's (linear, angular) layout
        and confirms a wp.spatial_vector reads back the same flat [1,2,3,4,5,6].

        NOTE: warp's spatial_vector is semantically (angular, linear) -- the opposite
        halves -- so DLPack only preserves the raw [N, 6] storage; a caller must NOT
        assume wp.spatial_top() corresponds to the binding's angular part.
        """
        _load_boxes(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            raise_if_empty=True, pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_VELOCITY
        )
        n = b.shape[0]
        # binding layout: (linear xyz, angular xyz)
        expected = np.tile(np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dtype=np.float32), (n, 1))
        # WRITE through a wp.spatial_vector-typed buffer...
        b.write(wp.array(expected.copy(), dtype=wp.spatial_vector, device=_DEVICE))
        # ...READ back as plain float32 [N, 6] -- storage order preserved 1:1
        out_f32 = wp.zeros((n, 6), dtype=wp.float32, device=_DEVICE)
        b.read(out_f32)
        assert np.allclose(out_f32.numpy(), expected, atol=1e-4)
        # ...and into a wp.spatial_vector buffer -- same flat data
        out_sv = wp.zeros(n, dtype=wp.spatial_vector, device=_DEVICE)
        b.read(out_sv)
        assert np.allclose(out_sv.numpy().reshape(n, 6), expected, atol=1e-4)
        b.destroy()

    def test_wp_vec3_mass_center(self, physx_sdk):
        """wp.vec3 exports [N, 3] -> matches ARTICULATION_MASS_CENTER_WORLD (read-only)."""
        _load_articulations(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            raise_if_empty=True,
            pattern=_ARTI_PATTERN,
            tensor_type=TensorType.ARTICULATION_MASS_CENTER_WORLD,
        )
        n = b.shape[0]
        buf = wp.zeros(n, dtype=wp.vec3, device=_DEVICE)
        b.read(buf)
        host = buf.numpy().reshape(n, 3)
        assert host.shape == (n, 3)
        assert np.all(np.isfinite(host))
        b.destroy()

    def test_wp_mat33_inertia_contract(self, physx_sdk):
        """Documents the contract: RIGID_BODY_INERTIA is flat [N, 9], so a
        wp.mat33 array (which exports [N, 3, 3]) does NOT line up -- the binding
        rejects the rank-3 buffer. A user must pass a flat [N, 9] float32 array
        (or reshape their mat33 buffer) instead."""
        _load_boxes(physx_sdk)
        b = physx_sdk.create_tensor_binding(
            raise_if_empty=True, pattern=_RB_PATTERN, tensor_type=TensorType.RIGID_BODY_INERTIA
        )
        n = b.shape[0]
        assert tuple(b.shape) == (n, 9)
        mat = wp.zeros(n, dtype=wp.mat33, device=_DEVICE)
        assert mat.numpy().shape == (n, 3, 3)  # [N,3,3] != binding's [N,9]
        with pytest.raises(RuntimeError, match=r"expected 2D tensor, got 3D"):
            b.read(mat)
        # The supported form: a flat [N, 9] float32 warp array -- and it round-trips.
        flat = wp.zeros(b.shape, dtype=wp.float32, device=_DEVICE)
        b.read(flat)
        assert flat.numpy().shape == (n, 9)
        assert np.all(np.isfinite(flat.numpy()))
        known = np.zeros((n, 9), dtype=np.float32)
        known[:, 0], known[:, 4], known[:, 8] = 0.5, 0.6, 0.7  # diagonal inertia
        b.write(wp.array(known, dtype=wp.float32, device=_DEVICE))
        out = wp.zeros(b.shape, dtype=wp.float32, device=_DEVICE)
        b.read(out)
        assert np.allclose(out.numpy(), known, atol=1e-4)
        b.destroy()


# ---------------------------------------------------------------------------
# Raw contact data -- int32 counts/start-indices + int64 actor-ids (GPU-only;
# raw-contact reads are GPU-only when PhysX is GPU, so there is no CPU mirror)
# ---------------------------------------------------------------------------


class TestWarpGpuRawContact:
    """read_raw_contact_data into warp cuda buffers: float32 + int32 + int64."""

    def test_raw_contact_data_int32_int64(self, physx_sdk):
        load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
        physx_sdk.wait_all()
        cap = 256
        cb = physx_sdk.create_contact_binding(
            sensor_patterns=[_RB_PATTERN],
            filter_patterns=[_GROUND_PATTERN],
            filters_per_sensor=1,
            max_contact_data_count=cap,
        )
        physx_sdk.warmup_gpu()
        for _ in range(60):  # let the boxes settle onto the ground so contacts exist
            physx_sdk.step(1.0 / 60.0)
        physx_sdk.wait_all()

        s = cb.sensor_count
        forces = wp.zeros((cap, 1), dtype=wp.float32, device=_DEVICE)
        positions = wp.zeros((cap, 3), dtype=wp.float32, device=_DEVICE)
        normals = wp.zeros((cap, 3), dtype=wp.float32, device=_DEVICE)
        separations = wp.zeros((cap, 1), dtype=wp.float32, device=_DEVICE)
        counts = wp.zeros(s, dtype=wp.int32, device=_DEVICE)
        start_indices = wp.zeros(s, dtype=wp.int32, device=_DEVICE)
        other_actor_ids = wp.zeros(cap, dtype=wp.int64, device=_DEVICE)

        cb.read_raw_contact_data(forces, positions, normals, separations, counts, start_indices, other_actor_ids)

        counts_np = counts.numpy()
        start_np = start_indices.numpy()

        # Shape smoke.
        assert counts_np.shape == (s,)
        assert start_np.shape == (s,)
        assert other_actor_ids.numpy().shape == (cap,)

        # The 60 settle steps guarantee boxes rest on the ground, so at least
        # one sensor must have reported contacts.
        assert counts_np.sum() > 0, "expected non-zero contact counts after settling"

        # Counts and start-indices must be non-negative and within the cap.
        assert np.all(counts_np >= 0)
        total = int(counts_np.sum())
        assert total <= cap, f"total contact count {total} overflows cap {cap}"
        assert np.all(start_np >= 0)
        assert np.all(start_np < cap)

        # Positions for populated contacts must be finite.
        assert np.all(np.isfinite(positions.numpy()[:total]))
        cb.destroy()
