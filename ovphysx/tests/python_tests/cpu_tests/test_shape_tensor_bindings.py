# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Tests for shape-level tensor bindings: material properties, contact offsets, rest offsets.

Scenes:
  - simple_physics_scene.usda: rigid body (Cube1) with at least 1 shape
  - two_articulations.usda: 2 articulations with 3 links each
"""

import os

import numpy as np
import pytest
from ovphysx.types import TensorType
from test_utils import load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


RB_PATTERN = "/World/Cube*"
ARTI_PATTERN = "/World/articulation*"


# ---------------------------------------------------------------------------
# Rigid body shape-level tensors
# ---------------------------------------------------------------------------


class TestRigidBodyShapeTensors:

    def _make_binding(self, sdk, tensor_type):
        load_usd_with_ovstage(sdk, data_path("simple_physics_scene.usda"))
        sdk.wait_all()
        return sdk.create_tensor_binding(pattern=RB_PATTERN, tensor_type=tensor_type)

    def test_material_properties_shape(self, physx_sdk_cpu):
        """Material properties binding should have shape [N, S, 3]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION)
        assert b.ndim == 3
        N, S, C = b.shape
        assert N >= 1
        assert S >= 1
        assert C == 3
        b.destroy()

    def test_material_properties_read(self, physx_sdk_cpu):
        """Material properties should be readable and contain finite values."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out))
        b.destroy()

    def test_contact_offset_shape(self, physx_sdk_cpu):
        """Contact offset binding should have shape [N, S]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_CONTACT_OFFSET)
        assert b.ndim == 2
        N, S = b.shape
        assert N >= 1
        assert S >= 1
        b.destroy()

    def test_contact_offset_read(self, physx_sdk_cpu):
        """Contact offsets should be readable and contain finite values."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_CONTACT_OFFSET)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out))
        b.destroy()

    def test_rest_offset_shape(self, physx_sdk_cpu):
        """Rest offset binding should have shape [N, S]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_REST_OFFSET)
        assert b.ndim == 2
        N, S = b.shape
        assert N >= 1
        assert S >= 1
        b.destroy()

    def test_rest_offset_read(self, physx_sdk_cpu):
        """Rest offsets should be readable and contain finite values."""
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_REST_OFFSET)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out))
        b.destroy()

    def test_material_properties_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original.copy()
        modified[:, :, 0] = 0.7  # static_friction
        modified[:, :, 1] = 0.5  # dynamic_friction
        modified[:, :, 2] = 0.3  # restitution
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()

    def test_material_properties_pool_reuse(self, physx_sdk_cpu):
        """Regression for NVBugs 6489465 / OMPE-102536.

        Writing shape friction/restitution interns PxMaterials in a refcounted
        pool (BaseSimulationView::mMaterials) keyed by a formatted value string.
        A prior key-drift bug erased pool entries with a reconstructed 3-component
        key that never matched the stored key, so a released material was recycled
        and repurposed for a new tuple while the stale pool entry still pointed at
        it. Re-requesting the original tuple then returned the recycled material
        with foreign values (the write reported success and read-back was wrong).

        Drive the A -> B -> C -> A write sequence that reproduced the corruption:
        after B every shape is off tuple A (its material refcount hits 0 and is
        recycled), C repurposes that material, and re-requesting A must return A's
        values -- not C's.
        """
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_SHAPE_FRICTION_AND_RESTITUTION)

        def uniform(static_friction, dynamic_friction, restitution):
            props = np.zeros(b.shape, dtype=np.float32)
            props[:, :, 0] = static_friction
            props[:, :, 1] = dynamic_friction
            props[:, :, 2] = restitution
            return props

        mat_a = uniform(0.1, 0.2, 0.3)
        mat_b = uniform(0.4, 0.5, 0.6)
        mat_c = uniform(0.7, 0.8, 0.9)

        b.write(mat_a)
        b.write(mat_b)
        b.write(mat_c)
        b.write(mat_a)  # re-request the recycled tuple

        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(
            result, mat_a, rtol=1e-5,
            err_msg="Re-requesting a previously-used material tuple returned the "
            "wrong material (NVBugs 6489465)",
        )
        b.destroy()

    def test_contact_offset_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_CONTACT_OFFSET)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original + 0.01
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()

    def test_rest_offset_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.RIGID_BODY_REST_OFFSET)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original + 0.005
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()


# ---------------------------------------------------------------------------
# Articulation shape-level tensors
# ---------------------------------------------------------------------------


class TestArticulationShapeTensors:

    def _make_binding(self, sdk, tensor_type):
        load_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
        sdk.wait_all()
        return sdk.create_tensor_binding(pattern=ARTI_PATTERN, tensor_type=tensor_type)

    def test_material_properties_shape(self, physx_sdk_cpu):
        """Articulation material properties binding should have shape [N, S, 3]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION)
        assert b.ndim == 3
        N, S, C = b.shape
        assert N >= 1
        assert S >= 1
        assert C == 3
        b.destroy()

    def test_material_properties_read(self, physx_sdk_cpu):
        """Articulation material properties should be readable and finite."""
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION)
        out = np.zeros(b.shape, dtype=np.float32)
        b.read(out)
        assert np.all(np.isfinite(out))
        b.destroy()

    def test_contact_offset_shape(self, physx_sdk_cpu):
        """Articulation contact offset binding should have shape [N, S]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_CONTACT_OFFSET)
        assert b.ndim == 2
        N, S = b.shape
        assert N >= 1
        assert S >= 1
        b.destroy()

    def test_rest_offset_shape(self, physx_sdk_cpu):
        """Articulation rest offset binding should have shape [N, S]."""
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_REST_OFFSET)
        assert b.ndim == 2
        N, S = b.shape
        assert N >= 1
        assert S >= 1
        b.destroy()

    def test_material_properties_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original.copy()
        modified[:, :, 0] = 0.6  # static_friction
        modified[:, :, 1] = 0.4  # dynamic_friction
        modified[:, :, 2] = 0.2  # restitution
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()

    def test_material_properties_pool_reuse(self, physx_sdk_cpu):
        """Regression for NVBugs 6489465 / OMPE-102536 on the articulation path.

        Articulation shape material writes share the same refcounted material
        pool and release path (BaseSimulationView::releaseSharedMaterial) that
        the rigid-body writes use, so the same A -> B -> C -> A key-drift repro
        must return A's values on re-request rather than the recycled C material.
        """
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_SHAPE_FRICTION_AND_RESTITUTION)

        def uniform(static_friction, dynamic_friction, restitution):
            props = np.zeros(b.shape, dtype=np.float32)
            props[:, :, 0] = static_friction
            props[:, :, 1] = dynamic_friction
            props[:, :, 2] = restitution
            return props

        mat_a = uniform(0.1, 0.2, 0.3)
        mat_b = uniform(0.4, 0.5, 0.6)
        mat_c = uniform(0.7, 0.8, 0.9)

        b.write(mat_a)
        b.write(mat_b)
        b.write(mat_c)
        b.write(mat_a)  # re-request the recycled tuple

        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(
            result, mat_a, rtol=1e-5,
            err_msg="Re-requesting a previously-used material tuple returned the "
            "wrong material (NVBugs 6489465)",
        )
        b.destroy()

    def test_contact_offset_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_CONTACT_OFFSET)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original + 0.01
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()

    def test_rest_offset_write_roundtrip(self, physx_sdk_cpu):
        b = self._make_binding(physx_sdk_cpu, TensorType.ARTICULATION_REST_OFFSET)
        original = np.zeros(b.shape, dtype=np.float32)
        b.read(original)
        modified = original + 0.005
        b.write(modified)
        result = np.zeros(b.shape, dtype=np.float32)
        b.read(result)
        np.testing.assert_allclose(result, modified, rtol=1e-5)
        b.destroy()
