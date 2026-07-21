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
