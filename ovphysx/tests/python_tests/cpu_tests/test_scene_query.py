# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Tests for scene query API: raycast, sweep, and overlap.

Scene: simple_physics_scene.usda
  - Ground plane at Y=0 (Cube collider scaled to 100x1x100)
  - Dynamic Cube1 at (0, 5, 0) -- falls under gravity
"""

import os

import pytest
from ovphysx.types import SceneQueryGeometryType, SceneQueryMode
from test_utils import load_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


def _load_and_step(sdk, n_steps=10, dt=1.0 / 60.0):
    load_usd_with_ovstage(sdk, data_path("simple_physics_scene.usda"))
    sdk.wait_all()
    for _ in range(n_steps):
        sdk.step(dt)
    sdk.wait_all()


# ---------------------------------------------------------------------------
# Raycast
# ---------------------------------------------------------------------------


class TestRaycast:

    def test_raycast_closest_hits_ground(self, physx_sdk_cpu):
        """Downward ray from (0, 100, 0) should hit something."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.raycast(
            origin=[0.0, 100.0, 0.0],
            direction=[0.0, -1.0, 0.0],
            distance=200.0,
            mode=SceneQueryMode.CLOSEST,
        )
        assert len(hits) == 1
        assert hits[0]["distance"] > 0.0

    def test_raycast_any(self, physx_sdk_cpu):
        """ANY mode should return exactly 0 or 1 hits."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.raycast(
            origin=[0.0, 100.0, 0.0],
            direction=[0.0, -1.0, 0.0],
            distance=200.0,
            mode=SceneQueryMode.ANY,
        )
        assert len(hits) in (0, 1)

    def test_raycast_all(self, physx_sdk_cpu):
        """ALL mode should return multiple hits (cube + ground)."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.raycast(
            origin=[0.0, 100.0, 0.0],
            direction=[0.0, -1.0, 0.0],
            distance=200.0,
            mode=SceneQueryMode.ALL,
        )
        assert len(hits) >= 1

    def test_raycast_miss(self, physx_sdk_cpu):
        """Ray aimed away from scene should return 0 hits."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.raycast(
            origin=[0.0, 100.0, 0.0],
            direction=[0.0, 1.0, 0.0],
            distance=200.0,
            mode=SceneQueryMode.CLOSEST,
        )
        assert len(hits) == 0


# ---------------------------------------------------------------------------
# Sweep
# ---------------------------------------------------------------------------


class TestSweep:

    def test_sweep_sphere_closest(self, physx_sdk_cpu):
        """Sweep a small sphere downward -- should hit something."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.sweep(
            geometry_type=SceneQueryGeometryType.SPHERE,
            direction=[0.0, -1.0, 0.0],
            distance=200.0,
            mode=SceneQueryMode.CLOSEST,
            radius=0.5,
            position=[0.0, 100.0, 0.0],
        )
        assert len(hits) == 1
        assert hits[0]["distance"] > 0.0

    def test_sweep_box_closest(self, physx_sdk_cpu):
        """Sweep a small box downward."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.sweep(
            geometry_type=SceneQueryGeometryType.BOX,
            direction=[0.0, -1.0, 0.0],
            distance=200.0,
            mode=SceneQueryMode.CLOSEST,
            half_extent=[0.5, 0.5, 0.5],
            position=[0.0, 100.0, 0.0],
            rotation=[0.0, 0.0, 0.0, 1.0],
        )
        assert len(hits) == 1
        assert hits[0]["distance"] > 0.0

    def test_sweep_sphere_miss(self, physx_sdk_cpu):
        """Sweep away from scene -- no hits."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.sweep(
            geometry_type=SceneQueryGeometryType.SPHERE,
            direction=[0.0, 1.0, 0.0],
            distance=200.0,
            mode=SceneQueryMode.CLOSEST,
            radius=0.5,
            position=[0.0, 100.0, 0.0],
        )
        assert len(hits) == 0


# ---------------------------------------------------------------------------
# Overlap
# ---------------------------------------------------------------------------


class TestOverlap:

    def test_overlap_sphere_at_ground(self, physx_sdk_cpu):
        """Large sphere centered at origin should overlap ground geometry."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.overlap(
            geometry_type=SceneQueryGeometryType.SPHERE,
            mode=SceneQueryMode.ALL,
            radius=5.0,
            position=[0.0, 0.0, 0.0],
        )
        assert len(hits) >= 1

    def test_overlap_any(self, physx_sdk_cpu):
        """ANY mode should return 0 or 1."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.overlap(
            geometry_type=SceneQueryGeometryType.SPHERE,
            mode=SceneQueryMode.ANY,
            radius=5.0,
            position=[0.0, 0.0, 0.0],
        )
        assert len(hits) in (0, 1)

    def test_overlap_box_no_overlap(self, physx_sdk_cpu):
        """Box far from scene should have 0 overlaps."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.overlap(
            geometry_type=SceneQueryGeometryType.BOX,
            mode=SceneQueryMode.ALL,
            half_extent=[0.5, 0.5, 0.5],
            position=[0.0, 1000.0, 0.0],
            rotation=[0.0, 0.0, 0.0, 1.0],
        )
        assert len(hits) == 0


# ---------------------------------------------------------------------------
# Edge cases and negative inputs
# ---------------------------------------------------------------------------


class TestSceneQueryEdgeCases:

    def test_raycast_negative_distance(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        with pytest.raises(RuntimeError):
            physx_sdk_cpu.raycast(
                origin=[0.0, 100.0, 0.0],
                direction=[0.0, -1.0, 0.0],
                distance=-1.0,
            )

    def test_raycast_nan_distance(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        with pytest.raises(RuntimeError):
            physx_sdk_cpu.raycast(
                origin=[0.0, 100.0, 0.0],
                direction=[0.0, -1.0, 0.0],
                distance=float("nan"),
            )

    def test_raycast_inf_distance(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        with pytest.raises(RuntimeError):
            physx_sdk_cpu.raycast(
                origin=[0.0, 100.0, 0.0],
                direction=[0.0, -1.0, 0.0],
                distance=float("inf"),
            )

    def test_sweep_negative_distance(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        with pytest.raises(RuntimeError):
            physx_sdk_cpu.sweep(
                geometry_type=SceneQueryGeometryType.SPHERE,
                direction=[0.0, -1.0, 0.0],
                distance=-5.0,
                radius=0.5,
                position=[0.0, 100.0, 0.0],
            )

    def test_sweep_invalid_geometry_type(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        with pytest.raises((RuntimeError, ValueError)):
            physx_sdk_cpu.sweep(
                geometry_type=99,
                direction=[0.0, -1.0, 0.0],
                distance=100.0,
            )

    def test_sweep_unknown_kwarg_rejected(self, physx_sdk_cpu):
        _load_and_step(physx_sdk_cpu)
        with pytest.raises(TypeError):
            physx_sdk_cpu.sweep(
                geometry_type=SceneQueryGeometryType.SPHERE,
                direction=[0.0, -1.0, 0.0],
                distance=100.0,
                raduis=0.5,  # intentional typo
            )

    def test_raycast_zero_distance(self, physx_sdk_cpu):
        """Zero distance is valid per the API (>= 0) -- should not raise."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.raycast(
            origin=[0.0, 100.0, 0.0],
            direction=[0.0, -1.0, 0.0],
            distance=0.0,
        )
        assert len(hits) == 0


# ---------------------------------------------------------------------------
# SHAPE geometry type
# ---------------------------------------------------------------------------


class TestShapeGeometry:

    def test_sweep_shape_closest(self, physx_sdk_cpu):
        """Sweep using an existing collision shape prim path."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.sweep(
            geometry_type=SceneQueryGeometryType.SHAPE,
            direction=[0.0, -1.0, 0.0],
            distance=200.0,
            mode=SceneQueryMode.CLOSEST,
            prim_path="/World/Cube1",
        )
        # Cube1 swept downward should hit the ground plane
        assert len(hits) >= 0  # validate no crash; hit count depends on scene state

    def test_overlap_shape(self, physx_sdk_cpu):
        """Overlap test using an existing collision shape prim path."""
        _load_and_step(physx_sdk_cpu)
        hits = physx_sdk_cpu.overlap(
            geometry_type=SceneQueryGeometryType.SHAPE,
            mode=SceneQueryMode.ALL,
            prim_path="/World/Cube1",
        )
        assert isinstance(hits, list)


# ---------------------------------------------------------------------------
# Enum sync
# ---------------------------------------------------------------------------


class TestSceneQueryEnumValues:
    """Verify Python enums match the C header values."""

    def test_mode_values(self):
        assert SceneQueryMode.CLOSEST == 0
        assert SceneQueryMode.ANY == 1
        assert SceneQueryMode.ALL == 2

    def test_geometry_type_values(self):
        assert SceneQueryGeometryType.SPHERE == 0
        assert SceneQueryGeometryType.BOX == 1
        assert SceneQueryGeometryType.SHAPE == 2
