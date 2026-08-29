# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""GPU-mode scene query tests: raycast, sweep, and overlap.

No scene query tests exist for GPU mode. This file mirrors the structure of
cpu_tests/test_scene_query.py and cpu_tests/test_scene_query_advanced.py but
verifies that queries work correctly when the PhysX instance is running in
GPU / DirectGPU mode.

Scene: simple_physics_scene.usda
  - Ground plane (Cube collider scaled 100×1×100) at Y=0
  - Dynamic Cube1 at (0, 5, 0) — falls under gravity
"""

import pytest
from ovphysx.types import SceneQueryGeometryType, SceneQueryMode
from test_utils import data_path
from test_utils import load_usd_with_ovstage


def _load_and_step(sdk, n_steps=10, dt=1.0 / 60.0):
    load_usd_with_ovstage(sdk, data_path("simple_physics_scene.usda"))
    sdk.wait_all()
    for _ in range(n_steps):
        sdk.step_sync(dt)


# ---------------------------------------------------------------------------
# Raycast
# ---------------------------------------------------------------------------


def test_raycast_closest_gpu(physx_sdk):
    """Downward raycast in GPU mode must hit the ground plane."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.raycast(
        origin=[0.0, 100.0, 0.0],
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
        mode=SceneQueryMode.CLOSEST,
    )
    assert len(hits) == 1
    assert hits[0]["distance"] > 0.0


def test_raycast_any_gpu(physx_sdk):
    """ANY mode in GPU mode returns 0 or 1 hit without error."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.raycast(
        origin=[0.0, 100.0, 0.0],
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
        mode=SceneQueryMode.ANY,
    )
    assert isinstance(hits, list)
    assert len(hits) <= 1


def test_raycast_all_gpu(physx_sdk):
    """ALL mode in GPU mode returns a list of >= 1 hits."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.raycast(
        origin=[0.0, 100.0, 0.0],
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
        mode=SceneQueryMode.ALL,
    )
    assert isinstance(hits, list)
    assert len(hits) >= 1


def test_raycast_miss_gpu(physx_sdk):
    """Ray pointing away from all scene geometry must return 0 hits on GPU."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.raycast(
        origin=[0.0, 100.0, 0.0],
        direction=[0.0, 1.0, 0.0],  # upward — away from ground
        distance=10.0,
        mode=SceneQueryMode.CLOSEST,
    )
    assert len(hits) == 0


def test_raycast_both_sides_false_kwarg_is_accepted_gpu(physx_sdk):
    """raycast() with both_sides=False is accepted on the GPU path and returns a list.

    The scene's ground is a flattened Cube — and backface behaviour for cube
    geometry under the both_sides=False flag is implementation-defined, so this
    test only verifies that the kwarg is honoured at the API surface (no raise,
    returns a list). See test_raycast_both_sides_true_gpu for the behavioural
    assertion in the both_sides=True case.
    """
    _load_and_step(physx_sdk)
    hits = physx_sdk.raycast(
        origin=[0.0, -5.0, 0.0],
        direction=[0.0, 1.0, 0.0],
        distance=20.0,
        mode=SceneQueryMode.CLOSEST,
        both_sides=False,
    )
    assert isinstance(hits, list)


def test_raycast_both_sides_true_gpu(physx_sdk):
    """Ray from below with both_sides=True must hit the underside of ground plane."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.raycast(
        origin=[0.0, -5.0, 0.0],
        direction=[0.0, 1.0, 0.0],
        distance=20.0,
        mode=SceneQueryMode.CLOSEST,
        both_sides=True,
    )
    assert len(hits) == 1
    assert hits[0]["distance"] > 0.0


# ---------------------------------------------------------------------------
# Sweep
# ---------------------------------------------------------------------------


def test_sweep_sphere_gpu(physx_sdk):
    """Downward sphere sweep must hit scene geometry in GPU mode."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.sweep(
        geometry_type=SceneQueryGeometryType.SPHERE,
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
        mode=SceneQueryMode.CLOSEST,
        radius=0.5,
        position=[0.0, 50.0, 0.0],
    )
    assert len(hits) >= 1
    assert hits[0]["distance"] > 0.0


def test_sweep_box_gpu(physx_sdk):
    """Downward box sweep must hit scene geometry in GPU mode."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.sweep(
        geometry_type=SceneQueryGeometryType.BOX,
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
        mode=SceneQueryMode.CLOSEST,
        half_extent=[0.5, 0.5, 0.5],
        position=[0.0, 50.0, 0.0],
        rotation=[0.0, 0.0, 0.0, 1.0],
    )
    assert len(hits) >= 1


# ---------------------------------------------------------------------------
# Overlap
# ---------------------------------------------------------------------------


def test_overlap_sphere_gpu(physx_sdk):
    """Sphere overlap in GPU mode must return >= 1 hits."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.overlap(
        geometry_type=SceneQueryGeometryType.SPHERE,
        mode=SceneQueryMode.ALL,
        radius=50.0,
        position=[0.0, 0.0, 0.0],
    )
    assert isinstance(hits, list)
    assert len(hits) >= 1


def test_overlap_closest_raises_value_error_gpu(physx_sdk):
    """overlap(mode=CLOSEST) must raise ValueError on GPU (NVBug 6172863)."""
    _load_and_step(physx_sdk)
    geo_kwargs = {"radius": 50.0, "position": [0.0, 0.0, 0.0]}
    with pytest.raises(ValueError, match="overlap\\(\\) does not support SceneQueryMode.CLOSEST"):
        physx_sdk.overlap(
            geometry_type=SceneQueryGeometryType.SPHERE,
            mode=SceneQueryMode.CLOSEST,
            **geo_kwargs,
        )


# ---------------------------------------------------------------------------
# Hit field types on GPU
# ---------------------------------------------------------------------------


def test_hit_dict_field_types_gpu(physx_sdk):
    """Every field in a GPU raycast hit dict must have the expected Python type."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.raycast(
        origin=[0.0, 100.0, 0.0],
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
        mode=SceneQueryMode.CLOSEST,
    )
    assert len(hits) >= 1
    h = hits[0]
    assert isinstance(h["collision"], int)
    assert isinstance(h["rigid_body"], int)
    assert isinstance(h["proto_index"], int)
    # The API returns 3-element sequences; accept both list and tuple.
    assert isinstance(h["normal"], (list, tuple)) and len(h["normal"]) == 3
    assert isinstance(h["position"], (list, tuple)) and len(h["position"]) == 3
    assert isinstance(h["distance"], float)
    assert isinstance(h["face_index"], int)
    assert isinstance(h["material"], int)


# ---------------------------------------------------------------------------
# Missing kwargs errors on GPU
# ---------------------------------------------------------------------------


def test_sweep_missing_radius_raises_gpu(physx_sdk):
    """sweep(SPHERE, ...) without 'radius' kwarg must raise an error on GPU.

    The API raises KeyError when the required geometry parameter is absent
    (the TypeError guard in _make_geometry_desc only fires for *unexpected*
    kwargs, not missing ones).
    """
    _load_and_step(physx_sdk)
    with pytest.raises((TypeError, KeyError)):
        physx_sdk.sweep(
            geometry_type=SceneQueryGeometryType.SPHERE,
            direction=[0.0, -1.0, 0.0],
            distance=10.0,
            mode=SceneQueryMode.CLOSEST,
            position=[0.0, 50.0, 0.0],
            # radius omitted
        )
