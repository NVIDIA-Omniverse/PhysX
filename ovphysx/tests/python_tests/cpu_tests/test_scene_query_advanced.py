# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
"""Advanced scene query tests: both_sides, overlap CLOSEST rejection, hit field types,
proto_index, result lifetime across queries, and missing kwargs errors.

Does NOT duplicate test_scene_query.py (which covers basic raycast/sweep/overlap,
mode variants, negative/nan/inf distance, zero-distance, SHAPE geometry, and
enum value sync).

Scene: simple_physics_scene.usda
  - Ground plane (flat box) at Y=0
  - Dynamic Cube1 at (0, 5, 0) — falls under gravity
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
        sdk.step_sync(dt)


# ---------------------------------------------------------------------------
# both_sides parameter
# ---------------------------------------------------------------------------


def test_raycast_both_sides_false_kwarg_is_accepted(physx_sdk):
    """raycast() with both_sides=False is accepted and returns a list.

    The scene's ground is a flattened Cube — and backface behaviour for cube
    geometry under the both_sides=False flag is implementation-defined, so this
    test only verifies that the kwarg is honoured at the API surface (no raise,
    returns a list). See test_raycast_both_sides_true_hits_backface for the
    behavioural assertion in the both_sides=True case.
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


def test_raycast_both_sides_true_hits_backface(physx_sdk):
    """Same ray with both_sides=True must hit the ground plane's underside."""
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


def test_sweep_both_sides_true_hits(physx_sdk):
    """Sphere sweep from below with both_sides=True must register a hit."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.sweep(
        geometry_type=SceneQueryGeometryType.SPHERE,
        direction=[0.0, 1.0, 0.0],
        distance=20.0,
        mode=SceneQueryMode.CLOSEST,
        both_sides=True,
        radius=0.3,
        position=[0.0, -5.0, 0.0],
    )
    # Should hit the underside of the ground plane
    assert len(hits) >= 1


# ---------------------------------------------------------------------------
# overlap CLOSEST rejected at Python layer
# ---------------------------------------------------------------------------


def test_overlap_closest_mode_raises_value_error(physx_sdk):
    """overlap(mode=CLOSEST) must raise ValueError (NVBug 6172863)."""
    _load_and_step(physx_sdk)
    geo_kwargs = {"radius": 50.0, "position": [0.0, 0.0, 0.0]}

    with pytest.raises(ValueError, match="overlap\\(\\) does not support SceneQueryMode.CLOSEST"):
        physx_sdk.overlap(
            geometry_type=SceneQueryGeometryType.SPHERE,
            mode=SceneQueryMode.CLOSEST,
            **geo_kwargs,
        )


# ---------------------------------------------------------------------------
# Multiple hits in ALL mode
# ---------------------------------------------------------------------------


def test_raycast_all_mode_returns_multiple_hits(physx_sdk):
    """raycast ALL mode must return all hits along the ray (>= 1 for a loaded scene)."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.raycast(
        origin=[0.0, 100.0, 0.0],
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
        mode=SceneQueryMode.ALL,
    )
    assert isinstance(hits, list)
    assert len(hits) >= 1


# ---------------------------------------------------------------------------
# Hit dict field types
# ---------------------------------------------------------------------------


def test_hit_dict_field_types(physx_sdk):
    """Every field in a raycast hit dict must have the expected Python type."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.raycast(
        origin=[0.0, 100.0, 0.0],
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
        mode=SceneQueryMode.CLOSEST,
    )
    assert len(hits) >= 1
    h = hits[0]

    assert isinstance(h["collision"], int), f"collision must be int, got {type(h['collision'])}"
    assert isinstance(h["rigid_body"], int), f"rigid_body must be int, got {type(h['rigid_body'])}"
    assert isinstance(h["proto_index"], int), f"proto_index must be int, got {type(h['proto_index'])}"
    # The API returns 3-element sequences; accept both list and tuple.
    assert isinstance(h["normal"], (list, tuple)), f"normal must be list or tuple, got {type(h['normal'])}"
    assert len(h["normal"]) == 3
    assert isinstance(h["position"], (list, tuple)), f"position must be list or tuple, got {type(h['position'])}"
    assert len(h["position"]) == 3
    assert isinstance(h["distance"], float), f"distance must be float, got {type(h['distance'])}"
    assert isinstance(h["face_index"], int), f"face_index must be int, got {type(h['face_index'])}"
    assert isinstance(h["material"], int), f"material must be int, got {type(h['material'])}"
    # All entries inside normal and position must be numeric
    for val in list(h["normal"]) + list(h["position"]):
        assert isinstance(val, (int, float))


# ---------------------------------------------------------------------------
# Overlap location fields are zeroed
# ---------------------------------------------------------------------------


def test_overlap_location_fields_are_zeroed(physx_sdk):
    """Overlap results must have location fields (normal, position, distance,
    face_index, material) set to zero / 0."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.overlap(
        geometry_type=SceneQueryGeometryType.SPHERE,
        mode=SceneQueryMode.ALL,
        radius=50.0,
        position=[0.0, 0.0, 0.0],
    )
    assert len(hits) >= 1
    _zero3 = (0.0, 0.0, 0.0)
    for h in hits:
        # API returns tuples for vector fields; compare element-wise.
        assert tuple(h["normal"]) == _zero3, f"normal should be zeroed, got {h['normal']}"
        assert tuple(h["position"]) == _zero3, f"position should be zeroed, got {h['position']}"
        assert h["distance"] == 0.0, f"distance should be 0, got {h['distance']}"
        assert h["face_index"] == 0, f"face_index should be 0, got {h['face_index']}"
        assert h["material"] == 0, f"material should be 0, got {h['material']}"


# ---------------------------------------------------------------------------
# proto_index for non-instancer prims
# ---------------------------------------------------------------------------


def test_non_instancer_proto_index_is_max_uint32(physx_sdk):
    """Raycast hit on a regular (non-PointInstancer) rigid body must have
    proto_index == 0xFFFFFFFF (4294967295)."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.raycast(
        origin=[0.0, 100.0, 0.0],
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
        mode=SceneQueryMode.CLOSEST,
    )
    assert len(hits) >= 1
    assert hits[0]["proto_index"] == 0xFFFFFFFF, (
        f"Expected proto_index=0xFFFFFFFF for non-instancer prim, " f"got {hits[0]['proto_index']}"
    )


# ---------------------------------------------------------------------------
# Missing kwargs errors
# ---------------------------------------------------------------------------


def test_sweep_sphere_missing_radius_raises(physx_sdk):
    """sweep(SPHERE, ...) without 'radius' kwarg must raise an error.

    The API raises KeyError when a required geometry parameter is absent
    (_make_geometry_desc only guards against *unexpected* kwargs, not missing
    ones), so accept both TypeError and KeyError.
    """
    _load_and_step(physx_sdk)
    with pytest.raises((TypeError, KeyError)):
        physx_sdk.sweep(
            geometry_type=SceneQueryGeometryType.SPHERE,
            direction=[0.0, -1.0, 0.0],
            distance=10.0,
            mode=SceneQueryMode.CLOSEST,
            # radius intentionally omitted
            position=[0.0, 10.0, 0.0],
        )


def test_sweep_box_missing_half_extent_raises(physx_sdk):
    """sweep(BOX, ...) without 'half_extent' kwarg must raise an error.

    Same as SPHERE: missing required param propagates as KeyError from the
    geometry descriptor builder, so accept both TypeError and KeyError.
    """
    _load_and_step(physx_sdk)
    with pytest.raises((TypeError, KeyError)):
        physx_sdk.sweep(
            geometry_type=SceneQueryGeometryType.BOX,
            direction=[0.0, -1.0, 0.0],
            distance=10.0,
            mode=SceneQueryMode.CLOSEST,
            # half_extent intentionally omitted
            position=[0.0, 10.0, 0.0],
            rotation=[0.0, 0.0, 0.0, 1.0],
        )


# ---------------------------------------------------------------------------
# Result lifetime across successive queries
# ---------------------------------------------------------------------------


def test_raycast_results_survive_subsequent_query(physx_sdk):
    """Earlier raycast hit dicts must not be overwritten by a later query.

    Python copies hit fields into owned dicts before returning. The C runtime
    reuses an internal per-instance buffer, but Python callers must not see
    stale data when retaining results across calls.
    """
    _load_and_step(physx_sdk)
    hits_a = physx_sdk.raycast(
        origin=[0.0, 100.0, 0.0],
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
        mode=SceneQueryMode.CLOSEST,
    )
    assert len(hits_a) >= 1
    distance_a = hits_a[0]["distance"]
    normal_a = hits_a[0]["normal"]
    position_a = hits_a[0]["position"]

    hits_b = physx_sdk.raycast(
        origin=[5.0, 100.0, 0.0],
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
        mode=SceneQueryMode.CLOSEST,
    )
    assert len(hits_b) >= 1
    assert hits_a[0]["distance"] == distance_a
    assert hits_a[0]["normal"] == normal_a
    assert hits_a[0]["position"] == position_a


# ---------------------------------------------------------------------------
# Default mode
# ---------------------------------------------------------------------------


def test_raycast_default_mode_is_closest(physx_sdk):
    """raycast() without a mode argument must return at most 1 hit (CLOSEST default)."""
    _load_and_step(physx_sdk)
    hits = physx_sdk.raycast(
        origin=[0.0, 100.0, 0.0],
        direction=[0.0, -1.0, 0.0],
        distance=200.0,
    )
    assert isinstance(hits, list)
    assert len(hits) <= 1
