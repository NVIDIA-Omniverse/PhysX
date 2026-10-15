# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# PARTIALLY DEPRECATED (tensor-binding-deprecation): the two create_tensor_binding NUL-path tests retire with the
# binding, and the object-type, SDF view, scene-query and contact-binding ones with those APIs, deprecated as well.
# The session selection and clone NUL tests stay. A session prim_paths selection takes interned handles, not
# strings, so the prim-path case has no session form.

"""Regression coverage for NVBugs 6433621.

Embedded NUL bytes in prim paths must be rejected consistently across
get_object_type, create_tensor_binding, the read/write selection pattern,
create_sdf_view, clone, create_contact_binding, and the scene-query SHAPE
geometry (sweep / overlap).
"""

from __future__ import annotations

import pytest

from ovphysx import SceneQueryGeometryType, SceneQueryMode, TensorType
from ovphysx.types import SimObjectType
from test_utils import data_path, load_usd_with_ovstage

_NUL_PATH = "/World/Cube1\x00GARBAGE"


@pytest.fixture
def boxes_scene(physx_sdk):
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    return physx_sdk


def test_get_object_type_rejects_embedded_nul(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        physx.get_object_type(_NUL_PATH)

    # A nonexistent path without a NUL is INVALID, not an error.
    assert physx.get_object_type("/World/NoSuchShape") == 0


def test_create_tensor_binding_rejects_embedded_nul_pattern(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        physx.create_tensor_binding(
            pattern=_NUL_PATH,
            tensor_type=TensorType.RIGID_BODY_POSE,
        )


# Session counterpart of test_create_tensor_binding_rejects_embedded_nul_pattern's check.
def test_selection_rejects_embedded_nul_pattern(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        with physx.read_sync(SimObjectType.RIGID_BODY, ["position"], pattern=_NUL_PATH):
            pass
    with pytest.raises(RuntimeError, match="embedded NUL"):
        with physx.write_sync(SimObjectType.RIGID_BODY, "position", pattern=_NUL_PATH):
            pass


def test_create_tensor_binding_rejects_embedded_nul_prim_paths(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        physx.create_tensor_binding(
            prim_paths=[_NUL_PATH],
            tensor_type=TensorType.RIGID_BODY_POSE,
        )


def test_create_sdf_view_rejects_embedded_nul(physx_sdk):
    with pytest.warns(DeprecationWarning, match="SDF views are deprecated"):
        with pytest.raises(RuntimeError, match="embedded NUL"):
            physx_sdk.create_sdf_view(pattern=_NUL_PATH, max_query_points=1)


def test_sweep_shape_rejects_embedded_nul(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        physx.sweep(
            geometry_type=SceneQueryGeometryType.SHAPE,
            mode=SceneQueryMode.CLOSEST,
            direction=(0.0, -1.0, 0.0),
            distance=1.0,
            prim_path=_NUL_PATH,
        )


def test_overlap_shape_rejects_embedded_nul(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        physx.overlap(
            geometry_type=SceneQueryGeometryType.SHAPE,
            mode=SceneQueryMode.ALL,
            prim_path=_NUL_PATH,
        )


def test_clone_rejects_embedded_nul_source(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        physx.clone(_NUL_PATH, ["/World/env_clone"])


def test_clone_rejects_embedded_nul_target(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        physx.clone("/World/Cube1", [_NUL_PATH])


def test_create_contact_binding_rejects_embedded_nul_sensor(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        physx.create_contact_binding(sensor_patterns=[_NUL_PATH])


def test_create_contact_binding_rejects_embedded_nul_filter(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        physx.create_contact_binding(
            sensor_patterns=["/World/Cube1"],
            filter_patterns=[_NUL_PATH],
            filters_per_sensor=1,
        )
