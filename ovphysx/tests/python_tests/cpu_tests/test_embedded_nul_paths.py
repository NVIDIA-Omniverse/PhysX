# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Regression coverage for NVBugs 6433621.

Embedded NUL bytes in prim paths must be rejected consistently across
get_object_type, create_tensor_binding, create_sdf_view, clone,
create_contact_binding, and the scene-query SHAPE geometry (sweep / overlap).
"""

from __future__ import annotations

import pytest

from ovphysx import SceneQueryGeometryType, SceneQueryMode, TensorType
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

    # Control: nonexistent path without NUL is INVALID, not an error.
    assert physx.get_object_type("/World/NoSuchShape") == 0


def test_create_tensor_binding_rejects_embedded_nul_pattern(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        physx.create_tensor_binding(
            pattern=_NUL_PATH,
            tensor_type=TensorType.RIGID_BODY_POSE,
        )


def test_create_tensor_binding_rejects_embedded_nul_prim_paths(boxes_scene):
    physx = boxes_scene

    with pytest.raises(RuntimeError, match="embedded NUL"):
        physx.create_tensor_binding(
            prim_paths=[_NUL_PATH],
            tensor_type=TensorType.RIGID_BODY_POSE,
        )


def test_create_sdf_view_rejects_embedded_nul(physx_sdk):
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
