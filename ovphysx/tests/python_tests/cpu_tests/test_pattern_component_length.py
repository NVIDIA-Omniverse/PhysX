# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-PATTERN-001
# @covers AC-1 AC-2 AC-3

# PARTIALLY DEPRECATED (tensor-binding-deprecation): the create_tensor_binding cases retire with
# the binding; the SDF view and contact binding cases stay.

"""Regression coverage for NVBugs 6714840.

A path pattern component longer than the runtime's limit used to crash the process inside the
regex compile. Every pattern-taking entry point now rejects it as an argument error, and a
component at exactly the limit still works.
"""

from __future__ import annotations

import pytest

from ovphysx import TensorType
from test_utils import data_path, load_usd_with_ovstage

_LIMIT = 4096
_LIMIT_MESSAGE = f"longer than {_LIMIT} characters"
_OVERSIZED_LITERAL = "/World/" + "a" * 65536
_OVERSIZED_GLOB = "/World/" + "?" * 65536
_AT_LIMIT = "/World/" + "a" * _LIMIT
# A parenthesized group is one component to the matcher even when it spans a '/'.
_OVERSIZED_GROUP = "/World/(" + "a" * 3000 + "/" + "b" * 3000 + "|Cube1)"
_SHORT_GROUP_WITH_SLASH = "/World/(a/b|Cube1)"


@pytest.fixture
def boxes_scene(physx_sdk):
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    return physx_sdk


# Explicit ids: pytest would otherwise embed the 65536-character value in the node id, which it
# exports through PYTEST_CURRENT_TEST and Windows caps environment variables at 32767 characters.
@pytest.mark.parametrize("pattern", [_OVERSIZED_LITERAL, _OVERSIZED_GLOB], ids=["literal", "glob"])
def test_create_tensor_binding_rejects_oversized_pattern_component(boxes_scene, pattern):
    with pytest.raises(RuntimeError, match=_LIMIT_MESSAGE):
        boxes_scene.create_tensor_binding(pattern=pattern, tensor_type=TensorType.RIGID_BODY_POSE)


def test_create_tensor_binding_rejects_oversized_prim_path_component(boxes_scene):
    with pytest.raises(RuntimeError, match=_LIMIT_MESSAGE):
        boxes_scene.create_tensor_binding(
            prim_paths=[_OVERSIZED_LITERAL], tensor_type=TensorType.RIGID_BODY_POSE
        )


def test_create_tensor_binding_accepts_component_at_limit(boxes_scene):
    binding = boxes_scene.create_tensor_binding(pattern=_AT_LIMIT, tensor_type=TensorType.RIGID_BODY_POSE)
    try:
        assert binding.count == 0
    finally:
        binding.destroy()


def test_create_tensor_binding_rejects_oversized_group_component(boxes_scene):
    # Neither '/'-split half exceeds the limit; the whole group does, and that is what the
    # runtime would have to compile.
    with pytest.raises(RuntimeError, match=_LIMIT_MESSAGE):
        boxes_scene.create_tensor_binding(pattern=_OVERSIZED_GROUP, tensor_type=TensorType.RIGID_BODY_POSE)


def test_create_tensor_binding_accepts_short_group_with_slash(boxes_scene):
    binding = boxes_scene.create_tensor_binding(
        pattern=_SHORT_GROUP_WITH_SLASH, tensor_type=TensorType.RIGID_BODY_POSE
    )
    try:
        assert binding.count == 1
    finally:
        binding.destroy()


def test_create_sdf_view_rejects_oversized_pattern_component(physx_sdk):
    with pytest.raises(RuntimeError, match=_LIMIT_MESSAGE):
        physx_sdk.create_sdf_view(pattern=_OVERSIZED_LITERAL, max_query_points=1)


def test_create_contact_binding_rejects_oversized_sensor_pattern_component(boxes_scene):
    with pytest.raises(RuntimeError, match=_LIMIT_MESSAGE):
        boxes_scene.create_contact_binding([_OVERSIZED_LITERAL])


def test_create_contact_binding_rejects_oversized_filter_pattern_component(boxes_scene):
    with pytest.raises(RuntimeError, match=_LIMIT_MESSAGE):
        boxes_scene.create_contact_binding(["/World/Cube1"], [_OVERSIZED_LITERAL], filters_per_sensor=1)
