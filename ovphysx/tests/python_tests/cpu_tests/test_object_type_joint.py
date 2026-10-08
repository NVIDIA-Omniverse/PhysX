# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-OBJECTTYPE-001
# @covers AC-1 AC-2 AC-3 AC-4
# @maps_to TEST-CAPI-OBJECTTYPE-001

"""Regression coverage for NVBugs 6560084.

Standalone maximal-coordinate joints must classify as ObjectType.JOINT; custom
joints as ObjectType.CUSTOM_JOINT; both distinct from INVALID (missing prim) and
from ARTICULATION_JOINT.
"""

from __future__ import annotations

import pytest

from ovphysx.types import ObjectType
from test_utils import data_path, load_usd_with_ovstage


# The classification is schema-independent, so both standalone-joint scenes run
# the same checks.
@pytest.mark.parametrize(
    "usda, joint_path, body_path",
    [
        ("standalone_prismatic_joint.usda", "/World/Anchor_Slide", "/World/Slider"),
        ("revolute_joint_scene.usda", "/World/revoluteJoint", "/World/box1"),
    ],
)
def test_standalone_joint_classifies_as_joint(physx_sdk_cpu, usda, joint_path, body_path):
    load_usd_with_ovstage(physx_sdk_cpu, data_path(usda))

    assert physx_sdk_cpu.get_object_type(joint_path) == ObjectType.JOINT
    assert physx_sdk_cpu.get_object_type(body_path) == ObjectType.RIGID_BODY
    assert physx_sdk_cpu.get_object_type("/World/Does_Not_Exist") == ObjectType.INVALID


def test_articulation_joint_stays_articulation_joint(physx_sdk_cpu):
    load_usd_with_ovstage(physx_sdk_cpu, data_path("links_chain_sample.usda"))

    assert physx_sdk_cpu.get_object_type("/World/articulation/articulatedRevoluteJoint1") == (
        ObjectType.ARTICULATION_JOINT
    )


def test_object_type_joint_enum_values():
    assert int(ObjectType.JOINT) == 6
    assert int(ObjectType.CUSTOM_JOINT) == 7
    assert ObjectType.CUSTOM_JOINT != ObjectType.JOINT
    assert ObjectType.CUSTOM_JOINT != ObjectType.INVALID
