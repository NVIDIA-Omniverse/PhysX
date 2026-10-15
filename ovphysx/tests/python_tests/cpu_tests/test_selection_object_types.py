# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-SELECT-001
# @covers AC-7
# @maps_to TEST-CAPI-SELECT-001

"""Which objects of each type a prim-set selection selects, pattern by pattern and path by path.

An articulation query selects an articulation through its root, any link or any joint, as the articulation
binding does (the deprecated tensor binding is checked against the same table). The member types
(links, joints, standalone bodies) select only the objects whose own prim the selection names. The fixture
(tests/data/articulation_selection.usda) has RobotA, rooted on a transform above its links with a fixed
base, RobotB, rooted on its root link and floating, and a standalone box.
"""

import warnings

import ovstage
import pytest
from ovphysx.types import SimObjectType, TensorType
from test_utils import attach_usd_with_ovstage, data_path

_ROBOT_A = "/World/RobotA"
_ROBOT_B = "/World/RobotB/base_link"

# (pattern, selected articulations). RobotA's fixed root joint, the plain RobotB transform above RobotB's
# root link, and the standalone box select nothing.
_PATTERNS = [
    ("/World/RobotA", [_ROBOT_A]),
    ("/World/RobotA/base_link", [_ROBOT_A]),
    ("/World/RobotA/link_0", [_ROBOT_A]),
    ("/World/RobotA/joint_0", [_ROBOT_A]),
    ("/World/RobotA/rootJoint", []),
    ("/World/RobotB", []),
    ("/World/RobotB/base_link", [_ROBOT_B]),
    ("/World/RobotB/link_0", [_ROBOT_B]),
    ("/World/RobotB/joint_0", [_ROBOT_B]),
    ("/World/Robot*", [_ROBOT_A]),
    ("/World/Robot*/*", [_ROBOT_A, _ROBOT_B]),
    ("/World/Robot*/**", [_ROBOT_A, _ROBOT_B]),
    ("/World/*/link_0", [_ROBOT_A, _ROBOT_B]),
    ("/World/*/joint_0", [_ROBOT_A, _ROBOT_B]),
    ("/World/box", []),
]

# Links and joints of both robots, each selecting its articulation when named explicitly.
_MEMBERS = [
    ("/World/RobotA/base_link", _ROBOT_A),
    ("/World/RobotA/link_0", _ROBOT_A),
    ("/World/RobotA/joint_0", _ROBOT_A),
    ("/World/RobotB/link_0", _ROBOT_B),
    ("/World/RobotB/joint_0", _ROBOT_B),
]

_ATTRIBUTE = {
    SimObjectType.ARTICULATION: "rootPosition",
    SimObjectType.ARTICULATION_LINK: "position",
    SimObjectType.ARTICULATION_JOINT: "jointPosition",
    SimObjectType.RIGID_BODY: "position",
}


def _read_paths(sdk, stage, object_type, *, pattern=None, paths=None):
    """Sorted prim paths a selected read of `object_type` covers; `paths` are interned to native handles."""
    out = []
    with ovstage.PathDictionary(stage) as dictionary:
        handles = [dictionary.intern_path(p) for p in paths] if paths else None
        with sdk.read_sync(object_type, [_ATTRIBUTE[object_type]], prim_paths=handles, pattern=pattern) as result:
            for group in result.groups:
                listed = dictionary.get_paths(group.prim_list)
                out.extend(
                    dictionary.path_to_string(h)
                    for h in listed[group.prim_offset : group.prim_offset + group.prim_count]
                )
    return sorted(out)


def _articulation_binding_paths(sdk, *, pattern=None, paths=None):
    """Sorted articulation paths an articulation tensor binding over the same selection covers."""
    # The tensor binding is deprecated; this parity check is an intentional compat use.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        binding = sdk.create_tensor_binding(
            pattern=pattern, prim_paths=paths, tensor_type=TensorType.ARTICULATION_ROOT_POSE
        )
        try:
            return sorted(binding.prim_paths)
        finally:
            binding.destroy()


@pytest.fixture
def articulation_scene(physx_sdk_cpu):
    stage = attach_usd_with_ovstage(physx_sdk_cpu, data_path("articulation_selection.usda"))
    physx_sdk_cpu.step(1.0 / 60.0)
    physx_sdk_cpu.wait_all()
    return physx_sdk_cpu, stage


def test_articulation_selection_matches_the_articulation_binding(articulation_scene):
    """A pattern or path list selects exactly the articulations the table lists for it, as an articulation
    binding over the same selection does."""
    sdk, stage = articulation_scene
    assert _read_paths(sdk, stage, SimObjectType.ARTICULATION, pattern="/World/**") == [
        "/World/RobotA",
        "/World/RobotB/base_link",
    ]
    mismatches = []
    for pattern, expected in _PATTERNS:
        read = _read_paths(sdk, stage, SimObjectType.ARTICULATION, pattern=pattern)
        binding = _articulation_binding_paths(sdk, pattern=pattern)
        if read != expected or binding != expected:
            mismatches.append(f"pattern {pattern!r}: read {read}, binding {binding}, expected {expected}")
    for path, articulation in _MEMBERS:
        read = _read_paths(sdk, stage, SimObjectType.ARTICULATION, paths=[path])
        binding = _articulation_binding_paths(sdk, paths=[path])
        if read != [articulation] or binding != [articulation]:
            mismatches.append(f"path {path!r}: read {read}, binding {binding}, expected {[articulation]}")
    assert not mismatches, "\n".join(mismatches)


def test_member_types_select_only_their_own_objects(articulation_scene):
    """Link, joint and rigid-body selections are not widened to the rest of an articulation."""
    sdk, stage = articulation_scene
    link, joint, rigid = SimObjectType.ARTICULATION_LINK, SimObjectType.ARTICULATION_JOINT, SimObjectType.RIGID_BODY
    assert _read_paths(sdk, stage, link, pattern="/World/*/link_0") == ["/World/RobotA/link_0", "/World/RobotB/link_0"]
    assert _read_paths(sdk, stage, link, pattern="/World/RobotA") == []
    assert _read_paths(sdk, stage, link, paths=["/World/RobotA/joint_0"]) == []
    joints = ["/World/RobotA/joint_0", "/World/RobotB/joint_0"]
    assert _read_paths(sdk, stage, joint, pattern="/World/Robot*/*") == joints
    assert _read_paths(sdk, stage, joint, pattern="/World/RobotA/link_0") == []
    assert _read_paths(sdk, stage, rigid, pattern="/World/**") == ["/World/box"]
