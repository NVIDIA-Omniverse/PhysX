# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-WRITE-001
# @covers AC-9
# @maps_to TEST-CAPI-WRITE-001
# @implements REQ-INPUT-COVERAGE-001
# @covers AC-8
# @maps_to TEST-INPUT-COVERAGE-001

"""Session-API read-only / write-only access-mode enforcement: read-only attributes refuse a write, write-only attributes take a write but emit no read column."""

import os

import pytest
from ovphysx.types import SimObjectType
from session_helpers import read_column, write_is_refused
from test_utils import attach_usd_with_ovstage

_TEST_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def data_path(filename):
    return os.path.join(_TEST_DIR, "data", filename)


# Rigid body prims in boxes_falling_on_groundplane.usda
_RB_PATTERN = "/World/Cube*"
# Articulation pattern in two_articulations.usda
_ARTI_PATTERN = "/World/articulation*"
# Their links and joints: a bare last `*` matches direct children only.
_ARTI_MEMBER_PATTERN = "/World/articulation*/*"


def _load_rb(sdk, n_steps=3):
    stage = attach_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    sdk.wait_all()
    for _ in range(n_steps):
        sdk.step_sync(1.0 / 60.0)
    return stage


def _load_artic(sdk, n_steps=3):
    stage = attach_usd_with_ovstage(sdk, data_path("two_articulations.usda"))
    sdk.wait_all()
    for _ in range(n_steps):
        sdk.step_sync(1.0 / 60.0)
    return stage


# ---------------------------------------------------------------------------
# Read-only attributes: write must raise
# ---------------------------------------------------------------------------

# (object_type, attribute, pattern, scene_loader)
_READ_ONLY_PARAMS = [
    # Rigid body read-only
    (SimObjectType.RIGID_BODY, "linearAcceleration", _RB_PATTERN, "rb"),
    (SimObjectType.RIGID_BODY, "angularAcceleration", _RB_PATTERN, "rb"),
    (SimObjectType.RIGID_BODY, "inverseMass", _RB_PATTERN, "rb"),
    (SimObjectType.RIGID_BODY, "inverseInertia", _RB_PATTERN, "rb"),
    # Articulation link read-only
    (SimObjectType.ARTICULATION_LINK, "position", _ARTI_MEMBER_PATTERN, "artic"),
    (SimObjectType.ARTICULATION_LINK, "orientation", _ARTI_MEMBER_PATTERN, "artic"),
    (SimObjectType.ARTICULATION_LINK, "linearVelocity", _ARTI_MEMBER_PATTERN, "artic"),
    (SimObjectType.ARTICULATION_LINK, "angularVelocity", _ARTI_MEMBER_PATTERN, "artic"),
    (SimObjectType.ARTICULATION_LINK, "linearAcceleration", _ARTI_MEMBER_PATTERN, "artic"),
    (SimObjectType.ARTICULATION_LINK, "angularAcceleration", _ARTI_MEMBER_PATTERN, "artic"),
    # Articulation body inverse
    (SimObjectType.ARTICULATION_LINK, "inverseMass", _ARTI_MEMBER_PATTERN, "artic"),
    (SimObjectType.ARTICULATION_LINK, "inverseInertia", _ARTI_MEMBER_PATTERN, "artic"),
    # Inverse dynamics queries
    (SimObjectType.ARTICULATION, "jacobian", _ARTI_PATTERN, "artic"),
    (SimObjectType.ARTICULATION, "massMatrix", _ARTI_PATTERN, "artic"),
    (SimObjectType.ARTICULATION, "coriolisForce", _ARTI_PATTERN, "artic"),
    (SimObjectType.ARTICULATION, "gravityForce", _ARTI_PATTERN, "artic"),
    (SimObjectType.ARTICULATION_LINK, "linkIncomingJointForce", _ARTI_MEMBER_PATTERN, "artic"),
    (SimObjectType.ARTICULATION_JOINT, "jointProjectedForce", _ARTI_MEMBER_PATTERN, "artic"),
]

_READ_ONLY_IDS = [f"{t[0].name}-{t[1]}" for t in _READ_ONLY_PARAMS]


@pytest.mark.parametrize("object_type,attribute,pattern,scene", _READ_ONLY_PARAMS, ids=_READ_ONLY_IDS)
def test_read_only_type_write_raises(physx_sdk, object_type, attribute, pattern, scene):
    """Opening a session write of a read-only attribute must raise RuntimeError."""
    if scene == "rb":
        stage = _load_rb(physx_sdk)
    else:
        stage = _load_artic(physx_sdk)

    assert read_column(physx_sdk, stage, object_type, attribute, pattern=pattern), \
        f"{object_type.name} {attribute} read is empty; a write refusal could not be told from a missing name"
    assert write_is_refused(physx_sdk, object_type, attribute, pattern=pattern)


# ---------------------------------------------------------------------------
# Write-only attributes: no read-back
# ---------------------------------------------------------------------------

_WRITE_ONLY_PARAMS = [
    (SimObjectType.RIGID_BODY, "force", _RB_PATTERN, "rb"),
    (SimObjectType.RIGID_BODY, "wrench", _RB_PATTERN, "rb"),
    (SimObjectType.ARTICULATION_LINK, "wrench", _ARTI_MEMBER_PATTERN, "artic"),
]
_WRITE_ONLY_IDS = [f"{t[0].name}-{t[1]}" for t in _WRITE_ONLY_PARAMS]


@pytest.mark.parametrize("object_type,attribute,pattern,scene", _WRITE_ONLY_PARAMS, ids=_WRITE_ONLY_IDS)
def test_write_only_type_read_raises(physx_sdk, object_type, attribute, pattern, scene):
    """A write-only attribute has no read-back: the session takes its write, and its read emits no column."""
    if scene == "rb":
        stage = _load_rb(physx_sdk)
    else:
        stage = _load_artic(physx_sdk)

    assert read_column(physx_sdk, stage, object_type, "position", pattern=pattern), \
        f"{object_type.name} has no prims matching '{pattern}'"
    # The accepted write tells a write-only attribute apart from one the type does not know.
    assert not write_is_refused(physx_sdk, object_type, attribute, pattern=pattern)
    assert read_column(physx_sdk, stage, object_type, attribute, pattern=pattern) == {}
