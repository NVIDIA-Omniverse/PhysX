# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""CPU-mode pre-step session write (REQ-CAPI-WRITE-001 AC-5a).

On CPU a write before the first step commits and is applied. DirectGPU is the
mode that refuses; that case lives in tests/python_tests/test_write_api.py
(the GPU suite's physx_sdk fixture enables suppressReadback).
"""

# @implements REQ-CAPI-WRITE-001
# @covers AC-5a
# @maps_to TEST-CAPI-WRITE-001

import numpy as np
import pytest
from ovphysx.types import SimObjectType
from test_utils import data_path, load_usd_with_ovstage


def _to_host(t) -> np.ndarray:
    return t if isinstance(t, np.ndarray) else t.numpy()


def _fill(t, block) -> None:
    if isinstance(t, np.ndarray):
        t.reshape(np.asarray(block).shape)[:] = block
        return
    t.assign(np.ascontiguousarray(block).reshape(t.shape))


def _read_positions(physx):
    with physx.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        assert result.groups, "CPU can read authored state before the first step"
        return np.concatenate([_to_host(g.tensors[0]).reshape(g.prim_count, -1) for g in result.groups])


def test_prestep_write_applies_on_cpu(physx_sdk):
    """CPU: commit before the first step succeeds and the values read back (AC-5a)."""
    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    physx_sdk.wait_all()

    with physx_sdk.write_sync(SimObjectType.RIGID_BODY, "position") as w:
        assert w.groups, "expected at least one writable group"
        float_off = 0
        chunks = []
        for g in w.groups:
            n = g.prim_count
            block = np.arange(n * 3, dtype=np.float32).reshape(n, 3) + 100.0 + float_off
            _fill(g.tensors[0], block)
            w.commit_sync(g)
            chunks.append(block)
            float_off += n * 3
    target = np.concatenate(chunks)
    after = _read_positions(physx_sdk)
    np.testing.assert_allclose(after, target, rtol=0, atol=1e-3)


@pytest.mark.parametrize(
    "object_type,attribute,pattern,value",
    [
        (SimObjectType.ARTICULATION, "rootPosition", "/World/articulation2", [2.5, 0.0, 30.0]),
        (SimObjectType.ARTICULATION_LINK, "mass", "/World/articulation2/articulationLink0", [2.0]),
        (SimObjectType.ARTICULATION_JOINT, "jointPosition",
         "/World/articulation2/articulatedRevoluteJoint1", [0.05]),
    ],
    ids=["root", "link", "joint"],
)
def test_articulation_prestep_write_applies_on_cpu(physx_sdk, object_type, attribute, pattern, value):
    """Each articulation family can be the first operation after attach (NVBug 6895491)."""
    load_usd_with_ovstage(physx_sdk, data_path("mixed_base_articulations.usda"))
    physx_sdk.wait_all()

    target = np.asarray(value, dtype=np.float32)
    with physx_sdk.write_sync(object_type, attribute, pattern=pattern, metadata=True) as w:
        assert w.metadata.total_prim_count == 1
        assert w.metadata.attributes[0].group_count == 1
        assert len(w.groups) == 1
        assert w.groups[0].prim_count == 1
        _fill(w.groups[0].tensors[0], target)
        w.commit_sync(w.groups[0])

    # Read immediately: a step or warmup here would hide a missing scene insertion.
    with physx_sdk.read_sync(object_type, [attribute], pattern=pattern) as result:
        assert len(result.groups) == 1
        np.testing.assert_allclose(_to_host(result.groups[0].tensors[0]).reshape(-1), target, rtol=0, atol=1e-5)

    if attribute == "rootPosition":
        # Gravity would have changed this if the insertion had stepped the scene.
        with physx_sdk.read_sync(object_type, ["rootLinearVelocity"], pattern=pattern) as result:
            velocity = _to_host(result.groups[0].tensors[0]).reshape(-1)
            np.testing.assert_array_equal(velocity, np.zeros(3, dtype=np.float32))
        physx_sdk.step_sync(1e-4)
        with physx_sdk.read_sync(object_type, [attribute], pattern=pattern) as result:
            np.testing.assert_allclose(_to_host(result.groups[0].tensors[0]).reshape(-1), target, rtol=0, atol=1e-4)


@pytest.mark.parametrize(
    "object_type,attribute",
    [(SimObjectType.ARTICULATION, name) for name in (
        "rootPosition", "rootOrientation", "rootLinearVelocity", "rootAngularVelocity",
    )] + [(SimObjectType.ARTICULATION_LINK, name) for name in (
        "mass", "inertia", "centerOfMassPosition", "centerOfMassOrientation", "disableGravity",
        "staticFriction", "dynamicFriction", "restitution", "contactOffset", "restOffset", "force", "wrench",
    )] + [(SimObjectType.ARTICULATION_JOINT, name) for name in (
        "jointPosition", "jointVelocity", "jointPositionTarget", "jointVelocityTarget", "jointActuationForce",
        "jointStiffness", "jointDamping", "jointLimit", "jointMaxVelocity", "jointMaxForce", "jointArmature",
        "jointStaticFriction", "jointDynamicFriction", "jointViscousFriction", "jointSpeedEffortGradient",
        "jointMaxActuatorVelocity", "jointVelocityDependentResistance", "jointDriveType",
    )],
)
def test_articulation_prestep_write_plans_all_columns(physx_sdk, object_type, attribute):
    """Every writable root, link and joint column plans one group before the first step."""
    load_usd_with_ovstage(physx_sdk, data_path("mixed_base_articulations.usda"))
    physx_sdk.wait_all()
    with physx_sdk.write_sync(object_type, attribute, metadata=True) as w:
        assert w.metadata.total_prim_count > 0
        assert w.metadata.attributes[0].group_count == 1
        assert len(w.groups) == 1
        assert w.groups[0].prim_count == w.metadata.total_prim_count
