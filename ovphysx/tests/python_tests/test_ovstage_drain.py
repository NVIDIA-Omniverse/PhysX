# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Drain fan-out: a value authored into ovstage and drained via update_from_ovstage is applied
through the write backend (applyOvstageValueBatch) and reads back, across every write-backed object
type the change feed delivers: rigid velocity, articulation DOF, particle sets. Two families are
deliberately NOT scattered by the drain because they are coupled/structural, not self-contained fields:
the rigid MASS FRAME (mass/centerOfMass/inertia, which the parse path recomputes holistically) and a
deformable's `deformablePose` BIND POSE (the parse path re-cooks it). Those cases assert the drain
leaves the live state untouched (the coupled recompute / re-cook lands at the next step instead).

The read-back is the oracle: on the GPU (DirectGPU) fixture the legacy per-object host setter is inert,
so a correct read-back is attributable to the backend drain alone.
"""

# @implements REQ-SIM-OVSTAGE-WRITEAPPLY-001
# @covers AC-1 AC-2 AC-6 AC-7 AC-8 AC-13
# @maps_to TEST-SIM-OVSTAGE-WRITEAPPLY-001

import numpy as np
import ovstage
from ovstage._src.dlpack import DLDataType, DLDataTypeCode, make_dltensor
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import attach_usd_with_ovstage, data_path


def _drain_scalar(physx, stage, path, attr, value, ordinal):
    """Author a fixed-size scalar/vec attribute and drain it."""
    pd = ovstage.PathDictionary(stage)
    q = stage.query_from_path_list(pd.create_path_list_from_strings([path]))
    stage.write_attribute(q, attr, ordinal, np.asarray(value, np.float32).reshape(1, -1),
                          is_array=False, prim_mode=ovstage.PrimMode.UPSERT).wait()
    q.release()
    stage.advance_write_floor(ordinal=ordinal).wait()
    physx.update_from_ovstage(ordinal, ordinal)
    physx.wait_all()


def _drain_scalar_multi(physx, stage, path_values, attr, ordinal):
    """Author `attr` on several prims at ONE ordinal (the feed then delivers a single batch carrying every
    key) and drain once."""
    pd = ovstage.PathDictionary(stage)
    for path, value in path_values:
        q = stage.query_from_path_list(pd.create_path_list_from_strings([path]))
        stage.write_attribute(q, attr, ordinal, np.asarray(value, np.float32).reshape(1, -1),
                              is_array=False, prim_mode=ovstage.PrimMode.UPSERT).wait()
        q.release()
    stage.advance_write_floor(ordinal=ordinal).wait()
    physx.update_from_ovstage(ordinal, ordinal)
    physx.wait_all()


def _drain_array(physx, stage, path, attr, rows, ordinal):
    """Author a ragged per-prim vec3 array (POINT semantic -> lanes 3) and drain it."""
    vec3 = DLDataType()
    vec3.code, vec3.bits, vec3.lanes = DLDataTypeCode.kDLFloat, 32, 3
    arr = np.ascontiguousarray(np.asarray(rows, np.float32))
    t = make_dltensor(arr, dtype=vec3, shape=[arr.shape[0]], ndim=1)
    pd = ovstage.PathDictionary(stage)
    q = stage.query_from_path_list(pd.create_path_list_from_strings([path]))
    stage.write_attribute(q, attr, ordinal, [t], is_array=True,
                          prim_mode=ovstage.PrimMode.UPSERT, semantic=int(ovstage.AttributeSemantic.POINT)).wait()
    q.release()
    stage.advance_write_floor(ordinal=ordinal).wait()
    physx.update_from_ovstage(ordinal, ordinal)
    physx.wait_all()


def _read(physx, obj_type, name, comp):
    with physx.read(obj_type, [name], scope=ObjectScope.ALL) as r:
        for g in r.groups:
            if g.tensors:
                t = g.tensors[0]
                a = t.numpy() if hasattr(t, "numpy") else np.asarray(t)
                return np.array(a, np.float32).reshape(-1, comp)
    return None


def _rigid_usda(tmp_path):
    p = tmp_path / "drain_rigid.usda"
    p.write_text("\n".join([
        '#usda 1.0', '(', '    defaultPrim = "World"', '    metersPerUnit = 1', '    upAxis = "Z"', ')', '',
        'def Xform "World"', '{', '    def PhysicsScene "physicsScene"', '    {',
        '        float physics:gravityMagnitude = 0', '    }',
        '    def Cube "Body" ( prepend apiSchemas = ["PhysicsRigidBodyAPI","PhysicsCollisionAPI","PhysicsMassAPI"] )',
        '    {', '        double size = 0.25', '        float physics:mass = 1',
        '        vector3f physics:velocity = (0, 0, 0)', '        vector3f physics:angularVelocity = (0, 0, 0)',
        '        point3f physics:centerOfMass = (0, 0, 0)', '        float3 physics:diagonalInertia = (1, 1, 1)', '    }',
        '}', '']))
    return str(p)


def _rigid_mixed_usda(tmp_path):
    """One dynamic body and one kinematic body, both authoring physics:velocity -- the shape that puts a
    key the drain skips (the kinematic one) and a key it services in the same batch."""
    p = tmp_path / "drain_rigid_mixed.usda"
    p.write_text("\n".join([
        '#usda 1.0', '(', '    defaultPrim = "World"', '    metersPerUnit = 1', '    upAxis = "Z"', ')', '',
        'def Xform "World"', '{', '    def PhysicsScene "physicsScene"', '    {',
        '        float physics:gravityMagnitude = 0', '    }',
        '    def Cube "Dyn" ( prepend apiSchemas = ["PhysicsRigidBodyAPI","PhysicsCollisionAPI","PhysicsMassAPI"] )',
        '    {', '        double size = 0.25', '        float physics:mass = 1',
        '        vector3f physics:velocity = (0, 0, 0)', '    }',
        '    def Cube "Kin" ( prepend apiSchemas = ["PhysicsRigidBodyAPI","PhysicsCollisionAPI","PhysicsMassAPI"] )',
        '    {', '        double size = 0.25', '        float physics:mass = 1',
        '        bool physics:kinematicEnabled = 1',
        '        double3 xformOp:translate = (2, 0, 0)', '        uniform token[] xformOpOrder = ["xformOp:translate"]',
        '        vector3f physics:velocity = (0, 0, 0)', '    }',
        '}', '']))
    return str(p)


def test_drain_rigid_velocity_mixed_kinematic(physx_sdk, tmp_path):
    """A velocity batch mixing a kinematic body (a key the drain SKIPS -> surface velocity on the parse path)
    with a dynamic one must still drain the dynamic body's velocity through the backend -- the drain splits
    the batch rather than falling back whole. On this DirectGPU fixture the per-object host setter is inert,
    so a whole-batch fallback (the pre-fix behavior) would leave the dynamic body at zero; the read-back of
    the authored velocity is attributable to the backend drain alone."""
    stage = attach_usd_with_ovstage(physx_sdk, _rigid_mixed_usda(tmp_path))
    for _ in range(2):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    # One batch, both keys, same ordinal: the dynamic body is serviced, the kinematic one is skipped.
    _drain_scalar_multi(physx_sdk, stage,
                        [("/World/Dyn", [7, 8, 9]), ("/World/Kin", [1, 2, 3])], "physics:velocity", 2)

    got = _read(physx_sdk, SimObjectType.RIGID_BODY, "linearVelocity", 3)
    assert got is not None
    # EXACTLY one body holds the drained velocity -- the dynamic one; the kinematic body cannot hold a body
    # velocity (its authored velocity becomes a surface/conveyor velocity on the parse path), so `== 1`
    # effectively pins the match to /World/Dyn. A whole-batch fallback drops the dynamic velocity too (inert
    # setter on GPU), leaving every row at zero -- no row would match.
    assert sum(1 for row in got if np.allclose(row, [7, 8, 9], atol=1e-3)) == 1, \
        "dynamic body's velocity dropped -- the mixed batch fell back to the inert per-object path"


def test_drain_rigid_velocity(physx_sdk, tmp_path):
    """velocity and angularVelocity (deg->rad) drain through the write backend, as self-contained fields."""
    stage = attach_usd_with_ovstage(physx_sdk, _rigid_usda(tmp_path))
    for _ in range(2):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    _drain_scalar(physx_sdk, stage, "/World/Body", "physics:velocity", [7, 8, 9], 2)
    assert np.allclose(_read(physx_sdk, SimObjectType.RIGID_BODY, "linearVelocity", 3)[0], [7, 8, 9], atol=1e-3)

    _drain_scalar(physx_sdk, stage, "/World/Body", "physics:angularVelocity", [1, 2, 3], 3)
    got = _read(physx_sdk, SimObjectType.RIGID_BODY, "angularVelocity", 3)[0]
    assert np.allclose(got, np.array([1, 2, 3]) * (np.pi / 180.0), atol=1e-3)  # authored deg -> read rad


def test_drain_rigid_mass_frame_not_scattered(physx_sdk, tmp_path):
    """The mass frame (mass / centerOfMass / diagonalInertia) is coupled. The parse path recomputes it
    holistically, so the drain does NOT scatter it. It falls back to that recompute, which is applied at the
    next step; with no step between the drain and the read, the read-backs are unchanged. Scattering these
    (the reverted behavior) would move them immediately. A positive control then steps once and asserts the
    recompute LANDED -- so "fell back to the recompute" is distinguished from "silently dropped" (which the
    unchanged-with-no-step check alone cannot do, and which on this DirectGPU fixture would also cover a
    recompute that never takes effect). The scene has no gravity, so the body itself does not move."""
    stage = attach_usd_with_ovstage(physx_sdk, _rigid_usda(tmp_path))
    for _ in range(2):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    mass0 = _read(physx_sdk, SimObjectType.RIGID_BODY, "mass", 1)[0].copy()
    com0 = _read(physx_sdk, SimObjectType.RIGID_BODY, "centerOfMassPosition", 3)[0].copy()

    _drain_scalar(physx_sdk, stage, "/World/Body", "physics:mass", [5.0], 4)
    _drain_scalar(physx_sdk, stage, "/World/Body", "physics:centerOfMass", [0.1, 0.2, 0.3], 5)
    # No step ran, so the deferred mass recompute has not applied: the drain left these untouched.
    assert np.allclose(_read(physx_sdk, SimObjectType.RIGID_BODY, "mass", 1)[0], mass0, atol=1e-4)
    assert np.allclose(_read(physx_sdk, SimObjectType.RIGID_BODY, "centerOfMassPosition", 3)[0], com0, atol=1e-4)

    # Positive control: step once so the dirty-mass queue flushes, then the recompute must have landed the
    # authored mass (5.0) and moved the COM to the authored (0.1,0.2,0.3) (this body has no local scale, so no
    # AC-10 rescale). A dropped key would leave both at their pre-drain values.
    physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()
    assert np.allclose(_read(physx_sdk, SimObjectType.RIGID_BODY, "mass", 1)[0], [5.0], atol=1e-3)
    com1 = _read(physx_sdk, SimObjectType.RIGID_BODY, "centerOfMassPosition", 3)[0]
    assert np.allclose(com1, [0.1, 0.2, 0.3], atol=1e-3)
    assert not np.allclose(com1, com0, atol=1e-3)  # the COM actually moved from its pre-drain value


def test_drain_articulation_dof_target(physx_sdk):
    """A single-DOF joint's drive target drains through the joint write path."""
    stage = attach_usd_with_ovstage(physx_sdk, data_path("CartRailDriveLinear.usda"))
    for _ in range(2):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()
    _read(physx_sdk, SimObjectType.ARTICULATION_JOINT, "jointPositionTarget", 1)  # warm the joint cache

    _drain_scalar(physx_sdk, stage, "/cartpole/cartJoint", "drive:linear:physics:targetPosition", [5.0], 2)
    got = _read(physx_sdk, SimObjectType.ARTICULATION_JOINT, "jointPositionTarget", 1)
    assert got is not None and np.allclose(got[0], [5.0], atol=1e-3)


def test_drain_articulation_dof_target_revolute(physx_sdk):
    """A revolute DOF drive target round-trips through the deg<->rad fold.

    The linear test above cannot see a unit-fold bug: a prismatic axis has invAngScale == 1. Here the axis is
    angular, so the write setter folds the authored degrees to radians and the read folds them back. Authoring
    90 must read back 90. A double fold in the drain (applying invAngScale before the setter does) would land
    ~57x too small and read back ~1.57.
    """
    stage = attach_usd_with_ovstage(physx_sdk, data_path("CartRailDriveAngular.usda"))
    for _ in range(2):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()
    _read(physx_sdk, SimObjectType.ARTICULATION_JOINT, "jointPositionTarget", 1)  # warm the joint cache

    _drain_scalar(physx_sdk, stage, "/cartpole/poleJoint", "drive:angular:physics:targetPosition", [90.0], 2)
    got = _read(physx_sdk, SimObjectType.ARTICULATION_JOINT, "jointPositionTarget", 1)
    assert got is not None and np.allclose(got[0], [90.0], atol=1e-2)


def test_drain_particle_points(physx_sdk):
    """A particle set's ragged points array drains through the particle write path."""
    stage = attach_usd_with_ovstage(physx_sdk, data_path("particles_simple.usda"))
    for _ in range(2):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    new = [[1., 1., 1.], [2., 2., 2.], [3., 3., 3.]]
    _drain_array(physx_sdk, stage, "/World/particles", "points", new, 2)
    got = _read(physx_sdk, SimObjectType.PARTICLE_SET, "points", 3)
    assert got is not None and got.shape == (3, 3)
    assert np.allclose(np.sort(got, axis=0), np.sort(np.asarray(new, np.float32), axis=0), atol=1e-3)


def test_drain_deformable_pose_does_not_touch_live_state(physx_sdk):
    """`deformablePose:<inst>:omniphysics:points` is the sim mesh's BIND POSE, not live solver state, so the
    drain must NOT scatter it onto the running vertices. It falls back to the parse-path resync, whose
    re-cook lands at the next step -- so with no step between the drain and the read, the live points are
    unchanged. Scattering the bind pose here (the reverted behavior) would teleport them.

    NOTE: this case has NO positive control. A bind-pose change updates the sim mesh's REST configuration; the
    live points relax toward it over MANY solver steps, so a single step barely moves them (empirically the
    re-cooked live points still sit at the old pose after one step). There is thus no cheap short-horizon
    oracle that separates "fell back to the resync" from "silently dropped" through the live-points read --
    unlike the mass frame, whose recompute lands at the next step. The negative (no-step, unchanged) assertion
    stands on its own."""
    stage = attach_usd_with_ovstage(physx_sdk, data_path("volume_deformable_simple.usda"))
    for _ in range(2):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    before = _read(physx_sdk, SimObjectType.DEFORMABLE_VOLUME, "points", 3)
    assert before is not None and before.shape == (5, 3)

    new = [[0., 0., 0.], [2., 0., 0.], [0., 2., 0.], [0., 0., 2.], [2., 2., 2.]]  # 5 tet vertices
    _drain_array(physx_sdk, stage, "/World/DeformableBody", "deformablePose:default:omniphysics:points", new, 2)
    after = _read(physx_sdk, SimObjectType.DEFORMABLE_VOLUME, "points", 3)
    # Drain did not teleport the live vertices to the authored bind pose (no step -> resync not yet applied).
    assert np.allclose(after, before, atol=1e-4)
