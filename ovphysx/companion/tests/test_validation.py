# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-VALIDATE-001
# @covers AC-1 AC-2 AC-4
from dataclasses import replace
import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from ovphysx_companion.usd import EditSession, StaleResultError
from ovphysx_companion.usd.descriptions import capture_descriptions
from ovphysx_companion.usd.validation import apply_repairs
from ovphysx_companion.validation import validate_descriptions


def test_validator_extension_flow_sample():
    from asset_validation import run
    stage=run()
    assert stage.GetPrimAtPath('/Joint').GetAttribute('physics:localRot1').Get().GetReal()==1


def asset():
    stage=Usd.Stage.CreateInMemory()
    body=UsdGeom.Xform.Define(stage,'/Body')
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(-1)
    shape=UsdGeom.Cube.Define(stage,'/Body/Collider')
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    shape.AddScaleOp().Set((0,1,1))
    joint=UsdPhysics.FixedJoint.Define(stage,'/Joint')
    joint.CreateBody1Rel().SetTargets([body.GetPath()])
    joint.CreateLocalRot1Attr(Gf.Quatf(2))
    return stage


def test_pure_checks_classify_and_selected_repairs_are_reversible(monkeypatch):
    import ovphysx
    def forbidden(*args,**kwargs):raise AssertionError('pure check initialized physics')
    monkeypatch.setitem(ovphysx.__dict__,'PhysX',forbidden)  # Avoid lazy native API import.
    stage=asset()
    with EditSession(stage) as session:
        before=stage.GetRootLayer().ExportToString()
        report=validate_descriptions(capture_descriptions(session))
        assert {issue.code for issue in report.issues}=={'MASS_INVALID','JOINT_FRAME_NON_UNIT','COLLIDER_SINGULAR_TRANSFORM'}
        assert all(check.requires=='usd-data' for check in report.checks)
        assert all(issue.source==session.source and issue.path.startswith('/') for issue in report.issues)
        assert report.complete and not report.valid
        assert stage.GetRootLayer().ExportToString()==before
        index=next(i for i,issue in enumerate(report.issues) if issue.code=='JOINT_FRAME_NON_UNIT')
        patch=apply_repairs(session,report,[index],layer=stage.GetRootLayer())
        assert stage.GetPrimAtPath('/Joint').GetAttribute('physics:localRot1').Get().GetReal()==1
        assert stage.GetPrimAtPath('/Body').GetAttribute('physics:mass').Get()==-1
        patch.undo()
        assert stage.GetRootLayer().ExportToString()==before
        patch.redo()
        after=stage.GetRootLayer().ExportToString()
        with pytest.raises(StaleResultError):apply_repairs(session,report,[index],layer=stage.GetRootLayer())
        assert stage.GetRootLayer().ExportToString()==after


def test_validation_repair_preflight_is_atomic_and_stale_deletion_rejected():
    stage=asset()
    with EditSession(stage) as session:
        report=validate_descriptions(capture_descriptions(session))
        before=stage.GetRootLayer().ExportToString()
        with pytest.raises(ValueError,match='no applicable repair'):
            apply_repairs(session,report,list(range(len(report.issues))),layer=stage.GetRootLayer())
        assert stage.GetRootLayer().ExportToString()==before
        stage.RemovePrim('/Joint')
        after=stage.GetRootLayer().ExportToString()
        with pytest.raises(StaleResultError):apply_repairs(session,report,[0],layer=stage.GetRootLayer())
        assert stage.GetRootLayer().ExportToString()==after


def test_unavailable_cooking_is_not_reported_as_success():
    stage=Usd.Stage.CreateInMemory()
    mesh=UsdGeom.Mesh.Define(stage,'/Mesh')
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    with EditSession(stage) as session:
        report=validate_descriptions(capture_descriptions(session),checks=['collision-cook'])
        assert not report.complete and not report.valid
        assert report.issues[0].code=='VALIDATION_UNAVAILABLE'
        assert report.issues[0].check.requires=='cooking'
        with pytest.raises(ValueError,match='ValidationPreview'):
            validate_descriptions(capture_descriptions(session),checks=['preview-motion'])


def test_sampled_repair_targets_captured_time_and_preserves_other_samples():
    stage=asset()
    attr=stage.GetPrimAtPath('/Joint').GetAttribute('physics:localRot1')
    attr.Set(Gf.Quatf(1),10);attr.Set(Gf.Quatf(2),15);attr.Set(Gf.Quatf(1),20)
    with EditSession(stage) as session:
        report=validate_descriptions(capture_descriptions(session,time_code=15),checks=['joint-frames'])
        assert len(report.issues)==1 and report.issues[0].repair.time_code==15
        before=stage.GetRootLayer().ExportToString()
        patch=apply_repairs(session,report,[0],layer=stage.GetRootLayer())
        assert attr.Get(15).GetReal()==1
        assert attr.Get().GetReal()==2 and attr.Get(10).GetReal()==1 and attr.Get(20).GetReal()==1
        assert validate_descriptions(capture_descriptions(session,time_code=15),checks=['joint-frames']).valid
        patch.undo();assert stage.GetRootLayer().ExportToString()==before
