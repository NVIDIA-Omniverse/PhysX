# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DESCRIPTION-001
# @covers AC-1 AC-2 AC-3 AC-4
import pytest
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from ovphysx_companion.usd import EditSession, StaleResultError
from ovphysx_companion.usd.descriptions import capture_descriptions


def test_stopped_description_families_defaults_relationships_units_and_lifetime():
    stage=Usd.Stage.CreateInMemory()
    world=UsdGeom.Xform.Define(stage,'/World')
    world.AddTranslateOp().Set((10,20,30))
    UsdGeom.SetStageMetersPerUnit(stage,.01)
    UsdPhysics.SetStageKilogramsPerUnit(stage,.001)
    box=UsdGeom.Cube.Define(stage,'/World/Box')
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
    peer=UsdGeom.Cube.Define(stage,'/World/Peer')
    UsdPhysics.CollisionAPI.Apply(peer.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(peer.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(world.GetPrim())
    joint=UsdPhysics.RevoluteJoint.Define(stage,'/World/Joint')
    joint.CreateBody1Rel().SetTargets([box.GetPath()])
    joint.CreateBody0Rel().SetTargets([peer.GetPath()])
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1))
    assert joint.GetPrim().ApplyAPI('PhysxTendonAxisRootAPI','main')
    vehicle=stage.DefinePrim('/World/Vehicle','Xform')
    assert vehicle.ApplyAPI('PhysxVehicleAPI')
    stage.DefinePrim('/World/Particles','PhysxParticleSystem')
    with EditSession(stage) as session:
        before=stage.GetRootLayer().ExportToString()
        snapshot=capture_descriptions(session)
        assert {d.family for d in snapshot.descriptions}=={'body','shape','joint','articulation','tendon','vehicle','particle'}
        assert not snapshot.diagnostics
        shape=snapshot.query('shape').descriptions[0]
        assert shape.attribute('size')==2 # unauthored schema default
        assert shape.meters_per_unit==.01 and shape.kilograms_per_unit==.001
        assert shape.local_to_world[3][:3]==(10,20,30)
        description=snapshot.query('joint').descriptions[0]
        assert description.targets('physics:body1')==('/World/Box',)
        assert description.attribute('physics:localRot1')==(0,0,0,1)
        assert snapshot.query('unknown').diagnostics[-1].code=='DESCRIPTION_UNSUPPORTED_FAMILY'
        assert stage.GetRootLayer().ExportToString()==before
        box.GetSizeAttr().Set(8)
        assert shape.attribute('size')==2
        with pytest.raises(StaleResultError):snapshot.validate(session)
    assert description.targets('physics:body1')==('/World/Box',)


def test_stopped_descriptions_distinguish_instance_paths():
    stage=Usd.Stage.CreateInMemory()
    prototype=UsdGeom.Xform.Define(stage,'/Source')
    cube=UsdGeom.Cube.Define(stage,'/Source/Box')
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    for path in ('/One','/Two'):
        prim=stage.DefinePrim(path,'Xform')
        prim.GetReferences().AddInternalReference('/Source')
        prim.SetInstanceable(True)
    with EditSession(stage) as session:
        result=capture_descriptions(session,['/One/Box','/Two/Box','/Missing'])
        assert {d.instance_path for d in result.descriptions}=={'/One','/Two'}
        assert {d.path for d in result.descriptions}=={'/One/Box','/Two/Box'}
        assert result.diagnostics[0].code=='DESCRIPTION_MISSING_PRIM'


def test_description_time_and_copied_mesh_topology():
    stage=Usd.Stage.CreateInMemory()
    mesh=UsdGeom.Mesh.Define(stage,'/Mesh')
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    mesh.CreatePointsAttr([(0,0,0),(1,0,0),(0,1,0)])
    mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
    mesh.AddTranslateOp().Set((1,0,0),1)
    mesh.GetOrderedXformOps()[0].Set((3,0,0),2)
    with EditSession(stage) as session:
        result=capture_descriptions(session,time_code=2).descriptions[0]
        assert result.local_to_world[3][0]==3
        assert result.attribute('faceVertexIndices')==(0,1,2)
        assert result.attribute('points')[1]==(1,0,0)
        with pytest.raises(ValueError):capture_descriptions(session,['relative'])
