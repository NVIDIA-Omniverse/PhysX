# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CONFIG-001
# @covers AC-1 AC-2 AC-3
import pytest
from pxr import UsdGeom,UsdPhysics,Usd
from ovphysx_companion.configuration import Solver
from ovphysx_companion.usd.configuration import SceneOptions,apply_scene_options
from ovphysx_companion.usd.session import StaleResultError


def test_scene_settings_units_explicit_layer_atomic_undo(scene):
    stage,session=scene
    prim=UsdPhysics.Scene.Define(stage,'/World/Scene').GetPrim()
    UsdGeom.SetStageMetersPerUnit(stage,.01)
    before=stage.GetRootLayer().ExportToString()
    patch=apply_scene_options(session,str(prim.GetPath()),
        SceneOptions((0,0,-9.81),Solver.TGS,False),source=session.source,layer=stage.GetRootLayer())
    assert tuple(prim.GetAttribute('physics:gravityDirection').Get())==(0,0,-1)
    assert prim.GetAttribute('physics:gravityMagnitude').Get()==pytest.approx(981)
    assert prim.GetAttribute('physxScene:solverType').Get()=='TGS'
    assert prim.GetAttribute('physxScene:enableGPUDynamics').Get() is False
    assert all(not attr.GetTimeSamples() for attr in prim.GetAttributes())
    after=stage.GetRootLayer().ExportToString();patch.undo()
    assert stage.GetRootLayer().ExportToString()==before
    patch.redo();assert stage.GetRootLayer().ExportToString()==after
    apply_scene_options(session,str(prim.GetPath()),SceneOptions((0,0,0)),source=session.source,layer=stage.GetRootLayer())
    assert prim.GetAttribute('physics:gravityMagnitude').Get()==0
    assert prim.GetAttribute('physxScene:solverType').Get()=='TGS'


def test_scene_settings_reject_invalid_and_stale_without_partial_edits(scene):
    stage,session=scene
    scene_prim=UsdPhysics.Scene.Define(stage,'/World/Scene').GetPrim()
    source=session.source;scene_prim.SetCustomDataByKey('host:changed',True)
    before=stage.GetRootLayer().ExportToString()
    with pytest.raises(StaleResultError):apply_scene_options(session,'/World/Scene',SceneOptions((0,-1,0)),source=source,layer=stage.GetRootLayer())
    for path,options in (('/World/A',SceneOptions((0,-1,0))),('/World/Scene',SceneOptions()),('/World/Scene',SceneOptions((1e300,0,0)))):
        with pytest.raises(ValueError):apply_scene_options(session,path,options,source=session.source,layer=stage.GetRootLayer())
    assert stage.GetRootLayer().ExportToString()==before
    for kwargs in ({'solver':0.0},{'solver':True},{'gravity_world_mps2':(0,float('nan'),0)},{'gpu_enabled':1}):
        with pytest.raises((TypeError,ValueError)):SceneOptions(**kwargs)


def test_settings_authoring_sample():
    from scoped_settings import author
    stage=author()
    assert stage.GetPrimAtPath('/Scene').GetAttribute('physxScene:solverType').Get()=='TGS'
