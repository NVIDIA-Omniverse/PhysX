# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PARTICLES-001
# @covers AC-1 AC-2 AC-3 AC-4
from dataclasses import replace
import pytest
from ovphysx_companion.usd import EditSession,StaleResultError
from ovphysx_companion.usd.particles import capture_sampling_input,apply_particle_set,apply_surface,Surface
from ovphysx_companion.particles import Samples


def test_particle_apply_transform_undo_stale_and_selected_system(scene):
    from pxr import UsdGeom,Gf
    stage,session=scene
    UsdGeom.SetStageMetersPerUnit(stage,1)
    UsdGeom.Xformable(stage.GetPrimAtPath('/World')).AddTranslateOp().Set(Gf.Vec3d(5,0,0))
    stage.DefinePrim('/World/System','PhysxParticleSystem')
    request=capture_sampling_input(session,'/World/A')
    assert request.points[0]==(5,0,0)
    sampled=Samples(request.source,((5.25,.25,0),),(0,),((.5,.25,.25),),'world',1,.1,0)
    before=stage.GetRootLayer().ExportToString()
    patch=apply_particle_set(session,sampled,path='/World/Particles',particle_system='/World/System',layer=stage.GetRootLayer(),owner='particles')
    prim=stage.GetPrimAtPath('/World/Particles')
    assert prim.HasAPI('PhysxParticleSetAPI') and tuple(prim.GetAttribute('points').Get()[0])==pytest.approx((.25,.25,0))
    assert list(map(str,prim.GetRelationship('physxParticle:particleSystem').GetTargets()))==['/World/System']
    assert prim.GetAttribute('physxParticle:fluid').Get() is True
    patch.undo();assert stage.GetRootLayer().ExportToString()==before
    patch.redo();assert stage.GetPrimAtPath('/World/Particles')
    with pytest.raises(StaleResultError):apply_particle_set(session,sampled,path='/World/Particles',particle_system='/World/System',layer=stage.GetRootLayer(),owner='particles')
    sampled=replace(sampled,source=session.source)
    before=stage.GetRootLayer().ExportToString()
    with pytest.raises(ValueError):apply_particle_set(session,sampled,path='/World/A',particle_system='/World/System',layer=stage.GetRootLayer(),owner='particles')
    assert stage.GetRootLayer().ExportToString()==before
    stage.RemovePrim('/World/A')
    with pytest.raises(StaleResultError):apply_particle_set(session,sampled,path='/World/Other',particle_system='/World/System',layer=stage.GetRootLayer(),owner='particles')


def test_surface_output_foreign_revision_units_and_atomicity(scene):
    from pxr import Usd,UsdGeom
    stage,session=scene
    UsdGeom.SetStageMetersPerUnit(stage,1)
    source=session.source
    result=Surface(source,((0,0,0),(1,0,0),(0,1,0)),((0,1,2),))
    patch=apply_surface(session,result,path='/World/Surface',layer=stage.GetRootLayer(),owner='surface')
    assert list(stage.GetPrimAtPath('/World/Surface').GetAttribute('faceVertexIndices').Get())==[0,1,2]
    patch.undo();patch.redo()
    with EditSession(Usd.Stage.CreateInMemory()) as foreign:
        with pytest.raises(StaleResultError):apply_surface(foreign,result,path='/World/Surface',layer=foreign.stage.GetRootLayer(),owner='surface')
    before=stage.GetRootLayer().ExportToString()
    for invalid in (replace(result,source=session.source,meters_per_unit=.01),replace(result,source=session.source,triangles=((0,1,9),))):
        with pytest.raises(ValueError):apply_surface(session,invalid,path='/World/Surface',layer=stage.GetRootLayer(),owner='surface')
        assert stage.GetRootLayer().ExportToString()==before


def test_particle_sampling_editor_sample():
    from particle_authoring import run
    stage=run()
    assert len(stage.GetPrimAtPath('/World/Particles').GetAttribute('points').Get())>10
