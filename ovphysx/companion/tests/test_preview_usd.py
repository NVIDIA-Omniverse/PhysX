# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PREVIEW-001
# @covers AC-2 AC-3 AC-4
from pathlib import Path
import sys
import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Sdf, Tf
from ovphysx_companion.usd.session import EditSession, StaleResultError
from ovphysx_companion.usd.preview import capture_preview, commit_preview, simulate_snapshot
from ovphysx_companion.preview import Pose


@pytest.fixture
def inspector():
    stage = Usd.Stage.Open(str(Path(__file__).resolve().parents[1] / 'samples/scenes/two_boxes.usda'))
    # Work on an anonymous copy; never mutate the checked-in sample.
    stage = Usd.Stage.Open(stage.Flatten())
    parent = UsdGeom.Xformable(stage.GetPrimAtPath('/World'))
    parent.AddTranslateOp().Set((10, 0, 0))
    body = UsdGeom.Xformable(stage.GetPrimAtPath('/World/BoxA'))
    body.AddScaleOp().Set((2, 3, 4))
    with EditSession(stage) as session:
        yield stage, session


def test_inspector_capture_commit_and_undo_preserve_unselected_opinions(inspector):
    stage, session = inspector
    layer = stage.GetRootLayer()
    before = layer.ExportToString()
    snapshot = capture_preview(session, ['/World/BoxA'])
    assert layer.ExportToString() == before
    patch = commit_preview(session, snapshot, {'/World/BoxA': Pose((12, 1, 8), (0, 0, 0, 1))}, layer=layer)
    body = stage.GetPrimAtPath('/World/BoxA')
    world = UsdGeom.XformCache().GetLocalToWorldTransform(body)
    assert tuple(world.ExtractTranslation()) == pytest.approx((12, 1, 8))
    assert tuple(Gf.Transform(world).GetScale()) == pytest.approx((2, 3, 4))
    assert tuple(body.GetAttribute('xformOp:scale').Get()) == (2, 3, 4)
    after = layer.ExportToString()
    patch.undo()
    assert layer.ExportToString() == before
    patch.redo()
    assert layer.ExportToString() == after


def test_inspector_invalidates_for_selection_play_and_source_edits(inspector):
    stage, session = inspector
    snapshot = capture_preview(session, ['/World/BoxA'])
    session.invalidate()  # host Play / selection transition
    with pytest.raises(StaleResultError):
        commit_preview(session, snapshot, {'/World/BoxA': Pose((0, 0, 0), (0, 0, 0, 1))}, layer=stage.GetRootLayer())
    snapshot = capture_preview(session, ['/World/BoxA'])
    stage.GetPrimAtPath('/World/BoxB').GetAttribute('size').Set(2)
    with pytest.raises(StaleResultError):
        session.validate(snapshot.source)


def test_inspector_rejects_incomplete_batch_atomically(inspector):
    stage, session = inspector
    snapshot = capture_preview(session, ['/World/BoxA', '/World/BoxB'])
    before = stage.GetRootLayer().ExportToString()
    with pytest.raises(ValueError, match='exactly'):
        commit_preview(session, snapshot, {}, layer=stage.GetRootLayer())
    with pytest.raises(ValueError, match='unit quaternion'):
        commit_preview(session, snapshot, {
            '/World/BoxA': Pose((1, 2, 3), (0, 0, 0, 1)),
            '/World/BoxB': Pose((1, 2, 3), (0, 0, 0, 2))}, layer=stage.GetRootLayer())
    assert stage.GetRootLayer().ExportToString() == before


def test_inspector_snapshot_freezes_numeric_usd_time(inspector):
    stage, session = inspector
    attr = stage.GetPrimAtPath('/World/BoxA').GetAttribute('xformOp:translate')
    attr.Set((0, 0, 10), 10)
    attr.Set((0, 0, 20), 20)
    snapshot = capture_preview(session, ['/World/BoxA'], time_code=15)
    layer = Sdf.Layer.CreateAnonymous()
    assert layer.ImportFromString(snapshot.usda)
    frozen = Usd.Stage.Open(layer)
    value = frozen.GetPrimAtPath('/World/BoxA').GetAttribute('xformOp:translate')
    assert value.GetNumTimeSamples() == 0
    assert tuple(value.Get()) == pytest.approx((0, 0, 15))


def test_inspector_customer_usd_to_real_native_worker_and_back(inspector):
    stage, session = inspector
    # Expects external forces applied once per step.
    scene = stage.GetPrimAtPath('/World/PhysicsScene')
    scene.AddAppliedSchema('PhysxSceneAPI')
    scene.CreateAttribute('physxScene:enableExternalForcesEveryIteration', Sdf.ValueTypeNames.Bool).Set(False)
    snapshot = capture_preview(session, ['/World/BoxA'])
    before = stage.GetRootLayer().ExportToString()
    poses = simulate_snapshot(session, snapshot, python_executable=sys.executable, dt=.01, steps=3)
    assert set(poses) == {'/World/BoxA'}
    assert poses['/World/BoxA'].position[2] < snapshot.bodies[0].world[3][2]
    assert stage.GetRootLayer().ExportToString() == before
    patch = commit_preview(session, snapshot, poses, layer=stage.GetRootLayer())
    patch.undo()
    assert stage.GetRootLayer().ExportToString() == before


def test_preview_display_cancel_preserves_new_authored_input_and_excludes_notices(inspector):
    from ovphysx_companion.usd.preview import PreviewDisplay
    from ovphysx_companion.exchange import OutputNotices
    stage,session=inspector
    layer=Sdf.Layer.CreateAnonymous('inspector-display')
    stage.GetSessionLayer().subLayerPaths.append(layer.identifier)
    captured=[]
    notices=OutputNotices(lambda *args:captured.append('input'))
    session._output_guard=notices.suppress
    listener=Tf.Notice.Register(Usd.Notice.ObjectsChanged,
        notices.notice,stage)
    try:
        snapshot=capture_preview(session,['/World/BoxA']);revision=session.source
        with PreviewDisplay(session,layer=layer) as display:
            with pytest.raises(RuntimeError,match='writer'):PreviewDisplay(session,layer=layer)
            display.publish(snapshot,{'/World/BoxA':Pose((1,2,3),(0,0,0,1))})
            assert session.source==revision and not captured
            # Source changes while display is live. Cancel clears only output.
            stage.GetPrimAtPath('/World/BoxB').GetAttribute('size').Set(7)
            with pytest.raises(StaleResultError):display.publish(snapshot,{'/World/BoxA':Pose((1,2,3),(0,0,0,1))})
        assert layer.empty and stage.GetPrimAtPath('/World/BoxB').GetAttribute('size').Get()==7
        assert captured==['input']
        session.close()
        display.close() # safe after both display and owning session have closed
    finally:listener.Revoke()


def test_articulation_worker_snapshot_selected_commit_and_play_undo_transitions():
    stage=Usd.Stage.Open(str(Path(__file__).resolve().parents[1]/'samples/scenes/inspection.usda'))
    stage=Usd.Stage.Open(stage.Flatten())
    with EditSession(stage) as session:
        snapshot=capture_preview(session,['/World/Robot/Tip'])
        before=stage.GetRootLayer().ExportToString()
        poses=simulate_snapshot(session,snapshot,python_executable=sys.executable,dt=.001,steps=2)
        assert set(poses)=={'/World/Robot/Tip'}
        patch=commit_preview(session,snapshot,poses,layer=stage.GetRootLayer())
        patch.undo();assert stage.GetRootLayer().ExportToString()==before
        patch.redo()
        with pytest.raises(StaleResultError):session.validate(snapshot.source)
        fresh=capture_preview(session,['/World/Robot/Tip'])
        session.invalidate() # host Play, selection, topology or undo transition
        with pytest.raises(StaleResultError):commit_preview(session,fresh,poses,layer=stage.GetRootLayer())


def test_animated_commit_rejected_without_changing_effective_animation(inspector):
    stage,session=inspector
    attr=stage.GetPrimAtPath('/World/BoxA').GetAttribute('xformOp:translate')
    attr.Set((0,0,10),10);attr.Set((0,0,20),20)
    snapshot=capture_preview(session,['/World/BoxA'],time_code=15)
    before=stage.GetRootLayer().ExportToString()
    with pytest.raises(ValueError,match='Animated transform commits'):
        commit_preview(session,snapshot,{'/World/BoxA':Pose((99,0,0),(0,0,0,1))},layer=stage.GetRootLayer())
    assert stage.GetRootLayer().ExportToString()==before
    for time,height in ((10,10),(15,15),(20,20)):
        matrix=UsdGeom.XformCache(Usd.TimeCode(time)).GetLocalToWorldTransform(stage.GetPrimAtPath('/World/BoxA'))
        assert matrix.ExtractTranslation()[2]==pytest.approx(height)


def test_physics_capture_excludes_render_payload_and_retains_dependencies(inspector,monkeypatch):
    from pxr import UsdPhysics,UsdShade
    stage,session=inspector
    render=UsdGeom.Mesh.Define(stage,'/World/BoxA/Render')
    render.CreatePointsAttr([(12345,23456,34567)]*100000)
    material=UsdShade.Material.Define(stage,'/Materials/Physics')
    UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateStaticFrictionAttr(.7)
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath('/World/BoxA')).Bind(material,materialPurpose='physics')
    joint=UsdPhysics.FixedJoint.Define(stage,'/Joints/Link')
    joint.CreateBody0Rel().SetTargets(['/World/BoxA']);joint.CreateBody1Rel().SetTargets(['/World/BoxB'])
    def forbidden(*args,**kwargs):raise AssertionError('whole stage flatten')
    monkeypatch.setattr(Usd.Stage,'Flatten',forbidden)
    snapshot=capture_preview(session,['/World/BoxA'])
    assert len(snapshot.usda)<20000 and '12345' not in snapshot.usda
    layer=Sdf.Layer.CreateAnonymous();assert layer.ImportFromString(snapshot.usda)
    copied=Usd.Stage.Open(layer)
    assert not copied.GetPrimAtPath('/World/BoxA/Render')
    assert copied.GetPrimAtPath('/World/BoxB').HasAPI(UsdPhysics.CollisionAPI)
    assert copied.GetPrimAtPath('/Joints/Link').GetRelationship('physics:body1').GetTargets()==[Sdf.Path('/World/BoxB')]
    assert copied.GetPrimAtPath('/Materials/Physics').GetAttribute('physics:staticFriction').Get()==pytest.approx(.7)
    assert UsdGeom.XformCache().GetLocalToWorldTransform(copied.GetPrimAtPath('/World/BoxA'))==UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/BoxA'))


def test_large_sibling_selection_and_nested_rejection(inspector,monkeypatch):
    from pxr import UsdPhysics
    stage,session=inspector
    paths=[]
    for index in range(2048):
        prim=UsdGeom.Xform.Define(stage,f'/World/Many/B{index}').GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim);paths.append(str(prim.GetPath()))
    prefix_calls=0
    original=Sdf.Path.HasPrefix
    def counted(path,prefix):
        nonlocal prefix_calls
        prefix_calls+=1
        return original(path,prefix)
    monkeypatch.setattr(Sdf.Path,'HasPrefix',counted)
    assert len(capture_preview(session,paths).bodies)==2048
    assert prefix_calls<=len(paths)*4
    with pytest.raises(ValueError,match='Nested'):
        capture_preview(session,['/World/Many/B0','/World/Many/B0/Child'])


def test_preview_retains_rigid_point_instancer_inputs_and_cct_geometry(inspector):
    from pxr import UsdPhysics
    stage,session=inspector
    instancer=UsdGeom.PointInstancer.Define(stage,'/World/Instances')
    instancer.CreatePrototypesRel().SetTargets(['/World/BoxB'])
    instancer.CreateProtoIndicesAttr([0]);instancer.CreatePositionsAttr([(10,0,3)])
    capsule=UsdGeom.Capsule.Define(stage,'/World/Cct');capsule.CreateRadiusAttr(.7);capsule.CreateHeightAttr(2)
    capsule.GetPrim().ApplyAPI('PhysxCharacterControllerAPI')
    snapshot=capture_preview(session,['/World/BoxA'])
    layer=Sdf.Layer.CreateAnonymous();assert layer.ImportFromString(snapshot.usda)
    copied=Usd.Stage.Open(layer)
    assert copied.GetPrimAtPath('/World/Instances').GetAttribute('protoIndices').Get()==[0]
    assert copied.GetPrimAtPath('/World/Instances').GetRelationship('prototypes').GetTargets()==[Sdf.Path('/World/BoxB')]
    assert tuple(copied.GetPrimAtPath('/World/Instances').GetAttribute('positions').Get()[0])==(10,0,3)
    assert copied.GetPrimAtPath('/World/Cct').IsA(UsdGeom.Capsule)
    assert copied.GetPrimAtPath('/World/Cct').GetAttribute('radius').Get()==pytest.approx(.7)


def copied_snapshot(session):
    snapshot=capture_preview(session,['/World/BoxA'])
    layer=Sdf.Layer.CreateAnonymous();assert layer.ImportFromString(snapshot.usda)
    return Usd.Stage.Open(layer)


@pytest.mark.parametrize('collection',[False,True])
def test_preview_preserves_resolved_strong_and_collection_physics_materials(inspector,collection):
    from pxr import UsdShade
    stage,session=inspector
    strong=UsdShade.Material.Define(stage,'/Materials/Strong')
    weak=UsdShade.Material.Define(stage,'/Materials/Weak')
    UsdPhysics.MaterialAPI.Apply(strong.GetPrim()).CreateDensityAttr(100)
    UsdPhysics.MaterialAPI.Apply(weak.GetPrim()).CreateDensityAttr(1000)
    parent=stage.GetPrimAtPath('/World');box=stage.GetPrimAtPath('/World/BoxA')
    UsdShade.MaterialBindingAPI.Apply(box).Bind(weak,materialPurpose='physics')
    binding=UsdShade.MaterialBindingAPI.Apply(parent)
    if collection:
        selection=Usd.CollectionAPI.Apply(parent,'Bodies');selection.CreateIncludeRootAttr(True)
        binding.Bind(selection,strong,'selected',bindingStrength=UsdShade.Tokens.strongerThanDescendants,materialPurpose='physics')
    else:binding.Bind(strong,bindingStrength=UsdShade.Tokens.strongerThanDescendants,materialPurpose='physics')
    original=UsdShade.MaterialBindingAPI(box).ComputeBoundMaterial('physics')[0]
    copied=copied_snapshot(session)
    resolved=UsdShade.MaterialBindingAPI(copied.GetPrimAtPath('/World/BoxA')).ComputeBoundMaterial('physics')[0]
    assert resolved.GetPath()==original.GetPath()==strong.GetPath()
    assert resolved.GetPrim().GetAttribute('physics:density').Get()==100
    from ovphysx_companion._worker import run_worker
    worker=Path(__file__).with_name('native_mass_worker.py')
    original_mass=run_worker(worker,dict(scene=stage.GetRootLayer().ExportToString(),path='/World/BoxA'))
    copied_mass=run_worker(worker,dict(scene=copied.GetRootLayer().ExportToString(),path='/World/BoxA'))
    assert copied_mass['mass']==pytest.approx(original_mass['mass'])


def test_preview_preserves_face_material_subsets_and_joint_drive_limit_state(inspector):
    from pxr import UsdShade
    stage,session=inspector
    mesh=UsdGeom.Mesh.Define(stage,'/World/FaceCollider');mesh.AddTranslateOp().Set((40,0,0));UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    mesh.CreatePointsAttr([(0,0,0),(1,0,0),(0,1,0)]);mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
    subset=UsdGeom.Subset.CreateGeomSubset(mesh,'Faces','face',[0],'materialBind')
    material=UsdShade.Material.Define(stage,'/Materials/Face');UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateStaticFrictionAttr(.2)
    base=UsdShade.Material.Define(stage,'/Materials/Mesh');UsdPhysics.MaterialAPI.Apply(base.GetPrim()).CreateStaticFrictionAttr(.8)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(base,materialPurpose='physics')
    UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material,materialPurpose='physics')
    joint=UsdPhysics.Joint.Define(stage,'/World/Joint');joint.CreateBody0Rel().SetTargets(['/World/BoxA'])
    drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'transX');drive.CreateStiffnessAttr(321);drive.CreateTargetPositionAttr(7)
    limit=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),'transX');limit.CreateLowAttr(-2);limit.CreateHighAttr(3)
    joint.GetPrim().CreateAttribute('state:transX:physics:position',Sdf.ValueTypeNames.Float).Set(4)
    copied=copied_snapshot(session)
    face=UsdGeom.Subset(copied.GetPrimAtPath(subset.GetPath()))
    assert list(face.GetIndicesAttr().Get())==[0] and face.GetElementTypeAttr().Get()=='face'
    assert UsdShade.MaterialBindingAPI(face.GetPrim()).ComputeBoundMaterial('physics')[0].GetPath()==material.GetPath()
    assert UsdShade.MaterialBindingAPI(copied.GetPrimAtPath(mesh.GetPath())).ComputeBoundMaterial('physics')[0].GetPath()==base.GetPath()
    from ovphysx_companion._worker import run_worker
    worker=Path(__file__).with_name('native_mass_worker.py')
    for input_stage in (stage,copied):
        friction=run_worker(worker,dict(scene=input_stage.GetRootLayer().ExportToString(),ray=(50.2,.2,5)))
        assert friction['friction']==pytest.approx(.2)
    for attr in joint.GetPrim().GetAttributes():
        if attr.GetName().startswith(('drive:','limit:','state:')):
            assert copied.GetPrimAtPath(joint.GetPath()).GetAttribute(attr.GetName()).Get()==attr.Get()


@pytest.mark.parametrize('ids',[None,[100,200]])
def test_preview_preserves_inactive_point_instances(inspector,ids):
    stage,session=inspector
    instancer=UsdGeom.PointInstancer.Define(stage,'/World/Instances')
    instancer.CreatePrototypesRel().SetTargets(['/World/BoxB']);instancer.CreateProtoIndicesAttr([0,0])
    instancer.CreatePositionsAttr([(0,0,0),(1,0,0)])
    if ids is not None:instancer.CreateIdsAttr(ids)
    instancer.DeactivateId(1 if ids is None else 200)
    copied=copied_snapshot(session)
    assert list(UsdGeom.PointInstancer(copied.GetPrimAtPath(instancer.GetPath())).ComputeMaskAtTime(Usd.TimeCode.Default()))==[True,False]


def test_numeric_capture_static_commit_has_default_pose_and_can_repeat(inspector):
    stage,session=inspector
    for position in ((12,1,8),(13,2,9)):
        snapshot=capture_preview(session,['/World/BoxA'],time_code=0)
        commit_preview(session,snapshot,{'/World/BoxA':Pose(position,(0,0,0,1))},layer=stage.GetRootLayer())
        prim=stage.GetPrimAtPath('/World/BoxA')
        for time in (Usd.TimeCode.Default(),Usd.TimeCode(0),Usd.TimeCode(20)):
            assert tuple(UsdGeom.XformCache(time).GetLocalToWorldTransform(prim).ExtractTranslation())==pytest.approx(position)
        assert not prim.GetAttribute('xformOp:transform:companionPreview').ValueMightBeTimeVarying()



def spline_attribute(attribute):
    from pxr import Ts
    spline=Ts.Spline()
    knot=Ts.Knot();knot.SetTime(0);knot.SetValue(0.);spline.SetKnot(knot)
    knot.SetTime(10);knot.SetValue(10.);spline.SetKnot(knot)
    assert attribute.SetSpline(spline)
    assert attribute.GetNumTimeSamples()==0 and attribute.ValueMightBeTimeVarying()


def test_spline_animation_commit_is_rejected_atomically(inspector):
    stage,session=inspector
    xform=UsdGeom.Xformable(stage.GetPrimAtPath('/World/BoxA'))
    spline_attribute(xform.AddTranslateXOp(UsdGeom.XformOp.PrecisionDouble).GetAttr())
    snapshot=capture_preview(session,['/World/BoxA'],time_code=5)
    before=stage.GetRootLayer().ExportToString()
    with pytest.raises(ValueError,match='Animated transform commits'):
        commit_preview(session,snapshot,{'/World/BoxA':Pose((12,1,8),(0,0,0,1))},layer=stage.GetRootLayer())
    assert stage.GetRootLayer().ExportToString()==before


def test_spline_mass_geometry_is_rejected(scene):
    from ovphysx_companion.usd.mass import capture_mass_request
    stage,session=scene
    cube=UsdGeom.Cube.Define(stage,'/Body');UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim());UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    spline_attribute(cube.CreateSizeAttr())
    with pytest.raises(ValueError,match='animated'):capture_mass_request(session,'/Body')


def test_installed_preview_worker_uses_selected_interpreter_packages(inspector,tmp_path,monkeypatch):
    import json,venv
    from ovphysx_companion.usd import preview
    stage,session=inspector
    # A copied POSIX interpreter loses its $ORIGIN-relative libpython; symlink it.
    worker=tmp_path/'worker';venv.EnvBuilder(with_pip=False,symlinks=sys.platform!='win32').create(worker)
    executable=worker/('Scripts/python.exe' if sys.platform=='win32' else 'bin/python')
    import subprocess
    site=Path(subprocess.check_output([str(executable),'-c','import sysconfig;print(sysconfig.get_path("purelib"))'],text=True).strip())
    for name in ('ovstage','ovphysx'):
        (site/(name+'.py')).write_text('origin="worker"')
    package=site/'ovphysx_companion';package.mkdir();(package/'__init__.py').write_text('')
    (package/'preview_worker.py').write_text("""import argparse,json
from pathlib import Path
import ovstage,ovphysx
assert ovstage.origin==ovphysx.origin=='worker'
p=argparse.ArgumentParser();p.add_argument('--scene');p.add_argument('--output');p.add_argument('--dt');p.add_argument('--steps');p.add_argument('--paths',nargs='+');a=p.parse_args()
Path(a.output).write_text(json.dumps({path:{'position':[12,1,8],'orientation':[0,0,0,1]} for path in a.paths}))
""")
    host=tmp_path/'host-site';host.mkdir()
    for name in ('ovstage','ovphysx'):(host/(name+'.py')).write_text('raise AssertionError("host package leaked")')
    monkeypatch.setenv('PYTHONPATH',str(host));monkeypatch.syspath_prepend(str(host))
    monkeypatch.setattr(preview,'__file__',str(host/'ovphysx_companion/usd/preview.py'))
    result=simulate_snapshot(session,capture_preview(session,['/World/BoxA']),python_executable=str(executable))
    assert result['/World/BoxA'].position==(12,1,8)



def test_preview_retains_typed_and_unknown_particle_input(inspector):
    stage,session=inspector
    particle=stage.DefinePrim('/World/Particles','PhysxParticleSystem')
    particle.CreateAttribute('enabled',Sdf.ValueTypeNames.Bool).Set(False)
    points=UsdGeom.Points.Define(stage,'/World/ParticlePoints').GetPrim()
    points.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(['PhysxParticleCustomAPI']))
    points.CreateAttribute('points',Sdf.ValueTypeNames.Point3fArray).Set([(1,2,3)])
    copied=copied_snapshot(session)
    assert copied.GetPrimAtPath(particle.GetPath()).GetTypeName()=='PhysxParticleSystem'
    assert copied.GetPrimAtPath(particle.GetPath()).GetAttribute('enabled').Get() is False
    assert copied.GetPrimAtPath(points.GetPath()).GetMetadata('apiSchemas').ApplyOperations([])==['PhysxParticleCustomAPI']
    assert tuple(copied.GetPrimAtPath(points.GetPath()).GetAttribute('points').Get()[0])==(1,2,3)


def native_reply(stage,**request):
    from ovphysx_companion._worker import run_worker
    return run_worker(Path(__file__).with_name('native_mass_worker.py'),dict(scene=stage.GetRootLayer().ExportToString(),**request))


@pytest.mark.parametrize('purpose',['physics','allPurpose'])
def test_preview_inactive_bound_material_stays_unbound(inspector,purpose):
    from pxr import UsdShade
    stage,session=inspector
    material=UsdShade.Material.Define(stage,'/Materials/Inactive');UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateDensityAttr(100)
    # Bound on the selected body and on an unselected environment collider.
    for path in ('/World/BoxA','/World/Floor'):
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(material,materialPurpose=purpose if purpose=='physics' else UsdShade.Tokens.allPurpose)
    active=native_reply(stage,path='/World/BoxA')['mass']
    material.GetPrim().SetActive(False)
    copied=copied_snapshot(session)
    assert not copied.GetPrimAtPath(material.GetPath())
    source,snapshot=(native_reply(s,path='/World/BoxA')['mass'] for s in (stage,copied))
    assert active==pytest.approx(2400) and source==pytest.approx(24000) and snapshot==pytest.approx(source)


@pytest.mark.parametrize('rule',['explicitOnly','expandPrims'])
def test_preview_preserves_material_collection_membership_rules(inspector,rule):
    from pxr import UsdShade
    stage,session=inspector
    material=UsdShade.Material.Define(stage,'/Materials/Collected');UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateDensityAttr(100)
    world=stage.GetPrimAtPath('/World')
    members=Usd.CollectionAPI.Apply(world,'physicsMaterial');members.CreateIncludesRel().AddTarget('/World');members.CreateExpansionRuleAttr(rule)
    UsdShade.MaterialBindingAPI.Apply(world).Bind(members,material,'collected',materialPurpose='physics')
    copied=copied_snapshot(session)
    expected=None if rule=='explicitOnly' else material.GetPath()
    for input_stage in (stage,copied):
        bound=UsdShade.MaterialBindingAPI(input_stage.GetPrimAtPath('/World/BoxA')).ComputeBoundMaterial('physics')[0]
        assert (bound.GetPath() if bound else None)==expected
    source,snapshot=(native_reply(s,path='/World/BoxA')['mass'] for s in (stage,copied))
    assert source==pytest.approx(24000 if rule=='explicitOnly' else 2400) and snapshot==pytest.approx(source)


def test_preview_retains_mesh_merge_collision_geometry(inspector):
    stage,session=inspector
    merge=UsdGeom.Xform.Define(stage,'/World/Merge');merge.AddTranslateOp().Set((0,8,0))
    UsdPhysics.CollisionAPI.Apply(merge.GetPrim());merge.GetPrim().ApplyAPI('PhysxMeshMergeCollisionAPI')
    Usd.CollectionAPI.Apply(merge.GetPrim(),'collisionmeshes').CreateIncludesRel().AddTarget(merge.GetPath())
    # Contributes only through the collection: no physics schema of its own.
    child=UsdGeom.Mesh.Define(stage,'/World/Merge/Part');child.AddTranslateOp().Set((0,0,1))
    child.CreatePointsAttr([(x,y,z) for x in (0,1) for y in (0,1) for z in (0,1)])
    child.CreateFaceVertexCountsAttr([4]*6)
    child.CreateFaceVertexIndicesAttr([0,1,3,2, 4,6,7,5, 0,4,5,1, 2,3,7,6, 0,2,6,4, 1,5,7,3])
    render=UsdGeom.Mesh.Define(stage,'/World/RenderOnly');render.CreatePointsAttr([(0,0,0),(1,0,0),(0,1,0)])
    copied=copied_snapshot(session)
    assert copied.GetPrimAtPath(child.GetPath()).GetTypeName()=='Mesh'
    assert len(copied.GetPrimAtPath(child.GetPath()).GetAttribute('points').Get())==8
    assert copied.GetPrimAtPath(render.GetPath()).IsValid() is False
    source,snapshot=(native_reply(s,probe=(10.5,8.5,5))['distances'] for s in (stage,copied))
    assert source and snapshot==pytest.approx(source)


def test_preview_retains_geometry_of_non_gprim_collider(inspector):
    stage,session=inspector
    holder=UsdGeom.Xform.Define(stage,'/World/Holder');holder.AddTranslateOp().Set((0,8,0))
    UsdPhysics.CollisionAPI.Apply(holder.GetPrim())
    # Consumed by the Xform's collider: no physics schema of their own.
    mesh=UsdGeom.Mesh.Define(stage,'/World/Holder/Part')
    mesh.CreatePointsAttr([(x,y,z) for x in (0,1) for y in (0,1) for z in (0,1)])
    mesh.CreateFaceVertexCountsAttr([4]*6)
    mesh.CreateFaceVertexIndicesAttr([0,1,3,2, 4,6,7,5, 0,4,5,1, 2,3,7,6, 0,2,6,4, 1,5,7,3])
    cube=UsdGeom.Cube.Define(stage,'/World/Holder/Block');cube.CreateSizeAttr(1);cube.AddTranslateOp().Set((0,3,0))
    copied=copied_snapshot(session)
    assert copied.GetPrimAtPath(mesh.GetPath()).GetTypeName()=='Mesh'
    assert copied.GetPrimAtPath(cube.GetPath()).GetTypeName()=='Cube'
    for probe,distance in (((10.5,8.5,5),4.0),((10,11,5),4.5)):
        source,snapshot=(native_reply(s,probe=probe)['distances'] for s in (stage,copied))
        assert source==pytest.approx([distance]) and snapshot==pytest.approx(source)


def test_public_layout_preview_worker_uses_exported_checkout(inspector,tmp_path,monkeypatch):
    import shutil
    from ovphysx_companion.usd import preview
    stage,session=inspector
    # The public export ships the package and its build file, not AGENTS.md.
    source=Path(preview.__file__).resolve().parents[2]
    exported=tmp_path/'companion'/'python'
    shutil.copytree(source,exported,ignore=shutil.ignore_patterns('__pycache__','*.egg-info','build'))
    assert (exported/'pyproject.toml').is_file() and not (exported.parent/'AGENTS.md').exists()
    monkeypatch.setattr(preview,'__file__',str(exported/'ovphysx_companion'/'usd'/'preview.py'))
    monkeypatch.setenv('PYTHONPATH',str(tmp_path/'unrelated'))
    poses=simulate_snapshot(session,capture_preview(session,['/World/BoxA']),python_executable=sys.executable,dt=.01,steps=3)
    assert set(poses)=={'/World/BoxA'}
