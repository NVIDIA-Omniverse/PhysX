# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MASS-001
# @covers AC-1 AC-2 AC-3 AC-4
from dataclasses import replace
from concurrent.futures import CancelledError
import pytest
from pxr import UsdGeom,UsdPhysics,UsdShade
from ovphysx_companion.usd.mass import capture_mass_request,apply_mass_result
from ovphysx_companion.usd.session import StaleResultError
from ovphysx_companion.host import DeferredEdit
from stopped_mass_editor import compute_in_worker,run


def prepare(stage):
    UsdGeom.SetStageMetersPerUnit(stage,.01);UsdPhysics.SetStageKilogramsPerUnit(stage,.1)
    body=UsdGeom.Xform.Define(stage,'/Body');UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    cube=UsdGeom.Cube.Define(stage,'/Body/Collider');cube.CreateSizeAttr(200);cube.AddScaleOp().Set((1,2,3))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim());UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateDensityAttr(.0001)
    return body.GetPrim(),cube


def test_mass_units_overrides_and_atomic_application(scene):
    stage,session=scene;body,cube=prepare(stage)
    request=capture_mass_request(session,'/Body')
    assert request.shapes[0].dimensions==pytest.approx((1,2,3))
    assert request.shapes[0].density==pytest.approx(10)
    result=compute_in_worker(request)
    assert result.properties.mass==pytest.approx(480)
    before=stage.GetRootLayer().ExportToString()
    patch=apply_mass_result(session,result,layer=stage.GetRootLayer())
    assert body.GetAttribute('physics:mass').Get()==pytest.approx(4800)
    assert sorted(body.GetAttribute('physics:diagonalInertia').Get())==pytest.approx([80000000,160000000,208000000])
    after=stage.GetRootLayer().ExportToString();patch.undo();assert stage.GetRootLayer().ExportToString()==before
    with pytest.raises(StaleResultError):apply_mass_result(session,result,layer=stage.GetRootLayer())
    patch.redo();assert stage.GetRootLayer().ExportToString()==after
    # Aggregate mass authored above must not also scale the child collider.
    request2=capture_mass_request(session,'/Body');assert request2.overrides.mass==pytest.approx(480)
    assert request2.shapes[0].overrides.mass is None


def test_mass_density_precedence_and_nested_body_filter(scene):
    stage,session=scene;body,cube=prepare(stage)
    child=UsdPhysics.MassAPI(cube.GetPrim());child.GetDensityAttr().Clear()
    parent=UsdPhysics.MassAPI.Apply(body);parent.CreateDensityAttr(.0002)
    material=UsdShade.Material.Define(stage,'/Material');UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateDensityAttr(.0003)
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material,materialPurpose='physics')
    assert capture_mass_request(session,'/Body').shapes[0].density==pytest.approx(20)
    parent.GetDensityAttr().Clear()
    assert capture_mass_request(session,'/Body').shapes[0].density==pytest.approx(30)
    child.CreateDensityAttr(.0004)
    assert capture_mass_request(session,'/Body').shapes[0].density==pytest.approx(40)
    nested=UsdGeom.Cube.Define(stage,'/Body/Nested');UsdPhysics.CollisionAPI.Apply(nested.GetPrim());UsdPhysics.RigidBodyAPI.Apply(nested.GetPrim())
    assert len(capture_mass_request(session,'/Body').shapes)==1
    child.CreateMassAttr(100);child.CreateCenterOfMassAttr((1,2,3));child.CreateDiagonalInertiaAttr((10,20,30))
    request=capture_mass_request(session,'/Body')
    assert request.shapes[0].overrides.mass==pytest.approx(10)
    assert request.shapes[0].overrides.center==pytest.approx((.01,.04,.09))
    assert request.shapes[0].overrides.diagonal==pytest.approx((.0001,.0002,.0003))


def test_mass_stale_cancel_delete_and_invalid_results_do_not_edit(scene):
    stage,session=scene;body,cube=prepare(stage)
    result=compute_in_worker(capture_mass_request(session,'/Body'))
    class Ready:
        done=True
        def result(self):return result
        def cancel(self):return False
    deliver=DeferredEdit(session,result.source,Ready(),lambda r:apply_mass_result(session,r,layer=stage.GetRootLayer()))
    before=stage.GetRootLayer().ExportToString();deliver.cancel()
    with pytest.raises(CancelledError):deliver.deliver()
    assert stage.GetRootLayer().ExportToString()==before
    invalid=replace(result,properties=replace(result.properties,mass=float('nan')))
    with pytest.raises(ValueError):apply_mass_result(session,invalid,layer=stage.GetRootLayer())
    assert stage.GetRootLayer().ExportToString()==before
    cube.GetSizeAttr().Set(400)
    with pytest.raises(StaleResultError):apply_mass_result(session,result,layer=stage.GetRootLayer())
    fresh=compute_in_worker(capture_mass_request(session,'/Body'));stage.RemovePrim('/Body')
    before=stage.GetRootLayer().ExportToString()
    with pytest.raises(StaleResultError):apply_mass_result(session,fresh,layer=stage.GetRootLayer())
    assert stage.GetRootLayer().ExportToString()==before


def test_mass_scaled_body_and_mesh_capture_rejects_unsupported(scene):
    stage,session=scene;body,cube=prepare(stage)
    UsdGeom.Xformable(body).AddScaleOp().Set((2,1,1))
    result=compute_in_worker(capture_mass_request(session,'/Body'))
    assert result.properties.mass==pytest.approx(960)
    stage.RemovePrim('/Body/Collider')
    mesh=UsdGeom.Mesh.Define(stage,'/Body/Collider');UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    mesh.CreatePointsAttr([(0,0,0),(1,0,0),(0,1,0),(0,0,1)])
    mesh.CreateFaceVertexCountsAttr([3]*4);mesh.CreateFaceVertexIndicesAttr([0,2,1,0,1,3,0,3,2,1,2,3])
    request=capture_mass_request(session,'/Body');assert len(request.shapes[0].triangles)==4
    mesh.GetFaceVertexCountsAttr().Set([4,4,4])
    with pytest.raises(ValueError,match='triangular'):capture_mass_request(session,'/Body')
    stage.RemovePrim('/Body/Collider');sphere=UsdGeom.Sphere.Define(stage,'/Body/Collider');UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    with pytest.raises(ValueError,match='nonuniform sphere'):capture_mass_request(session,'/Body')


def test_stopped_mass_editor_sample():
    stage=run();assert stage.GetPrimAtPath('/Body').GetAttribute('physics:mass').Get()==480


def test_capsule_axis_scaled_center_and_authored_body_mass(scene):
    from pxr import Gf
    stage,session=scene
    UsdGeom.SetStageMetersPerUnit(stage,1);UsdPhysics.SetStageKilogramsPerUnit(stage,1)
    capsule=UsdGeom.Capsule.Define(stage,'/Capsule');capsule.CreateAxisAttr('Z')
    capsule.CreateRadiusAttr(1);capsule.CreateHeightAttr(4)
    UsdPhysics.RigidBodyAPI.Apply(capsule.GetPrim());UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
    api=UsdPhysics.MassAPI.Apply(capsule.GetPrim());api.CreateMassAttr(10)
    request=capture_mass_request(session,'/Capsule')
    assert request.shapes[0].overrides.mass==10 and request.overrides.mass==10
    result=compute_in_worker(request)
    # Capsule's long Z axis has its smallest principal moment; reconstruct the
    # diagonal in the body frame to avoid depending on eigenvector ordering.
    q=result.properties.principal_axes;r=Gf.Matrix3d(Gf.Quatd(q[3],Gf.Vec3d(*q[:3])))
    diagonal=[sum(float(r[k][i])**2*result.properties.diagonal_inertia[k] for k in range(3)) for i in range(3)]
    assert diagonal[2]<diagonal[0] and diagonal[0]==pytest.approx(diagonal[1])
    capsule.AddScaleOp().Set((2,2,3));api.CreateCenterOfMassAttr((1,2,3))
    result=compute_in_worker(capture_mass_request(session,'/Capsule'))
    assert result.properties.center_of_mass==pytest.approx((2,4,9))
    apply_mass_result(session,result,layer=stage.GetRootLayer())
    assert tuple(api.GetCenterOfMassAttr().Get())==pytest.approx((1,2,3))
    api.CreatePrincipalAxesAttr(Gf.Quatf(0,Gf.Vec3f(0)))
    with pytest.raises(ValueError,match='unit quaternion'):capture_mass_request(session,'/Capsule')


def test_inactive_mass_api_opinions_are_ignored(scene):
    stage,session=scene;body,cube=prepare(stage)
    api=UsdPhysics.MassAPI.Apply(body);api.CreateMassAttr(10);api.CreateDensityAttr(.0009)
    child=UsdPhysics.MassAPI(cube.GetPrim());child.CreateMassAttr(20)
    body.RemoveAPI(UsdPhysics.MassAPI);cube.GetPrim().RemoveAPI(UsdPhysics.MassAPI)
    request=capture_mass_request(session,'/Body')
    assert request.overrides.mass is None and request.shapes[0].overrides.mass is None
    assert request.shapes[0].density==1000


@pytest.mark.parametrize('input_name',['size','material-density'])
def test_mass_rejects_sampled_primitive_and_material_inputs(scene,input_name):
    stage,session=scene;body,cube=prepare(stage)
    if input_name=='size':cube.GetSizeAttr().Set(300,10)
    else:
        UsdPhysics.MassAPI(cube.GetPrim()).GetDensityAttr().Clear()
        material=UsdShade.Material.Define(stage,'/Material')
        UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateDensityAttr().Set(.0003,10)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material,materialPurpose='physics')
    with pytest.raises(ValueError,match='animated'):capture_mass_request(session,'/Body')


def test_mass_rejects_instance_proxy_collider_in_group(scene):
    stage,session=scene;body,cube=prepare(stage)
    group=UsdGeom.Xform.Define(stage,'/Template')
    shape=UsdGeom.Cube.Define(stage,'/Template/Collider');UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    instance=UsdGeom.Xform.Define(stage,'/Body/Instance').GetPrim()
    instance.GetReferences().AddInternalReference('/Template');instance.SetInstanceable(True)
    assert instance.IsInstance() and stage.GetPrimAtPath('/Body/Instance/Collider').IsInstanceProxy()
    with pytest.raises(ValueError,match='instance collider'):capture_mass_request(session,'/Body')


@pytest.mark.parametrize('axes_only',[False,True])
def test_child_capsule_inertia_overrides_use_authored_source_frame(scene,axes_only):
    from pxr import Gf
    stage,session=scene
    UsdGeom.SetStageMetersPerUnit(stage,1);UsdPhysics.SetStageKilogramsPerUnit(stage,1)
    body=UsdGeom.Xform.Define(stage,'/Body');UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    capsule=UsdGeom.Capsule.Define(stage,'/Body/Capsule');capsule.CreateAxisAttr('Z')
    capsule.CreateRadiusAttr(1);capsule.CreateHeightAttr(4);UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
    if axes_only:
        reference=compute_in_worker(capture_mass_request(session,'/Body'))
        UsdPhysics.MassAPI.Apply(capsule.GetPrim()).CreatePrincipalAxesAttr(Gf.Quatf(1))
    else:UsdPhysics.MassAPI.Apply(capsule.GetPrim()).CreateDiagonalInertiaAttr((1,2,3))
    result=compute_in_worker(capture_mass_request(session,'/Body'))
    if axes_only:
        assert result.properties.diagonal_inertia==pytest.approx(reference.properties.diagonal_inertia)
        assert result.properties.principal_axes==pytest.approx(reference.properties.principal_axes)
    else:
        q=result.properties.principal_axes;r=Gf.Matrix3d(Gf.Quatd(q[3],Gf.Vec3d(*q[:3])))
        diagonal=[sum(float(r[k][i])**2*result.properties.diagonal_inertia[k] for k in range(3)) for i in range(3)]
        assert diagonal==pytest.approx((1,2,3))


def test_material_schema_removal_ignores_leftover_density(scene):
    stage,session=scene;body,cube=prepare(stage)
    UsdPhysics.MassAPI(cube.GetPrim()).GetDensityAttr().Clear()
    material=UsdShade.Material.Define(stage,'/Material')
    UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateDensityAttr(.0003)
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material,materialPurpose='physics')
    assert capture_mass_request(session,'/Body').shapes[0].density==pytest.approx(30)
    material.GetPrim().RemoveAPI(UsdPhysics.MaterialAPI)
    assert material.GetPrim().GetAttribute('physics:density').Get()==pytest.approx(.0003)
    assert capture_mass_request(session,'/Body').shapes[0].density==1000


@pytest.mark.parametrize('active',[True,False])
def test_inactive_material_density_is_ignored(scene,active):
    stage,session=scene
    UsdGeom.SetStageMetersPerUnit(stage,1);UsdPhysics.SetStageKilogramsPerUnit(stage,1)
    cube=UsdGeom.Cube.Define(stage,'/Body');cube.CreateSizeAttr(1)
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim());UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    material=UsdShade.Material.Define(stage,'/Material')
    UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateDensityAttr(100)
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material,materialPurpose='physics')
    material.GetPrim().SetActive(active)
    result=compute_in_worker(capture_mass_request(session,'/Body')).properties
    assert result.mass==pytest.approx(100 if active else 1000)
    assert_native_mass_matches(stage,'/Body',result)


def test_body_collider_compound_mass_weights_own_shape(scene):
    stage,session=scene
    UsdGeom.SetStageMetersPerUnit(stage,1);UsdPhysics.SetStageKilogramsPerUnit(stage,1)
    body=UsdGeom.Cube.Define(stage,'/Compound');body.CreateSizeAttr(2)
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim());UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(10)
    child=UsdGeom.Cube.Define(stage,'/Compound/Child');child.CreateSizeAttr(2);child.AddTranslateOp().Set((4,0,0))
    UsdPhysics.CollisionAPI.Apply(child.GetPrim())
    result=compute_in_worker(capture_mass_request(session,'/Compound')).properties
    assert result.mass==pytest.approx(10)
    assert result.center_of_mass==pytest.approx((4*8000/8010,0,0),abs=1e-5)
    assert sorted(result.diagonal_inertia)==pytest.approx([20/3,6.866168,6.866168],abs=1e-4)
    assert_native_mass_matches(stage,'/Compound',result)


@pytest.mark.parametrize('axes_only',[True,False])
def test_body_inertia_override_keeps_authored_or_computed_axes(scene,axes_only):
    from pxr import Gf
    stage,session=scene
    UsdGeom.SetStageMetersPerUnit(stage,1);UsdPhysics.SetStageKilogramsPerUnit(stage,1)
    body=UsdGeom.Xform.Define(stage,'/Rotated');UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    shape=UsdGeom.Cube.Define(stage,'/Rotated/Shape');shape.CreateSizeAttr(2)
    shape.AddRotateZOp().Set(30);shape.AddScaleOp().Set((1,2,3));UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    api=UsdPhysics.MassAPI.Apply(body.GetPrim())
    if axes_only:api.CreatePrincipalAxesAttr(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,0,1),20).GetQuat()))
    else:api.CreateDiagonalInertiaAttr((10,20,30))
    request=capture_mass_request(session,'/Rotated')
    assert (request.overrides.axes is not None)==axes_only
    result=compute_in_worker(request).properties
    q=result.principal_axes;rotation=Gf.Matrix3d(Gf.Quatd(q[3],Gf.Vec3d(*q[:3])))
    tensor=rotation.GetTranspose()*Gf.Matrix3d(Gf.Vec3d(*result.diagonal_inertia))*rotation
    assert abs(tensor[0][1])>1
    assert_native_mass_matches(stage,'/Rotated',result)



def assert_native_mass_matches(stage,path,properties):
    from pathlib import Path
    from pxr import Gf
    from ovphysx_companion._worker import run_worker
    actual=run_worker(Path(__file__).with_name('native_mass_worker.py'),dict(scene=stage.GetRootLayer().ExportToString(),path=path))
    assert properties.mass==pytest.approx(actual['mass'],rel=1e-5)
    assert properties.center_of_mass==pytest.approx(actual['center'],abs=1e-5)
    def tensor(diagonal,axes):
        rotation=Gf.Matrix3d(Gf.Quatd(axes[3],Gf.Vec3d(*axes[:3])))
        matrix=rotation.GetTranspose()*Gf.Matrix3d(Gf.Vec3d(*diagonal))*rotation
        return tuple(float(v) for row in matrix for v in row)
    assert tensor(properties.diagonal_inertia,properties.principal_axes)==pytest.approx(tensor(actual['diagonal'],actual['axes']),rel=1e-5,abs=1e-3)
