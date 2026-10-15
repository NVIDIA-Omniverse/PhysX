# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MASS-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Customer USD capture/application around stopped native SI mass computation.

Capture default-time collider geometry. No source objects cross the job boundary.
The mass frame is the body's world rigid frame (scale baked into geometry).
"""
import math
from ..mass import Shape,ShapeKind,Request,Result,Overrides
from .session import _usd

def _static(attr):
    if attr and (attr.GetNumTimeSamples() > 0 or attr.ValueMightBeTimeVarying()):
        raise ValueError(f'{attr.GetPath()}: animated mass input is unsupported')
    return attr.Get() if attr else None

def _positive(prim,name,*,material=False):
    from pxr import UsdPhysics
    if not prim.HasAPI(UsdPhysics.MaterialAPI if material else UsdPhysics.MassAPI):return None
    attr=prim.GetAttribute(name)
    value=_static(attr)
    if value is None or value==0:return None
    if not math.isfinite(value) or value<0:raise ValueError(f'{prim.GetPath()}: invalid {name}')
    return float(value)

def _frame(matrix,path):
    from pxr import Gf
    scales=tuple(Gf.Vec3d(*matrix[i][:3]).GetLength() for i in range(3))
    if not all(math.isfinite(s) and s>0 for s in scales):raise ValueError(f'{path}: singular/nonfinite transform')
    basis=tuple(Gf.Vec3d(*matrix[i][:3])/scales[i] for i in range(3))
    if any(abs(Gf.Dot(basis[i],basis[j]))>1e-6 for i in range(3) for j in range(i)) or Gf.Dot(Gf.Cross(basis[0],basis[1]),basis[2])<0:
        raise ValueError(f'{path}: shear/reflection mass transforms are unsupported')
    rigid=Gf.Matrix4d(1)
    for i in range(3):rigid.SetRow(i,Gf.Vec4d(*basis[i],0))
    rigid.SetRow(3,Gf.Vec4d(*matrix.ExtractTranslation(),1))
    return scales,rigid

def _quaternion(rotation):
    q=rotation.GetQuat()
    return (*tuple(q.GetImaginary()),float(q.GetReal()))

def _overrides(prim,scale,meters,kg,axis=None,*,aggregate=False):
    from pxr import Gf
    from pxr import UsdPhysics
    if not prim.HasAPI(UsdPhysics.MassAPI):return Overrides()
    values={}
    mass=_positive(prim,'physics:mass')
    if mass is not None:values['mass']=mass*kg
    center=prim.GetAttribute('physics:centerOfMass')
    center=_static(center)
    # Schema sentinel (-inf,-inf,-inf) means derive; a partially invalid authored
    # center is an error, rather than silently changing authored semantics.
    if center is not None and not all(v==-math.inf for v in center):
        if not all(math.isfinite(v) for v in center):raise ValueError(f'{prim.GetPath()}: invalid center of mass')
        vector=Gf.Vec3d(*(center[i]*scale[i]*meters for i in range(3)))
        if axis is not None:vector=axis.GetInverse().TransformDir(vector)
        values['center']=tuple(vector)
    diagonal=prim.GetAttribute('physics:diagonalInertia')
    diagonal=_static(diagonal)
    if diagonal is not None and any(v!=0 for v in diagonal):
        if not all(math.isfinite(v) and v>0 for v in diagonal):raise ValueError(f'{prim.GetPath()}: invalid diagonal inertia')
        values['diagonal']=tuple(v*kg*meters*meters for v in diagonal)
    axes=prim.GetAttribute('physics:principalAxes')
    # Collider tensors default to identity axes. Body tensors retain computed
    # axes unless axes are authored, or mass/center/diagonal fully specify them.
    authored_axes=bool(axes and axes.HasAuthoredValueOpinion())
    use_axes=authored_axes if aggregate else 'diagonal' in values
    use_axes=use_axes or (aggregate and all(k in values for k in ('mass','center','diagonal')))
    if use_axes:
        q=_static(axes) if axes and axes.HasAuthoredValueOpinion() else Gf.Quatf(1)
        if q is not None:
            components=(*tuple(q.GetImaginary()),float(q.GetReal()))
            if not all(math.isfinite(v) for v in components) or abs(sum(v*v for v in components)-1)>1e-5:
                raise ValueError(f'{prim.GetPath()}: principal axes must be a unit quaternion')
            rotation=Gf.Rotation(Gf.Quatd(q))
            if axis is not None:rotation=Gf.Rotation(axis.GetInverse().GetQuat()*rotation.GetQuat())
            values['axes']=_quaternion(rotation)
    return Overrides(**values)

def capture_mass_request(session,body_path):
    """Snapshot supported colliders with USD density/material/override precedence.

    Nested rigid bodies, disabled collisions and triggers are excluded. Meshes
    require triangle topology and collision approximation 'none'; approximations
    must be explicitly cooked/snapshotted by a consumer before this operation.
    Sphere scale must be uniform, capsule radial scales equal. Shear/reflection,
    prototypes/instances and time-sampled inputs are not approximated silently.
    This computes geometric mass for the supported collider selection; runtime
    convex/fallback approximations or disabled-shape contributions may differ.
    """
    Sdf,_,Usd,UsdGeom,_=_usd()
    from pxr import Gf,UsdPhysics,UsdShade
    source=session.source
    path=Sdf.Path(body_path);body=session.stage.GetPrimAtPath(path)
    if not path.IsAbsolutePath() or not path.IsPrimPath() or not body or not body.IsActive() or body.IsInstance() or body.IsInstanceProxy() or body.IsInPrototype() or not body.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError('Select an editable rigid body')
    meters=UsdGeom.GetStageMetersPerUnit(session.stage);kg=UsdPhysics.GetStageKilogramsPerUnit(session.stage)
    if not all(math.isfinite(v) and v>0 for v in (meters,kg)):raise ValueError('Stage units must be positive')
    cache=UsdGeom.XformCache(Usd.TimeCode.Default())
    body_scale,body_rigid=_frame(cache.GetLocalToWorldTransform(body),path)
    density_scale=kg/meters**3
    body_density=_positive(body,'physics:density')
    shapes=[]
    iterator=iter(Usd.PrimRange(body,Usd.TraverseInstanceProxies()))
    for prim in iterator:
        if prim!=body and prim.HasAPI(UsdPhysics.RigidBodyAPI):iterator.PruneChildren();continue
        if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
        if prim.IsInstance() or prim.IsInstanceProxy() or prim.IsInPrototype():raise ValueError(f'{prim.GetPath()}: instance collider requires explicit snapshot')
        enabled=prim.GetAttribute('physics:collisionEnabled')
        if (enabled and enabled.Get() is False) or prim.HasAPI('PhysxTriggerAPI'):continue
        # Default-time authoring deliberately refuses animation instead of
        # claiming a mass snapshot represents an arbitrary sample.
        ancestor=prim
        while ancestor and not ancestor.IsPseudoRoot():
            if any((a.GetNumTimeSamples() > 0 or a.ValueMightBeTimeVarying()) for a in ancestor.GetAttributes() if a.GetName().startswith(('xformOp:','physics:'))):
                raise ValueError(f'{prim.GetPath()}: animated mass/transform input is unsupported')
            ancestor=ancestor.GetParent()
        scale,rigid=_frame(cache.GetLocalToWorldTransform(prim)*body_rigid.GetInverse(),prim.GetPath())
        density=_positive(prim,'physics:density') or body_density
        if density is None:
            material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
            # Runtime population follows USD's default traversal: an inactive,
            # unloaded, undefined or abstract material contributes no density.
            if material and Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)(material.GetPrim()):
                density=_positive(material.GetPrim(),'physics:density',material=True)
        density=density*density_scale if density is not None else 1000
        kind=None;points=();triangles=();axis=None
        dimensions=(0,0,0)
        if prim.IsA(UsdGeom.Cube):
            size=_static(UsdGeom.Cube(prim).GetSizeAttr())
            dimensions=tuple(size*s*meters/2 for s in scale);kind=ShapeKind.BOX
        elif prim.IsA(UsdGeom.Sphere):
            if max(scale)-min(scale)>1e-6*max(scale):raise ValueError(f'{prim.GetPath()}: nonuniform sphere requires mesh snapshot')
            dimensions=(_static(UsdGeom.Sphere(prim).GetRadiusAttr())*scale[0]*meters,0,0);kind=ShapeKind.SPHERE
        elif prim.IsA(UsdGeom.Capsule):
            capsule=UsdGeom.Capsule(prim);axis_index={'X':0,'Y':1,'Z':2}[_static(capsule.GetAxisAttr())]
            radial=[i for i in range(3) if i!=axis_index]
            if abs(scale[radial[0]]-scale[radial[1]])>1e-6*max(scale):raise ValueError(f'{prim.GetPath()}: unequal capsule radial scales')
            target=Gf.Vec3d(0);target[axis_index]=1
            axis=Gf.Rotation(Gf.Vec3d(1,0,0),target)
            dimensions=(_static(capsule.GetRadiusAttr())*scale[radial[0]]*meters,_static(capsule.GetHeightAttr())*scale[axis_index]*meters/2,0)
            kind=ShapeKind.CAPSULE
        elif prim.IsA(UsdGeom.Mesh):
            mesh=UsdGeom.Mesh(prim)
            approx=prim.GetAttribute('physics:approximation')
            if approx and approx.Get() not in (None,'none'):raise ValueError(f'{prim.GetPath()}: cook the requested collision approximation before mass calculation')
            attrs=(mesh.GetPointsAttr(),mesh.GetFaceVertexCountsAttr(),mesh.GetFaceVertexIndicesAttr())
            if any((a.GetNumTimeSamples() > 0 or a.ValueMightBeTimeVarying()) for a in attrs):raise ValueError(f'{prim.GetPath()}: animated mesh requires explicit snapshot')
            counts=attrs[1].Get();indices=attrs[2].Get();vertices=attrs[0].Get()
            if counts is None or indices is None or vertices is None or any(c!=3 for c in counts) or len(indices)!=3*len(counts):
                raise ValueError(f'{prim.GetPath()}: mass requires explicit triangular closed mesh')
            if _static(mesh.GetHoleIndicesAttr()):raise ValueError(f'{prim.GetPath()}: mesh holes are unsupported')
            points=tuple(tuple(p[i]*scale[i]*meters for i in range(3)) for p in vertices)
            triangles=tuple(tuple(int(indices[j+i]) for i in range(3)) for j in range(0,len(indices),3));kind=ShapeKind.MESH
        else:raise ValueError(f'{prim.GetPath()}: unsupported mass collider {prim.GetTypeName()}')
        rotation=rigid.ExtractRotation()
        if axis is not None:rotation=Gf.Rotation(rotation.GetQuat()*axis.GetQuat())
        # Runtime first weights every collider (including the body prim), then
        # normalizes the aggregate to the body's authored mass.
        overrides=_overrides(prim,scale,meters,kg,axis)
        shapes.append(Shape(str(prim.GetPath()),kind,dimensions,density,
            tuple(v*meters for v in rigid.ExtractTranslation()),_quaternion(rotation),points,triangles,overrides))
    session.validate(source)
    return Request(tuple(shapes),source,str(path),_overrides(body,body_scale,meters,kg,aggregate=True))

def apply_mass_result(session,result,*,layer):
    """Atomically author a matching-revision result; return the undoable patch."""
    if not isinstance(result,Result) or result.units!='SI' or result.frame!='body-local':raise TypeError('Pass an SI body-local mass Result')
    Sdf,_,Usd,UsdGeom,_=_usd()
    from pxr import Gf,UsdPhysics
    session.validate(result.source);session._validate_layer(layer)
    body=session.stage.GetPrimAtPath(result.body_path)
    if not body or not body.IsActive() or body.IsInstance() or body.IsInstanceProxy() or body.IsInPrototype() or not body.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError('Mass result body is no longer editable')
    meters=UsdGeom.GetStageMetersPerUnit(session.stage);kg=UsdPhysics.GetStageKilogramsPerUnit(session.stage)
    scale,_=_frame(UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(body),body.GetPath())
    p=result.properties
    mass=p.mass/kg;center=tuple(p.center_of_mass[i]/(meters*scale[i]) for i in range(3))
    diagonal=tuple(v/(kg*meters*meters) for v in p.diagonal_inertia);axes=p.principal_axes
    if not all(math.isfinite(v) and abs(v)<=3.402823466e38 for v in (mass,*center,*diagonal,*axes)) or mass<=0 or min(diagonal)<=0 or abs(sum(v*v for v in axes)-1)>1e-5:
        raise ValueError('Mass result cannot be represented in stage units')
    session.validate(result.source)
    def edit():
        api=UsdPhysics.MassAPI.Apply(body)
        if not api:raise RuntimeError('USD rejected MassAPI')
        for attr,value in ((api.CreateMassAttr(),mass),(api.CreateCenterOfMassAttr(),Gf.Vec3f(*center)),
            (api.CreateDiagonalInertiaAttr(),Gf.Vec3f(*diagonal)),(api.CreatePrincipalAxesAttr(),Gf.Quatf(axes[3],Gf.Vec3f(*axes[:3])))):
            if not attr.Set(value):raise RuntimeError('USD rejected mass property')
    return session._apply(layer,edit)
