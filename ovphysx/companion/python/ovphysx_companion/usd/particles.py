# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PARTICLES-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Customer-USD capture and reversible particle/surface publication, no runtime."""
from dataclasses import dataclass
import math
from .session import _usd,StaleResultError,DependencyError
from .meshes import capture_mesh
from .deformables import _schema,_put,_set,_points,_elements
from ..particles import SamplingInput,Samples

_OWNER='ovphysxCompanion:particleOwner'


def capture_sampling_input(session,path):
    """Capture explicit triangles in world coordinates, stage length units.

    Triangulate polygons explicitly with the cooking service before using this
    helper; it never invents a fan for concave polygons. Native work should run in
    a separate worker if its ovstage runtime embeds another USD image.
    """
    snapshot=capture_mesh(session,path)
    if any(n!=3 for n in snapshot.face_counts):raise ValueError('Sampling capture requires triangulated mesh topology')
    from pxr import Gf
    matrix=Gf.Matrix4d(*[v for row in snapshot.local_to_world for v in row])
    points=tuple(tuple(matrix.Transform(Gf.Vec3d(*p))) for p in snapshot.points)
    triangles=tuple(tuple(snapshot.face_indices[i:i+3]) for i in range(0,len(snapshot.face_indices),3))
    return SamplingInput(snapshot.source,points,triangles,'world',snapshot.meters_per_unit)


@dataclass(frozen=True)
class Surface:
    """Copied world-space triangle surface from an explicitly selected provider.

    This is an authoring record, not a claim that native isosurface is available.
    """
    source:object
    points:tuple
    triangles:tuple
    meters_per_unit:float=1
    frame:str='world'
    units:str='stage'


def _prepare(session,result,path,layer,owner,kind):
    Sdf,_,Usd,UsdGeom,_=_usd()
    from pxr import Gf
    session.validate(result.source);session._validate_layer(layer)
    if result.frame!='world' or result.units!='stage' or result.meters_per_unit!=UsdGeom.GetStageMetersPerUnit(session.stage):
        raise ValueError('Output requires world coordinates in this stage length unit')
    if not isinstance(owner,str) or not owner:raise ValueError('Declare a stable authoring owner')
    path=Sdf.Path(path)
    if not path.IsAbsolutePath() or not path.IsPrimPath() or path==Sdf.Path.absoluteRootPath:raise ValueError('Expected absolute output prim path')
    prim=session.stage.GetPrimAtPath(path);parent=session.stage.GetPrimAtPath(path.GetParentPath())
    for candidate in (parent,prim if prim else parent):
        if not candidate or not candidate.IsActive() or not candidate.IsLoaded() or candidate.IsInstance() or candidate.IsInstanceProxy() or candidate.IsInPrototype():
            raise ValueError('Output parent/prim must be active, loaded and outside instances')
    if prim and (prim.GetTypeName()!=kind or prim.GetCustomDataByKey(_OWNER)!=owner):
        raise ValueError('Existing output must have the expected type and authoring owner')
    points=_points(result.points)
    matrix=UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim if prim else parent)
    if not math.isfinite(matrix.GetDeterminant()) or abs(matrix.GetDeterminant())<1e-12:raise ValueError('Output transform is singular')
    inverse=matrix.GetInverse()
    local=tuple(tuple(inverse.Transform(Gf.Vec3d(*p))) for p in points)
    _points(local)
    return path,local


def apply_particle_set(session,result,*,path,particle_system,layer,owner,fluid=True,particle_group=0):
    """Publish a selected Points set with zero initial velocities, explicit system.

    Existing unrelated properties remain intact. System creation and fluid solver
    settings are caller decisions; this helper never attaches or steps physics.
    """
    if not isinstance(result,Samples):raise TypeError('Pass copied Samples')
    Sdf,_,Usd,UsdGeom,Vt=_usd()
    if not Usd.SchemaRegistry().FindAppliedAPIPrimDefinition('PhysxParticleSetAPI'):
        raise DependencyError('Register compatible PhysxParticleSetAPI before particle authoring')
    path,points=_prepare(session,result,path,layer,owner,'Points')
    if not isinstance(fluid,bool) or isinstance(particle_group,bool) or not isinstance(particle_group,int) or not 0<=particle_group<2**20:
        raise ValueError('Expected boolean fluid and particle group in 0..2**20-1')
    system_path=Sdf.Path(particle_system);system=session.stage.GetPrimAtPath(system_path)
    if not system_path.IsAbsolutePath() or not system or not system.IsActive() or system.GetTypeName()!='PhysxParticleSystem':
        raise ValueError('Select an existing active PhysxParticleSystem')
    def edit():
        geometry=UsdGeom.Points.Define(session.stage,path);prim=geometry.GetPrim()
        _schema(prim,'PhysxParticleSetAPI')
        _put(geometry.CreatePointsAttr(),Vt.Vec3fArray(points))
        _put(geometry.CreateVelocitiesAttr(),Vt.Vec3fArray([(0,0,0)]*len(points)))
        _set(prim,'physxParticle:fluid',Sdf.ValueTypeNames.Bool,fluid)
        _set(prim,'physxParticle:particleGroup',Sdf.ValueTypeNames.Int,particle_group)
        if not prim.CreateRelationship('physxParticle:particleSystem',custom=False).SetTargets([system_path]):
            raise RuntimeError('USD rejected particle system relationship')
        prim.SetCustomDataByKey(_OWNER,owner)
    return session._apply(layer,edit)


def apply_surface(session,result,*,path,layer,owner):
    """Publish an owned mesh from a provider's copied surface, in one undo batch."""
    if not isinstance(result,Surface):raise TypeError('Pass a copied Surface')
    _,_,_,UsdGeom,Vt=_usd()
    path,points=_prepare(session,result,path,layer,owner,'Mesh')
    triangles=_elements(result.triangles,len(points),3)
    def edit():
        mesh=UsdGeom.Mesh.Define(session.stage,path)
        _put(mesh.CreatePointsAttr(),Vt.Vec3fArray(points))
        _put(mesh.CreateFaceVertexCountsAttr(),Vt.IntArray([3]*len(triangles)))
        _put(mesh.CreateFaceVertexIndicesAttr(),Vt.IntArray([i for t in triangles for i in t]))
        _put(mesh.CreateSubdivisionSchemeAttr(),'none')
        _put(mesh.CreateOrientationAttr(),'rightHanded')
        mesh.GetPrim().SetCustomDataByKey(_OWNER,owner)
    return session._apply(layer,edit)
