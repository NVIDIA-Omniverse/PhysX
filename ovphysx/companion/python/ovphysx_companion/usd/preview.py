# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-PREVIEW-001
# @covers AC-2 AC-3 AC-4
"""Customer-USD Inspector snapshots and explicit, reversible pose commits.

Capture retains composed physics dependencies in the snapshot; only selected rigid
bodies are committed. Native simulation receives a file/data snapshot, never a
customer USD pointer. A worker helper is provided for installations where native
ovstage population and customer USD use different USD images.
"""
from dataclasses import dataclass
import math
import os
from pathlib import Path
import subprocess
import tempfile
import json
import weakref

from .session import SourceRevision, _usd


@dataclass(frozen=True)
class PreviewBody:
    path: str
    world: tuple
    parent_inverse: tuple
    reset_stack: bool


@dataclass(frozen=True)
class PreviewSnapshot:
    source: SourceRevision
    usda: str
    bodies: tuple
    time_code: object


def _matrix_tuple(matrix):
    return tuple(tuple(float(value) for value in row) for row in matrix)


def _physics_snapshot(stage,time,time_code):
    """Copy composed physics, dependency targets and ancestor transforms only.

    Read geometry arrays only on collision/deformable/particle prims. Render
    meshes, shaders and assets are never flattened as an intermediate step.
    Instances are expanded into copied composed prims with stable source paths.
    """
    from pxr import Sdf,Usd,UsdGeom,UsdPhysics,UsdShade
    prims = {prim.GetPath():prim for prim in Usd.PrimRange(stage.GetPseudoRoot(),Usd.TraverseInstanceProxies())
             if not prim.IsPseudoRoot()}
    def schemas(prim):
        authored=prim.GetMetadata('apiSchemas')
        return authored.ApplyOperations([]) if authored else prim.GetAppliedSchemas()
    def physical(prim):
        return (prim.IsA(UsdPhysics.Scene) or prim.IsA(UsdPhysics.Joint) or
                str(prim.GetTypeName()).startswith(('Physics','Physx','OmniPhysics','Newton')) or
                any(str(api).startswith(('Physics','Physx','OmniPhysics','Newton')) for api in schemas(prim)))
    retained = {path for path,prim in prims.items() if physical(prim)}
    retained.update(path for path,prim in prims.items() if prim.IsA(UsdGeom.Subset) and
                    prim.GetParent().HasAPI(UsdPhysics.CollisionAPI))
    # A mesh-merge collider consumes the meshes of its collisionmeshes
    # collection, which need no CollisionAPI of their own (runtime:
    # resolveCollection + parseMeshGeometry per member). A collider on a
    # non-geometry prim (e.g. an Xform) likewise consumes its descendant gprims
    # (runtime: emitShape's descent; the ovstage walker reads direct children,
    # a subset of these).
    merged = set()
    for path in tuple(retained):
        if prims[path].HasAPI('PhysxMeshMergeCollisionAPI'):
            query = Usd.CollectionAPI(prims[path],'collisionmeshes').ComputeMembershipQuery()
            merged.update(member for member in Usd.CollectionAPI.ComputeIncludedPaths(
                query,stage,Usd.TraverseInstanceProxies()) if member in prims and prims[member].IsA(UsdGeom.Mesh))
        elif prims[path].HasAPI(UsdPhysics.CollisionAPI) and not prims[path].IsA(UsdGeom.Gprim):
            merged.update(child.GetPath() for child in Usd.PrimRange(prims[path],Usd.TraverseInstanceProxies())
                          if child.GetPath() in prims and child.IsA(UsdGeom.Gprim))
    retained.update(merged)
    # Bake the resolved physics-purpose material on each collider/subset. This
    # preserves collection and ancestor-strength semantics without render data.
    # A resolved material outside the traversal (inactive, abstract or undefined)
    # is never populated, so the runtime treats its collider as unbound. Keep the
    # copied binding to the absent prim rather than defining or activating it.
    materials = {}
    for path in tuple(retained):
        prim = prims[path]
        if prim.HasAPI(UsdPhysics.CollisionAPI) or prim.IsA(UsdGeom.Subset):
            material,_ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
            if material and material.GetPath() in prims:
                materials[path] = material.GetPath()
                retained.add(material.GetPath())
    physics_ancestors={ancestor for path in retained for ancestor in path.GetAncestorsRange()}
    instancers={path for path,prim in prims.items() if prim.IsA(UsdGeom.PointInstancer) and
                any(target in physics_ancestors for target in UsdGeom.PointInstancer(prim).GetPrototypesRel().GetTargets())}
    retained.update(instancers)
    def relationship(rel):
        return rel.GetName()=='prototypes' or rel.GetName().startswith(('physics:','physx','omniphysics:','omni:physics:',
                                          'newton:','material:binding','collection:'))
    pending = list(retained)
    while pending:
        prim = prims[pending.pop()]
        dependencies = [p.GetPrimPath() for rel in prim.GetRelationships() if relationship(rel)
                        for p in rel.GetForwardedTargets()]
        dependencies.extend(prim.GetPath().GetAncestorsRange())
        for path in dependencies:
            if path in prims and path not in retained:
                retained.add(path)
                pending.append(path)
    copied = Usd.Stage.CreateInMemory()
    for name in ('metersPerUnit','kilogramsPerUnit','upAxis','timeCodesPerSecond',
                 'framesPerSecond','startTimeCode','endTimeCode'):
        if stage.HasAuthoredMetadata(name):copied.SetMetadata(name,stage.GetMetadata(name))
    for path in sorted(retained):
        prim = prims[path]
        is_physics = physical(prim)
        geometry = (prim.HasAPI(UsdPhysics.CollisionAPI) or prim.IsA(UsdGeom.Subset) or prim.IsA(UsdGeom.PointInstancer) or
                    path in merged or
                    prim.HasAPI('PhysxCharacterControllerAPI') or
                    str(prim.GetTypeName()).startswith(('PhysxParticle','OmniPhysicsParticle')) or
                    any(str(api).startswith(('OmniPhysicsDeformable','PhysxDeformable','PhysxParticle')) for api in schemas(prim)))
        # A render-only geometry prim can be a dependency or rigid-body owner;
        # retain its transform and physics APIs without its rendering geometry.
        type_name = prim.GetTypeName()
        if prim.IsA(UsdGeom.Gprim) and not geometry:type_name = 'Xform'
        destination = copied.DefinePrim(path,type_name)
        destination.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(list(dict.fromkeys((*schemas(prim),*prim.GetAppliedSchemas())))))
        if prim.IsA(UsdGeom.PointInstancer) and prim.HasAuthoredMetadata('inactiveIds'):
            destination.SetMetadata('inactiveIds',prim.GetMetadata('inactiveIds'))
        for attr in prim.GetAttributes():
            name = attr.GetName()
            if name.startswith('primvars:') or name=='normals':continue
            # Collection rules travel with the copied collection relationships:
            # material-binding and mesh-merge membership depend on them.
            if not (geometry or (is_physics and not prim.IsA(UsdGeom.Imageable)) or
                    name.startswith(('xformOp:','physics:','physx','omniphysics:','omni:physics:','newton:','drive:','limit:','state:',
                                     'collection:')) or
                    name == 'xformOpOrder'):
                continue
            target = destination.CreateAttribute(name,attr.GetTypeName(),custom=attr.IsCustom(),variability=attr.GetVariability())
            value = attr.Get(time if time_code is not None else Usd.TimeCode.Default())
            if value is not None:target.Set(value)
            if time_code is None:
                for sample in attr.GetTimeSamples():target.Set(attr.Get(sample),sample)
        for rel in prim.GetRelationships():
            if relationship(rel):destination.CreateRelationship(rel.GetName(),custom=rel.IsCustom()).SetTargets(rel.GetForwardedTargets())
    for path,material in materials.items():
        UsdShade.MaterialBindingAPI.Apply(copied.GetPrimAtPath(path)).Bind(
            UsdShade.Material(copied.GetPrimAtPath(material)),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,materialPurpose='physics')
    return copied.GetRootLayer().ExportToString()


def capture_preview(session, paths, *, time_code=None):
    """Capture immutable scene data and selection at the approved USD thread.

    Nested selected rigid bodies and instance proxies are rejected explicitly.
    time_code=None captures default transforms; numeric times use USD time codes,
    independently of the native simulation clock. No source opinions are changed.
    """
    Sdf, _, Usd, UsdGeom, _ = _usd()
    from pxr import Gf, UsdPhysics
    session._check()
    if time_code is not None and (isinstance(time_code, bool) or not math.isfinite(time_code)):
        raise ValueError('time_code must be None or a finite USD time')
    paths = tuple(Sdf.Path(path) for path in paths)
    if not paths or len(set(paths)) != len(paths):
        raise ValueError('Select unique rigid-body paths')
    selected = set(paths)
    for path in paths:
        if not path.IsAbsolutePath() or not path.IsPrimPath():
            raise ValueError('Select absolute prim paths')
        if any(ancestor in selected for ancestor in path.GetAncestorsRange() if ancestor != path):
            raise ValueError('Nested selected rigid bodies require an explicit host mapping')
    time = Usd.TimeCode.Default() if time_code is None else Usd.TimeCode(time_code)
    cache = UsdGeom.XformCache(time)
    bodies = []
    for path in paths:
        prim = session.stage.GetPrimAtPath(path)
        xform = UsdGeom.Xformable(prim)
        if not prim or not xform or not prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.IsInstanceProxy():
            raise ValueError(f'{path} must be an editable rigid body')
        world = cache.GetLocalToWorldTransform(prim)
        parent = cache.GetParentToWorldTransform(prim)
        if abs(parent.GetDeterminant()) < 1e-12:
            raise ValueError(f'{path} has a singular parent transform')
        bodies.append(PreviewBody(str(path), _matrix_tuple(world), _matrix_tuple(parent.GetInverse()),
                                  bool(xform.GetResetXformStack())))
    source = session.source
    usda = _physics_snapshot(session.stage,time,time_code)
    session.validate(source)
    return PreviewSnapshot(source, usda, tuple(bodies), time_code)


def commit_preview(session, snapshot, poses, *, layer):
    """Commit an exact selected pose batch, preserving scale and unrelated opinions.

    Poses map selected paths to objects with world `position` and xyzw
    `orientation`, in snapshot stage distance units. Use a separate local layer
    for preview display. Returned patches support host undo/redo. Selection,
    Play or undo changes must call session.invalidate() before old results commit.
    """
    return _apply_preview(session,snapshot,poses,layer=layer,output=False)


def _apply_preview(session,snapshot,poses,*,layer,output):
    _, _, Usd, UsdGeom, _ = _usd()
    from pxr import Gf
    session.validate(snapshot.source)
    session._validate_layer(layer)
    if set(poses) != {body.path for body in snapshot.bodies}:
        raise ValueError('Commit must contain exactly the captured selection')
    prepared = []
    for body in snapshot.bodies:
        pose = poses[body.path]
        position, rotation = tuple(pose.position), tuple(pose.orientation)
        if len(position) != 3 or len(rotation) != 4 or not all(math.isfinite(v) for v in position + rotation):
            raise ValueError('Pose requires finite xyz and xyzw')
        if abs(sum(v*v for v in rotation) - 1) > 1e-4:
            raise ValueError('Pose rotation must be a unit quaternion')
        world = Gf.Transform(Gf.Matrix4d(*body.world))
        world.SetTranslation(Gf.Vec3d(*position))
        world.SetRotation(Gf.Rotation(Gf.Quatd(rotation[3], Gf.Vec3d(*rotation[:3]))))
        matrix = world.GetMatrix()
        if not body.reset_stack:
            matrix = matrix * Gf.Matrix4d(*body.parent_inverse)
        prim = session.stage.GetPrimAtPath(body.path)
        if not prim or prim.IsInstanceProxy():
            raise ValueError(f'Invalid preview destination {body.path}')
        if not output:
            # Replacing the uniform op order cannot preserve interpolation of
            # arbitrary animated stacks. Refuse before editing any destination.
            ancestor = prim
            while ancestor and not ancestor.IsPseudoRoot():
                if any((attr.GetNumTimeSamples() > 0 or attr.ValueMightBeTimeVarying()) for attr in ancestor.GetAttributes()
                       if attr.GetName().startswith('xformOp:')):
                    raise ValueError('Animated transform commits require an explicit host animation mapping')
                ancestor_xform=UsdGeom.Xformable(ancestor)
                if ancestor_xform and ancestor_xform.GetResetXformStack():break
                ancestor = ancestor.GetParent()
        prepared.append((UsdGeom.Xformable(prim), matrix, body.reset_stack))
    # Capture time selects the evaluation pose. A static commit remains a
    # default opinion; numeric capture must not create an animated op stack.
    time = Usd.TimeCode.Default() if not output or snapshot.time_code is None else Usd.TimeCode(snapshot.time_code)
    def edit():
        for xform, matrix, reset in prepared:
            # Dedicated override op: preserve original authored transform values.
            op_name = 'xformOp:transform:companionPreview'
            attr = xform.GetPrim().GetAttribute(op_name)
            if attr:
                op = UsdGeom.XformOp(attr)
                if op.GetOpType() != UsdGeom.XformOp.TypeTransform:
                    raise ValueError('Preview transform property has an incompatible type')
            else:
                op = xform.AddTransformOp(UsdGeom.XformOp.PrecisionDouble, 'companionPreview')
            if not op.Set(matrix, time) or not xform.SetXformOpOrder([op], reset):
                raise RuntimeError('USD rejected preview transform')
    return session._apply(layer, edit, output=output)


def simulate_snapshot(session, snapshot, *, python_executable, dt=1/60, steps=1, timeout=60):
    """Explicit worker fallback using an installed native core/companion environment.

    Runs no native library in the customer-USD process. Host may execute this
    synchronous helper off-thread only after arranging revision checks back on
    the USD thread; this convenience function itself remains thread-affine.
    """
    from ..preview import Pose
    session.validate(snapshot.source)
    if isinstance(steps, bool) or not isinstance(steps, int) or not 1 <= steps <= 100000:
        raise ValueError('steps must be in 1..100000')
    if not math.isfinite(dt) or not 0 < dt <= 1:
        raise ValueError('dt must be finite in (0, 1]')
    with tempfile.TemporaryDirectory(prefix='ovphysx-inspector-') as directory:
        scene, output = Path(directory)/'snapshot.usda', Path(directory)/'poses.json'
        scene.write_text(snapshot.usda, encoding='utf-8')
        command = [str(python_executable), '-m', 'ovphysx_companion.preview_worker',
                   '--scene', str(scene), '--output', str(output), '--dt', str(dt), '--steps', str(steps),
                   '--paths', *(body.path for body in snapshot.bodies)]
        env = os.environ.copy()
        # Installed workers resolve their own dependencies. Only source-checkout
        # development receives the companion-only python/ directory; never a
        # host site-packages directory or inherited host PYTHONPATH. The
        # package's own build file marks a checkout, internal or public export.
        package_root = Path(__file__).resolve().parents[2]
        env.pop('PYTHONPATH',None)
        if package_root.name=='python' and (package_root/'pyproject.toml').is_file():
            env['PYTHONPATH'] = str(package_root)
        subprocess.run(command, env=env, capture_output=True, text=True, timeout=timeout, check=True)
        data = json.loads(output.read_text(encoding='utf-8'))
    session.validate(snapshot.source)
    return {path: Pose(tuple(value['position']), tuple(value['orientation'])) for path, value in data.items()}


_display_writers=weakref.WeakValueDictionary()


class PreviewDisplay:
    """Own one initially empty local output layer for transient Inspector poses.

    The host inserts the layer into the stage and supplies OutputNotices through
    EditSession. Publishing is output, not an authored-input revision. Closing
    clears only this owned layer; it never restores a snapshot over changed input.
    Commit fresh poses separately with commit_preview after closing this display.
    Exclusive playback-owner preview is not offered: use PreviewSession or the
    explicit worker fallback.
    """
    def __init__(self,session,*,layer):
        session._validate_layer(layer)
        if not layer.empty:raise ValueError('Preview display requires an initially empty dedicated layer')
        self._key=(session.stage,layer.identifier)
        if self._key in _display_writers:raise RuntimeError('Preview destination already has a writer')
        self._session,self._layer=session,layer
        self._closed=False;self._expected=layer.ExportToString();self._selection=None
        _display_writers[self._key]=self
        self._remove_close=session.on_close(self.close)

    def publish(self,snapshot,poses):
        self._session._check()
        if self._closed:raise RuntimeError('Preview display is closed')
        if self._layer.ExportToString()!=self._expected:
            raise RuntimeError('Preview output layer changed outside its writer')
        selection=tuple(body.path for body in snapshot.bodies)
        if self._selection is not None and selection!=self._selection:
            raise ValueError('Close the display before changing its selection')
        _apply_preview(self._session,snapshot,poses,layer=self._layer,output=True)
        self._selection=selection;self._expected=self._layer.ExportToString()

    def close(self):
        if self._closed:return
        self._session._check()
        if self._layer.ExportToString()!=self._expected:
            raise RuntimeError('Preview output layer changed outside its writer')
        self._session._apply(self._layer,self._layer.Clear,output=True)
        self._remove_close()
        if _display_writers.get(self._key) is self:del _display_writers[self._key]
        self._closed=True

    def __enter__(self):return self
    def __exit__(self,*args):self.close()
