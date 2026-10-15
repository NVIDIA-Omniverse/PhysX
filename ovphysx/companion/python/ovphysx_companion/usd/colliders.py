# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-HOST-001
# @covers AC-1 AC-3 AC-4
# @implements REQ-COMP-USD-EDIT-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Incremental host collider command with one atomic, undoable USD publication.

All methods run on the EditSession thread. advance() prepares bounded chunks so
an editor can report progress and cancel between ticks. No USD opinions change
until commit(). Cooking/simulation is a separate operation after population.
"""
from concurrent.futures import CancelledError
from dataclasses import dataclass
import math
from .session import _usd, DependencyError


@dataclass(frozen=True)
class ColliderProgress:
    prepared: int
    total: int
    mesh_count: int
    meters_per_unit: float

    @property
    def fraction(self):
        return self.prepared / self.total if self.total else 1.0


class ColliderBatch:
    """Create colliders on an explicit selection; never silently walk descendants.

    Existing colliders, opt-outs, unsupported geometry and instance edits are
    rejected during preflight. Mesh approximations use the core authoring helper;
    its documented dynamic-mesh convex-hull fallback is reported in
    ``approximations`` before mutation. Contact offset is supplied in meters.
    A commit returns one EditPatch for the host command stack. Cancellation,
    failure and commit are terminal; neither retries a partial publication.
    """
    def __init__(self, session, paths, *, layer, approximation='convexHull', contact_offset_meters=None):
        Sdf, _, Usd, UsdGeom, _ = _usd()
        try:
            from pxr import UsdPhysics
            from ovphysx.utils.authoring import set_collider
            from ovphysx.utils.constants import MESH_APPROXIMATIONS
        except ImportError as error:
            raise DependencyError('Collider authoring requires customer UsdPhysics and ovphysx.utils') from error
        session._validate_layer(layer)
        if approximation not in MESH_APPROXIMATIONS:
            raise ValueError('Unsupported mesh approximation: '+str(approximation))
        if contact_offset_meters is not None and (isinstance(contact_offset_meters,bool) or
                not isinstance(contact_offset_meters,(int,float)) or
                not math.isfinite(contact_offset_meters) or contact_offset_meters<=0):
            raise ValueError('contact_offset_meters must be finite and positive')
        paths=tuple(paths)
        if not paths or len(set(map(str,paths)))!=len(paths):
            raise ValueError('Select a nonempty set of distinct collider paths')
        mpu=UsdGeom.GetStageMetersPerUnit(session.stage)
        if not math.isfinite(mpu) or mpu<=0:raise ValueError('Invalid stage metersPerUnit')
        rows=[];schemas={'PhysicsCollisionAPI','PhysxCollisionAPI'}
        for value in paths:
            path=Sdf.Path(value)
            if not path.IsAbsolutePath() or not path.IsPrimPath():raise ValueError('Expected absolute prim paths')
            prim=session.stage.GetPrimAtPath(path)
            if (not prim or not prim.IsActive() or not prim.IsLoaded() or prim.IsInstance() or
                    prim.IsInstanceProxy() or prim.IsInPrototype()):
                raise ValueError(f'{path}: select an active loaded non-instance prim')
            if prim.GetTypeName() not in ('Mesh','Cube','Sphere','Capsule','Cylinder','Cone','Plane'):
                raise ValueError(f'{path}: unsupported collider geometry')
            if prim.HasAPI(UsdPhysics.CollisionAPI):raise ValueError(f'{path}: already a collider')
            optout=prim.GetAttribute('omni:no_collision')
            if optout and optout.Get():raise ValueError(f'{path}: collision is explicitly disabled')
            mesh=prim.IsA(UsdGeom.Mesh);chosen=approximation
            if mesh:
                parent=prim
                while parent and not parent.IsPseudoRoot():
                    if parent.HasAPI(UsdPhysics.RigidBodyAPI) and chosen=='none':chosen='convexHull'
                    parent=parent.GetParent()
                schemas.add('PhysicsMeshCollisionAPI')
                if MESH_APPROXIMATIONS[chosen]:schemas.add(MESH_APPROXIMATIONS[chosen])
            rows.append((str(path),mesh,chosen))
        # Probe dependencies without applying anything. Codeless schemas must be
        # registered in this customer USD image before preparing the command.
        registry=Usd.SchemaRegistry()
        for name in schemas:
            if not registry.FindAppliedAPIPrimDefinition(name):
                raise DependencyError(f'Collider authoring requires registered schema {name}')
        self._session,self._source,self._layer=session,session.source,layer
        self._rows=tuple(rows);self._set_collider=set_collider
        self._mpu=mpu;self._offset=None if contact_offset_meters is None else contact_offset_meters/mpu
        self._prepared=0;self._state='preparing';self.patch=None;self._error=None
        self._bounds=[]

    def _check(self):
        self._session._check()
        if self._state=='cancelled':raise CancelledError('Collider batch was cancelled')
        if self._error is not None:raise self._error
        self._session.validate(self._source)

    @property
    def progress(self):
        self._session._check()
        return ColliderProgress(self._prepared,len(self._rows),sum(row[1] for row in self._rows),self._mpu)

    @property
    def approximations(self):
        return tuple((path,chosen) for path,mesh,chosen in self._rows if mesh)

    @property
    def world_bounds_meters(self):
        """Copied (path, minimum XYZ, maximum XYZ) for prepared selection items."""
        self._session._check()
        return tuple(self._bounds)

    def advance(self, max_items=32):
        self._check()
        if self._state!='preparing':raise RuntimeError('Collider batch is terminal')
        if isinstance(max_items,bool) or not isinstance(max_items,int) or max_items<1:
            raise ValueError('max_items must be a positive integer')
        _,_,Usd,UsdGeom,_=_usd()
        cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'])
        end=min(len(self._rows),self._prepared+max_items)
        try:
            for path,_,_ in self._rows[self._prepared:end]:
                extent=cache.ComputeWorldBound(self._session.stage.GetPrimAtPath(path)).ComputeAlignedRange()
                if extent.IsEmpty():raise ValueError(f'{path}: empty collider bound')
                low=tuple(float(v)*self._mpu for v in extent.GetMin())
                high=tuple(float(v)*self._mpu for v in extent.GetMax())
                if not all(math.isfinite(v) for v in low+high):raise ValueError(f'{path}: non-finite collider bound')
                self._bounds.append((path,low,high))
                self._prepared+=1
        except BaseException as error:
            self._state='failed';self._error=error
            raise
        return self.progress

    def cancel(self):
        self._session._check()
        if self._state!='preparing':return False
        self._state='cancelled'
        return True

    def commit(self):
        self._session._check()
        if self._state=='committed':return self.patch
        self._check()
        if self._prepared!=len(self._rows):raise RuntimeError('Finish preparing the batch before commit')
        from pxr import Sdf, UsdPhysics
        self._state='failed'
        def edit():
            for path,mesh,chosen in self._rows:
                prim=self._session.stage.GetPrimAtPath(path)
                self._set_collider(prim,chosen)
                if not prim.HasAPI(UsdPhysics.CollisionAPI) or not prim.HasAPI('PhysxCollisionAPI'):
                    raise RuntimeError(f'{path}: USD rejected collider APIs')
                if mesh and UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()!=chosen:
                    raise RuntimeError(f'{path}: USD rejected collider approximation')
                if self._offset is not None:
                    attr=prim.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float,custom=False)
                    if not attr.Set(self._offset):raise RuntimeError(f'{path}: USD rejected contact offset')
        try:
            self.patch=self._session._apply(self._layer,edit)
        except BaseException as error:
            self._error=error
            raise
        self._state='committed'
        return self.patch
