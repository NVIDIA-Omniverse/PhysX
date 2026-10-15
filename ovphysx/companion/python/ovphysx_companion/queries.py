# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-QUERY-001
# @covers AC-1 AC-2 AC-4
"""Bounded exact-owner queries with copied, generation-qualified hit identities."""
import ctypes as C
from dataclasses import dataclass
from enum import IntEnum
import math
from .inspection import ObjectAccess,ObjectRef,ObjectKind,_Object,_reference,_native_reference
from .native import CompanionError,Status,_check,_host_scalar
from .cct import _vector


class Mode(IntEnum):
    CLOSEST=0
    ANY=1
    ALL=2


@dataclass(frozen=True)
class Geometry:
    """Primitive dimensions or existing cooked shape; query pose is always explicit.

    Sphere radius, box half extents, capsule radius and half height along X.
    Bound shape supports native sphere/box/capsule/convex geometry, with its scale.
    Mesh/plane/custom/compound query geometry fails UNAVAILABLE without cooking.
    """
    kind: int
    dimensions: tuple=(1,1,1)
    reference: ObjectRef|None=None

    @staticmethod
    def sphere(radius):return Geometry(1,(_host_scalar(radius),0,0))
    @staticmethod
    def box(half_extents):return Geometry(2,_vector(half_extents))
    @staticmethod
    def capsule(radius,half_height):return Geometry(3,(_host_scalar(radius),_host_scalar(half_height),0))
    @staticmethod
    def bound(shape):
        _native_reference(shape)
        if shape.kind!=ObjectKind.SHAPE:raise ValueError('Expected a shape reference')
        return Geometry(4,reference=shape)


class _Query(C.Structure):
    _fields_=[(x,C.c_uint32) for x in ('kind','mode','geometry','flags','capacity')]+[
        ('pose',C.c_float*7),('direction',C.c_float*3),('distance',C.c_float),('dimensions',C.c_float*3),
        ('shape',_Object),('include',C.POINTER(_Object)),('exclude',C.POINTER(_Object)),('includeCount',C.c_uint32),('excludeCount',C.c_uint32)]


class _Hit(C.Structure):
    _fields_=[(x,_Object) for x in ('body','shape','material')]+[('distance',C.c_float),('position',C.c_float*3),
        ('normal',C.c_float*3),('face',C.c_uint32),('validFields',C.c_uint32),('reserved',C.c_uint32)]


class _Result(C.Structure):
    _fields_=[(x,C.c_uint64) for x in ('attachment','step','revision','total')]+[('count',C.c_uint32),('truncated',C.c_uint32)]


class _Api(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),
        ('execute',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.POINTER(_Query),C.c_uint32,C.POINTER(_Hit),C.c_uint32,C.c_uint32,C.POINTER(_Result),C.c_uint32))]


@dataclass(frozen=True)
class Hit:
    body: ObjectRef|None
    shape: ObjectRef|None
    material: ObjectRef|None
    distance: float|None
    position: tuple|None
    normal: tuple|None
    face: int|None


@dataclass(frozen=True)
class QueryResult:
    attachment: int
    step: int
    revision: int
    total: int
    truncated: bool
    hits: tuple
    frame: str='world'
    units: str='stage'

    @property
    def any(self):return self.total>0


class SceneQueries(ObjectAccess):
    """Queries and Inspector reads on a selected ToolsSession or PreviewSession.

    Requires completed CPU owner state. No implicit wait or simulation and no USD.
    ANY stops at existence; CLOSEST scans every selected scene for nearest hit;
    ALL orders by distance then object/face identity and retains nearest capacity.
    total counts all matches (ANY: zero/one); truncated reports capacity loss.
    Overlaps have no distance/position/normal/face; CLOSEST is unsupported.
    There is one SDK hit per actor/shape, not one for each mesh triangle crossed.

    include/exclude accept resolved body/shape identities, OR within each group;
    exclusion wins. Empty include selects all. Native query-disabled shapes are
    excluded. A body ref selects a point instance; a shared shape selects all its
    instances. Filters are validated even if the query misses. Both-sides affects
    triangle sidedness; single-sided input honors the native geometry's flags.
    Inputs/outputs use scene length units, world coordinates and XYZW quaternions.
    Hit body references carry instance indices, including articulation links.
    """
    def __init__(self,owner):
        super().__init__(owner);self._query_api=_Api()
        try:acquire=self._library.ovphysx_companion_queries_acquire
        except AttributeError as error:raise CompanionError(Status.UNAVAILABLE,'queries table') from error
        acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
        _check(acquire(1,C.sizeof(_Api),C.byref(self._query_api)),'queries table')
        if (self._query_api.size!=C.sizeof(_Api) or self._query_api.version!=1 or self._query_api.capabilities!=7 or not self._query_api.execute):
            raise CompanionError(Status.ABI_MISMATCH,'queries layout')

    def raycast(self,origin,direction,distance,*,mode=Mode.CLOSEST,capacity=256,include=(),exclude=(),static=True,dynamic=True,both_sides=False):
        return self._execute(0,None,origin,(0,0,0,1),direction,distance,mode,capacity,include,exclude,static,dynamic,both_sides)

    def sweep(self,geometry,position,direction,distance,*,orientation=(0,0,0,1),mode=Mode.CLOSEST,capacity=256,
              include=(),exclude=(),static=True,dynamic=True,both_sides=False):
        return self._execute(1,geometry,position,orientation,direction,distance,mode,capacity,include,exclude,static,dynamic,both_sides)

    def overlap(self,geometry,position,*,orientation=(0,0,0,1),mode=Mode.ALL,capacity=256,include=(),exclude=(),static=True,dynamic=True):
        return self._execute(2,geometry,position,orientation,(1,0,0),1,mode,capacity,include,exclude,static,dynamic,False)

    def _execute(self,kind,geometry,position,orientation,direction,distance,mode,capacity,include,exclude,static,dynamic,both_sides):
        self._check()
        if isinstance(mode,bool):raise ValueError('Invalid query mode')
        if isinstance(capacity,bool) or not isinstance(capacity,int) or not 1<=capacity<=65536:raise ValueError('capacity must be in [1,65536]')
        if not all(isinstance(v,bool) for v in (static,dynamic,both_sides)):raise TypeError('Query flags must be bool')
        position=_vector(position);direction=_vector(direction);orientation=tuple(_host_scalar(v) for v in orientation)
        if len(orientation)!=4 or not all(math.isfinite(v) for v in orientation):raise ValueError('Expected finite XYZW quaternion')
        query=_Query(kind,int(Mode(mode)),0,int(static)|int(dynamic)<<1|int(both_sides)<<2,capacity)
        query.pose[:]=position+orientation;query.direction[:]=direction;query.distance=_host_scalar(distance)
        if kind:
            if not isinstance(geometry,Geometry):raise TypeError('Pass a Geometry')
            if isinstance(geometry.kind,bool) or not isinstance(geometry.kind,int) or not 0<=geometry.kind<2**32:raise ValueError('Invalid geometry kind')
            query.geometry=geometry.kind;query.dimensions[:]=_vector(geometry.dimensions)
            if geometry.reference is not None:query.shape=_native_reference(geometry.reference)
        arrays=[]
        for values in (include,exclude):
            values=tuple(values)
            if len(values)>256:raise ValueError('At most 256 include/exclude objects')
            arrays.append((_Object*len(values))(*(_native_reference(v) for v in values)))
        query.include,query.exclude=arrays;query.includeCount=len(arrays[0]);query.excludeCount=len(arrays[1])
        hits=(_Hit*capacity)();result=_Result()
        _check(self._query_api.execute(self._attachment,C.byref(query),C.sizeof(query),hits,C.sizeof(_Hit),capacity,
                                      C.byref(result),C.sizeof(result)),'scene query')
        if result.count>capacity or result.attachment!=self._attachment:raise CompanionError(Status.ABI_MISMATCH,'query result')
        return QueryResult(result.attachment,result.step,result.revision,result.total,bool(result.truncated),tuple(
            Hit(_reference(v.body),_reference(v.shape),_reference(v.material),v.distance if v.validFields&1 else None,
                tuple(v.position) if v.validFields&4 else None,tuple(v.normal) if v.validFields&8 else None,
                v.face if v.validFields&2 else None) for v in hits[:result.count]))
