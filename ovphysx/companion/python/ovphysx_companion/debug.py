# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-DEBUG-001
# @covers AC-1 AC-3 AC-4
"""Copied CPU debug primitives for renderer adapters, with explicit host policy."""
import atexit
import ctypes as C
from dataclasses import dataclass
from enum import IntFlag
import math
import threading
import warnings
import weakref
from .native import CompanionError, Status, _check, _host_scalar
from .cct import _vector


class Parameter(IntFlag):
    WORLD_AXES=1<<1
    BODY_AXES=1<<2
    BODY_MASS_AXES=1<<3
    BODY_LINEAR_VELOCITY=1<<4
    BODY_ANGULAR_VELOCITY=1<<5
    CONTACT_POINT=1<<6
    CONTACT_NORMAL=1<<7
    CONTACT_ERROR=1<<8
    CONTACT_IMPULSE=1<<9
    FRICTION_POINT=1<<10
    FRICTION_NORMAL=1<<11
    FRICTION_IMPULSE=1<<12
    ACTOR_AXES=1<<13
    COLLISION_AABBS=1<<14
    COLLISION_SHAPES=1<<15
    COLLISION_AXES=1<<16
    COLLISION_COMPOUNDS=1<<17
    COLLISION_FACE_NORMALS=1<<18
    COLLISION_EDGES=1<<19
    COLLISION_STATIC_PRUNER=1<<20
    COLLISION_DYNAMIC_PRUNER=1<<21
    JOINT_LOCAL_FRAMES=1<<22
    JOINT_LIMITS=1<<23
    MBP_REGIONS=1<<25
    SIMULATION_MESH=1<<26
    SDF=1<<27


class SnapshotLimitError(RuntimeError):
    """The frame exceeds the requested bound; no truncated frame was returned."""


class _Config(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('parameters',C.c_uint64),('scale',C.c_float),
              ('enabled',C.c_uint32),('culling',C.c_uint32),('reserved',C.c_uint32),
              ('minimum',C.c_float*3),('maximum',C.c_float*3)]


class _Vertex(C.Structure):
    _fields_=[('position',C.c_float*3),('color',C.c_uint32)]


class _Info(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('attachment',C.c_uint64),('step',C.c_uint64),
              ('points',C.c_uint64),('lines',C.c_uint64),('triangles',C.c_uint64)]


class _Api(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),('parameter_mask',C.c_uint64),
              ('configure',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.POINTER(_Config))),
              ('capture',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_uint64,C.POINTER(C.c_uint64))),
              ('describe',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_uint32,C.POINTER(_Info))),
              ('copy',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_uint32,C.POINTER(_Vertex),C.c_uint64,C.POINTER(C.c_uint64))),
              ('release',C.CFUNCTYPE(C.c_uint32,C.c_uint64))]


@dataclass(frozen=True)
class Vertex:
    position: tuple
    color: int


@dataclass(frozen=True)
class DebugFrame:
    """Copied PhysX debug primitives with the owner's native execution counter.

    Actor/shape wireframes use each scene's latest pre-step poses (the last
    substep when substepping). Contact/friction primitives describe that step's
    contacts. ``step`` includes internal substeps and scenes; it is not a host
    frame number or a guarantee of post-step vertex poses.
    """
    attachment: int
    step: int
    points: tuple
    lines: tuple
    triangles: tuple
    frame: str='world'
    units: str='stage'
    color_format: str='AARRGGBB'


_live=weakref.WeakSet()
_retained=[]
_registered=False


def _cleanup():
    for item in list(_live)+list(_retained):
        try:item.close()
        except Exception as error:warnings.warn(f'Debug policy retained after failed close: {error}',RuntimeWarning)


class DebugDraw:
    """Exclusive host visualization policy on one ToolsSession or PreviewSession.

    Configuration takes effect on the next completed simulation step. Configure
    and capture require an idle owner. Magnitudes are one unit times scale and
    the attachment viewportGizmoScale; bounds/vertices are world stage units.
    Existing owner selection scope is respected. close disables visualization;
    this adapter does not restore other clients' settings. Host code must serialize
    policy ownership. Frames contain immutable copies and outlive this object.
    Native C++ performs all debug extraction; Python only copies/presents results.
    """
    def __init__(self,owner):
        global _registered
        from .native import ToolsSession
        from .preview import PreviewSession
        self._closed=True
        self._lock=threading.Lock()
        if not isinstance(owner,(ToolsSession,PreviewSession)):
            raise TypeError('Pass a ToolsSession or PreviewSession')
        self._owner,self._library=owner,owner._library
        self._attachment=owner.identity.attachment
        self._api=_Api()
        try:acquire=self._library.ovphysx_companion_debug_acquire
        except AttributeError as error:raise CompanionError(Status.UNAVAILABLE,'debug table') from error
        acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
        _check(acquire(1,C.sizeof(_Api),C.byref(self._api)),'acquire debug')
        if (self._api.size!=C.sizeof(_Api) or self._api.version!=1 or not self._api.capabilities&1 or
                not all((self._api.configure,self._api.capture,self._api.describe,self._api.copy,self._api.release))):
            raise CompanionError(Status.ABI_MISMATCH,'debug table')
        self.parameter_mask=Parameter(self._api.parameter_mask)
        self._closed=False
        _live.add(self)
        if not _registered:atexit.register(_cleanup);_registered=True

    def _check(self):
        if self._closed:raise RuntimeError('Debug policy is closed')
        if self._owner.identity.attachment!=self._attachment:raise CompanionError(Status.STALE,'debug owner')

    def configure(self,parameters=Parameter.COLLISION_SHAPES,*,scale=1,enabled=True,culling=None):
        scale=_host_scalar(scale)
        if not math.isfinite(scale) or not 0<=scale<=3.402823466e38:
            raise ValueError('scale must be finite nonnegative float32')
        if isinstance(parameters,bool) or not isinstance(parameters,int) or parameters<0 or parameters&~int(self.parameter_mask):
            raise ValueError('Unsupported debug parameter mask')
        if not isinstance(enabled,bool):raise TypeError('enabled must be bool')
        config=_Config(C.sizeof(_Config),1,int(parameters),scale,int(enabled))
        if culling is not None:
            if len(culling)!=2:raise ValueError('culling requires minimum and maximum')
            minimum,maximum=map(_vector,culling)
            if any(a>b for a,b in zip(minimum,maximum)):raise ValueError('Invalid culling bounds')
            config.culling=1;config.minimum[:]=minimum;config.maximum[:]=maximum
        with self._lock:
            self._check()
            _check(self._api.configure(self._attachment,C.byref(config)),'configure debug')

    def capture(self,*,max_vertices=1000000):
        if isinstance(max_vertices,bool) or not isinstance(max_vertices,int) or not 0<max_vertices<=10000000:
            raise ValueError('max_vertices must be in [1,10000000]')
        with self._lock:
            self._check()
            handle=C.c_uint64()
            status=self._api.capture(self._attachment,max_vertices,C.byref(handle))
            if status==8:raise SnapshotLimitError(f'Debug frame exceeds {max_vertices} vertices')
            _check(status,'capture debug')
            try:
                info=_Info()
                _check(self._api.describe(handle.value,C.sizeof(info),C.byref(info)),'describe debug')
                if info.attachment!=self._attachment:raise CompanionError(Status.ABI_MISMATCH,'debug owner')
                arrays=[]
                for kind,count in enumerate((info.points,info.lines*2,info.triangles*3)):
                    if count>max_vertices:raise CompanionError(Status.ABI_MISMATCH,'debug vertex count')
                    array=(_Vertex*count)();required=C.c_uint64()
                    _check(self._api.copy(handle.value,kind,array,count,C.byref(required)),'copy debug')
                    if required.value!=count:raise CompanionError(Status.ABI_MISMATCH,'debug copy count')
                    values=tuple(Vertex(tuple(v.position),v.color) for v in array)
                    arrays.append(values if kind==0 else tuple(values[i:i+kind+1] for i in range(0,len(values),kind+1)))
                return DebugFrame(info.attachment,info.step,*arrays)
            finally:_check(self._api.release(handle.value),'release debug')

    def close(self):
        with self._lock:
            if self._closed:return
            config=_Config(C.sizeof(_Config),1,0,1,0)
            status=self._api.configure(self._attachment,C.byref(config))
            if status not in (Status.STALE,Status.NOT_FOUND,Status.UNAVAILABLE):_check(status,'disable debug')
            self._closed=True;self._owner=None;_live.discard(self)

    def __enter__(self):self._check();return self
    def __exit__(self,*args):self.close()
    def __del__(self):
        if not getattr(self,'_closed',True):
            try:self.close()
            except Exception:_retained.append(self)


@dataclass(frozen=True)
class CollisionOverlay:
    """Stopped shape representation, independent of a native attachment/frame.

    Triangles contain three Vertex objects; normals contain two. The cache key
    identifies the cooked topology/recipe. A host may color each shape separately.
    """
    path: str
    topology_version: str
    triangles: tuple
    normals: tuple
    frame: str
    units: str
    meters_per_unit: float
    color_format: str='AARRGGBB'


def collision_overlay(cooked, *, path, color=0xff00ff00, normal_length=0.1,
                      local_to_output=None, output_frame=None):
    """Convert copied cooked triangle/convex data for a stopped viewport.

    No scene, runtime acquisition or USD access. The optional affine matrix uses
    row-vector convention and the same length unit as cooked data. Normals are
    computed after transformation, including nonuniform scaling. Tetrahedra and
    sphere sets require explicit surface extraction by their provider.
    """
    import numpy as np
    from .cooking import Cooked
    if not isinstance(cooked,Cooked):raise TypeError('Pass a copied Cooked result')
    if not isinstance(path,str) or not path.startswith('/') or '\0' in path:raise ValueError('Name an absolute object path')
    if isinstance(color,bool) or not isinstance(color,int) or not 0<=color<2**32:raise ValueError('Expected AARRGGBB uint32 color')
    if isinstance(normal_length,bool) or not math.isfinite(normal_length) or normal_length<0:raise ValueError('Expected nonnegative finite normal length')
    points=np.array(cooked.points,dtype=np.float64,copy=True)
    indices=np.array(cooked.indices,copy=True)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():raise ValueError('Expected finite XYZ points')
    if (indices.ndim!=2 or indices.shape[1]!=3 or indices.dtype.kind not in 'iu' or
            np.any(indices<0) or (indices.size and indices.max()>=len(points))):
        raise ValueError('Stopped overlay requires valid triangle indices')
    if not indices.size:
        raise ValueError('This cooking recipe does not expose a cooked triangle surface; blobs cannot be drawn as source geometry')
    frame=cooked.frame
    if local_to_output is not None:
        matrix=np.asarray(local_to_output,dtype=np.float64)
        if (matrix.shape!=(4,4) or not np.isfinite(matrix).all() or
                not np.allclose(matrix[:,3],(0,0,0,1),rtol=0,atol=1e-12) or
                abs(np.linalg.det(matrix[:3,:3]))<1e-12):raise ValueError('Expected nonsingular affine row-vector matrix')
        if not isinstance(output_frame,str) or not output_frame:raise ValueError('Declare the output coordinate frame')
        points=points@matrix[:3,:3]+matrix[3,:3];frame=output_frame
    elif output_frame is not None and output_frame!=frame:
        raise ValueError('Changing coordinate frames requires an explicit transform')
    triangles=points[indices]
    vectors=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    lengths=np.linalg.norm(vectors,axis=1)
    invalid=np.flatnonzero(~np.isfinite(lengths)|(lengths<=1e-15))
    if len(invalid):raise ValueError(f'Degenerate overlay triangles: {invalid.tolist()}')
    centers=triangles.mean(axis=1)
    endpoints=centers+vectors*(normal_length/lengths)[:,None]
    vertex=lambda point:Vertex(tuple(float(v) for v in point),color)
    return CollisionOverlay(path,cooked.cache_key,tuple(tuple(vertex(p) for p in t) for t in triangles),
        tuple((vertex(a),vertex(b)) for a,b in zip(centers,endpoints)),frame,cooked.units,cooked.meters_per_unit)


def export_availability(operation):
    """Diagnostic export discovery is separate from core OmniPVD recording."""
    from .host import Availability
    reasons={'repx':'RepX export is not provided by the companion',
             'legacy-pvd':'Legacy PVD connectivity is not provided; use explicit OVD recording'}
    return Availability(str(operation),False,reasons.get(operation,'No diagnostic export provider for this operation'))
