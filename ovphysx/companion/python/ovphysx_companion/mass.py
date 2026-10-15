# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-MASS-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Stopped, copied SI mass computation; no USD, native owner or solver required."""
import ctypes as C
from dataclasses import dataclass
from enum import IntEnum
from numbers import Integral
import math
from .native import _MassResult,MassProperties,CompanionError,Status,_check,_host_sequence,_host_scalar

class ShapeKind(IntEnum):
    BOX=0
    SPHERE=1
    CAPSULE=2
    MESH=3

def _vector(value,width):
    result=tuple(_host_scalar(v) for v in _host_sequence(value))
    if len(result)!=width or not all(math.isfinite(v) and abs(v)<=3.402823466e38 for v in result):
        raise ValueError(f'Expected {width} finite float-range components')
    return result

@dataclass(frozen=True)
class Overrides:
    mass:float|None=None
    center:tuple|None=None
    diagonal:tuple|None=None
    axes:tuple|None=None

    def __post_init__(self):
        if self.mass is not None:
            value=_host_scalar(self.mass)
            if not math.isfinite(value) or not 0<value<=3.402823466e38:raise ValueError('Mass must be positive finite float')
            object.__setattr__(self,'mass',value)
        for name,width in (('center',3),('diagonal',3),('axes',4)):
            value=getattr(self,name)
            if value is not None:object.__setattr__(self,name,_vector(value,width))
        if self.diagonal is not None and min(self.diagonal)<=0:raise ValueError('Diagonal inertia must be positive')
        if self.axes is not None and abs(sum(v*v for v in self.axes)-1)>1e-5:raise ValueError('Axes must be a unit xyzw quaternion')

@dataclass(frozen=True)
class Shape:
    """Copied collider-local SI geometry. Capsule axis X; box half extents.

    Mesh must be a closed, connected, oriented, non-self-intersecting triangle
    boundary. Global inward winding is accepted. No repair/approximation occurs.
    """
    path:str
    kind:ShapeKind
    dimensions:tuple=(0,0,0)
    density:float=1000
    position:tuple=(0,0,0)
    rotation:tuple=(0,0,0,1)
    points:tuple=()
    triangles:tuple=()
    overrides:Overrides=Overrides()

    def __post_init__(self):
        if not isinstance(self.path,str) or not self.path:raise ValueError('Collider identity is required')
        if isinstance(self.kind,bool):raise ValueError('Invalid shape kind')
        object.__setattr__(self,'kind',ShapeKind(self.kind))
        for name,width in (('dimensions',3),('position',3),('rotation',4)):
            object.__setattr__(self,name,_vector(getattr(self,name),width))
        density=_host_scalar(self.density)
        if not math.isfinite(density) or not 0<density<=3.402823466e38:raise ValueError(f'{self.path}: invalid density')
        object.__setattr__(self,'density',density)
        if abs(sum(v*v for v in self.rotation)-1)>1e-5:raise ValueError(f'{self.path}: rotation must be unit xyzw')
        points=tuple(_vector(p,3) for p in _host_sequence(self.points))
        triangles=tuple(tuple(_host_sequence(t)) for t in _host_sequence(self.triangles))
        if len(points)>1000000 or len(triangles)>2000000:raise ValueError('Mesh exceeds native limits')
        if any(len(t)!=3 or any(isinstance(i,bool) or not isinstance(i,Integral) or not 0<=i<len(points) for i in t) for t in triangles):
            raise ValueError(f'{self.path}: triangle indices must identify three vertices')
        object.__setattr__(self,'points',points);object.__setattr__(self,'triangles',tuple(tuple(int(i) for i in t) for t in triangles))
        if not isinstance(self.overrides,Overrides):raise TypeError('Pass Overrides')

@dataclass(frozen=True)
class Request:
    shapes:tuple
    source:object=None
    body_path:str=''
    overrides:Overrides=Overrides()
    units:str='SI'
    frame:str='body-local'

    def __post_init__(self):
        shapes=tuple(_host_sequence(self.shapes))
        if not 1<=len(shapes)<=65536 or not all(isinstance(s,Shape) for s in shapes):raise ValueError('Pass 1..65536 Shape snapshots')
        if len({s.path for s in shapes})!=len(shapes):raise ValueError('Collider identities must be distinct')
        if not isinstance(self.body_path,str) or not isinstance(self.overrides,Overrides):raise TypeError('Invalid body metadata')
        if self.units!='SI' or self.frame!='body-local':raise ValueError('Mass inputs use SI body-local frame')
        object.__setattr__(self,'shapes',shapes)

@dataclass(frozen=True)
class Result:
    properties:MassProperties
    source:object
    body_path:str
    collider_paths:tuple
    units:str='SI'
    frame:str='body-local'

class _Overrides(C.Structure):
    _fields_=[('flags',C.c_uint32),('mass',C.c_float),('center',C.c_float*3),('diagonal',C.c_float*3),('axes',C.c_float*4)]
class _Shape(C.Structure):
    _fields_=[('size',C.c_uint32),('kind',C.c_uint32),('dimensions',C.c_float*3),('position',C.c_float*3),
        ('rotation',C.c_float*4),('density',C.c_float),('points',C.POINTER(C.c_float)),('triangles',C.POINTER(C.c_uint32)),
        ('point_count',C.c_uint32),('triangle_count',C.c_uint32),('overrides',_Overrides)]
_Compute=C.CFUNCTYPE(C.c_uint32,C.POINTER(_Shape),C.c_uint32,C.c_uint32,C.POINTER(_Overrides),C.c_uint32,C.POINTER(_MassResult),C.POINTER(C.c_uint32))
class _Api(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),('compute',_Compute)]

def _overrides(value):
    flags=sum(flag for name,flag in (('mass',1),('center',2),('diagonal',4),('axes',8)) if getattr(value,name) is not None)
    return _Overrides(flags,value.mass or 0,(C.c_float*3)(*(value.center or (0,0,0))),
        (C.c_float*3)(*(value.diagonal or (0,0,0))),(C.c_float*4)(*(value.axes or (0,0,0,1))))

def compute_mass_properties(library,request):
    """Use the existing core CDLL. Inputs/results are CPU copies, valid after close."""
    if not isinstance(request,Request):raise TypeError('Pass a copied mass Request')
    try:acquire=library.ovphysx_companion_mass_acquire
    except AttributeError as exc:raise CompanionError(Status.UNAVAILABLE,'general mass table') from exc
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    api=_Api();_check(acquire(1,C.sizeof(api),C.byref(api)),'mass acquire')
    if api.size!=C.sizeof(api) or api.version!=1:raise CompanionError(Status.ABI_MISMATCH,'mass table')
    if api.capabilities&7!=7 or not api.compute:raise CompanionError(Status.UNAVAILABLE,'mass computation')
    arrays=[];records=[]
    for shape in request.shapes:
        points=(C.c_float*(3*len(shape.points)))(*(v for p in shape.points for v in p))
        triangles=(C.c_uint32*(3*len(shape.triangles)))(*(v for p in shape.triangles for v in p))
        arrays.extend((points,triangles))
        records.append(_Shape(C.sizeof(_Shape),shape.kind,(C.c_float*3)(*shape.dimensions),
            (C.c_float*3)(*shape.position),(C.c_float*4)(*shape.rotation),shape.density,
            points,triangles,len(shape.points),len(shape.triangles),_overrides(shape.overrides)))
    inputs=(_Shape*len(records))(*records);body=_overrides(request.overrides);output=_MassResult();invalid=C.c_uint32(0xffffffff)
    status=api.compute(inputs,len(inputs),C.sizeof(body),C.byref(body),C.sizeof(output),C.byref(output),C.byref(invalid))
    if status!=Status.OK:
        path=request.shapes[invalid.value].path if invalid.value<len(request.shapes) else request.body_path or '<aggregate>'
        error=CompanionError(status,f'mass for {path}');error.object_path=path;error.invalid_shape=None if invalid.value==0xffffffff else invalid.value
        raise error
    if output.size!=C.sizeof(output) or output.version!=1:raise CompanionError(Status.ABI_MISMATCH,'mass result')
    properties=MassProperties(output.mass,output.volume,tuple(output.center_of_mass),tuple(output.diagonal_inertia),
        tuple(output.principal_axes),tuple(output.bounds_min),tuple(output.bounds_max))
    return Result(properties,request.source,request.body_path,tuple(s.path for s in request.shapes))
