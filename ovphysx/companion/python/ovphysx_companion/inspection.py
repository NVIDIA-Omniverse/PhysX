# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-QUERY-001
# @covers AC-2 AC-3 AC-4
"""Copied live physics inspection; no USD, SDK pointers or implicit completion."""
import ctypes as C
from dataclasses import dataclass
from enum import IntEnum
import threading
from .native import CompanionError, Status, _check


class ObjectKind(IntEnum):
    BODY=1
    JOINT=2
    MATERIAL=3
    ARTICULATION=4
    SHAPE=5


class _Object(C.Structure):
    _fields_=[(x,C.c_uint64) for x in ('attachment','object','key','generation')]+[(x,C.c_uint32) for x in ('kind','instance')]


class _Body(C.Structure):
    _fields_=[('identity',_Object),('step',C.c_uint64)]+[(x,C.c_uint32) for x in ('type','flags','locks','reserved')]+[
        ('pose',C.c_float*7),('centerOfMass',C.c_float*7),('mass',C.c_float),('inertia',C.c_float*3),
        ('linearVelocity',C.c_float*3),('angularVelocity',C.c_float*3)]


class _Joint(C.Structure):
    _fields_=[(x,_Object) for x in ('identity','body0','body1')]+[('step',C.c_uint64)]+[
        (x,C.c_uint32) for x in ('type','flags','validFields','reserved')]+[
        (x,C.c_float*7) for x in ('local0','local1','world0','world1')]+[(x,C.c_float) for x in ('breakForce','breakTorque')]


class _Material(C.Structure):
    _fields_=[('identity',_Object),('step',C.c_uint64)]+[(x,C.c_float) for x in ('staticFriction','dynamicFriction','restitution')]+[
        (x,C.c_uint32) for x in ('frictionCombine','restitutionCombine','flags')]


class _Dof(C.Structure):
    _fields_=[('link',_Object),('joint',_Object),('index',C.c_uint32),('axis',C.c_uint32)]+[
        (x,C.c_float) for x in ('position','velocity','acceleration','force')]


class _Articulation(C.Structure):
    _fields_=[('identity',_Object),('step',C.c_uint64),('flags',C.c_uint32),('sleeping',C.c_uint32),
              ('rootPose',C.c_float*7),('rootLinearVelocity',C.c_float*3),('rootAngularVelocity',C.c_float*3)]


class _Api(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),
        ('resolve',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_char_p,C.c_uint32,C.c_uint32,C.c_uint32,C.POINTER(_Object))),
        ('read',C.CFUNCTYPE(C.c_uint32,C.POINTER(_Object),C.c_uint32,C.c_void_p,C.c_uint32)),
        ('articulation',C.CFUNCTYPE(C.c_uint32,C.POINTER(_Object),C.c_uint32,C.POINTER(_Articulation),C.c_uint32,
                                  C.POINTER(_Body),C.c_uint32,C.POINTER(C.c_uint32),C.POINTER(_Dof),C.c_uint32,C.POINTER(C.c_uint32))),
        ('path',C.CFUNCTYPE(C.c_uint32,C.POINTER(_Object),C.c_uint32,C.c_void_p,C.c_uint32,C.POINTER(C.c_uint32)))]


@dataclass(frozen=True)
class ObjectRef:
    attachment: int
    object: int
    key: int
    generation: int
    kind: ObjectKind
    instance: int|None=None


def _reference(value):
    return ObjectRef(value.attachment,value.object,value.key,value.generation,ObjectKind(value.kind),
                     None if value.instance==0xffffffff else value.instance) if value.object else None


def _native_reference(ref):
    if not isinstance(ref,ObjectRef):raise TypeError('Pass an ObjectRef returned by resolve or query')
    for value in (ref.attachment,ref.object,ref.key,ref.generation):
        if isinstance(value,bool) or not isinstance(value,int) or not 0<value<2**64:raise ValueError('Invalid object identity')
    if ref.instance is not None and (isinstance(ref.instance,bool) or not isinstance(ref.instance,int) or not 0<=ref.instance<0xffffffff):
        raise ValueError('Invalid instance index')
    return _Object(ref.attachment,ref.object,ref.key,ref.generation,int(ObjectKind(ref.kind)),
                   0xffffffff if ref.instance is None else ref.instance)


@dataclass(frozen=True)
class BodyState:
    identity: ObjectRef
    step: int
    type: str
    pose: tuple
    center_of_mass: tuple
    mass: float|None
    inertia: tuple|None
    linear_velocity: tuple|None
    angular_velocity: tuple|None
    kinematic: bool
    sleeping: bool
    ccd: bool
    simulation_disabled: bool
    locks: int


def _body(v):
    return BodyState(_reference(v.identity),v.step,('static','dynamic','link')[v.type],tuple(v.pose),tuple(v.centerOfMass),
        v.mass if v.type else None,tuple(v.inertia) if v.type else None,tuple(v.linearVelocity) if v.type else None,
        tuple(v.angularVelocity) if v.type else None,bool(v.flags&1),bool(v.flags&2),bool(v.flags&4),bool(v.flags&8),v.locks)


@dataclass(frozen=True)
class JointState:
    identity: ObjectRef
    step: int
    type: str
    body0: ObjectRef|None
    body1: ObjectRef|None
    local0: tuple
    local1: tuple
    world0: tuple
    world1: tuple
    flags: int|None
    break_force: float|None
    break_torque: float|None


@dataclass(frozen=True)
class MaterialState:
    identity: ObjectRef
    step: int
    static_friction: float
    dynamic_friction: float
    restitution: float
    friction_combine: int
    restitution_combine: int
    flags: int


@dataclass(frozen=True)
class DofState:
    link: ObjectRef
    joint: ObjectRef|None
    index: int
    axis: str
    position: float
    velocity: float
    acceleration: float
    applied_force: float


@dataclass(frozen=True)
class ArticulationState:
    identity: ObjectRef
    step: int
    flags: int
    sleeping: bool
    root_pose: tuple
    root_linear_velocity: tuple
    root_angular_velocity: tuple
    links: tuple
    dofs: tuple


class ObjectAccess:
    """Read a ToolsSession or independent PreviewSession on its host thread.

    Host completes pending work first. Every read revalidates the exact native
    record generation. Unrelated edits leave a reference valid; replacement or
    detach makes it stale. A reference from another Inspector owner is rejected.
    Shapes can be resolved for queries; they have no generic inspection payload.

    Poses are xyz + xyzw, world for actors/root and joint world frames; COM and
    joint local frames are actor-local. Values use scene length/mass units and
    seconds; angular DOFs use radians. DOF axes follow the native joint frames/cache order; they need not match the authored USD axis token.
    applied_force is the commanded joint force, not a constraint sensor reading.
    Static mass/inertia/velocities and articulation-joint break fields are None.
    This facade retains the selected owner object; close the facade before owner
    retirement. Returned immutable copies outlive both. No new native owner,
    background work or customer USD is loaded. Live state is CPU-only; material
    metadata is also available for GPU owners after host completion.
    """
    def __init__(self,owner):
        from .native import ToolsSession
        from .preview import PreviewSession
        if not isinstance(owner,(ToolsSession,PreviewSession)):raise TypeError('Pass a ToolsSession or PreviewSession')
        self._owner=owner;self._attachment=owner.identity.attachment;self._library=owner._library
        self._thread=threading.get_ident();self._closed=False;self._api=_Api()
        try:acquire=self._library.ovphysx_companion_inspection_acquire
        except AttributeError as error:raise CompanionError(Status.UNAVAILABLE,'inspection table') from error
        acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
        _check(acquire(1,C.sizeof(_Api),C.byref(self._api)),'inspection table')
        if (self._api.size!=C.sizeof(_Api) or self._api.version!=1 or self._api.capabilities!=15 or
                not all((self._api.resolve,self._api.read,self._api.articulation,self._api.path))):
            raise CompanionError(Status.ABI_MISMATCH,'inspection layout')

    def _check(self):
        if self._closed:raise RuntimeError('Inspector is closed')
        if threading.get_ident()!=self._thread:raise RuntimeError('Inspection must run on its host thread')
        if self._owner.identity.attachment!=self._attachment:raise CompanionError(Status.STALE,'inspection owner')

    def resolve(self,path,kind=ObjectKind.BODY,*,instance=None):
        self._check()
        if not isinstance(path,str) or not path.startswith('/') or '\0' in path:raise ValueError('Expected absolute object path')
        if instance is not None and (isinstance(instance,bool) or not isinstance(instance,int) or not 0<=instance<0xffffffff):
            raise ValueError('Expected nonnegative point-instance index')
        if isinstance(kind,bool):raise ValueError('Invalid object kind')
        value=_Object();_check(self._api.resolve(self._attachment,path.encode(),int(ObjectKind(kind)),
            0xffffffff if instance is None else instance,C.sizeof(value),C.byref(value)),'resolve object')
        return _reference(value)

    def path(self,reference):
        """Resolve a live reference to an authored diagnostic path in this owner."""
        self._check();ref=_native_reference(reference)
        if ref.attachment!=self._attachment:raise CompanionError(Status.STALE,'foreign inspection owner')
        size=C.c_uint32()
        status=self._api.path(C.byref(ref),C.sizeof(ref),None,0,C.byref(size))
        if status!=8:_check(status,'size object path')
        if not 0<size.value<=1048576:raise CompanionError(Status.ABI_MISMATCH,'object path bound')
        value=C.create_string_buffer(size.value)
        _check(self._api.path(C.byref(ref),C.sizeof(ref),value,len(value),C.byref(size)),'object path')
        return value.value.decode('utf-8')

    def read(self,reference):
        self._check();ref=_native_reference(reference)
        if ref.attachment!=self._attachment:raise CompanionError(Status.STALE,'foreign inspection owner')
        if reference.kind==ObjectKind.ARTICULATION:return self._read_articulation(ref)
        types={ObjectKind.BODY:_Body,ObjectKind.JOINT:_Joint,ObjectKind.MATERIAL:_Material}
        if reference.kind not in types:raise ValueError('No inspection payload for this object kind')
        value=types[reference.kind]()
        _check(self._api.read(C.byref(ref),C.sizeof(ref),C.byref(value),C.sizeof(value)),'inspect object')
        if reference.kind==ObjectKind.BODY:return _body(value)
        if reference.kind==ObjectKind.MATERIAL:return MaterialState(_reference(value.identity),value.step,
            value.staticFriction,value.dynamicFriction,value.restitution,value.frictionCombine,value.restitutionCombine,value.flags)
        return JointState(_reference(value.identity),value.step,('generic','fixed','revolute','prismatic','spherical','distance','D6','articulation')[value.type],
            _reference(value.body0),_reference(value.body1),tuple(value.local0),tuple(value.local1),tuple(value.world0),tuple(value.world1),
            value.flags if value.validFields&1 else None,value.breakForce if value.validFields&1 else None,value.breakTorque if value.validFields&1 else None)

    def _read_articulation(self,ref):
        value=_Articulation();nl=C.c_uint32();nd=C.c_uint32()
        status=self._api.articulation(C.byref(ref),C.sizeof(ref),C.byref(value),C.sizeof(value),None,0,C.byref(nl),None,0,C.byref(nd))
        if status!=8:_check(status,'size articulation')
        if not 0<nl.value<=65536 or nd.value>65536:raise CompanionError(Status.ABI_MISMATCH,'articulation counts')
        links=(_Body*nl.value)();dofs=(_Dof*nd.value)()
        _check(self._api.articulation(C.byref(ref),C.sizeof(ref),C.byref(value),C.sizeof(value),links,len(links),C.byref(nl),dofs,len(dofs),C.byref(nd)),
               'inspect articulation')
        return ArticulationState(_reference(value.identity),value.step,value.flags,bool(value.sleeping),tuple(value.rootPose),
            tuple(value.rootLinearVelocity),tuple(value.rootAngularVelocity),tuple(_body(v) for v in links[:nl.value]),
            tuple(DofState(_reference(v.link),_reference(v.joint),v.index,('twist','swing1','swing2','x','y','z')[v.axis],
                v.position,v.velocity,v.acceleration,v.force) for v in dofs[:nd.value]))

    def close(self):
        if self._closed:return
        if threading.get_ident()!=self._thread:raise RuntimeError('Close inspection on its host thread')
        self._closed=True;self._owner=None

    def __enter__(self):self._check();return self
    def __exit__(self,*args):self.close()


class Inspector(ObjectAccess):
    """Copied physics inspection on an explicitly borrowed or owned attachment.

    ObjectAccess supplies runtime identity/read operations shared by interaction
    and inspection. Inspector itself owns no UI or preview simulation.
    """
