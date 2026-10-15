# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-CCT-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Native capsule control with copied collision results; no input/UI/USD ownership."""
import atexit
import ctypes as C
from dataclasses import dataclass
from enum import IntFlag
import math
import threading
import warnings
import weakref
from .native import CompanionError, Status, _check, _host_sequence, _host_scalar


class Collision(IntFlag):
    SIDES = 1
    ABOVE = 2
    BELOW = 4


class Filter(IntFlag):
    STATIC = 1
    DYNAMIC = 2
    CONTROLLERS = 4
    ALL = 7


class _Command(C.Structure):
    _fields_ = [('size',C.c_uint32),('version',C.c_uint32),('action',C.c_uint32),('reserved',C.c_uint32),
                ('generation',C.c_uint64),('position',C.c_double*3),('displacement',C.c_float*3),
                ('height',C.c_float),('radius',C.c_float),('dt',C.c_float),('min_distance',C.c_float),('filters',C.c_uint32)]


class _State(C.Structure):
    _fields_ = [('size',C.c_uint32),('version',C.c_uint32),('attachment',C.c_uint64),('key',C.c_uint64),
                ('generation',C.c_uint64),('position',C.c_double*3),('up',C.c_float*3),('gravity',C.c_float*3),
                ('height',C.c_float),('radius',C.c_float),('collisions',C.c_uint32)]


class _Api(C.Structure):
    _fields_ = [('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),
                ('execute',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_char_p,C.POINTER(_Command),C.c_uint32,C.POINTER(_State)))]


def _vector(value):
    value = tuple(_host_scalar(v) for v in _host_sequence(value))
    if len(value)!=3 or not all(math.isfinite(v) for v in value):
        raise ValueError('Expected finite ready CPU XYZ data')
    return value


@dataclass(frozen=True)
class ControllerState:
    attachment: int
    key: int
    generation: int
    position: tuple
    up: tuple
    scene_gravity: tuple
    height: float
    radius: float
    collisions: Collision
    frame: str = 'world'
    units: str = 'stage'


_live = weakref.WeakSet()
_retained = []
_registered = False


def _cleanup():
    for controller in list(_live) + list(_retained):
        try:controller.close()
        except Exception as error:warnings.warn(f'Controller retained after failed close: {error}',RuntimeWarning)


class CharacterController:
    """Own control/removal of one ovstage-created CPU capsule controller.

    Pass a live ToolsSession or PreviewSession. Keep that owner open until this
    controller closes. The host populates PhysxCharacterControllerAPI on a capsule
    (include rendering population so the capsule is present) and attaches first.
    close removes this native CCT only; source USD/ovstage data stays unchanged.
    A later explicit repopulation or reattachment recreates it with a new identity.

    move accepts world displacement in stage units, dt in seconds, and an explicit
    static/dynamic/controller query mask. Its return value copies collision flags.
    Host gravity policy is optional: set_gravity('scene') or an explicit world
    acceleration, then move adds semi-implicit gravity displacement. Movement and
    collision resolution execute natively; no Python callback runs inside them.
    Teleport/resize do not test overlaps. Resize keeps the current foot position.
    """
    def __init__(self, owner, path):
        global _registered
        from .native import ToolsSession
        from .preview import PreviewSession
        self._closed = True
        self._lock = threading.Lock()
        if not isinstance(owner,(ToolsSession,PreviewSession)):
            raise TypeError('Pass a ToolsSession or PreviewSession')
        if not isinstance(path,str) or not path.startswith('/') or '\0' in path:
            raise ValueError('Expected an absolute controller path')
        self._owner, self._library, self.path = owner, owner._library, path
        self._attachment = owner.identity.attachment
        self._api = _Api()
        try:acquire = self._library.ovphysx_companion_cct_acquire
        except AttributeError as error:raise CompanionError(Status.UNAVAILABLE,'CCT table') from error
        acquire.argtypes = [C.c_uint32,C.c_uint32,C.POINTER(_Api)]
        acquire.restype = C.c_uint32
        _check(acquire(1,C.sizeof(_Api),C.byref(self._api)),'acquire CCT')
        if self._api.version!=1 or self._api.size!=C.sizeof(_Api) or not self._api.execute or not self._api.capabilities&1:
            raise CompanionError(Status.ABI_MISMATCH,'CCT table')
        self._generation = 0
        self._gravity, self._velocity = (0.,0.,0.), (0.,0.,0.)
        self._state = self._execute(0)
        self._generation = self._state.generation
        self._closed = False
        _live.add(self)
        if not _registered:
            atexit.register(_cleanup)
            _registered = True

    def _execute(self, action, **values):
        command = _Command()
        command.size, command.version, command.action, command.generation = C.sizeof(command),1,action,self._generation
        command.filters = int(Filter.ALL)
        for name,value in values.items():
            if name in ('position','displacement'):getattr(command,name)[:] = value
            else:setattr(command,name,value)
        state = _State()
        _check(self._api.execute(self._attachment,self.path.encode('utf-8'),C.byref(command),C.sizeof(state),C.byref(state)),
               ('describe','move','teleport','resize','remove')[action]+' CCT')
        return ControllerState(state.attachment,state.key,state.generation,tuple(state.position),tuple(state.up),
                                tuple(state.gravity),state.height,state.radius,Collision(state.collisions))

    def _check(self):
        if self._closed:raise RuntimeError('Character controller is closed')
        if self._owner.identity.attachment!=self._attachment:raise CompanionError(Status.STALE,'CCT owner')

    @property
    def state(self):
        with self._lock:
            self._check()
            self._state = self._execute(0)
            return self._state

    def set_gravity(self, acceleration=None):
        """None disables gravity; 'scene' snapshots scene gravity; XYZ uses stage units/sec²."""
        with self._lock:
            self._check()
            if isinstance(acceleration,str):
                if acceleration!='scene':raise ValueError('Expected scene, None or a world acceleration')
                acceleration = self._execute(0).scene_gravity
            self._gravity = (0.,0.,0.) if acceleration is None else _vector(acceleration)
            self._velocity = (0.,0.,0.)

    def move(self, displacement=(0,0,0), *, dt, min_distance=0, filters=Filter.ALL):
        displacement, dt, min_distance = _vector(displacement), _host_scalar(dt), _host_scalar(min_distance)
        if not math.isfinite(dt) or not 0<dt<=1 or not math.isfinite(min_distance) or min_distance<0:
            raise ValueError('Expected dt in (0,1] and nonnegative min_distance')
        if isinstance(filters,bool) or not isinstance(filters,int) or int(filters)&~7 or filters<0:
            raise ValueError('Use static/dynamic/controller filter bits')
        with self._lock:
            self._check()
            velocity = tuple(v+g*dt for v,g in zip(self._velocity,self._gravity))
            step = tuple(d+v*dt for d,v in zip(displacement,velocity))
            state = self._execute(1,displacement=step,dt=dt,min_distance=min_distance,filters=int(filters))
            vertical = sum(v*u for v,u in zip(velocity,state.up))
            if (state.collisions&Collision.BELOW and vertical<0) or (state.collisions&Collision.ABOVE and vertical>0):
                velocity = tuple(v-vertical*u for v,u in zip(velocity,state.up))
            self._velocity, self._state = velocity,state
            return state

    def teleport(self, position):
        with self._lock:
            self._check()
            self._state = self._execute(2,position=_vector(position))
            self._velocity = (0.,0.,0.)
            return self._state

    def resize(self, *, height, radius):
        with self._lock:
            self._check()
            self._state = self._execute(3,height=_host_scalar(height),radius=_host_scalar(radius))
            return self._state

    def close(self):
        with self._lock:
            if self._closed:return
            try:self._execute(4)
            except CompanionError as error:
                if error.status not in (Status.STALE,Status.NOT_FOUND):raise
            self._closed = True
            self._owner = None
            _live.discard(self)

    def __enter__(self):
        self._check()
        return self

    def __exit__(self,*args):self.close()

    def __del__(self):
        if not getattr(self,'_closed',True):
            try:self.close()
            except Exception:_retained.append(self)
