# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-INTERACTION-001
# @covers AC-2 AC-3 AC-4 AC-5 AC-6
"""Typed CPU interaction; SDK pointer leases are exclusively in the C++ SDK."""
import atexit
import math
import ctypes as C
from dataclasses import dataclass
from enum import IntEnum
import threading
import warnings
import weakref
from .inspection import ObjectAccess,ObjectKind,_Object,_native_reference,_reference
from .native import CompanionError,Status,_check,_host_scalar
from .cct import _vector


class ForceMode(IntEnum):
    FORCE=0
    IMPULSE=1


class GrabMode(IntEnum):
    FORCE=0
    JOINT=1


class EndVelocity(IntEnum):
    PRESERVE=0
    ZERO=1
    RESTORE=2


class _Force(C.Structure):
    _fields_=[(x,C.c_uint32) for x in ('mode','frame','atPoint','wake')]+[('force',C.c_float*3),('point',C.c_float*3)]


class _Options(C.Structure):
    _fields_=[('target',C.c_float*3)]+[(x,C.c_float) for x in ('stiffness','damping','maxForce')]+[
        ('flags',C.c_uint32),('velocityPolicy',C.c_uint32)]


class _State(C.Structure):
    _fields_=[('body',_Object),('target',C.c_float*3),('active',C.c_uint32)]


class _Api(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),
        ('force',C.CFUNCTYPE(C.c_uint32,C.POINTER(_Object),C.c_uint32,C.POINTER(_Force),C.c_uint32)),
        ('begin',C.CFUNCTYPE(C.c_uint32,C.POINTER(_Object),C.c_uint32,C.POINTER(_Options),C.c_uint32,C.POINTER(C.c_uint64))),
        ('move',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.POINTER(C.c_float),C.POINTER(_State),C.c_uint32)),
        ('read',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.POINTER(_State),C.c_uint32)),
        ('close',C.CFUNCTYPE(C.c_uint32,C.c_uint64))]


class _PickingOptions(C.Structure):
    _fields_=[('point',C.c_float*3),('pickingForce',C.c_float),('pickDistance',C.c_float),('velocityPolicy',C.c_uint32)]


class _PickingApi(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),
        ('begin',C.CFUNCTYPE(C.c_uint32,C.POINTER(_Object),C.c_uint32,C.POINTER(_PickingOptions),C.c_uint32,C.POINTER(C.c_uint64))),
        ('move',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.POINTER(C.c_float),C.POINTER(_State),C.c_uint32)),
        ('read',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.POINTER(_State),C.c_uint32)),
        ('close',C.CFUNCTYPE(C.c_uint32,C.c_uint64)),
        ('move_ray',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.POINTER(C.c_float),C.POINTER(C.c_float),C.POINTER(_State),C.c_uint32)),
        ('set_force_coefficient',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_float))]


@dataclass(frozen=True)
class DragState:
    body: object
    target: tuple
    active: bool
    frame: str='world'
    units: str='stage'


class Interaction(ObjectAccess):
    """Host-thread interaction on ToolsSession or independent PreviewSession.

    Calls require idle owner state; route inputs through the host's pre-step phase.
    Force grabbing supports movable CPU rigid bodies and articulation links (with
    preserve-velocity policy). D6 dragging and direct apply_force require dynamic
    rigid bodies. Static, kinematic, disabled and GPU/DirectGPU bodies are unavailable. No implicit
    fetch or step. Forces use scene mass*length/seconds^2, impulses mass*length/seconds.
    The existing core read/write APIs remain suitable for their tensor semantics;
    this service adds exact-instance force-at-point and native managed force grabbing.
    Grab/drag default to FORCE; JOINT explicitly selects the existing D6 spring.
    """
    def __init__(self,owner):
        super().__init__(owner);self._interaction_api=_Api()
        try:acquire=self._library.ovphysx_companion_interaction_acquire
        except AttributeError as error:raise CompanionError(Status.UNAVAILABLE,'interaction table') from error
        acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
        _check(acquire(1,C.sizeof(_Api),C.byref(self._interaction_api)),'interaction table')
        api=self._interaction_api
        if api.size!=C.sizeof(_Api) or api.version!=1 or api.capabilities!=3 or not all((api.force,api.begin,api.move,api.read,api.close)):
            raise CompanionError(Status.ABI_MISMATCH,'interaction layout')

    def _picking(self):
        # Acquire lazily so older providers still support explicitly requested
        # joint dragging. A missing force service never falls back to a joint.
        if hasattr(self,'_picking_api'):return self._picking_api
        api=_PickingApi()
        try:acquire=self._library.ovphysx_companion_picking_acquire
        except AttributeError as error:raise CompanionError(Status.UNAVAILABLE,'force picking table') from error
        acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_PickingApi)];acquire.restype=C.c_uint32
        _check(acquire(1,C.sizeof(_PickingApi),C.byref(api)),'force picking table')
        if api.size!=C.sizeof(_PickingApi) or api.version!=1 or api.capabilities!=1 or not all((api.begin,api.move,api.read,api.close,api.move_ray,api.set_force_coefficient)):
            raise CompanionError(Status.ABI_MISMATCH,'force picking layout')
        self._picking_api=api
        return api

    def _body(self,reference):
        self._check();ref=_native_reference(reference)
        if ref.attachment!=self._attachment:raise CompanionError(Status.STALE,'foreign interaction owner')
        if reference.kind!=ObjectKind.BODY:raise ValueError('Pass a body reference')
        return ref

    def apply_force(self,body,force,*,mode=ForceMode.FORCE,point=None,frame='world',wake=True):
        """Apply at COM or an explicit point. Actor-local points use the actor pose.

        Forces and impulses accumulate under idle admission and take effect during
        the next simulation step. Multiple point instances are selected
        by the body reference, never just their shared prototype path.
        """
        ref=self._body(body)
        if isinstance(mode,bool) or frame not in ('world','actor-local') or not isinstance(wake,bool):raise ValueError('Invalid force mode/frame/wake')
        command=_Force(int(ForceMode(mode)),int(frame=='actor-local'),int(point is not None),int(wake))
        command.force[:]=_vector(force);command.point[:]=_vector(point) if point is not None else (0,0,0)
        _check(self._interaction_api.force(C.byref(ref),C.sizeof(ref),C.byref(command),C.sizeof(command)),'apply force')

    def grab(self,body,picked_world_point,*,mode=GrabMode.FORCE,picking_force=1,pick_distance=0,
             end_velocity=EndVelocity.PRESERVE,**joint_options):
        """Retain a picked point; FORCE is the default, JOINT is explicit.

        Force grabbing runs the complete legacy impulse, point damping and gravity
        assistance in native code for every actual substep. move() or move_ray()
        only updates the retained target. pick_distance is the original ray hit
        distance, used for ray movement and gravity assistance. No USD or step is
        performed here. Close on release, pause, stop, focus loss or input disable.
        stiffness/damping/max_force/clear_locks/speculative_ccd apply only to JOINT.
        """
        self._body(body)
        return Drag(self,body,picked_world_point,mode=mode,picking_force=picking_force,pick_distance=pick_distance,
                    end_velocity=end_velocity,**joint_options)

    def drag(self,body,picked_world_point,**options):
        """Alias of grab(); force grabbing is the default."""
        return self.grab(body,picked_world_point,**options)

    def joint_drag(self,body,picked_world_point,**options):
        """Explicit D6 joint drag using stiffness/damping/force-limit options."""
        return self.grab(body,picked_world_point,mode=GrabMode.JOINT,**options)

    def pick(self,origin,direction,*,max_distance=1000000,mode=GrabMode.FORCE,picking_force=1,
             end_velocity=EndVelocity.PRESERVE,include=(),exclude=(),capacity=256,**joint_options):
        """Pick the closest eligible CPU body and return its grab, or None.

        include/exclude are exact body/shape refs; the host supplies visibility
        filtering through those lists, or performs its own SceneQueries and grab.
        Static bodies are excluded and do not occlude eligible bodies behind them.
        Hosts needing closest-hit occlusion query static and dynamic bodies first.
        Unsupported dynamic bodies are skipped without
        changing modes; duplicate grabs and busy owners report errors. A truncated
        result with no eligible hit reports unavailable rather than claiming a miss.
        """
        self._check()
        from .queries import SceneQueries,Mode
        if isinstance(mode,bool):raise ValueError('Invalid grab mode')
        mode=GrabMode(mode)
        if mode==GrabMode.FORCE:self._picking()
        direction=_vector(direction)
        length=math.hypot(*direction)
        if length==0:raise ValueError('Expected a nonzero ray direction')
        direction=tuple(value/length for value in direction)
        with SceneQueries(self._owner) as queries:
            result=queries.raycast(origin,direction,max_distance,mode=Mode.ALL,capacity=capacity,
                                   static=False,include=include,exclude=exclude)
        for hit in result.hits:
            if hit.body is None or hit.position is None or hit.distance is None:continue
            try:
                return self.grab(hit.body,hit.position,mode=mode,picking_force=picking_force,
                                 pick_distance=hit.distance,end_velocity=end_velocity,**joint_options)
            except CompanionError as error:
                if error.status!=Status.UNAVAILABLE:raise
        if result.truncated:raise CompanionError(Status.UNAVAILABLE,'picking result truncated; increase capacity')
        return None


_live=weakref.WeakSet()
_retained=[]
_registered=False


def _cleanup():
    for drag in list(_live)+list(_retained):
        try:drag.close()
        except Exception as error:warnings.warn(f'Drag retained after failed close: {error}',RuntimeWarning)


class Drag:
    """Native managed grab on one body, with host-owned stepping.

    FORCE retains a local anchor and applies legacy impulses at each native
    substep, temporarily raising angular damping and restoring it on close.
    JOINT retains the original D6 spring semantics. Neither mode steps physics.
    Both modes close before actor deletion/reset; state then reports inactive.
    Busy close is retryable after host completion. End velocity preserves motion
    by default; CPU links allow only that policy.
    """
    def __init__(self,interaction,body,picked_world_point,*,mode=GrabMode.FORCE,picking_force=1,pick_distance=0,
                 end_velocity=EndVelocity.PRESERVE,**joint_options):
        global _registered
        self._closed=True
        if not isinstance(interaction,Interaction):raise TypeError('Pass an Interaction')
        ref=interaction._body(body)
        if isinstance(mode,bool) or isinstance(end_velocity,bool):raise ValueError('Invalid grab mode or velocity policy')
        self.mode=GrabMode(mode)
        self.pick_distance=_host_scalar(pick_distance)
        if not math.isfinite(self.pick_distance) or not 0<=self.pick_distance<=1000000:raise ValueError('Invalid pick distance')
        policy=int(EndVelocity(end_velocity))
        if self.mode==GrabMode.FORCE:
            if joint_options:raise ValueError('Joint options require mode=GrabMode.JOINT')
            options=_PickingOptions();options.point[:]=_vector(picked_world_point)
            options.pickingForce=_host_scalar(picking_force);options.pickDistance=self.pick_distance;options.velocityPolicy=policy
            self._api=interaction._picking()
        else:
            options=_Options();options.target[:]=_vector(picked_world_point)
            defaults={'stiffness':1000,'damping':100,'max_force':100000,'clear_locks':True,'speculative_ccd':True}
            unknown=joint_options.keys()-defaults.keys()
            if unknown:raise TypeError('Unknown joint options: '+', '.join(sorted(unknown)))
            defaults.update(joint_options)
            if not isinstance(defaults['clear_locks'],bool) or not isinstance(defaults['speculative_ccd'],bool):raise ValueError('Invalid drag flags')
            options.stiffness=_host_scalar(defaults['stiffness']);options.damping=_host_scalar(defaults['damping']);options.maxForce=_host_scalar(defaults['max_force'])
            options.flags=int(defaults['clear_locks'])|int(defaults['speculative_ccd'])<<1;options.velocityPolicy=policy
            self._api=interaction._interaction_api
        self._owner,self._library=interaction._owner,interaction._library
        self._thread=threading.get_ident();self._handle=C.c_uint64()
        from ovphysx.api import _acquire_process_lifecycle,_release_process_lifecycle
        self._release_lifecycle=_release_process_lifecycle;_acquire_process_lifecycle(self)
        try:_check(self._api.begin(C.byref(ref),C.sizeof(ref),C.byref(options),C.sizeof(options),C.byref(self._handle)),'begin grab')
        except BaseException:
            self._lifecycle_acquired=False;self._release_lifecycle();raise
        self._closed=False;_live.add(self)
        if not _registered:atexit.register(_cleanup);_registered=True

    def _check(self):
        if threading.get_ident()!=self._thread:raise RuntimeError('Drag must run on its host thread')
        if self._closed:raise RuntimeError('Drag is closed')

    @staticmethod
    def _state(value):return DragState(_reference(value.body),tuple(value.target),bool(value.active))

    @property
    def state(self):
        self._check();value=_State()
        _check(self._api.read(self._handle.value,C.byref(value),C.sizeof(value)),'read drag')
        return self._state(value)

    def move(self,target):
        self._check();target=(C.c_float*3)(*_vector(target));value=_State()
        _check(self._api.move(self._handle.value,target,C.byref(value),C.sizeof(value)),'move drag')
        return self._state(value)

    def move_ray(self,origin,direction):
        """Normalize the pointer ray and keep the initial hit distance."""
        self._check();origin=_vector(origin);direction=_vector(direction)
        if self.mode==GrabMode.JOINT:
            length=math.hypot(*direction)
            if length==0:raise ValueError('Expected nonzero ray direction')
            return self.move(tuple(o+d/length*self.pick_distance for o,d in zip(origin,direction)))
        output=_State();native_origin=(C.c_float*3)(*origin);native_direction=(C.c_float*3)(*direction)
        _check(self._api.move_ray(self._handle.value,native_origin,native_direction,C.byref(output),C.sizeof(output)),'move grab ray')
        return self._state(output)

    def set_picking_force(self,coefficient):
        """Change the per-grab coefficient (including zero) for the next substep.

        Legacy angular damping is established once on press; tuning strength does
        not change its saved/temporary values. No shared settings are modified.
        """
        self._check()
        if self.mode!=GrabMode.FORCE:raise ValueError('Picking force requires FORCE mode')
        _check(self._api.set_force_coefficient(self._handle.value,_host_scalar(coefficient)),'set picking force')

    def close(self):
        if self._closed:return
        self._check();_check(self._api.close(self._handle.value),'close drag')
        self._closed=True;self._owner=None;_live.discard(self)
        if self._lifecycle_acquired:self._lifecycle_acquired=False;self._release_lifecycle()

    cancel=close

    def __enter__(self):self._check();return self
    def __exit__(self,*args):self.close()
    def __del__(self):
        if not getattr(self,'_closed',True):
            try:self.close()
            except Exception:_retained.append(self)
