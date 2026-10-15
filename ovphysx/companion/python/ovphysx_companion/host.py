# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-HOST-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Headless host coordination; UI, selection and command-stack adapters stay outside."""
from concurrent.futures import CancelledError
from dataclasses import dataclass
import math
import threading
import weakref
from .native import CompanionError, Identity, Status, _host_scalar


@dataclass(frozen=True)
class CompletedFrame:
    """Coordinator-local step/time since its creation, not native counters.

    Core stepping before coordinator creation, substeps and rebuilding a
    coordinator can give Inspector/DebugDraw different step numbers.
    """
    owner: Identity
    host_frame: int
    step: int
    simulation_time: float


class CoordinatorConflict(RuntimeError):
    """Another coordinator already owns this attachment's host update loop."""


_coordinators=weakref.WeakValueDictionary()
_coordinators_lock=threading.Lock()


class FrameCoordinator:
    """One host-approved-thread update loop per native attachment.

    Pass the same coordinator to umbrella and PhysX stageupdate adapters. Their
    duplicate tick(frame_id, dt) calls reuse the same completed record, so only
    one adapter may update input/capture output for that frame. The first call
    supplies before_step/after_step callbacks, invoked on this creating thread.
    A second coordinator for the attachment is rejected even through a different
    ToolsSession. Preview owners schedule separately and do not collect playback.

    The host must route *all* steps/updates for this attachment through this object;
    this Python election cannot intercept unrelated direct C/core callers. Callback
    reentry is rejected. Failures pause/latch the loop to prevent accidental replay.
    close relinquishes scheduling, never resets/detaches/closes its borrowed owner.
    For reset: close tools/controllers/coordinator, let the host retire/repopulate
    the owner, then construct new adapters for the new attachment generation.
    """
    def __init__(self,owner):
        from .native import ToolsSession
        from .preview import PreviewSession
        if not isinstance(owner,(ToolsSession,PreviewSession)):
            raise TypeError('Pass a ToolsSession or PreviewSession')
        self._owner=owner
        self.identity=owner.identity
        self._preview=isinstance(owner,PreviewSession)
        self._thread=threading.get_ident()
        self._closed=True
        self._entered=False
        self._paused=False
        self._failure=None
        self._frame=None
        self._dt=None
        self._update=None
        self._steps=0
        self._time=0.
        with _coordinators_lock:
            if self.identity.attachment in _coordinators:
                raise CoordinatorConflict('Attachment already has a host coordinator; share it between adapters')
            _coordinators[self.identity.attachment]=self
        self._closed=False

    def _check(self):
        if threading.get_ident()!=self._thread:raise RuntimeError('Host coordination must run on its approved thread')
        if self._closed:raise RuntimeError('Host coordinator is closed')
        if self._entered:raise RuntimeError('Host callback reentry is not permitted')
        if self._owner.identity!=self.identity:raise CompanionError(Status.STALE,'host owner')

    @property
    def paused(self):return self._paused

    def pause(self):
        self._check();self._paused=True

    def resume(self):
        self._check()
        if self._failure is not None:raise RuntimeError('Repair the failed owner and replace its coordinator') from self._failure
        self._paused=False

    def tick(self,frame_id,dt,*,update=None,before_step=None,after_step=None):
        """Complete one host frame. update=(from_ordinal,to_ordinal) is optional.

        Only this frame's update and step tasks are collected; no wait_all is issued. Preview
        uses its own simulate/fetch interval. Output exceptions latch the loop
        after recording the completed physics step, never cause it to be replayed.
        Callbacks belong to the first adapter's tick; duplicates do not call them.
        """
        self._check()
        if isinstance(frame_id,bool) or not isinstance(frame_id,int) or frame_id<0:raise ValueError('frame_id must be nonnegative int')
        dt=_host_scalar(dt)
        if not math.isfinite(dt) or not 0<dt<=1:raise ValueError('dt must be in (0,1] seconds')
        if self._failure is not None:raise RuntimeError('Host loop has a latched failure') from self._failure
        if self._frame is not None and frame_id<=self._frame.host_frame:
            if frame_id<self._frame.host_frame:raise ValueError('Host frame ids must increase')
            if dt!=self._dt or (tuple(update) if update is not None else None)!=self._update:
                raise ValueError('Duplicate host frame changed its timestep or source update')
            return self._frame
        if self._paused:return None
        for callback in (before_step,after_step):
            if callback is not None and not callable(callback):raise TypeError('Host callbacks must be callable')
        if update is not None:
            if (not isinstance(update,(tuple,list)) or len(update)!=2 or
                    any(isinstance(v,bool) or not isinstance(v,int) or v<1 for v in update) or update[0]>update[1]):
                raise ValueError('update requires positive ordered source ordinals')
        self._entered=True
        try:
            if update is not None:
                if self._preview:self._owner.update(*update)
                else:
                    core=self._owner._owner
                    core.wait_task(core.update_from_ovstage(*update))
            if before_step is not None:before_step()
            if self._preview:self._owner.step(dt)
            else:
                core=self._owner._owner
                task=core.step(dt)
                core.wait_task(task)
            self._steps+=1;self._time+=dt
            self._frame=CompletedFrame(self.identity,frame_id,self._steps,self._time)
            self._dt=dt
            self._update=tuple(update) if update is not None else None
            if after_step is not None:after_step(self._frame)
            return self._frame
        except BaseException as error:
            self._failure=error;self._paused=True
            raise
        finally:self._entered=False

    def close(self):
        if threading.get_ident()!=self._thread:raise RuntimeError('Close on the host approved thread')
        if self._closed:return
        if self._entered:raise RuntimeError('Cannot close from a host callback')
        with _coordinators_lock:
            if _coordinators.get(self.identity.attachment) is self:
                del _coordinators[self.identity.attachment]
        self._closed=True;self._owner=None

    def __enter__(self):self._check();return self
    def __exit__(self,*args):self.close()


@dataclass(frozen=True)
class Availability:
    """An action-level capability resolved by the host for its actual backend."""
    action: str
    available: bool
    reason: str=''

    def __post_init__(self):
        if not isinstance(self.action,str) or not self.action:raise ValueError('Name the action')
        if not isinstance(self.available,bool):raise TypeError('available must be bool')
        if not self.available and (not isinstance(self.reason,str) or not self.reason):
            raise ValueError('An unavailable action requires a specific diagnostic')

    def require(self):
        if not self.available:raise CompanionError(Status.UNAVAILABLE,f'{self.action}: {self.reason}')


class DeferredEdit:
    """Deliver one copied authoring job on the customer's USD-approved thread.

    source is captured with the immutable input, not when the job finishes. apply
    receives the job result after revision validation and returns an EditPatch for
    the host command stack. cancel suppresses publication even when native work
    has already completed and job.cancel() cannot stop it. Service ownership/drain
    remains with the authoring service. No worker thread may touch USD.
    """
    def __init__(self,session,source,job,apply):
        session.validate(source)
        if not callable(apply):raise TypeError('apply must be callable')
        self._session,self._source,self._job,self._apply=session,source,job,apply
        self._thread=threading.get_ident()
        self._terminal=False
        self._cancelled=False
        self.patch=None
        self._error=None

    def _check(self):
        if threading.get_ident()!=self._thread:raise RuntimeError('Deliver/cancel edits on the host approved thread')

    @property
    def done(self):return self._terminal

    def cancel(self):
        self._check()
        if self._terminal:return False
        self._cancelled=self._terminal=True
        self._job.cancel()
        self._apply=None
        return True

    def deliver(self):
        self._check()
        if self._cancelled:raise CancelledError('Host cancelled authoring publication')
        if self._error is not None:raise self._error
        if self._terminal:return self.patch
        if not self._job.done:return None
        # Terminal before application: any failure must never retry a partial edit.
        self._terminal=True
        apply,self._apply=self._apply,None
        try:
            self._session.validate(self._source)
            self.patch=apply(self._job.result())
            return self.patch
        except BaseException as error:
            self._error=error
            raise

    def close(self):self.cancel()


def cpu_tool_availability(owner,action):
    """Discover qualified CPU CCT/debug actions without creating another runtime.

    This conservative host policy requires hard CPU mode. A runtime started in
    GPU-capable mode may contain CPU scenes too, but those mixed modes require
    their own host qualification. Never infer availability from header presence.
    """
    import ctypes as C
    from ovphysx import PhysX
    from .native import ToolsSession
    from .preview import PreviewSession
    if not isinstance(owner,(ToolsSession,PreviewSession)):
        raise TypeError('Pass a ToolsSession or PreviewSession')
    owner.identity
    if action=='character-controller':
        from .cct import _Api
        symbol='ovphysx_companion_cct_acquire'
    elif action=='vehicle-controller':
        from .vehicle import _Api
        symbol='ovphysx_companion_vehicle_acquire'
    elif action=='typed-inspection':
        from .inspection import _Api
        symbol='ovphysx_companion_inspection_acquire'
    elif action=='scene-queries':
        from .queries import _Api
        symbol='ovphysx_companion_queries_acquire'
    elif action=='interaction':
        from .interaction import _Api
        symbol='ovphysx_companion_interaction_acquire'
    elif action=='debug-geometry':
        from .debug import _Api
        symbol='ovphysx_companion_debug_acquire'
    else:
        return Availability(str(action),False,'This host adapter has no qualified provider for the action')
    try:acquire=getattr(owner._library,symbol)
    except AttributeError:return Availability(action,False,'The loaded runtime has no companion service table')
    acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
    table=_Api()
    status=acquire(1,C.sizeof(table),C.byref(table))
    if status!=Status.OK or table.version!=1 or table.size!=C.sizeof(table):
        return Availability(action,False,'The loaded companion table is ABI-incompatible')
    if not table.capabilities&1:return Availability(action,False,'CPU support is absent from the service capabilities')
    if not PhysX.get_cpu_mode():return Availability(action,False,'This adapter qualifies the service only in hard CPU mode')
    return Availability(action,True)
