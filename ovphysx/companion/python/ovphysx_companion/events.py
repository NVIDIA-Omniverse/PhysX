# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-EVENTS-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Bounded native callback queues, copied and delivered on the creating host thread."""
import atexit
import ctypes as C
from dataclasses import dataclass
from enum import IntEnum,IntFlag
import threading
import warnings
import weakref
from .native import CompanionError,Status,_check


class Mask(IntFlag):
    PRE_STEP=1
    POST_STEP=2
    SUBSTEPS=4
    CONTACT=8
    TRIGGER=16
    OBJECT=32
    ERROR=64
    ALL=127


class Kind(IntEnum):
    PRE_STEP=1
    SUBSTEP_BEGIN=2
    SUBSTEP_END=3
    POST_STEP=4
    SIMULATION_COMPLETE=5
    CONTACT=6
    TRIGGER=7
    OBJECT_CREATED=8
    OBJECT_REMOVED=9
    OBJECTS_RESET=10
    RUNTIME_ERROR=11
    TRIGGER_STAY=12


class Phase(IntEnum):
    PRE_STEP=0
    SUBSTEP=1
    POST_STEP=2
    SOURCE_LIFECYCLE=3
    PROCESS=4


class ContactType(IntEnum):
    FOUND=0
    LOST=1
    PERSISTS=2


class TriggerType(IntEnum):
    ENTER=0
    LEAVE=1


class ErrorType(IntEnum):
    USD_LOAD=0
    PHYSX=1
    CUDA=2
    TOO_MANY_ERRORS=3


class _Event(C.Structure):
    _fields_=[(n,C.c_uint64) for n in ('sequence','attachment','generation','step','substep','pair_sequence',
        'actor0','actor1','collider0','collider1','material0','material1','object_id')]+[
        (n,C.c_uint32) for n in ('kind','phase','type','flags','instance0','instance1','point_index','point_count','face0','face1')]+[
        ('dt',C.c_float),('separation',C.c_float),('position',C.c_float*3),('normal',C.c_float*3),('impulse',C.c_float*3),('message',C.c_char*256)]


class _Batch(C.Structure):
    _fields_=[('attachment',C.c_uint64),('dropped',C.c_uint64),('completed_step',C.c_uint64),
              ('count',C.c_uint32),('overflow',C.c_uint32),('owner_live',C.c_uint32),('reserved',C.c_uint32)]


class _Api(C.Structure):
    _fields_=[('size',C.c_uint32),('version',C.c_uint32),('capabilities',C.c_uint64),
              ('open',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_uint32,C.c_uint32,C.c_char_p,C.POINTER(C.c_uint64))),
              ('poll',C.CFUNCTYPE(C.c_uint32,C.c_uint64,C.c_uint32,C.POINTER(_Event),C.c_uint32,C.c_uint32,C.POINTER(_Batch))),
              ('close',C.CFUNCTYPE(C.c_uint32,C.c_uint64))]


@dataclass(frozen=True)
class Pair:
    actor0:int
    actor1:int
    collider0:int
    collider1:int
    instance0:int|None
    instance1:int|None


@dataclass(frozen=True)
class ContactPoint:
    index:int
    count:int
    position:tuple
    normal:tuple
    impulse:tuple
    separation:float
    material0:int
    material1:int
    face0:int|None
    face1:int|None


@dataclass(frozen=True)
class Event:
    sequence:int
    attachment:int
    generation:int
    step:int
    substep:int
    kind:Kind
    phase:Phase
    type:int
    dt:float
    pair_sequence:int
    object_id:int
    pair:Pair
    point:ContactPoint|None
    message:str
    text_truncated:bool

    @property
    def contact_type(self):return ContactType(self.type) if self.kind==Kind.CONTACT else None
    @property
    def trigger_type(self):return TriggerType(self.type) if self.kind==Kind.TRIGGER else None
    @property
    def error_type(self):return ErrorType(self.type) if self.kind==Kind.RUNTIME_ERROR else None


@dataclass(frozen=True)
class Batch:
    attachment:int
    completed_step:int
    owner_live:bool
    events:tuple
    overflow:bool
    dropped:int


class EventOverflow(RuntimeError):
    """History is incomplete; batch is diagnostic only. Rebuild pair state explicitly."""
    def __init__(self,batch):
        self.batch=batch
        super().__init__(f'Event history overflowed ({batch.dropped} lost records). Close, reset/rebuild the scene and resubscribe before stepping.')


def _optional(value):return None if value==2**32-1 else value


def _event(value):
    point=ContactPoint(value.point_index,value.point_count,tuple(value.position),tuple(value.normal),tuple(value.impulse),
        value.separation,value.material0,value.material1,_optional(value.face0),_optional(value.face1)) if value.point_count else None
    return Event(value.sequence,value.attachment,value.generation,value.step,value.substep,Kind(value.kind),Phase(value.phase),
        value.type,value.dt,value.pair_sequence,value.object_id,Pair(value.actor0,value.actor1,value.collider0,value.collider1,
        _optional(value.instance0),_optional(value.instance1)),point,value.message.decode('utf-8',errors='replace'),bool(value.flags&1))


_live=weakref.WeakSet()
_retained=[]
_registered=False


def _cleanup():
    for queue in list(_live)+list(_retained):
        try:queue.close()
        except Exception as error:warnings.warn(f'Event queue cleanup failed: {error}',RuntimeWarning)


class EventQueue:
    """Owner-scoped native events, polled after results on the creating thread.

    Native callbacks copy into bounded storage and never enter Python. Phase marks
    where an event was produced, not permission to change physics during callback.
    step is the native owner's substep counter, not a UI frame number. Concurrent
    scene callbacks have observed FIFO sequence order, not deterministic scheduling.
    Contact reports can batch multiple substeps; their step is the flush timestamp.

    Contact/trigger APIs must already be authored. An optional filter_path matches
    either contact body/collider or either trigger collider. Contact and trigger instance values
    preserve native point-instancer indices, including separate trigger stay pairs.
    Stay derives only from observed enters, once per completed owner step. Subscribe
    before initial stepping for complete history; late subscriptions do not discover
    existing overlaps. Reset/deletion invalidates tracked pairs. Process error events
    carry attachment=0 because the native error stream has no reliable owner origin.

    poll raises EventOverflow on sticky overflow. allow_incomplete=True is an explicit
    diagnostic opt-in, never a repair of pair state. Close discards queued records;
    callbacks already admitted cannot enqueue after close. Copied batches remain valid.
    """
    def __init__(self,owner,*,mask=Mask.ALL,capacity=4096,filter_path=None):
        global _registered
        from .native import ToolsSession
        from .preview import PreviewSession
        if not isinstance(owner,(ToolsSession,PreviewSession)):raise TypeError('Pass a ToolsSession or PreviewSession')
        if isinstance(mask,bool) or not isinstance(mask,int) or not 0<int(mask)<=127:raise ValueError('Invalid event mask')
        if isinstance(capacity,bool) or not isinstance(capacity,int) or not 1<=capacity<=65536:raise ValueError('capacity requires 1..65536')
        if filter_path is not None and (not isinstance(filter_path,str) or not filter_path.startswith('/') or '\0' in filter_path):
            raise ValueError('filter_path requires an absolute path')
        self._closed=True;self._owner=owner;self._library=owner._library;self._thread=threading.get_ident()
        self._attachment=owner.identity.attachment;self._handle=C.c_uint64();self._api=_Api();self._capacity=capacity
        try:acquire=self._library.ovphysx_companion_events_acquire
        except AttributeError as error:raise CompanionError(Status.UNAVAILABLE,'events table') from error
        acquire.argtypes=[C.c_uint32,C.c_uint32,C.POINTER(_Api)];acquire.restype=C.c_uint32
        _check(acquire(1,C.sizeof(self._api),C.byref(self._api)),'acquire events')
        if self._api.size!=C.sizeof(self._api) or self._api.version!=1 or int(mask)&~self._api.capabilities or not all((self._api.open,self._api.poll,self._api.close)):
            raise CompanionError(Status.ABI_MISMATCH,'events table')
        from ovphysx.api import _acquire_process_lifecycle,_release_process_lifecycle
        self._release_lifecycle=_release_process_lifecycle
        _acquire_process_lifecycle(self)
        try:_check(self._api.open(self._attachment,int(mask),capacity,filter_path.encode() if filter_path else None,C.byref(self._handle)),'open event queue')
        except BaseException:
            self._lifecycle_acquired=False;self._release_lifecycle();raise
        self._closed=False;_live.add(self)
        if not _registered:atexit.register(_cleanup);_registered=True

    def _check_thread(self):
        if threading.get_ident()!=self._thread:raise RuntimeError('Event delivery requires the creating host thread')

    def poll(self,*,allow_incomplete=False):
        self._check_thread()
        if self._closed:raise RuntimeError('Event queue is closed')
        values=(_Event*self._capacity)();info=_Batch()
        status=self._api.poll(self._handle.value,C.sizeof(_Event),values,self._capacity,C.sizeof(info),C.byref(info))
        if status!=8:_check(status,'poll event queue')
        elif not info.overflow:raise CompanionError(Status.ABI_MISMATCH,'history-lost status without overflow')
        batch=Batch(info.attachment,info.completed_step,bool(info.owner_live),tuple(_event(values[i]) for i in range(info.count)),bool(info.overflow),info.dropped)
        if batch.overflow and not allow_incomplete:raise EventOverflow(batch)
        return batch

    def dispatch(self,callback):
        """Host/graph adapter hook with fail-stop consumption of a polled batch.

        Handler exceptions propagate; the already polled tail is not requeued.
        Hosts requiring retry must poll() and retain their batch/cursor, then
        invoke handlers themselves. Closing from a handler stops delivery.
        """
        if not callable(callback):raise TypeError('callback must be callable')
        batch=self.poll()
        for event in batch.events:
            if self._closed:break
            callback(event)
        return batch

    def close(self):
        self._check_thread()
        if self._closed:return
        _check(self._api.close(self._handle.value),'close event queue')
        self._closed=True;self._owner=None;_live.discard(self)
        if self._lifecycle_acquired:self._lifecycle_acquired=False;self._release_lifecycle()

    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def __del__(self):
        if not getattr(self,'_closed',True):
            try:self.close()
            except Exception:_retained.append(self)
