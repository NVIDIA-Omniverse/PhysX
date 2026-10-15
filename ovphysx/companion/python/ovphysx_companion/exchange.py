# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-COMP-OVSTAGE-001
# @covers AC-1 AC-2 AC-3 AC-4
"""Explicit host input/output ordering. Imports neither USD nor ovstage eagerly."""
from contextlib import contextmanager
from dataclasses import dataclass
import ctypes as C
import threading
import weakref
from .native import CompanionError,Status
from .schemas import OUTPUT_NAMESPACE


def _ordinal(value):
    if isinstance(value,bool) or not isinstance(value,int) or not 0<value<2**64:
        raise ValueError('Expected nonzero uint64 ordinal')
    return value


class OutputNotices:
    """Host USD notice adapter, independent of ovstage ordinal filtering.

    Pass output_guard=gate.suppress to EditSession and route USD notices through
    gate.notice. Each output/rollback/undo scope suppresses the solver-input
    callback. No automatic USD observer, Kit dependency or simulation call.
    """
    def __init__(self,authored_notice):
        if not callable(authored_notice):raise TypeError('Pass the host authored-input callback')
        self._callback=authored_notice;self._thread=threading.get_ident();self._depth=0

    def _check(self):
        if threading.get_ident()!=self._thread:raise RuntimeError('Notice adapter must use its host thread')

    @contextmanager
    def suppress(self):
        self._check();self._depth+=1
        try:yield
        finally:self._depth-=1

    def notice(self,*args,**kwargs):
        self._check()
        if not self._depth:return self._callback(*args,**kwargs)


@dataclass(frozen=True)
class OutputColumn:
    """An independently copied numeric column; explicit frame/unit, no SDK view.

    Fixed values have one leading row per selected path; array values are a tuple
    of arrays, one per path. Vector arrays declare lanes explicitly (e.g. 3
    for an N-by-3 point array); scalar arrays have lanes=1. No implicit GPU transfer, pose conversion or ID interning.
    Output paths/attributes are selected by the host and are never inferred from
    geometry. Attributes use the reserved companion:output: namespace, which no
    solver or custom-physics input may declare, so later input reparsing cannot
    consume them. Read this copy after producer/core read-session close if desired.
    """
    attribute:str
    values:object
    frame:str
    units:str
    is_array:bool=False
    semantic:int=0
    lanes:int=1

    def __post_init__(self):
        import numpy as np
        if (not isinstance(self.attribute,str) or '\x00' in self.attribute or not self.attribute.startswith(OUTPUT_NAMESPACE)
                or not self.attribute[len(OUTPUT_NAMESPACE):]):
            raise ValueError(f'Output attributes must use the reserved {OUTPUT_NAMESPACE} namespace; solver and custom-physics inputs cannot be published')
        if not all(isinstance(v,str) and v for v in (self.frame,self.units)) or not isinstance(self.is_array,bool):
            raise ValueError('Declare frame, units and logical array kind')
        if isinstance(self.semantic,bool) or not isinstance(self.semantic,int) or self.semantic not in (0,5,6,7,8,9,10,11,14,15):
            raise ValueError('Numeric geometric semantics only; dictionary IDs require a separate host adapter')
        if isinstance(self.lanes,bool) or not isinstance(self.lanes,int) or not 1<=self.lanes<=65535 or (not self.is_array and self.lanes!=1):
            raise ValueError('Array lanes must be 1..65535; fixed columns use their row shape')
        def copy(values):
            if not isinstance(values,np.ndarray):raise TypeError('Pass a ready CPU numpy.ndarray; explicitly complete/copy device output')
            scalar=values.dtype.subdtype[0] if values.dtype.subdtype else values.dtype
            if scalar.kind not in 'fiu' or not values.ndim or values.dtype.hasobject or not scalar.isnative or scalar.itemsize not in (1,2,4,8):
                raise ValueError('Expected non-object numeric rows')
            if scalar.kind=='f' and not np.isfinite(values).all():raise ValueError('Nonfinite output')
            if self.is_array and (values.ndim!=(1 if self.lanes==1 else 2) or (self.lanes>1 and values.shape[1]!=self.lanes)):
                raise ValueError('Array shape does not match declared element lanes')
            copied=np.array(values,order='C',copy=True)
            # Immutable byte owner prevents re-enabling writes to a captured copy.
            return np.frombuffer(copied.tobytes(),dtype=copied.dtype).reshape(copied.shape)
        copied=tuple(copy(v) for v in self.values) if self.is_array else copy(self.values)
        if self.is_array and (not copied or len({v.dtype for v in copied})!=1):raise ValueError('Array rows require one common scalar dtype')
        object.__setattr__(self,'values',copied)


_flows=weakref.WeakValueDictionary()
_flows_lock=threading.Lock()


class StageFlow:
    """One host-managed writer/input consumer for an existing attached ovstage.

    All stage writers and input updates must go through the host's serialization
    policy. Route authored writes outside this class, seal them, then consume().
    Publish only after all sealed input has been consumed. publish() writes one
    selected batch, seals its output ordinal, and advances the local cursor past
    it WITHOUT forwarding it to the runtime. The next consume starts after that
    cursor. No simulation step, core wait_all, attachment replacement or USD edit.

    Pass the exact Stage object retained by the owner and its last consumed input
    ordinal when opening this adapter. Keep it alive through owner detach. Native
    input/output failures latch the adapter; repair/recreate the source and owner
    before replacing it. Ovstage writes have no rollback guarantee on failure.
    """
    def __init__(self,owner,stage,*,consumed_ordinal):
        from .native import ToolsSession
        from .preview import PreviewSession
        import ovstage
        if not isinstance(owner,(ToolsSession,PreviewSession)) or not isinstance(stage,ovstage.Stage):
            raise TypeError('Pass a ToolsSession/PreviewSession and its application-owned Stage')
        self._preview=isinstance(owner,PreviewSession)
        actual=owner._stage if self._preview else owner._owner._attached_ovstage
        if actual is not stage:raise ValueError('Stage does not belong to the supplied owner')
        self._owner,self._stage=owner,stage;self._identity=owner.identity
        self._cursor=_ordinal(consumed_ordinal);self._thread=threading.get_ident()
        self._closed=True;self._failure=None;self._entered=False
        self.input_updates=0;self.output_batches=0
        if self._floor()<self._cursor:raise ValueError('Initial input ordinal must already be sealed')
        self._key=stage.handle() if hasattr(stage,'handle') else C.cast(stage._inst,C.c_void_p).value
        with _flows_lock:
            if self._key in _flows:raise RuntimeError('Share the existing StageFlow for this stage')
            _flows[self._key]=self
        self._closed=False

    def _floor(self):
        with self._stage.get_attribute_write_floor() as query:return query.fetch()

    def _check(self):
        if threading.get_ident()!=self._thread:raise RuntimeError('StageFlow must run on its host thread')
        if self._closed:raise RuntimeError('StageFlow is closed')
        if self._entered:raise RuntimeError('StageFlow reentry is not permitted')
        if self._failure is not None:raise RuntimeError('StageFlow has a latched failure; repair source/owner before replacement') from self._failure
        if self._owner.identity!=self._identity:raise CompanionError(Status.STALE,'stage flow owner')

    @property
    def cursor(self):return self._cursor

    def consume(self,to_ordinal):
        """Submit new sealed authored input; already handled ranges are no-ops.

        The runtime may realize deferred changes (such as mass) at the next host
        step. This method never steps or fetches to make those visible sooner."""
        self._check();to_ordinal=_ordinal(to_ordinal)
        if to_ordinal<=self._cursor:return False
        if to_ordinal>self._floor():raise ValueError('Seal authored input before consuming it')
        self._entered=True
        try:
            if self._preview:self._owner.update(self._cursor+1,to_ordinal)
            else:
                core=self._owner._owner
                core.wait_task(core.update_from_ovstage(self._cursor+1,to_ordinal))
            self._cursor=to_ordinal;self.input_updates+=1;return True
        except BaseException as error:self._failure=error;raise
        finally:self._entered=False

    def publish(self,ordinal,paths,columns):
        """Explicit copy-in numeric output; no transforms are guessed or converted."""
        self._check();ordinal=_ordinal(ordinal)
        import ovstage
        paths=tuple(paths);columns=tuple(columns)
        if not paths or len(set(paths))!=len(paths) or any(not isinstance(p,str) or not p.startswith('/') or p=='/' or '\x00' in p for p in paths):
            raise ValueError('Select unique absolute output paths')
        if not columns or any(not isinstance(c,OutputColumn) for c in columns):raise TypeError('Pass one or more OutputColumn records')
        if len({c.attribute for c in columns})!=len(columns):raise ValueError('Duplicate output attribute')
        if any(len(c.values)!=len(paths) for c in columns):raise ValueError('Each column needs one row per selected path')
        if self._floor()!=self._cursor:raise ValueError('Consume all sealed authored input before output publication')
        if ordinal<=self._cursor:raise ValueError('Output ordinal must increase')
        writes=[]
        for column in columns:
            values=column.values
            if column.is_array:
                values=[ovstage.make_dltensor(v,shape=[len(v)],ndim=1,
                    dtype=ovstage.DLDataType({'i':0,'u':1,'f':2}[v.dtype.kind],v.dtype.itemsize*8,column.lanes)) for v in values]
            writes.append(ovstage.WriteDesc(column.attribute,values,is_array=column.is_array,semantic=column.semantic))
        self._entered=True
        try:
            with ovstage.PathDictionary(self._stage) as dictionary:
                with dictionary.create_path_list_from_strings(paths) as selected:
                    with self._stage.query_from_path_list(selected) as query:
                        self._stage.write_attributes(query,writes,ordinal).wait()
            self._stage.advance_write_floor(ordinal=ordinal).wait()
            # This output ordinal is excluded even when the caller later asks to
            # consume an older range. A future authored edit starts strictly later.
            self._cursor=ordinal;self.output_batches+=1
        except BaseException as error:self._failure=error;raise
        finally:self._entered=False

    def close(self):
        if threading.get_ident()!=self._thread:raise RuntimeError('Close StageFlow on its host thread')
        if self._closed:return
        if self._entered:raise RuntimeError('Cannot close during publication')
        with _flows_lock:
            if _flows.get(self._key) is self:del _flows[self._key]
        self._closed=True;self._owner=None;self._stage=None

    def __enter__(self):self._check();return self
    def __exit__(self,*args):self.close()
