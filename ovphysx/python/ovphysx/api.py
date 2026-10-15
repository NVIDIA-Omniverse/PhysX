# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-DETACH-001
# @covers AC-3 AC-4 AC-6
# @implements REQ-PYTHON-FRAME-001
# @covers AC-1
# @implements REQ-CAPI-COMPANION-001
# @covers AC-2

# @implements REQ-CAPI-CONTACT-003
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-PYTHON-BINDING-SYNC-001
# @covers AC-1 AC-2 AC-3
#
# @implements REQ-CAPI-READ-CONTACT-002
# @covers AC-3

# @implements REQ-PYTHON-TASK-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10
# @implements REQ-CAPI-OVD-NAMES-001
# @covers AC-1 AC-2 AC-3 AC-4

# @implements REQ-CAPI-LOG-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
# @implements REQ-PYTHON-KEYWORD-001
# @covers AC-1 AC-2
# @implements REQ-CAPI-PATTERN-002
# @covers AC-8
# @implements REQ-CAPI-SELECT-001
# @covers AC-6
# @implements REQ-PYTHON-OMNIPVD-LATE-001
# @covers AC-3 AC-4 AC-5 AC-6 AC-7 AC-8
# @implements REQ-CAPI-WRITE-001
# @covers AC-5a AC-9 AC-11 AC-12
# @implements REQ-PYTHON-READPOOL-001
# @covers AC-3
# @implements REQ-PYTHON-BINDING-DEVICE-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-PYTHON-CLONE-001
# @covers AC-2
# @implements REQ-CAPI-OVSTAGE-SCHEMA-001
# @covers AC-5
# @implements REQ-PACKAGING-BUILDNUM-001
# @covers AC-7
# @implements REQ-CAPI-CONTACT-002
# @covers AC-2 AC-3 AC-4
# @implements REQ-CAPI-CONTACT-001
# @covers AC-5 AC-6 AC-7
# @implements REQ-CAPI-SCENEQUERY-001
# @covers AC-1
# @implements REQ-CAPI-INSTANCE-SCOPE-001
# @covers AC-1 AC-4

"""High-level Python API for the ovphysx library.

Stream-Ordered Execution Model
------------------------------
Operations on each instance execute in submission order. Submission may execute
inline; the task contract still separates admission from completion:

- Operations appear to complete in submission order
- Writes from operation N are visible to operation N+1
- Wait for a creator before passing its output handle to another operation
- Independent operations may execute concurrently internally for performance

Example using the explicit synchronous read convenience:

.. code-block:: python
   :caption: Stream-ordered operation sequence

    from ovphysx.types import ObjectScope, SimObjectType

    def step_and_read(physx, stage, initial_ordinal, from_ordinal, to_ordinal, dt):
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=initial_ordinal))
        # After the application authors later ovstage edits:
        physx.wait_task(physx.update_from_ovstage(from_ordinal, to_ordinal))
        physx.step(dt)  # Sees the drained stage edits
        with physx.read_sync(
            SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL
        ) as result:  # Reads current state
            for group in result.groups:
                for tensor in group.tensors:
                    print(tensor.numpy())

Use Operation.wait(), wait_task(), or wait_all() when:

- Before materializing an operation's typed result
- To ensure operations complete before program exit
- For explicit synchronization points in your application

Host task collection and CUDA stream dependencies are separate. Materializing a
read group orders its producer event onto the current Warp stream. A write
commit's stream and event describe the caller's producer completion; they do not
select the stream PhysX uses for its own work.

Thread Safety
-------------
- Each instance holds at most one live ovstage attach. Several instances may be
  attached at once, and attaching one does not detach another. The same ovstage
  instance cannot be attached to two PhysX instances.
- Different instances may step their own attachments at the same time. A single
  instance is NOT thread-safe. Use external synchronization if calling from
  multiple threads on the same instance.
- Process-wide limits still apply: one hard CPU-only mode, one SDK tolerance
  scale, and a recording start/stop that needs every attached instance idle.
- ctypes releases the GIL during native calls, so concurrent ``step()`` and
  ``PhysX.read()`` / ``PhysX.write()`` from different threads on the same
  instance is a data race.
  See the developer guide threading section for the recommended pattern.

"""

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9
# @implements REQ-PYTHON-SDFVIEW-001
# @covers AC-1
# @implements REQ-CAPI-CACHE-001
# @covers AC-5
# @implements REQ-PYTHON-READ-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
# @implements REQ-CAPI-CONFIG-001
# @covers AC-3 AC-5 AC-6 AC-8
# @implements REQ-CAPI-SDF-001
# @covers AC-3 AC-4

import atexit
import ctypes
import gc
import itertools
import logging
import math
import operator
import threading
import warnings
import weakref
from ctypes import (
    POINTER,
    byref,
    c_char_p,
    c_double,
    c_float,
    c_int32,
    c_int64,
    c_uint32,
    c_uint64,
    c_void_p,
    cast,
)
from dataclasses import dataclass
from enum import IntFlag
from ._operations import Operation, registered_tasks as _registered_tasks, dispatch as _dispatch_operations, drain_cleanups as _drain_operation_cleanups
from typing import TYPE_CHECKING, Any, Callable, NamedTuple

from packaging.version import Version

if TYPE_CHECKING:
    from .config import OmniPvdDestination, PhysXConfig

from . import __version__ as _python_version
from ._bindings import OVPHYSX_INVALID_HANDLE as _INVALID_HANDLE
from ._bindings import (
    OVPHYSX_CONFIG_VALUE_BOOL,
    OVPHYSX_CONFIG_VALUE_DOUBLE,
    OVPHYSX_CONFIG_VALUE_INT64,
    OVPHYSX_CONFIG_VALUE_STRING,
    _lib,
    ovphysx_all_objects_destroyed_fn,
    ovphysx_articulation_metadata_t,
    ovphysx_config_entry_t,
    ovphysx_config_value_t,
    ovphysx_create_args,
    ovphysx_log_callback_t,
    ovphysx_object_change_callbacks_t,
    ovphysx_object_created_fn,
    ovphysx_object_destroyed_fn,
    ovphysx_omnipvd_destination_t,
    ovphysx_enqueue_result_t,
    ovphysx_wait_task_result_t,
    ovphysx_string_t,
    ovphysx_contact_sensor_selection_t,
    ovphysx_query_selection_t,
    ovphysx_read_result_t,
    ovphysx_write_result_t,
    ovphysx_fetched_write_group_t,
    ovphysx_tensor_binding_desc_t,
    ovphysx_tensor_spec_t,
    ovstage_cuda_sync_t,
    ovstage_map_group_t,
    ovstage_ordinal_range_t,
    ovstage_query_result_t,
    ovstage_read_group_t,
    ovx_string_or_token_t,
    ovx_string_t,
    ovphysx_scene_query_geometry_desc_t,
    ovphysx_scene_query_hit_t,
)
from .types import (
    ApiStatus,
    ArticulationKinematicFlag,
    ConfigKey,
    ContactSelection,
    GlobalConfigKey,
    LogLevel,
    ObjectScope,
    ObjectType,
    SceneQueryGeometryType,
    SceneQueryMode,
    SimObjectType,
    TensorType,
)

# Set of tensor types for which articulation metadata (dof_count, body_names, etc.) is valid.
# Derived from the enum so it stays in sync automatically when new ARTICULATION_ types are added.
_ARTICULATION_TENSOR_TYPES: frozenset[int] = frozenset(t for t in TensorType if t.name.startswith("ARTICULATION_"))


@dataclass(frozen=True, slots=True)
class Task:
    """Immutable receipt for an accepted operation, without instance ownership.

    Pass the receipt to ``PhysX.wait_task`` on its owning instance. Completion
    is collected once, either by that wait or by ``PhysX.wait_all``. Discarding
    a receipt does not discard an uncollected failure. ``status`` describes
    admission, ``op_index`` identifies the task within its instance, and
    ``instance`` is the owning native handle without a lifetime reference.
    """

    status: ApiStatus
    op_index: int
    instance: int

    @classmethod
    def _from_native(cls, receipt: ovphysx_enqueue_result_t) -> "Task":
        return cls(ApiStatus(receipt.status), receipt.op_index, receipt.instance)

    def _as_native(self) -> ovphysx_enqueue_result_t:
        return ovphysx_enqueue_result_t(self.status, self.op_index, self.instance)


class TaskError(RuntimeError):
    """Collected task failures, copied independently of native result storage."""

    def __init__(self, errors: tuple[tuple[Task, str], ...]) -> None:
        self._errors = tuple((task, message) for task, message in errors)
        super().__init__("Task(s) failed:\n  " + "\n  ".join(
            f"task {task.op_index}: {message}" for task, message in self._errors
        ))

    @property
    def errors(self) -> tuple[tuple[Task, str], ...]:
        """Immutable ``(Task, message)`` entries in collection order."""
        return self._errors


class TaskTimeoutError(TimeoutError):
    """A wait deadline expired, possibly after collecting earlier failures."""

    def __init__(self, errors: tuple[tuple[Task, str], ...], pending_task: Task) -> None:
        self._errors = tuple((task, message) for task, message in errors)
        self._pending_task = pending_task
        super().__init__(f"Task {pending_task.op_index} timed out")

    @property
    def errors(self) -> tuple[tuple[Task, str], ...]:
        """Immutable completed failures collected before the pending task."""
        return self._errors

    @property
    def pending_task(self) -> Task:
        """Earliest unfinished task, which remains available for another wait."""
        return self._pending_task


def _wait_for_task(sdk, receipt, operation: str) -> None:
    """Collect a native operation hidden by a waiting Python convenience."""
    if receipt.status != ApiStatus.SUCCESS:
        raise RuntimeError(f"{operation} failed: {sdk._get_last_error()}")
    sdk.wait_task(Task._from_native(receipt))


def _chain_cleanup(primary, cleanup) -> None:
    """Append unique causes, removing cycles without losing cleanup diagnostics."""
    chain = []
    seen = set()
    for error in (primary, cleanup):
        while error is not None and id(error) not in seen:
            seen.add(id(error))
            chain.append(error)
            error = error.__cause__
    for index, error in enumerate(chain):
        error.__cause__ = chain[index + 1] if index + 1 < len(chain) else None
        if error.__context__ is not None and id(error.__context__) in seen:
            # The explicit cause chain already preserves this diagnostic. Its
            # automatic context back-edge would keep traceback owners in a cycle.
            error.__context__ = None


def _run_cleanups(cleanups, *, primary=None) -> None:
    """Attempt every cleanup and retain all failures in the exception chain."""
    failure = None
    try:
        for cleanup in cleanups:
            try:
                cleanup()
            except Exception as error:
                if failure is not None:
                    _chain_cleanup(error, failure)
                failure = error
        if failure is not None:
            if primary is not None:
                _chain_cleanup(primary, failure)
                raise primary
            raise failure
    finally:
        # Exceptions own their traceback frames; do not make the frames own them.
        failure = primary = None


def _stage_dictionary(stage):
    if stage is None:
        raise RuntimeError("session has no owning ovstage.Stage")
    from ovstage import PathDictionary
    return PathDictionary(stage)


def _release_handle(holder, field: str, native_release) -> Task | None:
    """Transfer a handle on admission; rejected releases remain retryable."""
    handle = getattr(holder, field)
    if not handle:
        return
    sdk = holder._sdk
    instance = getattr(getattr(sdk, "_omni_physx_sdk_handle", None), "value", None)
    if instance is None:
        setattr(holder, field, 0)
        return
    receipt = native_release(instance, handle)
    if receipt.status == ApiStatus.SUCCESS:
        # The SDK owns cleanup, including quarantine after terminal failure. Never resubmit it.
        setattr(holder, field, 0)
    return sdk._admit_session_task(receipt, "release " + field.lstrip("_"))


@dataclass(frozen=True, slots=True)
class QueryMetadata:
    """Creation-time discovery; attributes and counts are immutable snapshots."""

    total_prim_count: int
    attributes: tuple[int, ...]


class ReadOmission(IntFlag):
    """Reasons applicable selected partitions did not produce a requested column."""

    NONE = 0
    UNSUPPORTED_ATTRIBUTE = 1
    UNSUPPORTED_CONFIGURATION = 2
    NOT_READY = 4


@dataclass(frozen=True, slots=True)
class ReadAttributeResult:
    attribute: int
    group_count: int
    omission_reasons: ReadOmission


@dataclass(frozen=True, slots=True)
class ReadMetadata:
    total_prim_count: int
    attributes: tuple[ReadAttributeResult, ...]


@dataclass(frozen=True, slots=True)
class WriteAttributeResult:
    attribute: int
    group_count: int


@dataclass(frozen=True, slots=True)
class WriteMetadata:
    total_prim_count: int
    attributes: tuple[WriteAttributeResult, ...]


class Query:
    """A completed reusable selector owned by one instance and attachment.

    Release independently of sessions already opened from it. Whole-type queries
    evaluate membership at each session open; selected queries keep their resolved
    membership. Recreate a query to refresh its creation-time discovery metadata.
    """

    def __init__(self, sdk, handle, object_type, metadata, stage, attachment):
        self._sdk = sdk
        self._query = int(handle)
        self.object_type = SimObjectType(object_type)
        self.metadata = metadata
        self._stage = stage
        self._attachment = attachment

    @property
    def handle(self) -> int:
        self._validate()
        return self._query

    def _validate(self):
        self._sdk._check_valid()
        if not self._query:
            raise RuntimeError("query is closed")
        if self._sdk._get_attachment_identity() != self._attachment:
            raise RuntimeError("query belongs to an invalidated attachment")

    @property
    def dictionary(self):
        """Owner-aware PathDictionary for tokens and prim lists of this query."""
        self._validate()
        return _stage_dictionary(self._stage)

    def close(self) -> Operation[None]:
        task = _release_handle(self, "_query", self._sdk._lib.ovphysx_release_query)
        return Operation(self._sdk, () if task is None else (task,))

    release = close

    def close_sync(self) -> None:
        self.close().wait()

    def __enter__(self):
        self._validate()
        return self

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close()
        except Exception as cleanup_error:
            if exc is not None:
                _chain_cleanup(exc, cleanup_error)
                raise exc.with_traceback(traceback)
            raise
        finally:
            exc = traceback = None

    def __del__(self):
        try:
            if self._query:
                self._sdk._enqueue_write_release(_WriteRelease(self._sdk, self._query, 0))
                self._query = 0
        except Exception:
            pass


class WriteGroup(NamedTuple):
    """Writable zero-copy buffers for one original attribute request.

    ``attribute_index`` identifies the original request; ``attribute`` is its
    canonical token. Use the exact group with its session's commit operation.
    Accepted commit, session release, or attachment teardown ends buffer access,
    including when a commit subsequently fails. Commits are not transactional.
    """

    prim_list: int
    prim_offset: int
    prim_count: int
    tensors: "list"
    attribute_index: int
    attribute: int


class _WriteRelease:
    """Queue-only finalizer holder; native release runs on the owning thread."""

    def __init__(self, sdk, query: int, write: int):
        self._sdk_ref = weakref.ref(sdk)
        self._query = int(query)
        self._write = int(write)
        self._cleanup_operations = []

    @property
    def _sdk(self):
        sdk = self._sdk_ref()
        if sdk is None:
            raise ReferenceError("owning PhysX instance no longer exists")
        return sdk

    @property
    def pending(self) -> bool:
        return bool(self._write or self._query)

    def _free_native(self) -> tuple[Task, ...]:
        tasks = []
        def release(field, fn):
            task = _release_handle(self, field, fn)
            if task is not None:
                tasks.append(task)
        try:
            _run_cleanups((
                lambda: release("_write", self._sdk._lib.ovphysx_release_write),
                lambda: release("_query", self._sdk._lib.ovphysx_release_query),
            ))
        except Exception:
            if tasks:
                self._cleanup_operations.append(Operation(weakref.proxy(self._sdk), tasks))
            raise
        return tuple(tasks)


class WriteSession:
    """A completed write session; fetch, commit and close return uncollected operations.

    Fill fetched groups in place and commit each explicitly. A successful earlier
    commit remains applied when a later commit fails; an accepted failed commit
    may have partially applied rows and must never be retried. Caller commit order
    determines command/property ordering. The caller's Query is not owned here.
    Group arrays are borrowed: do not access them after commit admission or close.
    """

    def __init__(self, sdk, query: int, write: int, groups=None, native=None, *,
                 metadata=None, attributes=(), stage=None, attachment=None, object_type=None):
        self._sdk = sdk
        self._query = int(query)
        self._write = int(write)
        self.groups = [] if groups is None else groups
        self._native = [] if native is None else native
        self.metadata = metadata
        self._attributes = tuple(attributes)
        self._stage = stage
        self._attachment = attachment
        self._object_type = object_type
        self._write_generation = getattr(sdk, "_write_generation", 0)
        self._closed = False
        self._spent = set()
        self._sync = False
        self._release = _WriteRelease(sdk, self._query, self._write)

    def _validate(self):
        self._sdk._check_valid()
        if self._closed or not self._release._write:
            raise RuntimeError("write session is closed")
        if self._write_generation != getattr(self._sdk, "_write_generation", 0):
            raise RuntimeError("write session was retired by attachment teardown")
        if self._attachment is not None and self._sdk._get_attachment_identity() != self._attachment:
            raise RuntimeError("write session belongs to an invalidated attachment")

    def fetch_next(self) -> Operation[WriteGroup | None]:
        """Submit one fetch. A successful result of None is end of iteration."""
        self._validate()
        output = ovphysx_fetched_write_group_t()
        receipt = self._sdk._lib.ovphysx_fetch_write_next(
            self._sdk._omni_physx_sdk_handle.value, self._write, byref(output))
        task = self._sdk._admit_session_task(receipt, "write fetch")
        def convert():
            self._validate()
            if not output.group:
                return None
            index = int(output.attribute_index)
            if index >= len(self._attributes):
                raise RuntimeError("write fetch returned an invalid attribute request index")
            native = output.group
            data = native.contents
            attribute = self._attributes[index]
            if self.metadata is not None:
                token = self.metadata.attributes[index].attribute
            elif isinstance(attribute, int):
                token = int(attribute)
            elif self._stage is None and self._object_type == SimObjectType.DEBUG_RENDER:
                # Debug render has no prim dictionary. Other stage-less sessions still
                # fail here so a string attribute cannot invent a private dictionary.
                token = 0
            else:
                token = _stage_dictionary(self._stage).intern_token(attribute)
            import warp as wp
            _warn_if_warp_build_breaks_cpu_mode(wp)
            tensors = [self._sdk._dltensor_to_warp_array(data.data.tensors[i], wp, None)
                       for i in range(int(data.data.tensor_count))] if data.data.tensors else []
            group = WriteGroup(int(data.prims.list), int(data.prims.offset), int(data.prims.count),
                               tensors, index, token)
            self.groups.append(group)
            self._native.append(native)
            return group
        return Operation(self._sdk, (task,), convert=convert, keepalive=(output,))

    def commit(self, group: WriteGroup, cuda_stream: int = 0, cuda_wait_event: int = 0,
               *, event_owner=None) -> Operation[None]:
        """Submit a filled group's publication, spending it on admission.

        Complete host fills before calling. For outstanding CUDA fills, pass the
        producer stream, completion event, or both; both dependencies are honored
        when supplied. They do not select PhysX's execution stream. Only two zero
        handles assert that no producer work remains, even for host-resident data.
        Stream 1 names the legacy default stream in the caller's current CUDA
        context; make the producer context current when using this sentinel.

        Keep supplied streams and events valid through commit completion. Pass
        event_owner to retain their Python owner, or a tuple of owners, through
        collection. The current native implementation may block this call while
        completing producer dependencies. A successful task confirms CPU mutation
        or GPU submission; GPU execution can continue afterward. Synchronization
        errors fail the task and the admitted group remains spent.
        """
        self._validate()
        index = next((i for i, candidate in enumerate(self.groups) if candidate is group), None)
        if index is None:
            raise ValueError("group does not belong to this write session")
        if index in self._spent:
            raise RuntimeError("write group has already been committed")
        sync = ovstage_cuda_sync_t()
        sync.stream = int(cuda_stream)
        sync.wait_event = int(cuda_wait_event)
        receipt = self._sdk._lib.ovphysx_commit_group(
            self._sdk._omni_physx_sdk_handle.value, self._write, self._native[index], sync)
        task = self._sdk._admit_session_task(receipt, "commit")
        self._spent.add(index)
        return Operation(self._sdk, (task,), keepalive=(sync, event_owner), owners=(self,))

    def commit_sync(self, group: WriteGroup, cuda_stream: int = 0, cuda_wait_event: int = 0,
                    *, event_owner=None) -> None:
        self.commit(group, cuda_stream, cuda_wait_event, event_owner=event_owner).wait()

    def close(self) -> Operation[None]:
        """Submit release. Rejected admission leaves ownership available for retry.

        Finish CUDA fills of uncommitted groups first; close takes no producer
        synchronization handles.
        """
        try:
            tasks = self._release._free_native()
        finally:
            self._closed = not self._release._write
        return Operation(self._sdk, tasks)

    release = close

    def close_sync(self) -> None:
        _run_cleanups((
            lambda: self.close().wait(),
            lambda: _run_cleanups(operation.wait for operation in self._release._cleanup_operations),
        ))

    def __enter__(self):
        self._validate()
        return self

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close_sync() if self._sync else self.close()
        except Exception as cleanup_error:
            if exc is not None:
                _chain_cleanup(exc, cleanup_error)
                raise exc.with_traceback(traceback)
            raise
        finally:
            exc = traceback = None

    def __del__(self):
        try:
            if self._release.pending:
                self._closed = True
                self._sdk._enqueue_write_release(self._release)
        except Exception:
            pass


class ReadGroup(NamedTuple):
    """One physics-output column group returned by :meth:`ReadResult.fetch_next`.

    A flattened view of the native ``ovstage_read_group_t`` the read returns (its
    ``prims`` / ``data`` / ``meta`` sub-structs unpacked into these fields). The
    interned identifiers (``attribute`` token and ``prim_list`` handle) resolve
    through the owning attached Stage path dictionary,
    so an ``ovstage.PathDictionary(stage)`` resolves them, and ``prim_list`` feeds
    straight into ``stage.query_from_path_list`` for a no-repack write-back.
    ``object_type`` is NOT part of the native group (the read is opened over one
    type). It is stamped here from the ``object_type`` of the session's :class:`Query`.

    Constant on the physics-output path (so callers can rely on them): ``is_delete``
    is always ``False`` (this read never emits tombstones). ``prim_offset`` is ``0``
    and ``prim_index_map`` is ``None`` (each group carries its own full ``prim_list``).
    ``ordinal`` is ``0`` (groups are not ordinal-stamped here).

    Attributes:
        attribute: Interned EMITTED attribute token (resolve via the path
            dictionary). May differ from the requested name: a "position" request on
            a point-instancer is emitted as "positions" (instancer-local). Write back
            using this token, not the requested string.
        object_type: The queried :class:`SimObjectType` (stamped from the query,
            not carried by the native ovstage group).
        ordinal: The data ordinal of this group (``0`` on this path).
        is_array: Follows the SOURCE attribute kind: ``True`` for a ragged / USD-array
            / byte-string column (e.g. a point-instancer's positions, a deformable
            mesh's points), ``False`` for a fixed scalar column. It is NOT decided by
            whether the per-element dims happen to be uniform: a fixed-width array
            attribute is still ``is_array=True``. It is the only signal that tells e.g.
            a 1-byte string from a scalar uint8 apart.
        is_delete: Always ``False`` for physics output (tombstone flag).
        semantic: Authored USD interpretation (``ovstage_attribute_semantic_t`` value).
        prim_list: Interned prim-path-list handle covering this group's prims.
        prim_offset: Start index within ``prim_list`` (``0`` on this path).
        prim_count: Number of prims in this group.
        prim_index_map: CPU ``warp.array`` of ``uint32`` sparse indices into ``prim_list``,
            or ``None`` for a contiguous range (always ``None`` on this path).
        index_map: CPU ``warp.array`` of ``uint32`` values (gather/scatter over the OUTER element axis),
            or ``None``. ``None`` for point-instancer rigid-body output, which always
            emits the instancer's full instance array (by-index) so it forwards into
            the ovstage write path verbatim. Slots without a live body are zero-filled
            in every array; an all-zero ``orientations`` quaternion is the absent-slot
            marker because live orientations are normalized and never all zero.
        layout_generation: Reserved metadata field. The current output producer
            always returns ``0``. Do not use it for structural invalidation.
        write_floor_ordinal: Reserved metadata field. The current output producer
            always returns ``0``. The application owns ovstage write-floor advancement.
        tensors: One ``warp.array`` per native tensor (1 for a fixed column;
            per-prim for an array group). A tuple width above 1 is the trailing dim
            (a vec3 column is shape ``[N, 3]``); a single-lane column stays ``[N]``.
            Non-empty arrays alias the read session's snapshot storage and keep it
            alive, so CPU and CUDA arrays are both safe to keep after the
            :class:`ReadResult` closes.
        cuda_stream: Producer stream for a device column, or ``0``. The read emits
            ``0``: it does not ask a consumer to drain a stream, which is what a
            non-zero value means to ovstage.
        cuda_wait_event: CUDA event (as an integer handle) that a device column's
            producer work signals, or ``0`` for a host column. **A device column is
            handed over before its work has necessarily completed**. The read does
            not block on the consumer's behalf.

            The Python binding enqueues a wait on this event for the Warp stream
            current during the first fetch-operation result access. It does not block the host, and no other
            stream is ordered, so the caller must establish its own Warp stream
            dependency before using the array elsewhere. The handle stays valid
            until the read session is released. Do not destroy it.
    """

    attribute: int
    object_type: SimObjectType
    ordinal: int
    is_array: bool
    is_delete: bool
    semantic: int
    prim_list: int
    prim_offset: int
    prim_count: int
    prim_index_map: "object"  # warp.array | None
    index_map: "object"  # warp.array | None
    layout_generation: int
    write_floor_ordinal: int
    tensors: "list"  # list[warp.array]
    cuda_stream: int = 0
    cuda_wait_event: int = 0


# dtype.code is typed as DLDataTypeCode (a ctypes.c_uint8 *subclass*), so the field yields a ctypes
# instance, and int() on it would parse its raw byte through the buffer protocol (ValueError on
# b'\x02'). Read .value instead. The plain-int fields normalize through the same path harmlessly.
def _dl_int(v) -> int:
    return int(getattr(v, "value", v))


_CPU_MODE_WARP_BUILD_CHECKED = False

# Carbonite setting (create-time config entry, ``PhysXConfig.carbonite_overrides``):
# False silences the Newton schema registration check in ``attach_ovstage()``.
NEWTON_SCHEMA_WARNING_SETTING = "/ovphysx/schemas/warnMissingNewtonSchema"
_NEWTON_SCHEMA_CHECKED = False


def _carbonite_override_is_false(value) -> bool:
    """Interpret a ``carbonite_overrides`` value the way the native config parser does.

    Overrides travel as strings (``_make_carbonite_entry``), and the native side reads
    ``"false"`` / ``"False"`` / ``"FALSE"`` and ``"0"`` as false, so the Python-side reader of
    this one setting must agree with it rather than apply Python truthiness.
    """
    if isinstance(value, str):
        return value.strip().lower() in ("false", "0")
    return not value


_SCHEMA_PROBE_DIR = None


def _usd_schema_definitions_built(ovstage):
    """Tell whether USD has already built its schema definitions in this process.

    True once a population or export has run (the registry is frozen), False while it
    is still open, None when ovstage could not be asked. ovstage exposes no query for
    this; what it reports is a registration that arrives after the definitions were
    built. So a throwaway descriptor is registered: a uniquely named plugin that
    declares no schema types, which therefore selects no schema for anyone and changes
    nothing whether the registry is open or frozen. A fresh name is used on every call
    because a path registered once is deduplicated afterwards. The descriptors live in
    one process-lifetime directory, removed at exit.
    """
    global _SCHEMA_PROBE_DIR
    import json
    import os
    import shutil
    import tempfile
    import uuid

    try:
        if _SCHEMA_PROBE_DIR is None:
            _SCHEMA_PROBE_DIR = tempfile.mkdtemp(prefix="ovphysx-schema-registry-probe-")
            atexit.register(shutil.rmtree, _SCHEMA_PROBE_DIR, True)
        probe = os.path.join(_SCHEMA_PROBE_DIR, uuid.uuid4().hex)
        os.mkdir(probe)
        with open(os.path.join(probe, "plugInfo.json"), "w", encoding="utf-8") as fh:
            json.dump(
                {
                    "Plugins": [
                        {
                            "Name": "ovphysx_schema_registry_probe_" + os.path.basename(probe),
                            "Type": "resource",
                            "Root": ".",
                            "ResourcePath": ".",
                            "Info": {},
                        }
                    ]
                },
                fh,
            )
    except OSError:
        return None
    try:
        ovstage.population.register_usd_schemas([probe])
    except Exception as exc:
        return True if "already built" in str(exc) else None
    return False


def _warn_legacy_api(message: str, stacklevel: int = 3) -> None:
    warnings.warn(message, DeprecationWarning, stacklevel=stacklevel)


def _warn_if_newton_schema_unregistered() -> None:
    """Warn once per process when the Newton USD schema was not registered with ovstage.

    The parser reads the Newton schema's newton:* attributes as fallbacks for the PhysX
    spellings, and population drops every newton:* attribute the scene authors unless the
    schema was registered before the first USD schema read (population or export).
    ovstage keeps no queryable record
    of registered schema families; what it does report is a registration that arrives
    after USD built its schema definitions. So, once those definitions are built, the
    check re-registers the installed newton-usd-schemas package: a no-op when the family
    was registered in time (from that path or any other complete copy), an error when it
    was not.

    That probe would be a real registration while USD's registry is still open, so it
    runs only after ``_usd_schema_definitions_built`` says the registry is frozen. A stage
    authored procedurally and attached while USD schema definitions remain unbuilt leaves
    the registry open: the check then does nothing to the Newton family, the application
    still chooses its own copy, and the check stays armed for a later attach.

    Runs before the native attach so a warning promoted to an error leaves the instance
    detached. Latches once it has spoken or probed: USD's registry cannot change for the
    rest of the process, and the probe itself registers the plugins, so a second probe
    would report success.
    """
    global _NEWTON_SCHEMA_CHECKED
    if _NEWTON_SCHEMA_CHECKED:
        return

    from .schemas import (
        NEWTON_SCHEMA_INSTALL_HINT,
        NEWTON_SCHEMA_PIP_NAME,
        find_newton_schema_root,
    )

    silence = (
        f" Set {NEWTON_SCHEMA_WARNING_SETTING} to false "
        f"(PhysXConfig(carbonite_overrides={{'{NEWTON_SCHEMA_WARNING_SETTING}': False}})) to silence this check."
    )
    root = find_newton_schema_root()
    if root is None:
        # Not installed is the default state of a consumer whose scenes author no newton:*
        # attribute, and nothing is wrong there. A logged warning keeps the pointer visible
        # without turning attach into an exception for suites that promote warnings to errors.
        _NEWTON_SCHEMA_CHECKED = True
        logging.getLogger("ovphysx").warning(
            "attach_ovstage: the Newton USD schema (%s) is not installed in this Python environment, so "
            "ovphysx cannot verify that it was registered with ovstage. ovphysx reads its newton:* attributes "
            "(for example the NewtonJointAPI velocity limit, armature, friction and damping of a joint), "
            "and population drops every newton:* attribute a scene authors unless the schema was registered "
            "before the first USD schema read (population or export); scenes that author none are unaffected. %s%s",
            NEWTON_SCHEMA_PIP_NAME,
            NEWTON_SCHEMA_INSTALL_HINT,
            silence,
        )
        return

    try:
        import ovstage
    except ImportError:
        return
    if not ovstage.population.available():
        return
    if _usd_schema_definitions_built(ovstage) is not True:
        return  # registry still open (or unknown): registering now would select a schema for the app
    _NEWTON_SCHEMA_CHECKED = True
    try:
        ovstage.population.register_usd_schemas([str(root)])
    except Exception as exc:  # ovstage reports a late registration as OP_FAILED
        warnings.warn(
            f"attach_ovstage: the Newton USD schema at '{root}' was not registered with ovstage before the "
            "first USD schema read (population or export) in this process. Population can therefore drop "
            "newton:* attributes the scene authors (for example newton:velocityLimit), leaving the PhysX "
            "default in effect. Restart the process and register it "
            "together with the external PhysX schemas through ovstage.population.register_usd_schemas() "
            "before populating or exporting (see docs/physics_schemas.md and ovphysx.newton_schema_root()). A registration "
            "ovstage could not observe (another USD consumer read the schema definitions first) is not "
            f"detected.{silence} (ovstage: {exc})",
            RuntimeWarning,
            stacklevel=3,
        )


def _warp_build_has_cuda(wp) -> "bool | None":
    """Whether this Warp build was compiled with CUDA, asked WITHOUT loading the driver.

    `Runtime.__init__` reads the same flag off the native library, but only as part of an
    initialization that goes on to open the driver. Loading `warp.so` on its own does not:
    the symbol is a compile-time constant in that library, so it can be read before Warp has
    a runtime and before `libcuda` is ever mapped. Measured on Warp 1.11, `libcuda` is still
    unmapped after `CDLL(warp.so)` and after the call.

    None when the answer cannot be established, which is treated as "do not warn": a wrong
    warning about the CUDA driver is worse than none.
    """
    import ctypes
    import os
    import sys

    # Warp's own per-platform names (warp/_src/context.py). A hardcoded warp.so would return
    # None on Windows and macOS and leave the check silent there.
    name = "warp.dll" if sys.platform == "win32" else "libwarp.dylib" if sys.platform == "darwin" else "warp.so"
    try:
        lib = ctypes.CDLL(os.path.join(os.path.dirname(wp.__file__), "bin", name))
        fn = getattr(lib, "wp_is_cuda_enabled", None)
        if fn is None:
            return None
        fn.restype = ctypes.c_int
        fn.argtypes = None
        return bool(fn())
    except Exception:
        return None


def _warn_if_warp_build_breaks_cpu_mode(wp) -> None:
    """Warn once when CPU-only mode is on and the installed Warp will load the CUDA driver.

    GlobalConfigKey.CPU_ONLY keeps ovphysx's own code from loading the CUDA driver,
    but materializing a read or write group builds `warp.array` aliases and initializes
    the Warp runtime, which opens the driver on a CUDA-enabled Warp build. A CPU-only Warp keeps
    these frontend conversions driverless. Other dependencies, including ovstage, are
    outside this check and may already have opened the driver.

    A warning and not an exception: this is reached from an already-open read or write, and
    a process that has been silently doing this is not made better off by having array
    conversion start throwing.
    """
    global _CPU_MODE_WARP_BUILD_CHECKED
    if _CPU_MODE_WARP_BUILD_CHECKED:
        return

    try:
        cpu_only = bool(get_global_config(GlobalConfigKey.CPU_ONLY))
    except Exception:
        return  # cannot ask, so stay silent and leave the check armed for the next conversion

    # Latched only once the question was actually answered, so a conversion that could not ask
    # does not consume the process's one warning.
    _CPU_MODE_WARP_BUILD_CHECKED = True
    if not cpu_only:
        return

    if _warp_build_has_cuda(wp) is not True:
        return  # CPU-only build (or undeterminable): Warp array construction is driverless

    warnings.warn(
        "ovphysx is in CPU-only mode, but the installed Warp was built with CUDA. Building the "
        "warp.array for a read or write group initializes the Warp runtime, which opens the CUDA "
        "driver. Install a CPU-only Warp to keep these Python paths driverless: "
        '`conda install -c conda-forge "warp-lang=*=*cpu*"`, or build Warp from source with no CUDA '
        "toolkit configured. "
        "ovphysx's own code touches no CUDA driver either way.",
        RuntimeWarning,
        stacklevel=2,
    )


def _warp_device(dl_device) -> str:
    """Warp device string for a DLTensor device, or TypeError for one Warp cannot hold."""
    device_type = _dl_int(dl_device.device_type)
    if device_type == DLDeviceType.kDLCPU:
        return "cpu"
    if device_type == DLDeviceType.kDLCUDA:
        return f"cuda:{_dl_int(dl_device.device_id)}"
    raise TypeError(
        f"ovphysx column has unsupported DLPack device type {device_type}; " "the Python frontend supports CPU and CUDA"
    )


class _ReadRelease:
    """Refcounted, thread-safe deferred teardown for a read session so borrowed columns can't dangle.

    Read columns are borrowed ``warp.array`` aliases over native buffers owned by the read session,
    so the session is torn down only once the ReadResult is closed **and** every handed-out borrow
    (the array, a Warp slice, or a framework view imported from it) has had its Warp deleter fire.

    A Warp deleter runs on whatever thread garbage-collects the borrow, which may not be the thread
    that owns the ovphysx instance. So:

    * The refcount is mutated under a lock. A bare ``+=`` / ``-=`` is not atomic even under the GIL,
      so two concurrent drops could lose a decrement and leak the session.
    * The native teardown (``ovphysx_release_*``) is **never** called from the deleter's thread. A
      single ovphysx instance is not thread-safe. Hitting zero only enqueues the teardown. The owning
      thread performs it on its next read/step drain (``PhysX._drain_pending_read_releases``).
    """

    def __init__(self, sdk, query: int, read: int):
        self._sdk_ref = weakref.ref(sdk)
        self._query = int(query)
        self._read = int(read)
        self._group_ids: "list[int]" = []
        self._cleanup_operations = []
        self._refs = 1  # the owning ReadResult's ref, plus one per outstanding Warp array
        # Outstanding array borrows, counted apart from _refs: closing the ReadResult and dropping a
        # tensor both decrement _refs, so one live borrow leaves _refs == 1, indistinguishable from
        # an open ReadResult with none.
        self._borrows = 0
        self._freed = False
        self._lock = threading.Lock()

    @property
    def _sdk(self):
        sdk = self._sdk_ref()
        if sdk is None:
            raise ReferenceError("owning PhysX instance no longer exists")
        return sdk

    def add_group(self, gid: int) -> None:
        self._group_ids.append(int(gid))  # only called by owning-thread completion dispatch

    @property
    def has_live_borrows(self) -> bool:
        with self._lock:
            return not self._freed and self._borrows > 0

    def retain(self) -> None:
        with self._lock:
            self._refs += 1
            self._borrows += 1
        # One process-lifecycle reference per borrow, so ovphysx_shutdown(), which reclaims the
        # CUDA context the borrowed buffer lives in, cannot run while an array still points at it.
        # Taken outside self._lock: _acquire_process_lifecycle takes its own, and holding both would
        # fix a lock order that release_borrow (running on an arbitrary GC thread) cannot honour.
        _acquire_process_lifecycle()

    def release_borrow(self, *_ignored) -> None:
        """Drop one array borrow. May run on ANY thread (a Warp deleter on a GC thread).

        BOOKKEEPING ONLY. Nothing here may enter a native lifecycle transition, because the caller
        is whatever thread the collector happened to be on. If this borrow holds the last process
        reference, including during interpreter teardown, releasing it inline runs the whole of
        ``ovphysx_shutdown()`` on that GC thread, during a ``__del__``, possibly while the
        interpreter is finalizing. The reference is queued instead and retired by the next thread
        that reaches a lifecycle entry point, or by the interpreter-exit drain.
        """
        with self._lock:
            if not self._freed:
                self._borrows -= 1
        try:
            # The session reference FIRST, so the read teardown is queued before the process
            # reference is handed back. Releasing the process reference first can complete final
            # shutdown before this line runs, queueing the read holder against an SDK whose native
            # handle is already gone.
            self.release()
        finally:
            # Pairs the acquire in retain(): every borrow must return its process reference or the
            # process can never shut down. A counter increment under a lock cannot raise, so it
            # cannot strand the reference the way an inline release could.
            _defer_process_lifecycle_release()

    def retain_pending(self) -> None:
        """Keep raw fetch output alive without pretending it is an array borrow."""
        with self._lock:
            self._refs += 1

    def release_group(self, gid) -> Task:
        receipt = self._sdk._lib.ovphysx_release_group(
            self._sdk._omni_physx_sdk_handle.value, self._read, gid)
        task = self._sdk._admit_session_task(receipt, "release group")
        self._group_ids.remove(gid)
        return task

    def release_groups(self) -> tuple[Task, ...]:
        """Retire stage-derived metadata on the owning thread while the Stage is attached.

        Numeric snapshots remain borrowed independently. Admission transfers each group's
        cleanup to the SDK; only rejected groups remain available for a later release attempt.
        """
        instance = getattr(getattr(self._sdk, "_omni_physx_sdk_handle", None), "value", None)
        if instance is None or not self._read:
            self._group_ids.clear()
            return ()
        tasks = []
        try:
            _run_cleanups(lambda gid=gid: tasks.append(self.release_group(gid)) for gid in list(self._group_ids))
        except Exception:
            if tasks:
                self._cleanup_operations.append(Operation(weakref.proxy(self._sdk), tasks))
            raise
        return tuple(tasks)

    def release(self, *_ignored) -> None:
        # May run on ANY thread (ReadResult.close on the owner, a Warp deleter on a GC thread).
        # Thread-safe bookkeeping only. The native teardown is deferred to the owning thread.
        with self._lock:
            if self._freed:
                return
            self._refs -= 1
            if self._refs > 0:
                return
            self._freed = True
        sdk = self._sdk_ref()
        if sdk is not None:
            sdk._enqueue_read_release(self)

    @property
    def pending(self) -> bool:
        return bool(self._read or self._query)

    def _free_native(self) -> None:
        # Preserve observable outcomes while this holder is still reachable; the
        # SDK registry itself keeps outputless operations only weakly.
        tasks = []
        def release(field, fn):
            task = _release_handle(self, field, fn)
            if task is not None:
                tasks.append(task)
        try:
            _run_cleanups((
                lambda: tasks.extend(self.release_groups()),
                lambda: release("_read", self._sdk._lib.ovphysx_release_read),
                lambda: release("_query", self._sdk._lib.ovphysx_release_query),
            ))
        finally:
            if tasks:
                self._cleanup_operations.append(Operation(weakref.proxy(self._sdk), tasks))


class ReadResult:
    """A completed read session with explicit asynchronous group fetches.

    ``fetch_next().wait()`` exposes one group, or None at successful EOF. Opening
    a session does not fetch its groups. Numeric Warp aliases keep snapshot
    storage alive after logical close; dictionary metadata does not survive
    group release or attachment invalidation. The caller's Query is independent.
    """

    def __init__(self, sdk, query: int, read: int, groups=None, group_ids=(), release=None,
                 *, metadata=None, object_type=SimObjectType.RIGID_BODY, stage=None, attachment=None):
        self._sdk = sdk
        self._query = int(query)
        self._read = int(read)
        self.groups = [] if groups is None else groups
        self.metadata = metadata
        self._object_type = object_type
        self._stage = stage
        self._attachment = attachment
        self._closed = False
        self._sync = False
        if release is None:
            release = _ReadRelease(sdk, query, read)
            for gid in group_ids:
                release.add_group(gid)
        self._release = release
        sdk._live_read_holders.add(release)

    def _validate(self):
        self._sdk._check_valid()
        if self._closed:
            raise RuntimeError("read session is closed")
        if self._attachment is not None and self._sdk._get_attachment_identity() != self._attachment:
            raise RuntimeError("read session belongs to an invalidated attachment")

    @property
    def dictionary(self):
        """Owner-aware dictionary; never resolves through a retired query."""
        self._validate()
        return _stage_dictionary(self._stage)

    def fetch_next(self) -> Operation[ReadGroup | None]:
        """Submit one fetch; materialize its arrays only on typed result access."""
        self._validate()
        output = POINTER(ovstage_read_group_t)()
        receipt = self._sdk._lib.ovphysx_fetch_read_next(
            self._sdk._omni_physx_sdk_handle.value, self._read, byref(output))
        task = self._sdk._admit_session_task(receipt, "read fetch")
        holder = self._release
        holder.retain_pending()
        pending = [True]
        group_id = [0]
        attachment = self._attachment
        sdk_ref = weakref.ref(self._sdk)

        def finish():
            if pending[0]:
                pending[0] = False
                holder.release()

        def completed():
            # Copy only the stable identity. Warp conversion stays on result access.
            sdk = sdk_ref()
            if sdk is None or (attachment is not None and sdk._get_attachment_identity() != attachment):
                return
            if not holder._read:
                return
            if output:
                group_id[0] = int(output.contents.read_group_id)
                holder.add_group(group_id[0])

        def cleanup():
            if group_id[0] in holder._group_ids:
                holder.release_group(group_id[0])
            finish()

        def convert():
            self._validate()
            if not output:
                finish()
                return None
            if not holder._read or group_id[0] not in holder._group_ids:
                finish()
                raise RuntimeError("read group has been released before materialization")
            group = self._sdk._materialize_read_group(output.contents, self._object_type, holder)
            self.groups.append(group)
            finish()
            return group

        return Operation(self._sdk, (task,), convert=convert, cleanup=cleanup,
                         on_complete=completed, keepalive=(output,))

    def close(self) -> Operation[None]:
        """Logically close and submit group releases, without waiting for aliases.

        The owner reference drops once every group release is admitted. Rejected
        admission retains it for retry. Pending fetches and returned arrays keep
        the physical snapshot alive until their own references are retired.
        """
        if self._closed:
            return Operation(self._sdk)
        tasks = self._release.release_groups()
        self._closed = True
        self._release.release()
        operation = Operation(self._sdk, tasks)
        try:
            self._sdk._drain_pending_read_releases()
        except Exception:
            operation._sdk = weakref.proxy(self._sdk)
            self._release._cleanup_operations.append(operation)
            raise
        return operation

    release = close

    def close_sync(self) -> None:
        _run_cleanups((
            lambda: self.close().wait(),
            self._sdk._drain_pending_read_releases,
            lambda: _run_cleanups(operation.wait for operation in self._release._cleanup_operations),
        ))

    def __enter__(self):
        self._validate()
        return self

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.close_sync() if self._sync else self.close()
        except Exception as cleanup_error:
            if exc is not None:
                _chain_cleanup(exc, cleanup_error)
                raise exc.with_traceback(traceback)
            raise
        finally:
            exc = traceback = None

    def __del__(self):
        try:
            if not self._closed:
                self._closed = True
                self._release.release()
        except Exception:
            pass


from .dlpack import (
    DLDataType,
    DLDataTypeCode,
    DLDevice,
    DLDeviceType,
    DLTensor,
)


def _check_version_match() -> None:
    import logging

    from . import _bindings

    logger = logging.getLogger(__name__)
    native_version = _bindings.get_native_version_string()
    if not native_version:
        # No native version reported. Warn but do not fail.
        logger.warning(
            "ovphysx native library did not report a version string. "
            "Version compatibility check skipped. This may indicate an old or "
            "development build of the native library."
        )
        return

    try:
        python_base = Version(_python_version).base_version
        native_base = Version(native_version).base_version
    except Exception as exc:
        raise RuntimeError(
            "Failed to parse ovphysx version for compatibility check. "
            f"Python: '{_python_version}', native: '{native_version}'."
        ) from exc

    # Compare the release line only. A CI build stamps the pipeline id as a
    # fourth component (REQ-PACKAGING-BUILDNUM-001), and the two sides do not
    # always carry it: the native library is built from CMake, while the Python
    # version comes from _version.py in a wheel but falls back to the bare
    # VERSION file in a source-tree or editable install. Comparing the full
    # version would fail every CI run for a difference that says nothing about
    # API compatibility.
    def _line(version: str) -> tuple[int, ...]:
        return tuple(Version(version).release[:3])

    if _line(python_base) != _line(native_base):
        raise RuntimeError(
            "ovphysx Python package version does not match the native library. "
            f"Python: '{_python_version}' (base {python_base}), "
            f"native: '{native_version}' (base {native_base}). "
            "Reinstall the wheel or set OVPHYSX_LIB to the matching library. "
            "To bypass this check, pass ignore_version_mismatch=True."
        )

    if python_base != native_base:
        # Same release, different build. Not an API mismatch, but it does mean
        # the wheel and the library came from different pipelines.
        logger.warning(
            "ovphysx Python package and native library are the same release but "
            "different builds. Python: '%s', native: '%s'.",
            _python_version,
            native_version,
        )


from collections import namedtuple

_CacheEntry = namedtuple(
    "_CacheEntry",
    [
        "tensor",  # the tensor object (identity check)
        "c_func",  # C function pointer
        "sdk_handle",  # SDK handle integer
        "bind_handle",  # binding handle
        "dl_ptr",  # ctypes pointer to DLTensor
        "dl_tensor",  # DLTensor struct (prevents GC)
        "data_ptr",  # data pointer at cache time (int or None)
        "ptr_getter",  # callable to re-extract data pointer (or None)
    ],
)


def _copy_dl_data_type(dtype: DLDataType | None = None) -> DLDataType:
    """Return a Python-owned copy of a DLPack data type descriptor."""
    if dtype is None:
        return DLDataType(DLDataTypeCode.kDLFloat, 32, 1)
    return DLDataType.from_buffer_copy(dtype)


class TensorBindingSpec(NamedTuple):
    """Python-owned tensor binding metadata returned by :attr:`TensorBinding.spec`.

    .. deprecated:: 0.6.0
        The tensor-binding API is deprecated. Use :meth:`PhysX.read` for reads
        and :meth:`PhysX.write` for writes.
    """

    dtype: DLDataType
    ndim: int
    shape: tuple


def _warn_unclosed_resource(resource_name: str, source: object) -> None:
    warnings.warn(
        f"{resource_name} was garbage-collected without explicit destroy(); "
        "use destroy() or a context manager to release native resources promptly.",
        ResourceWarning,
        source=source,
    )


def _detect_data_ptr(tensor):
    """The cache checks ``tensor is cached_tensor`` (Python object identity),
    but a numpy array can be resized in place (``buf.resize((bigger,),
    refcheck=False)``), which reallocates the underlying memory while
    ``id(buf)`` stays the same.  Without this guard the cache would pass the
    old DLTensor (pointing to freed memory) to the C layer.

    Returns ``(current_ptr, getter_fn)`` where *getter_fn* re-extracts the
    pointer on the fast path, or ``(None, None)`` for providers whose storage
    ownership is not known.
    """
    module_root = type(tensor).__module__.partition(".")[0]
    if module_root == "numpy" and hasattr(tensor, "ctypes"):
        return tensor.ctypes.data, lambda t: t.ctypes.data
    if module_root == "torch" and hasattr(tensor, "data_ptr") and callable(tensor.data_ptr):
        return tensor.data_ptr(), lambda t: t.data_ptr()
    if module_root == "warp" and hasattr(tensor, "ptr"):
        return tensor.ptr, lambda t: t.ptr
    return None, None


def _dltensor_data_ptr(dl_tensor: DLTensor) -> int:
    """Return the effective first-element address of a DLTensor."""
    return int(dl_tensor.data or 0) + int(dl_tensor.byte_offset)


def _make_cache_entry(tensor, c_func, sdk_handle, bind_handle, dl_tensor):
    """Build a cache entry when the tensor has stable, known storage."""
    if isinstance(tensor, DLTensor):
        return _CacheEntry(
            tensor,
            c_func,
            sdk_handle,
            bind_handle,
            ctypes.byref(tensor),
            tensor,
            None,
            None,
        )

    cached_ptr, ptr_getter = _detect_data_ptr(tensor)
    if ptr_getter is None or cached_ptr != _dltensor_data_ptr(dl_tensor):
        return None

    from ._dlpack_utils import copy_dltensor

    cached_dl_tensor = copy_dltensor(dl_tensor)
    return _CacheEntry(
        tensor,
        c_func,
        sdk_handle,
        bind_handle,
        ctypes.byref(cached_dl_tensor),
        cached_dl_tensor,
        cached_ptr,
        ptr_getter,
    )


class TensorBinding:
    """Tensor binding for bulk physics data access via DLPack.

    .. deprecated:: 0.6.0
        The tensor-binding API is deprecated. Use :meth:`PhysX.read` for reads
        and :meth:`PhysX.write` for writes.

    A tensor binding connects a physics-object path pattern to a tensor type,
    enabling efficient bulk read/write for authored USD objects and runtime-only
    clones (poses, velocities, joint positions, etc.).
    The :attr:`shape`, :attr:`ndim`, and :attr:`dtype` metadata come from
    ``ovphysx_get_tensor_binding_spec()``. Use them to allocate compatible
    buffers instead of assuming every tensor type is ``float32``. Use
    :attr:`native_device` to choose the no-staging CPU or CUDA device.

    CPU-only property bindings cover standalone rigid-body mass/inertia/COM
    values, articulation DOF/body properties, rigid-body/articulation shape
    properties, deformable-material properties, and disable-simulation/gravity
    flags. Fixed and spatial tendon property bindings are not CPU-only. CPU-only
    property bindings require host-resident tensor, index, and mask buffers,
    including when the simulation runs on GPU.

    This is a synchronous API - operations complete before returning.
    Bindings are tied to the currently realized physics objects. Reuse them
    across simulation steps, but do not keep them across detach_ovstage(), removing USD
    data that contains bound objects, or replacing/reparsing the stage so bound
    objects are destroyed and recreated. Destroy cached bindings before those
    lifecycle operations when practical. If a stale binding survives, only
    destroy it. Create replacement bindings after the operation completes.

    Usage patterns:

    - Context manager (auto-cleanup)::

        import numpy as np
        from ovphysx import TensorType

        def raise_robot_poses(physx):
            with physx.create_tensor_binding(
                "/World/robot*", tensor_type=TensorType.RIGID_BODY_POSE
            ) as binding:
                poses = np.zeros(binding.shape, dtype=np.dtype(str(binding.dtype)))
                binding.read(poses)
                poses[:, 2] += 0.1  # raise z position
                binding.write(poses)
            # Auto-destroyed here

    - Manual (explicit cleanup)::

        import numpy as np
        from ovphysx import TensorType

        def read_robot_poses(physx):
            binding = physx.create_tensor_binding(
                "/World/robot*", tensor_type=TensorType.RIGID_BODY_POSE
            )
            poses = np.zeros(binding.shape, dtype=np.dtype(str(binding.dtype)))
            binding.read(poses)
            binding.destroy()
            return poses
    """

    def __init__(
        self,
        sdk,
        handle: int,
        tensor_type: int,
        ndim: int,
        shape: tuple,
        dtype: DLDataType | None = None,
        *,
        _from_factory: bool = False,
    ):
        """Initialize tensor binding (created by PhysX.create_tensor_binding)."""
        # Warn on construction so direct use of this documented constructor is diagnosed too, not
        # only the factory. create_tensor_binding passes _from_factory=True because it already emits
        # its own caller-precise warning. Skipping the duplicate here needs no process-global warning
        # filter state, which would not be thread-safe.
        if not _from_factory:
            warnings.warn(
                "ovphysx tensor bindings are deprecated and will be removed in a future "
                "release; use PhysX.read for reads and PhysX.write for writes.",
                DeprecationWarning,
                stacklevel=2,
            )
        self._sdk = sdk
        self._handle = handle
        self._tensor_type = tensor_type
        self._ndim = ndim
        self._shape = shape
        self._dtype = _copy_dl_data_type(dtype)
        self._destroyed = False
        self._lock = threading.Lock()
        self._artic_metadata = None
        self._read_cache = None
        self._write_cache = None

    def __repr__(self) -> str:
        state = "destroyed" if self._destroyed else "alive"
        return (
            f"TensorBinding(handle={self._handle}, tensor_type={self._tensor_type}, "
            f"shape={self._shape}, dtype={self.dtype_name}, state={state})"
        )

    @property
    def handle(self) -> int:
        """Get the binding handle."""
        return self._handle

    @property
    def tensor_type(self) -> int:
        """Get the tensor type enum value."""
        return self._tensor_type

    @property
    def ndim(self) -> int:
        """Get the number of dimensions reported by ``ovphysx_get_tensor_binding_spec()``."""
        return self._ndim

    @property
    def shape(self) -> tuple:
        """Get tensor shape as tuple reported by ``ovphysx_get_tensor_binding_spec()``.

        Returns:
            Tensor dimensions for this binding. Scalar-property bindings use
            ``(N,)``. Flat state tensors use ``(N, C)``. Articulation and
            deformable mesh tensors use ``(N, L, C)``.
        """
        return self._shape

    @property
    def dtype(self) -> DLDataType:
        """Get the required DLPack dtype for tensors passed to this binding.

        Most bindings use ``float32``. Index bindings (deformable element indices)
        use ``int32``. The bool bindings (DISABLE_SIMULATION, DISABLE_GRAVITY) use
        ``uint8``, as does ARTICULATION_DOF_DRIVE_TYPE, which is a per-DOF enum
        byte rather than a flag.
        See :attr:`dtype_name` for a compact string form.
        """
        return _copy_dl_data_type(self._dtype)

    @property
    def dtype_name(self) -> str:
        """Get the required tensor dtype as a short string such as ``float32``, ``int32``, or ``uint8``."""
        return str(self._dtype)

    @property
    def native_device(self) -> DLDevice:
        """Get the binding's native DLPack device.

        .. deprecated:: 0.6.0
            The tensor-binding API is deprecated. Use :meth:`PhysX.read` for reads
            and :meth:`PhysX.write` for writes.

        CPU-only property tensors report ``DLDevice(kDLCPU, 0)`` even when
        the scene uses GPU dynamics. Other bindings follow their native
        TensorAPI view and report ``DLDevice(kDLCUDA, ordinal)`` in DirectGPU
        mode or ``DLDevice(kDLCPU, 0)`` otherwise. This property identifies the
        no-staging device and does not change existing read/write behavior. A CUDA
        ``device_id`` is the process-visible runtime ordinal used by a framework device such as
        ``cuda:N``, not a physical PCI bus index. Compare ``device_type.value``
        with a constant such as ``DLDeviceType.kDLCUDA``. This is a live query,
        not cached metadata: it may wait for pending operations and rejects an
        invalidated simulation view.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")
            self._check_sdk_valid()
            native_device = DLDevice()
            result = self._sdk._lib.ovphysx_get_tensor_binding_native_device(
                self._sdk._omni_physx_sdk_handle.value, self._handle, byref(native_device)
            )
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to get tensor binding native device: {error_msg}")
            return native_device

    @property
    def spec(self) -> TensorBindingSpec:
        """Get a Python-owned tensor spec snapshot for this binding."""
        return TensorBindingSpec(dtype=self.dtype, ndim=self._ndim, shape=self._shape)

    @property
    def count(self) -> int:
        """Get number of entities (first dimension of shape)."""
        return self._shape[0] if self._shape else 0

    def _check_sdk_valid(self) -> None:
        """Ensure the parent PhysX instance is still alive."""
        if self._sdk._omni_physx_sdk_handle is None:
            raise RuntimeError("Cannot use TensorBinding: parent PhysX instance has been destroyed.")

    @property
    def prim_paths(self) -> list[str]:
        """Resolved physics-object paths in tensor row order.

        Rigid-body bindings return one path per rigid-body tensor row.
        Articulation bindings return one root object path per articulation row.
        For per-articulation link names, use :attr:`body_names`.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")
            self._check_sdk_valid()
            count = self.count
            if count == 0:
                return []
            paths_arr = (ovphysx_string_t * count)()
            out_count = c_uint32(0)
            result = _lib.ovphysx_tensor_binding_get_prim_paths(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                paths_arr,
                count,
                ctypes.byref(out_count),
            )
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to get tensor binding prim paths: {error_msg}")
            if out_count.value > count:
                # The native layer wrote past the buffer sized to `count`. This is
                # a bug in the C library, not an expected truncation. The min()
                # below avoids indexing past the buffer.
                import logging as _logging

                _logging.getLogger(__name__).warning(
                    "ovphysx_tensor_binding_get_prim_paths: native out_count=%d > buffer size=%d; truncating",
                    out_count.value,
                    count,
                )
            return [str(paths_arr[i]) for i in range(min(out_count.value, count))]

    def _get_artic_metadata(self) -> ovphysx_articulation_metadata_t:
        """Return cached articulation metadata struct (one C call total per binding)."""
        if self._tensor_type not in _ARTICULATION_TENSOR_TYPES:
            try:
                name = TensorType(self._tensor_type).name
            except ValueError:
                name = str(self._tensor_type)
            raise TypeError(
                f"Articulation metadata (dof_count, body_names, etc.) is not available "
                f"for tensor type {name!r}. Only ARTICULATION_* tensor types carry this "
                f"metadata. Use an articulation tensor type such as "
                f"TensorType.ARTICULATION_DOF_POSITION."
            )
        if self._artic_metadata is None:
            self._check_sdk_valid()
            meta = ovphysx_articulation_metadata_t()
            result = _lib.ovphysx_get_articulation_metadata(
                self._sdk._omni_physx_sdk_handle.value, self._handle, ctypes.byref(meta)
            )
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to get articulation metadata: {error_msg}")
            self._artic_metadata = meta
        return self._artic_metadata

    @property
    def dof_count(self) -> int:
        """Number of degrees of freedom (DOFs). 0 if not an articulation binding."""
        return self._get_artic_metadata().dof_count

    @property
    def body_count(self) -> int:
        """Number of links."""
        return self._get_artic_metadata().body_count

    @property
    def is_fixed_base(self) -> bool:
        """Whether the articulation has a fixed base."""
        return bool(self._get_artic_metadata().is_fixed_base)

    def _get_names(self, c_func, count_prop: str) -> list[str]:
        """Helper to fetch name lists from metadata queries."""
        self._check_sdk_valid()
        count = getattr(self, count_prop)
        if count == 0:
            return []
        names_arr = (ovphysx_string_t * count)()
        out_count = c_uint32(0)
        result = c_func(self._sdk._omni_physx_sdk_handle.value, self._handle, names_arr, count, ctypes.byref(out_count))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._sdk._get_last_error()
            raise RuntimeError(f"Failed to get names: {error_msg}")
        return [str(names_arr[i]) for i in range(out_count.value)]

    @property
    def dof_names(self) -> list[str]:
        """List of DOF names (one per DOF)."""
        return self._get_names(_lib.ovphysx_articulation_get_dof_names, "dof_count")

    @property
    def body_names(self) -> list[str]:
        """List of body/link names."""
        return self._get_names(_lib.ovphysx_articulation_get_body_names, "body_count")

    @property
    def joint_count(self) -> int:
        """Number of joints per articulation."""
        return self._get_artic_metadata().joint_count

    @property
    def joint_names(self) -> list[str]:
        """List of joint names."""
        return self._get_names(_lib.ovphysx_articulation_get_joint_names, "joint_count")

    @property
    def fixed_tendon_count(self) -> int:
        """Number of fixed tendons per articulation (0 if none).

        Use to decide whether to allocate buffers for fixed tendon property
        tensors (types 80-85) and to skip tendon code paths when T=0.
        """
        return self._get_artic_metadata().fixed_tendon_count

    @property
    def spatial_tendon_count(self) -> int:
        """Number of spatial tendons per articulation (0 if none).

        Use to decide whether to allocate buffers for spatial tendon property
        tensors (types 90-93) and to skip tendon code paths when T=0.
        """
        return self._get_artic_metadata().spatial_tendon_count

    def read(self, tensor) -> None:
        """Read simulation data into a user-provided tensor (synchronous).

        The tensor must have matching shape and dtype (:attr:`shape` and
        :attr:`dtype`). Can be a NumPy array, PyTorch tensor, or any object
        with __dlpack__ protocol.

        When called repeatedly with the same NumPy, PyTorch, Warp, or direct
        ``DLTensor`` buffer object, an internal cache reuses descriptor metadata
        and attribute lookups. CUDA DLPack producers still export a capsule on
        each call to establish stream readiness. The numpy
        writeable guard is preserved on the fast path. Other DLPack providers
        are reacquired on every call. Callers that want the fast path should
        reuse the same tensor object with unchanged backing storage across calls.
        Do not resize or rebind storage between cached calls. The staleness
        guard rebuilds the cache when it detects a pointer change, but storage
        mutations that reuse the same pointer violate the cache contract.
        Direct ``DLTensor`` inputs retain their caller-owned descriptor.

        Args:
            tensor: DLPack-compatible tensor with pre-allocated storage matching self.shape.
                   Must use ``self.dtype``. When CUDA is available, CPU/CUDA device
                   mismatches are staged for binding types whose storage follows the
                   simulation device. CPU-only property bindings require a host-resident
                   tensor (``kDLCPU`` or ``kDLCUDAHost``), including on GPU simulations;
                   CUDA and CUDA-managed tensors are rejected rather than staged. Cross-GPU
                   mismatches and CUDA tensors in process-wide CPU-only mode are rejected.

        Preconditions:
            - This binding is not destroyed.
            - tensor has matching shape and dtype and uses a supported device.
            - Raw ``DLTensor`` buffers must be ready for native access; callers must
              finish pending CUDA work on them before the call. Objects implementing
              the CUDA DLPack stream protocol are synchronized through Warp.
        Side effects:
            - Blocks until data is available and writes into the provided tensor.
        Ownership/Lifetime:
            - Caller owns tensor storage and must keep it alive for the duration of the call.
            - Do not mutate the tensor's backing storage (``resize()``, ``set_()``,
              etc.) between cached calls. A staleness guard detects pointer changes
              for NumPy, PyTorch, and Warp inputs and falls back to the slow path.
              Direct ``DLTensor`` inputs retain their caller-owned descriptor.
        Threading:
            - Serialized per binding via an internal lock.
        Errors:
            - RuntimeError if read fails (shape mismatch, device mismatch, etc.).
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")

            # Fast path: same tensor object as last successful read.
            cached = self._read_cache
            if cached is not None and cached.tensor is tensor:
                if cached.ptr_getter is not None and cached.ptr_getter(tensor) != cached.data_ptr:
                    self._read_cache = None
                    # Backing storage changed (e.g. numpy resize). Fall through to the slow path.
                else:
                    if self._sdk._omni_physx_sdk_handle is None:
                        raise RuntimeError("Cannot use TensorBinding: parent PhysX instance has been destroyed.")
                    if hasattr(tensor, "flags") and not getattr(tensor.flags, "writeable", True):
                        raise ValueError(
                            "Array passed to binding.read() must be writeable. "
                            "Use np.array(buf) to create a writeable copy."
                        )
                    from ._dlpack_utils import synchronize_tensor_producer

                    keepalive = synchronize_tensor_producer(tensor)
                    try:
                        result = cached.c_func(cached.sdk_handle, cached.bind_handle, cached.dl_ptr)
                    finally:
                        del keepalive
                    if result.status != ApiStatus.SUCCESS:
                        self._read_cache = None
                        error_msg = self._sdk._get_last_error()
                        raise RuntimeError(f"Failed to read tensor binding: {error_msg}")
                    return

            # Slow path: first call or different tensor.
            self._check_sdk_valid()

            if hasattr(tensor, "flags"):
                try:
                    writeable = tensor.flags["WRITEABLE"]
                except (KeyError, TypeError, AttributeError):
                    writeable = True
                if not writeable:
                    raise ValueError(
                        "Array passed to binding.read() must be writeable. "
                        "Use np.array(buf) to create a writeable copy."
                    )

            keepalive = None
            cache_entry = None
            try:
                dl_tensor, keepalive = self._acquire_dltensor(tensor)
                dl_ptr = ctypes.byref(dl_tensor)
                c_func = self._sdk._lib.ovphysx_read_tensor_binding
                sdk_handle_val = self._sdk._omni_physx_sdk_handle.value

                result = c_func(sdk_handle_val, self._handle, dl_ptr)
                if result.status == ApiStatus.SUCCESS:
                    cache_entry = _make_cache_entry(tensor, c_func, sdk_handle_val, self._handle, dl_tensor)
            finally:
                del keepalive

            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to read tensor binding: {error_msg}")

            self._read_cache = cache_entry

    def write(self, tensor, indices=None, mask=None) -> None:
        """Write data from a user-provided tensor into the simulation (synchronous).

        The tensor must have matching shape and dtype (:attr:`shape` and
        :attr:`dtype`). Can be a NumPy array, PyTorch tensor, or any object
        with __dlpack__ protocol.

        When called repeatedly with the same supported buffer object and no
        indices/mask, an internal cache reuses descriptor metadata and attribute
        lookups. Producer readiness is checked on every call. Callers that want this
        fast path should reuse the same tensor object with unchanged backing
        storage across calls. See :meth:`read` for the supported providers and
        full contract.

        Args:
            tensor: DLPack-compatible tensor with data to write, shape matching self.shape.
                   Must use ``self.dtype``. When CUDA is available, CPU/CUDA device
                   mismatches are staged for binding types whose storage follows the
                   simulation device. For CPU-only property bindings, this tensor and optional
                   ``indices`` or ``mask`` must be host-resident (``kDLCPU`` or
                   ``kDLCUDAHost``), including on GPU simulations; CUDA and CUDA-managed
                   tensors are rejected rather than staged. Cross-GPU mismatches and CUDA
                   tensors in process-wide CPU-only mode are rejected.
            indices: Optional int32 tensor of indices for partial update. If provided,
                    only the rows at the given indices are written. The tensor argument
                    must still be full shape [N, ...] matching the binding spec; only the
                    selected rows are applied. Shape of indices: [K] where K <= N.
            mask: Optional bool/uint8 tensor for masked update. If provided, only
                 elements where mask[i] != 0 are written. Shape: [N] matching the
                 binding's first dimension. When mask is provided, tensor must be
                 full shape [N, ...]. If both mask and indices are provided, mask
                 takes precedence and indices are ignored (with a warning).

                 Note: there is no corresponding ``read(..., mask=...)``. Reads always
                 return the full [N,...] tensor and callers can index the result themselves.
                 This write-only mask design matches other RL physics APIs such as Newton's
                 selectionAPI, where masks selectively apply actions but observations are
                 always returned in full.

        Preconditions:
            - This binding is not destroyed.
            - tensor matches shape and dtype and uses a supported device.
            - indices (if provided) is int32 and within bounds.
            - mask (if provided) is bool/uint8 with shape [N] on a supported device.
        Side effects:
            - Updates simulation state for the bound entities.
        Ownership/Lifetime:
            - Caller owns tensor/indices/mask storage and must keep it alive for the call.
        Threading:
            - Serialized per binding via an internal lock.
        Errors:
            - RuntimeError if write fails (shape mismatch, device mismatch, etc.).
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")

            # Fast path: same tensor, no indices, no mask.
            if indices is None and mask is None:
                cached = self._write_cache
                if cached is not None and cached.tensor is tensor:
                    if cached.ptr_getter is not None and cached.ptr_getter(tensor) != cached.data_ptr:
                        self._write_cache = None
                        # Backing storage changed. Fall through to the slow path.
                    else:
                        if self._sdk._omni_physx_sdk_handle is None:
                            raise RuntimeError("Cannot use TensorBinding: parent PhysX instance has been destroyed.")
                        from ._dlpack_utils import synchronize_tensor_producer

                        keepalive = synchronize_tensor_producer(tensor)
                        try:
                            result = cached.c_func(cached.sdk_handle, cached.bind_handle, cached.dl_ptr, None)
                        finally:
                            del keepalive
                        if result.status != ApiStatus.SUCCESS:
                            self._write_cache = None
                            error_msg = self._sdk._get_last_error()
                            raise RuntimeError(f"Failed to write tensor binding: {error_msg}")
                        return

            # Slow path: first call, different tensor, or indices/mask provided.
            self._check_sdk_valid()

            if mask is not None and indices is not None:
                import warnings

                warnings.warn(
                    "Both mask and indices provided to TensorBinding.write(); mask takes precedence, indices ignored.",
                    UserWarning,
                    stacklevel=2,
                )
                indices = None

            if mask is not None:
                keepalive = None
                mask_keepalive = None
                try:
                    dl_tensor, keepalive = self._acquire_dltensor(tensor)
                    dl_mask, mask_keepalive = self._acquire_dltensor(mask)

                    result = self._sdk._lib.ovphysx_write_tensor_binding_masked(
                        self._sdk._omni_physx_sdk_handle.value,
                        self._handle,
                        ctypes.byref(dl_tensor),
                        ctypes.byref(dl_mask),
                    )

                    if result.status != ApiStatus.SUCCESS:
                        error_msg = self._sdk._get_last_error()
                        raise RuntimeError(f"Failed to write tensor binding (masked): {error_msg}")
                    return
                finally:
                    del keepalive
                    del mask_keepalive

            keepalive = None
            idx_keepalive = None
            cache_entry = None
            try:
                dl_tensor, keepalive = self._acquire_dltensor(tensor)
                dl_ptr = ctypes.byref(dl_tensor)
                c_func = self._sdk._lib.ovphysx_write_tensor_binding
                sdk_handle_val = self._sdk._omni_physx_sdk_handle.value

                if indices is not None:
                    idx_dl_tensor, idx_keepalive = self._acquire_dltensor(indices)
                    idx_ptr = ctypes.byref(idx_dl_tensor)
                else:
                    idx_dl_tensor = None
                    idx_ptr = None

                result = c_func(sdk_handle_val, self._handle, dl_ptr, idx_ptr)
                if result.status == ApiStatus.SUCCESS and indices is None:
                    cache_entry = _make_cache_entry(tensor, c_func, sdk_handle_val, self._handle, dl_tensor)
            finally:
                del keepalive
                del idx_keepalive

            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to write tensor binding: {error_msg}")

            if indices is None:
                self._write_cache = cache_entry

    def _acquire_dltensor(self, obj) -> tuple[DLTensor, object | None]:
        """Extract DLTensor and a keepalive reference (if needed)."""
        from ._dlpack_utils import acquire_dltensor

        return acquire_dltensor(obj)

    def wake_up(self, indices=None) -> None:
        """Wake rigid bodies in this binding.

        Mirrors PhysX SDK ``PxRigidDynamic::wakeUp``. Bodies that still have
        ``RIGID_BODY_DISABLE_SIMULATION`` set are silently skipped (the
        engine refuses to wake disabled actors).

        Typical pair: clear the disable flag on an actor (re-add it to
        simulation in a sleep state) and then call this so the actor is
        active for the next ``step()``.

        Only valid on a rigid-body binding. Articulation bindings raise.

        Args:
            indices: Optional int32 DLPack-compatible tensor of indices into
                this binding. If None, every body in the binding is woken.

        Errors:
            RuntimeError if the binding is destroyed, is not a rigid-body
            binding, has been invalidated by a stage change, or the engine
            wake call fails.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")
            self._check_sdk_valid()

            idx_ptr = None
            idx_keepalive = None
            if indices is not None:
                idx_dl, idx_keepalive = self._acquire_dltensor(indices)
                idx_ptr = ctypes.byref(idx_dl)

            result = self._sdk._lib.ovphysx_rigid_body_view_wake_up(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                idx_ptr,
            )
            # Keep the indices buffer alive across the native call.
            _ = idx_keepalive
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"wake_up failed: {error_msg}")

    def sleep(self, indices=None) -> None:
        """Force rigid bodies in this binding to sleep.

        Mirrors PhysX SDK ``PxRigidDynamic::putToSleep``. Symmetric
        counterpart to :meth:`wake_up`. Bodies that have
        ``RIGID_BODY_DISABLE_SIMULATION`` set are silently skipped.

        Only valid on a rigid-body binding. Articulation bindings raise.

        Args:
            indices: Optional int32 DLPack-compatible tensor of indices into
                this binding. If None, every body in the binding is put to
                sleep.

        Errors:
            RuntimeError if the binding is destroyed, is not a rigid-body
            binding, has been invalidated by a stage change, or the engine
            call fails.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("TensorBinding has been destroyed")
            self._check_sdk_valid()

            idx_ptr = None
            idx_keepalive = None
            if indices is not None:
                idx_dl, idx_keepalive = self._acquire_dltensor(indices)
                idx_ptr = ctypes.byref(idx_dl)

            result = self._sdk._lib.ovphysx_rigid_body_view_sleep(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                idx_ptr,
            )
            _ = idx_keepalive
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"sleep failed: {error_msg}")

    def destroy(self) -> None:
        """Release binding resources.

        Safe to call multiple times. Called automatically on garbage collection
        or when exiting a context manager.

        Preconditions:
            - Binding must not be in use by other threads.
        Side effects:
            - Releases native resources and invalidates the binding.
        Ownership/Lifetime:
            - After destruction, the binding cannot be used.
        Threading:
            - Serialized per binding via an internal lock.
        Errors:
            - RuntimeError if destruction fails.
        """
        with self._lock:
            if self._destroyed:
                return
            self._check_sdk_valid()

            result = self._sdk._lib.ovphysx_destroy_tensor_binding(self._sdk._omni_physx_sdk_handle.value, self._handle)
            # Mark destroyed after the native destroy attempt so __del__ does not retry after SDK destruction.
            self._destroyed = True
            self._read_cache = None
            self._write_cache = None
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._sdk._get_last_error()
                raise RuntimeError(f"Failed to destroy tensor binding: {error_msg}")

    def __enter__(self):
        """Enter context manager.

        Preconditions:
            - Binding is valid and not destroyed.
        Side effects:
            - None.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Exit context manager - destroys binding.

        Side effects:
            - Releases binding resources.
        """
        self.destroy()
        return False

    def __del__(self):
        """Destructor - ensures cleanup on garbage collection.

        The process atexit callback normally handles still-live instances
        before interpreter finalization. If this destructor runs later, calling
        C functions may fail, so it checks ``sys.is_finalizing()`` and returns.
        """
        try:
            import sys

            if sys.is_finalizing():
                return
        except Exception:
            return
        if not self._destroyed:
            if getattr(getattr(self, "_sdk", None), "_omni_physx_sdk_handle", None) is not None:
                try:
                    _warn_unclosed_resource("TensorBinding", self)
                except Exception:
                    pass
            try:
                self.destroy()
            except Exception:
                pass


class SdfView:
    """SDF shape view for evaluating signed distance fields at query points.

    Created by PhysX.create_sdf_view(). Use as a context manager or call
    destroy() explicitly when done.

    Evaluation is GPU-only: the query and output tensors must live on the
    same CUDA device as the instance (e.g. Warp arrays).

    SdfView handles are tied to the attached USD stage. After
    :meth:`PhysX.detach_ovstage`, existing views
    are invalid. Destroy them and create replacements after re-attaching a stage.

    Example::

        import warp as wp

        with physx.create_sdf_view(pattern="/World/Mesh*", max_query_points=100) as sdf:
            pts = wp.zeros((sdf.count, 100, 3), dtype=wp.float32, device="cuda")
            out = wp.zeros((sdf.count, 100, 4), dtype=wp.float32, device="cuda")
            sdf.evaluate(pts, out)
    """

    def __init__(self, sdk, handle: int, count: int, max_query_points: int):
        """Initialize SDF view (created by PhysX.create_sdf_view)."""
        self._sdk = sdk
        self._handle = handle
        self._count = count
        self._max_query_points = max_query_points
        self._destroyed = False
        self._lock = threading.Lock()

    def __repr__(self) -> str:
        state = "destroyed" if self._destroyed else "alive"
        return f"SdfView(handle={self._handle}, count={self._count}, max_query_points={self._max_query_points}, state={state})"

    @property
    def count(self) -> int:
        """Number of SDF shapes in this view (first dimension of query tensors)."""
        return self._count

    @property
    def max_query_points(self) -> int:
        """Maximum number of query points per shape this view was created with."""
        return self._max_query_points

    def evaluate(self, query_points, out_distances_and_gradients) -> None:
        """Evaluate SDF at query points and write distances + gradients.

        Args:
            query_points: DLPack-compatible tensor with shape [N, Q, 3], float32.
                N must equal count and Q must equal max_query_points.
            out_distances_and_gradients: Pre-allocated DLPack-compatible tensor
                with shape [N, Q, 4], float32. Component layout per point:
                (grad.x, grad.y, grad.z, distance).

        Preconditions:
            - This view is not destroyed.
            - Tensors are float32 on the GPU (kDLCUDA). Requires a GPU instance.
        """
        from ._dlpack_utils import acquire_dltensor

        with self._lock:
            if self._destroyed:
                raise RuntimeError("SdfView has been destroyed")

            qp_dl, qp_keepalive = acquire_dltensor(query_points)
            out_dl, out_keepalive = acquire_dltensor(out_distances_and_gradients)
            result = _lib.ovphysx_evaluate_sdf(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(qp_dl),
                ctypes.byref(out_dl),
            )
            del qp_keepalive, out_keepalive
            if result.status != 0:
                raise RuntimeError(f"ovphysx_evaluate_sdf failed: {self._sdk._get_last_error()}")

    def destroy(self) -> None:
        """Release the SDF view and its resources.

        Safe to call multiple times, including on stale handles after
        detach_ovstage() cleanup removed the native view.
        """
        with self._lock:
            if self._destroyed:
                return
            # Guard against the SDK being torn down before an explicit destroy()
            # (GC may collect self._sdk's handle first). Mirrors ContactBinding.destroy.
            sdk_handle = self._sdk._omni_physx_sdk_handle
            if sdk_handle is None:
                self._destroyed = True
                return
            result = _lib.ovphysx_destroy_sdf_view(sdk_handle.value, self._handle)
            self._destroyed = True
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"ovphysx_destroy_sdf_view failed: {self._sdk._get_last_error()}")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.destroy()

    def __del__(self):
        try:
            import sys

            if sys.is_finalizing():
                return
        except Exception:
            return
        if not self._destroyed:
            if getattr(getattr(self, "_sdk", None), "_omni_physx_sdk_handle", None) is not None:
                try:
                    _warn_unclosed_resource("SdfView", self)
                except Exception:
                    pass
            try:
                self.destroy()
            except Exception:
                pass


def _decode_truncated_paths(paths_arr, out_count: int, capacity: int, api_name: str) -> list[str]:
    """Decode a native path array, warning if the native call reports more than fits.

    ``out_count`` may legitimately exceed ``capacity`` (OVPHYSX_API_BUFFER_TOO_SMALL);
    only the in-range prefix was written, so indexing stops at ``capacity``.
    """
    written = min(out_count, capacity)
    if out_count > capacity:
        logging.getLogger(__name__).warning(
            "%s: native out_count=%d > buffer size=%d; truncating",
            api_name,
            out_count,
            capacity,
        )
    return [str(paths_arr[i]) for i in range(written)]


class ContactBinding:
    """Contact tensor binding backed by IRigidContactView.

    .. deprecated:: 0.6.0
        The contact-binding API is deprecated and retained for compatibility.

    Do not instantiate directly. Use :meth:`PhysX.create_contact_binding` to
    obtain an instance. The :attr:`sensor_paths` and :attr:`filter_paths`
    properties expose the row/column metadata for the returned tensors.
    """

    def __init__(self, sdk, handle: int, sensor_count: int, filter_count: int, max_contact_data_count: int):
        self._sdk = sdk
        self._handle = handle
        self._sensor_count = sensor_count
        self._filter_count = filter_count
        self._max_contact_data_count = max_contact_data_count
        self._destroyed = False
        self._lock = threading.Lock()

    def _check_sdk_valid(self) -> None:
        if self._sdk._omni_physx_sdk_handle is None:
            raise RuntimeError("Cannot use ContactBinding: parent PhysX instance has been destroyed.")

    @property
    def sensor_count(self) -> int:
        """Number of sensor bodies matched."""
        return self._sensor_count

    @property
    def filter_count(self) -> int:
        """Number of filter bodies per sensor (0 when no filters specified)."""
        return self._filter_count

    @property
    def max_contact_data_count(self) -> int:
        """Flat-buffer capacity for detailed contact and friction reads."""
        return self._max_contact_data_count

    @property
    def sensor_paths(self) -> list[str]:
        """Resolved sensor physics-object paths in contact tensor row order.

        Returns:
            list[str]: One path per sensor row, in the same order as the
            contact data tensors.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            if self._sensor_count <= 0:
                return []
            paths_arr = (ovphysx_string_t * self._sensor_count)()
            out_count = c_uint32(0)
            result = _lib.ovphysx_contact_binding_get_sensor_paths(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                paths_arr,
                self._sensor_count,
                ctypes.byref(out_count),
            )
            if result.status not in (ApiStatus.SUCCESS, ApiStatus.BUFFER_TOO_SMALL):
                raise RuntimeError(f"Failed to get contact sensor paths: {self._sdk._get_last_error()}")
            return _decode_truncated_paths(
                paths_arr, out_count.value, self._sensor_count, "ovphysx_contact_binding_get_sensor_paths"
            )

    @property
    def filter_paths(self) -> list[list[str]]:
        """Resolved filter physics-object paths in contact tensor column order.

        The outer list is indexed by sensor row and the inner list by filter
        column. Each inner list is empty for unfiltered contact bindings
        (i.e. when ``filter_count == 0``).

        Returns:
            list[list[str]]: Nested list of shape [sensor_count][filter_count].
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            total = self._sensor_count * self._filter_count
            if total <= 0:
                return [[] for _ in range(max(self._sensor_count, 0))]
            paths_arr = (ovphysx_string_t * total)()
            out_count = c_uint32(0)
            result = _lib.ovphysx_contact_binding_get_filter_paths(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                paths_arr,
                total,
                ctypes.byref(out_count),
            )
            if result.status not in (ApiStatus.SUCCESS, ApiStatus.BUFFER_TOO_SMALL):
                raise RuntimeError(f"Failed to get contact filter paths: {self._sdk._get_last_error()}")
            if result.status == ApiStatus.SUCCESS and out_count.value != total:
                raise RuntimeError(f"Expected {total} contact filter paths, got {out_count.value}")
            flat = _decode_truncated_paths(
                paths_arr, out_count.value, total, "ovphysx_contact_binding_get_filter_paths"
            )
            return [
                flat[sensor_idx * self._filter_count : (sensor_idx + 1) * self._filter_count]
                for sensor_idx in range(self._sensor_count)
            ]

    def read_net_normal_forces(self, output) -> None:
        """Read net normal forces into output, shape ``[sensor_count, 3]``.

        .. deprecated:: 0.6.0
            The contact-binding API is deprecated and retained for compatibility.

        Sums world-space normal forces over all reported contacts of each sensor,
        regardless of filters or detailed-contact capacity. Add
        :meth:`read_net_friction_forces` from the same step for total contact force.
        Impulse-to-force conversion uses the dt of the last step that actually
        simulated; a zero-dt step is skipped and changes neither the contacts
        nor the dt.
        """
        self._read_forces(output, _lib.ovphysx_read_contact_net_normal_forces, "net contact forces")

    def read_net_friction_forces(self, output) -> None:
        """Read net friction forces into output, shape ``[sensor_count, 3]``.

        .. deprecated:: 0.6.0
            The contact-binding API is deprecated and retained for compatibility.

        Sums world-space friction-anchor forces over all reported contacts of each
        sensor, including contacts with bodies not matched by the configured
        filters. Filters and detailed-contact capacity are not required.
        Adding the result to :meth:`read_net_normal_forces` from the same step
        yields the total contact force. Impulse-to-force conversion uses that step's dt.

        For example, if a foot touches the floor and a wall but its filter selects
        only the floor, this function includes friction from both bodies.
        The friction force matrix includes only the floor contribution.

        Opposing anchor forces may cancel while producing torque.
        This vector sum does not report friction torque.
        """
        self._read_forces(output, _lib.ovphysx_read_contact_net_friction_forces, "net friction forces")

    def read_normal_force_matrix(self, output) -> None:
        """Read normal forces into output, shape ``[sensor_count, filter_count, 3]``.

        .. deprecated:: 0.6.0
            The contact-binding API is deprecated and retained for compatibility.

        Each entry sums world-space normal forces for its sensor/filter pair,
        independently of detailed-contact capacity. Adding
        :meth:`read_friction_force_matrix` from the same step yields total contact
        forces for the configured pairs. Impulse-to-force conversion uses the
        dt of the last step that actually simulated; a zero-dt step is skipped
        and changes neither the contacts nor the dt.
        """
        self._read_forces(output, _lib.ovphysx_read_contact_normal_force_matrix, "contact force matrix")

    def read_friction_force_matrix(self, output) -> None:
        """Read friction forces into output, shape ``[sensor_count, filter_count, 3]``.

        .. deprecated:: 0.6.0
            The contact-binding API is deprecated and retained for compatibility.

        Each entry sums world-space friction forces for its sensor/filter pair,
        independently of detailed-contact capacity. Only contacts with configured
        filter bodies contribute. Adding :meth:`read_normal_force_matrix` from
        the same step yields total contact forces for those pairs. Individual
        friction forces and anchor points are available through
        :meth:`read_friction_contact_data`.

        Impulse-to-force conversion uses the dt of the last step that actually
        simulated; a zero-dt step is skipped and changes neither the contacts
        nor the dt.
        """
        self._read_forces(output, _lib.ovphysx_read_contact_friction_force_matrix, "friction force matrix")

    def _read_forces(self, output, reader, description: str) -> None:
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            from ._dlpack_utils import acquire_dltensor

            dl_tensor, keepalive = acquire_dltensor(output)
            result = reader(
                self._sdk._omni_physx_sdk_handle.value, self._handle, ctypes.byref(dl_tensor)
            )
            _ = keepalive
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to read {description}: {self._sdk._get_last_error()}")

    def read_net_forces(self, output) -> None:
        """Deprecated alias of :meth:`read_net_normal_forces`; returns normal forces only."""
        warnings.warn(
            "read_net_forces() is deprecated; use read_net_normal_forces(). "
            "The result still contains normal forces only.",
            DeprecationWarning, stacklevel=2,
        )
        self.read_net_normal_forces(output)

    def read_force_matrix(self, output) -> None:
        """Deprecated alias of :meth:`read_normal_force_matrix`; returns normal forces only."""
        warnings.warn(
            "read_force_matrix() is deprecated; use read_normal_force_matrix(). "
            "The result still contains normal forces only.",
            DeprecationWarning, stacklevel=2,
        )
        self.read_normal_force_matrix(output)

    def read_normal_contact_data(self, contact_forces, positions, normals, separations, counts, start_indices) -> int:
        """Read detailed normal contact data into flat buffers.

        .. deprecated:: 0.6.0
            The contact-binding API is deprecated and retained for compatibility.

        ``contact_forces`` contains scalar normal forces. Multiply each scalar
        by the corresponding ``normals`` vector to obtain the world-space
        normal force vector. Friction forces are returned separately by
        :meth:`read_friction_contact_data`.

        Expected shapes are ``[C, 1]`` for ``contact_forces`` and ``separations``,
        ``[C, 3]`` for ``positions`` and ``normals``, and ``[sensor_count,
        filter_count]`` for ``counts`` and ``start_indices``. ``C`` is
        :attr:`max_contact_data_count`. Both ``C`` and ``filter_count`` must
        be positive. Count and start-index tensors may be int32 or uint32.
        Contact force magnitudes use the timestep from the last successful
        :meth:`PhysX.step` or :meth:`PhysX.step_sync` call that simulated; a
        zero-dt step is skipped and changes neither the contacts nor the timestep.

        Returns the total number of contacts produced for the step. When this
        exceeds ``max_contact_data_count``, the buffers contain the valid prefix
        that fit. A pair's count reports only contacts actually written, and
        its start index is clamped to ``max_contact_data_count``, so
        ``[start, start + count)`` is always an in-range (possibly empty) slice.
        Recreate the binding with at least the returned capacity, step again,
        and retry to obtain a complete result.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            if self._max_contact_data_count <= 0:
                raise RuntimeError(
                    "Detailed contact reads require max_contact_data_count > 0 " "when creating the contact binding."
                )
            if self._filter_count <= 0:
                raise RuntimeError(
                    "Detailed contact reads require filters_per_sensor > 0 " "when creating the contact binding."
                )
            from ._dlpack_utils import acquire_dltensor

            force_dl, force_keepalive = acquire_dltensor(contact_forces)
            pos_dl, pos_keepalive = acquire_dltensor(positions)
            normal_dl, normal_keepalive = acquire_dltensor(normals)
            sep_dl, sep_keepalive = acquire_dltensor(separations)
            count_dl, count_keepalive = acquire_dltensor(counts)
            start_dl, start_keepalive = acquire_dltensor(start_indices)
            required_contact_count = ctypes.c_uint32(0)

            result = _lib.ovphysx_read_normal_contact_data(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(force_dl),
                ctypes.byref(pos_dl),
                ctypes.byref(normal_dl),
                ctypes.byref(sep_dl),
                ctypes.byref(count_dl),
                ctypes.byref(start_dl),
                ctypes.byref(required_contact_count),
            )
            _ = (force_keepalive, pos_keepalive, normal_keepalive, sep_keepalive, count_keepalive, start_keepalive)
            if result.status not in (ApiStatus.SUCCESS, ApiStatus.BUFFER_TOO_SMALL):
                raise RuntimeError(f"Failed to read detailed contact data: {self._sdk._get_last_error()}")
            return required_contact_count.value

    def read_contact_data(self, contact_forces, positions, normals, separations, counts, start_indices) -> int:
        """Deprecated alias of :meth:`read_normal_contact_data`; forces contain only normal components."""
        warnings.warn(
            "read_contact_data() is deprecated; use read_normal_contact_data(). "
            "The force values still contain normal components only.",
            DeprecationWarning, stacklevel=2,
        )
        return self.read_normal_contact_data(
            contact_forces, positions, normals, separations, counts, start_indices,
        )

    def read_raw_contact_data(
        self,
        contact_forces,
        positions,
        normals,
        separations,
        sensor_layout,
        actor_ids,
    ) -> int:
        """Read raw (unfiltered) contact data into flat buffers.

        Filter-less variant of :meth:`read_normal_contact_data`. Returns every
        contact involving each sensor regardless of which other actor it
        collided with, plus per-contact actor-identity tensors for identifying
        both the sensor and the contacting body via
        :meth:`get_other_actor_paths_from_ids`.

        Expected shapes are ``[C, 1]`` for ``contact_forces`` and
        ``separations``, ``[C, 3]`` for ``positions`` and ``normals``,
        ``[sensor_count, 2]`` for ``sensor_layout`` (column 0 count, column 1
        start index), and ``[C, 2]`` for ``actor_ids`` (column 0 the reporting
        sensor's actor, column 1 the actor it contacted). ``C`` is
        :attr:`max_contact_data_count` and must be positive. No filter
        dimension is required. ``sensor_layout`` may be int32 or uint32.
        ``actor_ids`` must be int64 or uint64. Slicing the columns out as
        views costs no copy. Contact force
        magnitudes use the timestep from the last successful
        :meth:`PhysX.step` or :meth:`PhysX.step_sync` call that simulated; a
        zero-dt step is skipped and changes neither the contacts nor the timestep.

        Returns the total number of contacts produced for the step. When this
        exceeds ``max_contact_data_count``, the buffers contain the valid prefix
        that fit. A sensor's count reports only contacts actually written, and
        its start index is clamped to ``max_contact_data_count``, so
        ``[start, start + count)`` is always an in-range (possibly empty) slice.
        Recreate the binding with at least the returned capacity, step again,
        and retry to obtain a complete result.

        **Token lifetime**: tokens in ``actor_ids`` are opaque actor handles,
        not encoded paths, and are
        stable while the corresponding actor is alive on the attached stage.
        After an actor is removed, its token is stale and
        :meth:`get_other_actor_paths_from_ids` reports it as an empty path
        rather than as the path it used to name.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            if self._max_contact_data_count <= 0:
                raise RuntimeError(
                    "Raw contact reads require max_contact_data_count > 0 " "when creating the contact binding."
                )
            from ._dlpack_utils import acquire_dltensor

            force_dl, force_keepalive = acquire_dltensor(contact_forces)
            pos_dl, pos_keepalive = acquire_dltensor(positions)
            normal_dl, normal_keepalive = acquire_dltensor(normals)
            sep_dl, sep_keepalive = acquire_dltensor(separations)
            layout_dl, layout_keepalive = acquire_dltensor(sensor_layout)
            ids_dl, ids_keepalive = acquire_dltensor(actor_ids)
            required_contact_count = ctypes.c_uint32(0)

            result = _lib.ovphysx_read_raw_contact_data(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(force_dl),
                ctypes.byref(pos_dl),
                ctypes.byref(normal_dl),
                ctypes.byref(sep_dl),
                ctypes.byref(layout_dl),
                ctypes.byref(ids_dl),
                ctypes.byref(required_contact_count),
            )
            _ = (force_keepalive, pos_keepalive, normal_keepalive, sep_keepalive, layout_keepalive, ids_keepalive)
            if result.status not in (ApiStatus.SUCCESS, ApiStatus.BUFFER_TOO_SMALL):
                raise RuntimeError(f"Failed to read raw contact data: {self._sdk._get_last_error()}")
            return required_contact_count.value

    def get_other_actor_paths_from_ids(self, ids_array) -> list[str]:
        """Resolve actor IDs from :meth:`read_raw_contact_data` to physics-object paths.

        ``ids_array`` is a 1D int64/uint64 array (numpy / warp / torch with
        DLPack support) holding actor IDs, from either column of the
        ``actor_ids`` tensor. Both use the same namespace. A column is a
        strided view and this boundary requires C-contiguous input, so wrap a
        column slice in ``np.ascontiguousarray()`` before passing it. Path
        strings are copied into Python, so the caller can keep them across
        subsequent ovphysx calls.

        Every non-zero ID is checked against the attached stage first, so an ID
        whose actor has been removed yields an empty path rather than the path it
        used to name. Since the caller holds the IDs, that makes the failure
        explicit: a non-zero ID with an empty path is stale, while ID ``0`` simply
        means no actor. The check is as precise as the backend's notion of
        existence. On a USD stage a merely *deactivated* prim still resolves.

        Returns:
            list[str]: Physics-object paths in the same order as the input IDs.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            from ._dlpack_utils import acquire_dltensor

            ids_dl, ids_keepalive = acquire_dltensor(ids_array)
            # The native call writes exactly N paths (N = ids_dl.shape[0]).
            if ids_dl.ndim != 1:
                del ids_keepalive
                raise ValueError(f"ids_array must be a 1D int64/uint64 tensor; got ndim={ids_dl.ndim}")
            n = int(ids_dl.shape[0])
            if n == 0:
                _ = ids_keepalive
                return []

            buf = (ovphysx_string_t * n)()
            count = ctypes.c_uint32(0)
            result = _lib.ovphysx_contact_binding_get_other_actor_paths_from_ids(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(ids_dl),
                buf,
                ctypes.c_uint32(n),
                ctypes.byref(count),
            )
            _ = ids_keepalive
            if result.status not in (ApiStatus.SUCCESS, ApiStatus.BUFFER_TOO_SMALL):
                raise RuntimeError(f"Failed to resolve actor IDs to paths: {self._sdk._get_last_error()}")
            return _decode_truncated_paths(
                buf,
                int(count.value),
                n,
                "ovphysx_contact_binding_get_other_actor_paths_from_ids",
            )

    def read_friction_contact_data(self, friction_forces, friction_points, counts, start_indices) -> int:
        """Read detailed friction contact data into flat buffers.

        .. deprecated:: 0.6.0
            The contact-binding API is deprecated and retained for compatibility.

        Returns world-space friction force vectors at friction anchors. These
        anchors need not correspond one-to-one with the contact points from
        :meth:`read_normal_contact_data`; use this read's own counts and start indices.

        Expected shapes are ``[C, 3]`` for ``friction_forces`` and
        ``friction_points``, and ``[sensor_count, filter_count]`` for ``counts``
        and ``start_indices``. ``C`` is :attr:`max_contact_data_count` and must
        be positive, and ``filter_count`` must also be positive. Count and
        start-index tensors may be int32 or uint32.
        Friction entries are per-anchor. Sum each flat slice to build a
        pair-level ``[sensor_count, filter_count, 3]`` force tensor.
        Friction forces use the timestep from the last successful
        :meth:`PhysX.step` or :meth:`PhysX.step_sync` call that simulated; a
        zero-dt step is skipped and changes neither the contacts nor the timestep.

        Returns the total number of friction anchors produced for the step. When
        this exceeds ``max_contact_data_count``, the buffers contain the valid
        prefix that fit. A pair's count reports only anchors actually written,
        and its start index is clamped to ``max_contact_data_count``, so
        ``[start, start + count)`` is always an in-range (possibly empty) slice.
        Recreate the binding with at least the returned capacity, step again,
        and retry to obtain a complete result.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            if self._max_contact_data_count <= 0:
                raise RuntimeError(
                    "Detailed friction reads require max_contact_data_count > 0 " "when creating the contact binding."
                )
            if self._filter_count <= 0:
                raise RuntimeError(
                    "Detailed friction reads require filters_per_sensor > 0 " "when creating the contact binding."
                )
            from ._dlpack_utils import acquire_dltensor

            force_dl, force_keepalive = acquire_dltensor(friction_forces)
            point_dl, point_keepalive = acquire_dltensor(friction_points)
            count_dl, count_keepalive = acquire_dltensor(counts)
            start_dl, start_keepalive = acquire_dltensor(start_indices)
            required_friction_count = ctypes.c_uint32(0)

            result = _lib.ovphysx_read_friction_contact_data(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(force_dl),
                ctypes.byref(point_dl),
                ctypes.byref(count_dl),
                ctypes.byref(start_dl),
                ctypes.byref(required_friction_count),
            )
            _ = (force_keepalive, point_keepalive, count_keepalive, start_keepalive)
            if result.status not in (ApiStatus.SUCCESS, ApiStatus.BUFFER_TOO_SMALL):
                raise RuntimeError(f"Failed to read detailed friction data: {self._sdk._get_last_error()}")
            return required_friction_count.value

    def read_friction_data(self, friction_forces, friction_points, counts, start_indices) -> int:
        """Deprecated alias of :meth:`read_friction_contact_data`."""
        warnings.warn(
            "read_friction_data() is deprecated; use read_friction_contact_data().",
            DeprecationWarning, stacklevel=2,
        )
        return self.read_friction_contact_data(friction_forces, friction_points, counts, start_indices)

    def destroy(self) -> None:
        """Release contact binding resources.

        Safe to call multiple times. Captures strong references to the SDK and
        library before the C call to guard against GC ordering issues (Python
        may collect self._sdk before self if both go out of scope together).
        """
        with self._lock:
            if not self._destroyed:
                sdk = self._sdk
                sdk_handle = sdk._omni_physx_sdk_handle
                if sdk_handle is None:
                    self._destroyed = True
                    return
                result = _lib.ovphysx_destroy_contact_binding(sdk_handle.value, self._handle)
                self._destroyed = True
                if result.status != ApiStatus.SUCCESS:
                    raise RuntimeError(f"Failed to destroy contact binding: {sdk._get_last_error()}")

    def __del__(self):
        try:
            import sys

            if sys.is_finalizing():
                return
        except Exception:
            return
        if not self._destroyed:
            if getattr(getattr(self, "_sdk", None), "_omni_physx_sdk_handle", None) is not None:
                try:
                    _warn_unclosed_resource("ContactBinding", self)
                except Exception:
                    pass
            try:
                self.destroy()
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.destroy()


# =============================================================================
# Module-level process configuration
# =============================================================================

_GLOBAL_STRING_KEYS = {
    GlobalConfigKey.COOKED_COLLIDER_CACHE_DIRECTORY,
    GlobalConfigKey.PROFILER_OUTPUT_PATH,
}


def _make_scoped_config_value(value: bool | int | float | str) -> ovphysx_config_value_t:
    result = ovphysx_config_value_t()
    if isinstance(value, bool):
        result.type = OVPHYSX_CONFIG_VALUE_BOOL
        result.value.bool_value = value
    elif isinstance(value, int):
        if not -(2**63) <= value < 2**63:
            raise ValueError("Configuration integer must fit signed int64")
        result.type = OVPHYSX_CONFIG_VALUE_INT64
        result.value.int64_value = value
    elif isinstance(value, float):
        result.type = OVPHYSX_CONFIG_VALUE_DOUBLE
        result.value.double_value = value
    elif isinstance(value, str):
        if "\0" in value:
            raise ValueError("Configuration strings must not contain NUL")
        result.type = OVPHYSX_CONFIG_VALUE_STRING
        string_value = ovphysx_string_t(value)
        result.value.string_value = string_value
        # Assigning a nested ctypes structure copies only its fields. Retain the
        # object that owns the encoded buffer until the native call returns.
        result._string_keepalive = string_value
    else:
        raise TypeError("Configuration values must be bool, int, float, or str")
    return result


def _raise_config_error(result, operation: str) -> None:
    error = _get_last_error_from_lib()
    if result.status == ApiStatus.INVALID_ARGUMENT:
        raise ValueError(error or f"{operation} rejected an invalid key or value")
    raise RuntimeError(error or f"{operation} failed ({ApiStatus(result.status).name})")


def set_global_config(key: GlobalConfigKey, value: bool | int | float | str) -> None:
    """Set one process-global value. Environment overrides remain authoritative."""
    native_value = _make_scoped_config_value(value)
    result = _lib.ovphysx_set_global_config(int(key), native_value)
    if result.status != ApiStatus.SUCCESS:
        _raise_config_error(result, "set_global_config")


def _get_scoped_config(call, owner: int | None, key: int, expected_type: type):
    native_value = ovphysx_config_value_t()
    if expected_type is str:
        native_value.type = OVPHYSX_CONFIG_VALUE_STRING
        native_value.value.string_value.ptr = None
        native_value.value.string_value.length = 0
    result = call(key, byref(native_value)) if owner is None else call(owner, key, byref(native_value))
    if result.status == ApiStatus.NOT_FOUND:
        return None
    if expected_type is str and result.status == ApiStatus.BUFFER_TOO_SMALL:
        capacity = native_value.value.string_value.length
        buffer = ctypes.create_string_buffer(capacity)
        native_value.value.string_value.ptr = ctypes.cast(buffer, c_char_p)
        native_value.value.string_value.length = capacity
        result = call(key, byref(native_value)) if owner is None else call(owner, key, byref(native_value))
    if result.status != ApiStatus.SUCCESS:
        _raise_config_error(result, "get_config")
    if native_value.type == OVPHYSX_CONFIG_VALUE_BOOL:
        return bool(native_value.value.bool_value)
    if native_value.type == OVPHYSX_CONFIG_VALUE_INT64:
        return int(native_value.value.int64_value)
    if native_value.type == OVPHYSX_CONFIG_VALUE_DOUBLE:
        return float(native_value.value.double_value)
    if native_value.type == OVPHYSX_CONFIG_VALUE_STRING:
        return str(native_value.value.string_value)
    raise RuntimeError("Native configuration returned an unknown value type")


def get_global_config(key: GlobalConfigKey) -> bool | int | float | str | None:
    """Return one effective process-global value.

    Unset strings return ``None``. Scalar keys return their declared default
    when nothing has been set.
    """
    enum_key = GlobalConfigKey(key)
    expected_type = str if enum_key in _GLOBAL_STRING_KEYS else float
    return _get_scoped_config(_lib.ovphysx_get_global_config, None, int(enum_key), expected_type)


# =============================================================================
# Module-level logging compatibility
# =============================================================================


def set_log_level(level: int) -> None:
    """Set the process-scoped libovphysx source log level threshold.

    Messages emitted under the named Carbonite sources ``omni_physx_sdk``,
    ``omni.physx``, and ``ovphysx_internal`` below this level are suppressed for
    console and callback delivery. Every other process source and channel,
    including any unnamed source, remains unchanged and is subject only to the
    callback's severity and channel filter. Callable at any time, including
    before instance creation. ``LogLevel.NONE`` mutes only the three named
    sources. It is not a whole-runtime or process mute.

    Args:
        level: Log level threshold (LogLevel.DEFAULT through LogLevel.NONE).
               LogLevel.DEFAULT restores LogLevel.WARNING.

    Raises:
        ValueError: If level is out of range. No state change is applied.
        RuntimeError: If the native API rejects the call for another reason,
            including callback-time reconfiguration.
    """
    result = _lib.ovphysx_set_log_level(level)
    if result.status != ApiStatus.SUCCESS:
        err = _lib.ovphysx_get_last_error()
        err_msg = str(err) if err and err.ptr else ""
        if result.status == ApiStatus.INVALID_ARGUMENT:
            raise ValueError(err_msg or f"Invalid log level: {level}")
        raise RuntimeError(err_msg or "Failed to set log level")


def get_log_level() -> int:
    """Get the current process-scoped libovphysx source log level threshold.

    Returns:
        The current log level (int matching ovphysx_log_level_t constants).
    """
    return int(_lib.ovphysx_get_log_level())


def enable_default_log_output(enable: bool = True) -> None:
    """Enable or disable Carbonite's built-in console log output.

    By default, Carbonite logs to the console. When a custom callback is set
    (or :func:`enable_python_logging` is active), both the built-in console
    output and the callback receive messages, which may
    cause duplicate output.

    Call with ``False`` to suppress the built-in console output while
    keeping the callback active. Call with ``True`` to re-enable it.

    This is independent of callback registration and the libovphysx source log
    level. It controls Carbonite's process-global built-in console logger and
    therefore affects every Carbonite tenant in the process. Multi-tenant hosts
    should normally own this policy and leave the logger enabled.

    Args:
        enable: ``True`` to enable (default), ``False`` to disable.

    Raises:
        RuntimeError: If the native API rejects the call, including when it is
            made from the active native log callback.
    """
    result = _lib.ovphysx_enable_default_log_output(enable)
    if result.status != ApiStatus.SUCCESS:
        err = _lib.ovphysx_get_last_error()
        err_msg = str(err) if err and err.ptr else ""
        raise RuntimeError(err_msg or "Failed to set default log output")


# Internal state for Python logging bridge
_python_log_callback = None  # prevent GC of the ctypes callback
_python_log_logger_name = None
# A native callback transition can publish before Python observes its return
# status (for example, if KeyboardInterrupt arrives at the ctypes boundary).
# Retain every callback from an ambiguous transition until a later successful
# replace, disable, or shutdown proves that the native slot has been drained.
_python_log_retained_callbacks = ()
_python_log_callback_lock = threading.Lock()
_python_log_callback_condition = threading.Condition(_python_log_callback_lock)
_python_log_callback_context = threading.local()
_python_log_callback_transition = False


def enable_python_logging(
    logger_name: str = "ovphysx",
    *,
    min_severity: int = LogLevel.VERBOSE,
    channel_filter: str | None = None,
) -> None:
    """Route native log messages to Python's logging module.

    Sets the sole C-level callback slot and forwards matching native messages
    to ``logging.getLogger(logger_name)``. Calling this function again replaces
    any existing native callback, including one installed outside Python.
    Successful shutdown of the Python process-lifecycle scope, normally when
    the final :class:`PhysX` instance is destroyed, also disables this bridge.
    Enable it again after creating an instance in a new lifecycle scope.
    Each forwarded ``LogRecord`` includes ``ovphysx_channel`` and
    ``ovphysx_timestamp`` attributes for the native source and Unix-epoch
    timestamp seconds.

    Call :func:`disable_python_logging` to stop forwarding.

    Args:
        logger_name: Name of the Python logger to route to (default: "ovphysx").
        min_severity: Minimum severity for records observed from the process
            log stream (default: LogLevel.VERBOSE). Records from libovphysx's
            ``omni_physx_sdk``, ``omni.physx``, and ``ovphysx_internal`` sources
            are also subject to :func:`set_log_level`.
        channel_filter: Optional comma-separated ``channel=level`` rules. The
            native API copies this string, compares levels case-insensitively,
            and uses raw channel-prefix matching. The longest matching prefix
            wins. A later rule wins ties of equal length.

    Raises:
        RuntimeError: If called from a native log callback, another Python
            logging transition is in progress, or the native API rejects the
            severity, filter, or callback configuration.
    """
    import logging as _logging

    global _python_log_callback, _python_log_logger_name
    global _python_log_retained_callbacks
    global _python_log_callback_transition

    if getattr(_python_log_callback_context, "active", False):
        raise RuntimeError("Cannot configure Python logging from a native log callback")

    py_logger = _logging.getLogger(logger_name)

    # Map ovphysx levels to Python logging levels
    level_map = {
        LogLevel.ERROR: _logging.ERROR,
        LogLevel.WARNING: _logging.WARNING,
        LogLevel.INFO: _logging.INFO,
        LogLevel.VERBOSE: _logging.DEBUG,
    }

    @ovphysx_log_callback_t
    def _callback(level, message, channel, timestamp, user_data):
        previous_active = getattr(_python_log_callback_context, "active", False)
        _python_log_callback_context.active = True
        py_level = level_map.get(level, _logging.DEBUG)
        try:
            try:
                text = ctypes.string_at(message.ptr, message.length).decode("utf-8", errors="replace")
            except Exception:
                text = str(message)
            try:
                channel_text = ctypes.string_at(channel.ptr, channel.length).decode("utf-8", errors="replace")
            except Exception:
                channel_text = ""
            py_logger.log(
                py_level,
                "%s",
                text,
                extra={"ovphysx_channel": channel_text, "ovphysx_timestamp": timestamp},
            )
        finally:
            _python_log_callback_context.active = previous_active

    filter_value = ovphysx_string_t(channel_filter) if channel_filter is not None else None
    filter_pointer = ctypes.byref(filter_value) if filter_value is not None else None
    retained_before_candidate = ()
    transition_owned = False
    try:
        with _python_log_callback_condition:
            if _python_log_callback_transition:
                raise RuntimeError("Another Python logging transition is in progress")
            # The native call may publish the candidate before Python can
            # observe its status. Retain it before crossing the FFI boundary.
            retained_before_candidate = _python_log_retained_callbacks
            _python_log_retained_callbacks = (*_python_log_retained_callbacks, _callback)
            transition_owned = True
            _python_log_callback_transition = True

        result = _lib.ovphysx_set_log_callback(min_severity, filter_pointer, ctypes.cast(_callback, c_void_p), None)
        if result.status != ApiStatus.SUCCESS:
            err = _lib.ovphysx_get_last_error()
            err_msg = str(err) if err and err.ptr else ""
            if result.status == ApiStatus.INVALID_ARGUMENT:
                # Native validation precedes publication. Drop only this
                # rejected candidate. Older ambiguous owners may still back
                # the active native slot and must remain alive.
                with _python_log_callback_condition:
                    _python_log_retained_callbacks = retained_before_candidate
            raise RuntimeError(f"Failed to enable Python logging: {err_msg}")

        with _python_log_callback_condition:
            # Publish the known-active owner before dropping callbacks retained
            # from this or any earlier ambiguous transition.
            _python_log_callback = _callback
            _python_log_logger_name = logger_name
            _python_log_retained_callbacks = ()
    finally:
        if transition_owned:
            with _python_log_callback_condition:
                _python_log_callback_transition = False
                _python_log_callback_condition.notify_all()


def disable_python_logging() -> None:
    """Stop routing native log messages to Python's logging module.

    If :func:`enable_python_logging` was not called, this is a no-op.

    Raises:
        RuntimeError: If called from a native log callback or while another
            Python logging transition is in progress.
    """
    global _python_log_callback, _python_log_logger_name
    global _python_log_retained_callbacks
    global _python_log_callback_transition

    if getattr(_python_log_callback_context, "active", False):
        raise RuntimeError("Cannot configure Python logging from a native log callback")

    transition_owned = False
    disable_succeeded = False
    callback_owners = ()
    try:
        with _python_log_callback_condition:
            if _python_log_callback_transition:
                raise RuntimeError("Another Python logging transition is in progress")
            if _python_log_callback is None and not _python_log_retained_callbacks:
                return
            transition_owned = True
            _python_log_callback_transition = True
            callback_owners = (
                _python_log_callback,
                *_python_log_retained_callbacks,
            )

        result = _lib.ovphysx_set_log_callback(LogLevel.DEFAULT, None, None, None)
        if result.status != ApiStatus.SUCCESS:
            import logging as _logging

            err = _lib.ovphysx_get_last_error()
            err_msg = str(err) if err and err.ptr else "unknown error"
            _logging.getLogger(__name__).warning("Failed to unregister Python log callback: %s", err_msg)
            return
        disable_succeeded = True
    finally:
        if transition_owned:
            with _python_log_callback_condition:
                if disable_succeeded:
                    _python_log_callback = None
                    _python_log_logger_name = None
                    _python_log_retained_callbacks = ()
                _python_log_callback_transition = False
                _python_log_callback_condition.notify_all()
        # Keep every possibly published CFUNCTYPE alive until the native
        # disable has returned and drained the active registration.
        del callback_owners


def flush_log(timeout_ns: int = (1 << 64) - 1) -> None:
    """Wait for callback delivery already accepted before this call.

    The barrier covers records already handed to ovphysx by Carbonite. If the
    host enabled Carbonite asynchronous logging, records still buffered
    upstream are outside this barrier. Successful native shutdown flushes the
    upstream buffer before disabling and draining the callback.

    Args:
        timeout_ns: Maximum wait in nanoseconds. Zero polls; ``2**64 - 1``
            waits indefinitely.

    Raises:
        ValueError: If ``timeout_ns`` is outside the unsigned 64-bit range.
        TimeoutError: If the timeout expires.
        RuntimeError: If native log delivery cannot be flushed.
    """
    if timeout_ns < 0 or timeout_ns > (1 << 64) - 1:
        raise ValueError("timeout_ns must be in range 0 through 2**64 - 1")
    result = _lib.ovphysx_flush_log(timeout_ns)
    if result.status == ApiStatus.TIMEOUT:
        raise TimeoutError("Timed out waiting for native log delivery")
    if result.status != ApiStatus.SUCCESS:
        raise RuntimeError(_get_last_error_from_lib())


_PROCESS_LIFECYCLE_LOCK = threading.Lock()
_PROCESS_LIFECYCLE_CONDITION = threading.Condition(_PROCESS_LIFECYCLE_LOCK)
_PROCESS_LIFECYCLE_REFCOUNT = 0
# These mutually exclusive states are protected by the process condition.
# Native calls run without its lock. Concurrent constructors wait for
# initialization, while final-shutdown races fail before callback draining.
_PROCESS_LIFECYCLE_INITIALIZING = False
_PROCESS_LIFECYCLE_SHUTTING_DOWN = False
# Lifecycle references a Warp deleter could not release because a transition was in flight.
# Guarded by _PROCESS_LIFECYCLE_CONDITION. Drained at both lifecycle entry points.
_PROCESS_LIFECYCLE_DEFERRED_RELEASES = 0
# Reentrant: operations under this lock allocate, so CPython's cyclic collector
# can run on the same thread and finalize an unreachable PhysX whose __del__
# reaches destroy() -> _untrack_process_instance() and re-enters this lock.
# A plain Lock would self-deadlock there.
_PROCESS_LIFECYCLE_INSTANCES_LOCK = threading.RLock()
# Identity-keyed: WeakSet hashes the PhysX object, so an __eq__-only subclass
# is unhashable (TypeError after native construction) and two hashable
# instances that compare equal collapse to one tracked referent.
# Normally weak references; a finalizer unable to start its retry worker pins a
# known-live owner here through a strong callable until exit cleanup can retry.
_PROCESS_LIFECYCLE_INSTANCES: dict[int, Callable[[], Any]] = {}
_PROCESS_LIFECYCLE_ATEXIT_REGISTERED = False
# A deferred finalizer worker owns its instance's teardown until it returns.
# Process-exit cleanup sets the flag (no new workers; busy retries hand the
# owner back) and joins every worker before it touches any instance, so exit
# never runs native destruction or process shutdown concurrently with a worker.
# Both guarded by _PROCESS_LIFECYCLE_INSTANCES_LOCK.
_DEFERRED_FINALIZER_THREADS: set[threading.Thread] = set()
_PROCESS_EXIT_CLEANUP_STARTED = threading.Event()
# Attached Stages, pinned from module scope rather than only by their owner.
# ovstage.Stage's finalizer destroys the native stage unconditionally. When an
# unreachable host/owner cycle also contains the Stage, the collector runs every
# finalizer in that set before any teardown, so an owner-held reference alone
# lets the Stage be destroyed before its owner's immediate or deferred teardown
# detaches it. Keyed by a per-instance token, never by a reusable id().
_ATTACHED_OVSTAGES: dict[int, Any] = {}
_ATTACHED_OVSTAGE_KEYS = itertools.count(1)


def _start_deferred_finalizer(instance: Any, *, retry_busy: bool = False) -> bool:
    """Run the finalizer policy on a daemon worker that exit cleanup joins.

    Returns ``False`` without starting a worker once process-exit cleanup has
    begun; that cleanup owns the instance from then on. A thread start failure
    propagates to the caller.
    """

    def run() -> None:
        try:
            instance._finalize_unreleased(retry_busy=retry_busy)
        finally:
            if retry_busy:
                instance._busy_finalizer_pending = False
            with _PROCESS_LIFECYCLE_INSTANCES_LOCK:
                _DEFERRED_FINALIZER_THREADS.discard(worker)

    worker = threading.Thread(target=run, name="ovphysx-finalizer", daemon=True)
    with _PROCESS_LIFECYCLE_INSTANCES_LOCK:
        if _PROCESS_EXIT_CLEANUP_STARTED.is_set():
            return False
        _DEFERRED_FINALIZER_THREADS.add(worker)
        try:
            worker.start()
        except BaseException:
            _DEFERRED_FINALIZER_THREADS.discard(worker)
            raise
    return True


def _join_deferred_finalizers() -> None:
    """Stop new deferred finalizers and wait for in-flight ones to return."""
    with _PROCESS_LIFECYCLE_INSTANCES_LOCK:
        _PROCESS_EXIT_CLEANUP_STARTED.set()
    while True:
        with _PROCESS_LIFECYCLE_INSTANCES_LOCK:
            workers = [w for w in _DEFERRED_FINALIZER_THREADS if w is not threading.current_thread()]
        if not workers:
            return
        for worker in workers:
            worker.join()


def _destroy_process_instances_at_exit() -> None:
    """Destroy live Python instances before interpreter finalization."""
    # A worker returns after completing teardown or, if still busy, after
    # handing its owner back. Only then is the instance snapshot below safe.
    _join_deferred_finalizers()
    with _PROCESS_LIFECYCLE_INSTANCES_LOCK:
        # The lock makes the reentrancy described above safe, but not the
        # snapshot itself: walking the dict while a cyclic-collector __del__
        # lands in _untrack_process_instance() would change its size and
        # abandon remaining instances. Pause collection for the snapshot
        # only. The loop below runs with it back on.
        gc_was_enabled = gc.isenabled()
        gc.disable()
        try:
            instances = []
            for tracked in list(_PROCESS_LIFECYCLE_INSTANCES.values()):
                instance = tracked()
                if instance is not None:
                    instances.append(instance)
        finally:
            if gc_was_enabled:
                gc.enable()
    retained_busy = False
    for instance in instances:
        # _finalize_unreleased() is the non-throwing finalizer policy, but at this
        # exit boundary no caller is left to recover. Isolate each instance so one
        # failure cannot strand the remaining lifecycle tokens or skip the
        # final SDK shutdown and its platform-specific cache cleanup.
        try:
            instance._finalize_unreleased()
        except BaseException as exc:
            try:
                warnings.warn(
                    f"PhysX process-exit cleanup failed for one instance: {exc}",
                    RuntimeWarning,
                    stacklevel=1,
                )
            except BaseException:
                import sys

                sys.stderr.write(f"PhysX process-exit cleanup failed for one instance: {exc}\n")
        retained_busy = retained_busy or getattr(instance, "_retained_busy_at_exit", False)
    if retained_busy:
        # A still-busy instance keeps its lifecycle token. Native shutdown
        # refuses while it is live, so collapsing tokens would only misreport
        # ownership.
        return
    _shutdown_remaining_process_lifecycle_at_exit()


def _shutdown_remaining_process_lifecycle_at_exit() -> None:
    """Run native shutdown even if a read() array still holds a borrow token.

    Each Warp array from read() owns an independent process-lifecycle token so
    ovphysx_shutdown() cannot reclaim the CUDA context while the array aliases
    it. At interpreter exit those arrays can remain globally reachable: instance
    cleanup leaves the refcount at 1, the deferred-release drain sees an empty
    queue, and the array deleter runs only during later teardown, after atexit
    drainers are gone. ovphysx.cpp documents that skipping ovphysx_shutdown()
    then destroys the direct runtime from C++ statics, which can emit UJITSO
    "Leaked processor" errors and a Windows access violation. The arrays are
    about to die with the interpreter, so collapse leftover tokens and shut
    down while ctypes is still valid.
    """
    global _PROCESS_LIFECYCLE_REFCOUNT
    try:
        _drain_deferred_lifecycle_releases()
    except BaseException:
        pass
    with _PROCESS_LIFECYCLE_CONDITION:
        if _PROCESS_LIFECYCLE_REFCOUNT <= 0:
            return
        if _PROCESS_LIFECYCLE_REFCOUNT > 1:
            _PROCESS_LIFECYCLE_REFCOUNT = 1
    try:
        _release_process_lifecycle()
    except BaseException:
        pass


def _track_process_instance(instance: Any) -> None:
    """Track an instance and install process-exit cleanup after native bootstrap."""
    global _PROCESS_LIFECYCLE_ATEXIT_REGISTERED
    key = id(instance)

    def _forget(_ref: weakref.ref, tracked_key: int = key) -> None:
        with _PROCESS_LIFECYCLE_INSTANCES_LOCK:
            _PROCESS_LIFECYCLE_INSTANCES.pop(tracked_key, None)

    with _PROCESS_LIFECYCLE_INSTANCES_LOCK:
        _PROCESS_LIFECYCLE_INSTANCES[key] = weakref.ref(instance, _forget)
        if not _PROCESS_LIFECYCLE_ATEXIT_REGISTERED:
            # Registered from a fully bootstrapped instance so the callback only
            # ever sees instances with usable native handles. Carbonite's own
            # cache-removal backstop is a C `std::atexit` thunk, which the C
            # runtime drains after Py_FinalizeEx. This Python callback therefore
            # always runs first, while ctypes and the handles are still usable.
            atexit.register(_destroy_process_instances_at_exit)
            _PROCESS_LIFECYCLE_ATEXIT_REGISTERED = True


def _untrack_process_instance(instance: Any) -> None:
    """Stop tracking after native instance destruction becomes terminal."""
    with _PROCESS_LIFECYCLE_INSTANCES_LOCK:
        _PROCESS_LIFECYCLE_INSTANCES.pop(id(instance), None)


def _get_last_error_from_lib() -> str:
    err = _lib.ovphysx_get_last_error()
    if err and err.ptr:
        try:
            return ctypes.string_at(err.ptr, err.length).decode("utf-8", errors="replace") or "Unknown error"
        except Exception:
            return "Unknown error"
    return "Unknown error"


def _acquire_process_lifecycle(owner=None) -> None:
    global _PROCESS_LIFECYCLE_REFCOUNT, _PROCESS_LIFECYCLE_INITIALIZING
    global _PROCESS_LIFECYCLE_SHUTTING_DOWN
    if getattr(_python_log_callback_context, "active", False):
        # Native lifecycle transitions may synchronously deliver callbacks. A
        # callback must not recursively enter lifecycle work itself.
        raise RuntimeError(
            "PhysX() cannot be called from within a native log callback; " "retry after the callback returns"
        )

    _drain_deferred_lifecycle_releases()

    transition_owned = False
    initialize_may_have_run = False
    initialize_known_failure = False
    shared_token_published = False
    token_published = False
    try:
        with _PROCESS_LIFECYCLE_CONDITION:
            _PROCESS_LIFECYCLE_CONDITION.wait_for(lambda: not _PROCESS_LIFECYCLE_INITIALIZING)
            if _PROCESS_LIFECYCLE_SHUTTING_DOWN:
                raise RuntimeError("ovphysx process shutdown is in progress; retry PhysX() after it returns")
            if _PROCESS_LIFECYCLE_REFCOUNT > 0:
                previous_refcount = _PROCESS_LIFECYCLE_REFCOUNT
                try:
                    if owner is not None:
                        owner._lifecycle_acquired = True
                    _PROCESS_LIFECYCLE_REFCOUNT = previous_refcount + 1
                    shared_token_published = True
                except BaseException:
                    _PROCESS_LIFECYCLE_REFCOUNT = previous_refcount
                    shared_token_published = False
                    if owner is not None:
                        owner._lifecycle_acquired = False
                    raise
            else:
                transition_owned = True
                _PROCESS_LIFECYCLE_INITIALIZING = True

        if shared_token_published:
            return

        initialize_may_have_run = True
        result = _lib.ovphysx_initialize()
        initialize_status = result.status
        if initialize_status != ApiStatus.SUCCESS:
            initialize_known_failure = True
            raise RuntimeError(f"ovphysx_initialize() failed: {_get_last_error_from_lib()}")

        with _PROCESS_LIFECYCLE_CONDITION:
            try:
                _PROCESS_LIFECYCLE_REFCOUNT = 1
                token_published = True
                _PROCESS_LIFECYCLE_INITIALIZING = False
            finally:
                _PROCESS_LIFECYCLE_CONDITION.notify_all()
        if owner is not None:
            owner._lifecycle_acquired = True
    except BaseException:
        if shared_token_published:
            if owner is not None and owner._lifecycle_acquired:
                raise
            try:
                _release_process_lifecycle()
            except BaseException:
                pass
            raise
        if not transition_owned:
            raise
        if not initialize_may_have_run:
            with _PROCESS_LIFECYCLE_CONDITION:
                try:
                    _PROCESS_LIFECYCLE_INITIALIZING = False
                finally:
                    _PROCESS_LIFECYCLE_CONDITION.notify_all()
            raise
        if initialize_known_failure:
            with _PROCESS_LIFECYCLE_CONDITION:
                try:
                    _PROCESS_LIFECYCLE_INITIALIZING = False
                finally:
                    _PROCESS_LIFECYCLE_CONDITION.notify_all()
            raise

        # The native call may have committed before an asynchronous Python
        # exception became visible at the ctypes boundary. A published token
        # uses the normal release path so any concurrent token survives. An
        # unpublished outcome hands the transition directly to shutdown. No
        # waiter may acquire the ambiguous token in between.
        if token_published:
            with _PROCESS_LIFECYCLE_CONDITION:
                if _PROCESS_LIFECYCLE_INITIALIZING:
                    try:
                        _PROCESS_LIFECYCLE_INITIALIZING = False
                    finally:
                        _PROCESS_LIFECYCLE_CONDITION.notify_all()
            if owner is not None and owner._lifecycle_acquired:
                raise
            try:
                _release_process_lifecycle()
            except BaseException:
                pass
            raise

        # Cleanup must survive a second asynchronous exception while the
        # ambiguous initialize result is handed to shutdown. Re-publishing
        # this idempotent state is safe and prevents SHUTTING_DOWN from being
        # stranded before the native rollback runs.
        handoff_published = False
        while not handoff_published:
            try:
                with _PROCESS_LIFECYCLE_CONDITION:
                    try:
                        # Publish SHUTTING_DOWN before releasing INITIALIZING so
                        # no waiter can acquire an ambiguous native outcome.
                        _PROCESS_LIFECYCLE_SHUTTING_DOWN = True
                        _PROCESS_LIFECYCLE_REFCOUNT = 0
                        _PROCESS_LIFECYCLE_INITIALIZING = False
                    finally:
                        _PROCESS_LIFECYCLE_CONDITION.notify_all()
                handoff_published = True
            except BaseException:
                pass
        try:
            _finish_process_shutdown_transition()
        except BaseException:
            pass
        raise


def _finish_process_shutdown_transition() -> None:
    global _python_log_callback, _python_log_logger_name
    global _python_log_retained_callbacks
    global _python_log_callback_transition
    global _PROCESS_LIFECYCLE_SHUTTING_DOWN

    pending_transition_error = None
    operation_error = None
    clear_transition_error = None
    transition_owned = False
    shutdown_succeeded = False
    callback_owners = ()
    try:
        try:
            while not transition_owned:
                try:
                    with _python_log_callback_condition:
                        while _python_log_callback_transition:
                            _python_log_callback_condition.wait()
                        # Set the local owner first so an interruption at the
                        # following assignment is recoverable.
                        transition_owned = True
                        _python_log_callback_transition = True
                        callback_owners = (
                            _python_log_callback,
                            *_python_log_retained_callbacks,
                        )
                except BaseException as error:
                    if pending_transition_error is None:
                        pending_transition_error = error
                    if transition_owned:
                        # The condition lock serialized the ownership decision.
                        # Reassert the flag if interruption landed between the
                        # local and global assignments.
                        with _python_log_callback_condition:
                            _python_log_callback_transition = True
                            callback_owners = (
                                _python_log_callback,
                                *_python_log_retained_callbacks,
                            )

            result = _lib.ovphysx_shutdown()
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"ovphysx_shutdown() failed: {_get_last_error_from_lib()}")
            shutdown_succeeded = True
        finally:
            if transition_owned:
                with _python_log_callback_condition:
                    if shutdown_succeeded:
                        # Native shutdown disabled and drained the sole slot.
                        _python_log_callback = None
                        _python_log_logger_name = None
                        _python_log_retained_callbacks = ()
                    _python_log_callback_transition = False
                    _python_log_callback_condition.notify_all()
            # Keep every possibly published CFUNCTYPE alive until native
            # shutdown has returned and drained the callback slot.
            del callback_owners
    except BaseException as error:
        operation_error = error
    finally:
        # Once final shutdown owns the process transition, no interruption may
        # leave future constructors permanently observing SHUTTING_DOWN. Retry
        # the idempotent publication until both the state change and condition
        # notification complete, then propagate the first deferred error.
        transition_cleared = False
        while not transition_cleared:
            try:
                with _PROCESS_LIFECYCLE_CONDITION:
                    try:
                        _PROCESS_LIFECYCLE_SHUTTING_DOWN = False
                    finally:
                        _PROCESS_LIFECYCLE_CONDITION.notify_all()
                transition_cleared = True
            except BaseException as error:
                if clear_transition_error is None:
                    clear_transition_error = error

    if operation_error is not None:
        raise operation_error
    if pending_transition_error is not None:
        raise pending_transition_error
    if clear_transition_error is not None:
        raise clear_transition_error


def _defer_process_lifecycle_release() -> None:
    """Return one process reference from a Warp array deleter. Safe on ANY thread.

    Goes through :func:`_release_process_lifecycle` rather than touching the refcount, so there is
    one implementation of what a release means. What it adds is a thread-local veto on the FINAL
    release: a non-final release is a decrement under a lock and is safe wherever the collector
    happens to be, but the final one is ``ovphysx_shutdown()``, a long native teardown reclaiming
    the CUDA context, which must not run on a GC thread inside a ``__del__``, possibly while the
    interpreter is finalizing. Vetoed, it is queued and retired later by
    :func:`_drain_deferred_lifecycle_releases`.

    The veto is read under the process lock, so the decision cannot race a refcount change.

    Cannot raise: a deleter has nowhere to report an exception, and a lost reference would strand
    the process reference for good.
    """
    global _PROCESS_LIFECYCLE_DEFERRED_RELEASES
    _LIFECYCLE_DELETER_CONTEXT.veto_final = True
    try:
        _release_process_lifecycle()
    except BaseException:
        with _PROCESS_LIFECYCLE_CONDITION:
            _PROCESS_LIFECYCLE_DEFERRED_RELEASES += 1
    finally:
        _LIFECYCLE_DELETER_CONTEXT.veto_final = False


@atexit.register
def _drain_lifecycle_releases_at_exit() -> None:
    """Retire queued lifecycle references at interpreter exit.

    Without this, deferring in the deleter would trade "shutdown on a GC thread" for "shutdown never
    runs": the queue is otherwise only drained by a later lifecycle entry point, and a program whose
    last act is dropping an array reaches none. atexit runs on the main thread before finalization,
    which is a thread native shutdown may legitimately run on.

    Live read() arrays that remain reachable through exit are handled by
    :func:`_shutdown_remaining_process_lifecycle_at_exit`, which is registered later
    and therefore runs first (LIFO). This drain then sees an empty queue.
    """
    try:
        _drain_deferred_lifecycle_releases()
    except BaseException:
        pass


# Set only for the duration of a Warp deleter's release. Thread-local because it describes the
# CALLER, not the process: another thread may legitimately be completing final shutdown at the same
# moment.
_LIFECYCLE_DELETER_CONTEXT = threading.local()


def _drain_deferred_lifecycle_releases() -> None:
    """Retire lifecycle references a deleter could not release when it ran.

    Cannot recurse: the counter is decremented under the lock before the release is attempted,
    so a nested call sees zero. A release that fails again is put back and the drain stops.
    The next entry point retries it.
    """
    global _PROCESS_LIFECYCLE_DEFERRED_RELEASES
    while True:
        with _PROCESS_LIFECYCLE_CONDITION:
            if _PROCESS_LIFECYCLE_DEFERRED_RELEASES == 0:
                return
            if _PROCESS_LIFECYCLE_INITIALIZING or _PROCESS_LIFECYCLE_SHUTTING_DOWN:
                return
            _PROCESS_LIFECYCLE_DEFERRED_RELEASES -= 1
        try:
            _release_process_lifecycle()
        except BaseException:
            with _PROCESS_LIFECYCLE_CONDITION:
                _PROCESS_LIFECYCLE_DEFERRED_RELEASES += 1
            return


def _release_process_lifecycle() -> None:
    global _PROCESS_LIFECYCLE_REFCOUNT
    global _PROCESS_LIFECYCLE_SHUTTING_DOWN
    global _PROCESS_LIFECYCLE_DEFERRED_RELEASES

    if getattr(_python_log_callback_context, "active", False):
        # Check before taking the process lock: final shutdown may be draining
        # this callback and must never wait for it to acquire the same lock.
        raise RuntimeError("ovphysx_shutdown cannot be called from within a log callback")

    pending_transition_error = None
    transition_owned = False
    try:
        with _PROCESS_LIFECYCLE_CONDITION:
            if _PROCESS_LIFECYCLE_INITIALIZING:
                raise RuntimeError("ovphysx process initialization is already in progress")
            if _PROCESS_LIFECYCLE_SHUTTING_DOWN:
                raise RuntimeError("ovphysx process shutdown is already in progress")
            if _PROCESS_LIFECYCLE_REFCOUNT == 0:
                return
            if _PROCESS_LIFECYCLE_REFCOUNT > 1:
                _PROCESS_LIFECYCLE_REFCOUNT -= 1
                return

            # Everything below is FINAL shutdown. A Warp array deleter must not run it: it lands on
            # whatever thread the collector is on, inside a __del__, and ovphysx_shutdown() is a long
            # native teardown that reclaims the CUDA context. Queue it instead. The next lifecycle
            # entry point, or the interpreter-exit drain, retires it on a thread that can take it.
            #
            # Checked HERE rather than in the deleter so the decision is made under this lock, where
            # "would this be the final release?" cannot race another thread's acquire or release.
            if getattr(_LIFECYCLE_DELETER_CONTEXT, "veto_final", False):
                _PROCESS_LIFECYCLE_DEFERRED_RELEASES += 1
                return

            # Complete the final-shutdown publication while still holding the
            # condition lock. SHUTTING_DOWN is published before the refcount is
            # cleared, so an interrupted intermediate state remains fail-fast.
            while not transition_owned:
                try:
                    _PROCESS_LIFECYCLE_SHUTTING_DOWN = True
                    _PROCESS_LIFECYCLE_REFCOUNT = 0
                    transition_owned = True
                except BaseException as error:
                    if pending_transition_error is None:
                        pending_transition_error = error
    except BaseException as error:
        if not transition_owned:
            raise
        if pending_transition_error is None:
            pending_transition_error = error

    _finish_process_shutdown_transition()
    if pending_transition_error is not None:
        raise pending_transition_error


class _DestroyAdmissionBusy(RuntimeError):
    """Native destruction was rejected before changing instance ownership."""


class PhysX:
    """High-level wrapper around the C API using ctypes."""

    @property
    def _attached_ovstage(self) -> Any:
        """The attached Stage keepalive, pinned in ``_ATTACHED_OVSTAGES`` until cleared."""
        key = self.__dict__.get("_attached_ovstage_key")
        return None if key is None else _ATTACHED_OVSTAGES.get(key)

    @_attached_ovstage.setter
    def _attached_ovstage(self, stage: Any) -> None:
        key = self.__dict__.get("_attached_ovstage_key")
        if stage is None:
            if key is not None:
                _ATTACHED_OVSTAGES.pop(key, None)
            return
        if key is None:
            key = self.__dict__["_attached_ovstage_key"] = next(_ATTACHED_OVSTAGE_KEYS)
        _ATTACHED_OVSTAGES[key] = stage

    def __init__(
        self,
        *,
        config: "PhysXConfig | None" = None,
        ignore_version_mismatch: bool = False,
        active_cuda_gpus: str | None = None,
    ) -> None:
        """Initialize PhysX SDK.

        Args:
            config: Typed config dataclass. Only non-None fields are applied.
            ignore_version_mismatch: Skip Python/native version match check.
            active_cuda_gpus: Comma-separated CUDA device ordinals
                (default: None = no ovphysx ordinal override).
                Restricts which GPU ordinal(s) are used.
                Supported: None/"" (preserve current PhysX process selection;
                a fresh/default process selects automatically), "0" (GPU 0), "N" (GPU N),
                "-1" (PhysX automatic selection),
                "0,1,...,N-1" (all GPUs round-robin), "1,2,...,N-1" (skip first).
                Lists are normalized into ascending ordinal order. Input order
                does not control scene rotation.
                Any non-empty value overrides config.scene_multi_gpu_mode. A single
                ordinal disables multi-GPU scene distribution.
                A different deterministic ordinal after the first GPU scene requires
                a new process.

        To force process-wide CPU-only mode, call
        ``set_global_config(GlobalConfigKey.CPU_ONLY, True)`` before creating
        any PhysX instances.

        Raises:
            RuntimeError: If construction is attempted from a native log
                callback or while final shutdown is draining callbacks.
                Concurrent construction during initialization waits, then
                shares success or retries a known failure. Construction rejects
                during ambiguous rollback or final shutdown.
        """
        self._lib = _lib
        self._lifecycle_acquired = False
        # Keepalive for an attached ovstage Stage. ovphysx dereferences the raw
        # ovstage_instance_t* until detach, so this reference keeps the Stage and
        # its native instance alive for the duration of the attachment. Set in
        # attach_ovstage(), cleared in detach_ovstage()/destroy().
        self._attached_ovstage = None
        # The Newton schema check in attach_ovstage() runs in Python (the C API cannot
        # locate the package), so its Carbonite override is read from the config here.
        overrides = getattr(config, "carbonite_overrides", None) or {}
        self._warn_missing_newton_schema = not _carbonite_override_is_false(
            overrides.get(NEWTON_SCHEMA_WARNING_SETTING, True)
        )

        if not ignore_version_mismatch:
            _check_version_match()

        legacy_args = None
        config_arr = None
        if config is not None:
            warnings.warn(
                "PhysX(config=...) is deprecated; use ovphysx.set_global_config() "
                "for settings with a GlobalConfigKey before constructing PhysX. "
                "Carbonite-only settings still use PhysXConfig.carbonite_overrides.",
                DeprecationWarning,
                stacklevel=2,
            )
            legacy_args = ovphysx_create_args()
            legacy_args.active_cuda_gpus = ovphysx_string_t(active_cuda_gpus or "")
            legacy_args.bundled_deps_path = ovphysx_string_t()
            from .config import _to_c_config

            all_entries = _to_c_config(config)
            if all_entries:
                config_arr = (ovphysx_config_entry_t * len(all_entries))(*all_entries)
                legacy_args.config_entries = cast(config_arr, POINTER(ovphysx_config_entry_t))
                legacy_args.config_entry_count = len(all_entries)

        self._omni_physx_sdk_handle = c_uint64(_INVALID_HANDLE)
        # Read sessions whose refcount hit zero on a non-owning (GC) thread. Their native teardown
        # is deferred and drained here on the owning thread (see _drain_pending_read_releases), so a
        # Warp deleter never calls ovphysx_release_* off-thread against this not-thread-safe instance.
        self._pending_read_releases: "list[_ReadRelease]" = []
        self._pending_read_lock = threading.Lock()
        # Write sessions abandoned without close() and finalized on a GC thread. Same hazard, same
        # remedy as the read queue: __del__ enqueues, the owning thread drains (folded into
        # _drain_pending_read_releases so every owning-thread entry point retires both).
        self._pending_write_releases: "list[_WriteRelease]" = []
        self._pending_write_lock = threading.Lock()
        self._object_change_keepalive: dict[int, list] = {}
        # Weakly-held set of live read holders, so stage teardown can release path metadata and
        # destroy() rejects while an aliased CPU or CUDA array still owns native storage.
        self._live_read_holders: "weakref.WeakSet[_ReadRelease]" = weakref.WeakSet()
        self._write_generation = 0
        try:
            _acquire_process_lifecycle(self)
        except BaseException:
            if self._lifecycle_acquired:
                self._lifecycle_acquired = False
                _release_process_lifecycle()
            raise

        try:
            if legacy_args is not None:
                result = self._lib.ovphysx_create_instance_with_args(
                    byref(legacy_args), byref(self._omni_physx_sdk_handle)
                )
            else:
                result = self._lib.ovphysx_create_instance(byref(self._omni_physx_sdk_handle))
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._get_last_error()
                raise RuntimeError(f"ovphysx_create_instance() failed: {error_msg}")
            if self._omni_physx_sdk_handle.value == _INVALID_HANDLE:
                self._omni_physx_sdk_handle = None
                raise RuntimeError("ovphysx_create_instance() returned invalid handle")
            if active_cuda_gpus is not None and legacy_args is None:
                self.set_config(ConfigKey.ACTIVE_CUDA_GPUS, active_cuda_gpus)
        except BaseException:
            if (
                self._omni_physx_sdk_handle is not None
                and self._omni_physx_sdk_handle.value != _INVALID_HANDLE
            ):
                self._lib.ovphysx_destroy_instance(self._omni_physx_sdk_handle.value)
                self._omni_physx_sdk_handle = None
            if self._lifecycle_acquired:
                self._lifecycle_acquired = False
                _release_process_lifecycle()
            raise

        # Track explicit-destruction state for the ResourceWarning finalizer.
        self._released = False
        _track_process_instance(self)

    def _check_valid(self) -> None:
        if self._omni_physx_sdk_handle is None:
            raise RuntimeError("PhysX instance has been destroyed. Create a new PhysX() instance.")

    @property
    def handle(self) -> int:
        """The raw ``ovphysx_instance_handle_t`` for this instance (read-only).

        Use this when passing the handle to C/C++ code that calls the
        ovphysx C API directly.

        Raises:
            RuntimeError: If the instance has been destroyed.
        """
        self._check_valid()
        return self._omni_physx_sdk_handle.value

    @staticmethod
    def set_cpu_mode(cpu_only: bool) -> None:
        """Compatibility wrapper for ``GlobalConfigKey.CPU_ONLY``.

        Call before the first PhysX instance is ever created to keep ovphysx's
        own code from touching CUDA. The call requires no active instances. Once
        set to True successfully, the mode cannot be reversed for this process.

        When called before the first instance, True prevents CUDA driver use by
        ovphysx and makes all PhysX scenes use CPU dynamics regardless of their
        USD physxScene:enableGPUDynamics settings. Other libraries in the process
        may still open the driver. A call after an earlier instance was destroyed
        may succeed, but cannot provide ovphysx's no-CUDA-touch guarantee or
        retarget an already-bootstrapped runtime.
        For CPU-only deployments, setting OVPHYSX_DISABLE_GPU before ovphysx
        initialization provides the equivalent process-wide policy.

        Raises:
            RuntimeError: If any PhysX instances are currently active, or if
                attempting to set False after True has been applied (CPU-only
                mode is sticky as soon as enabling it succeeds).
        """
        result = _lib.ovphysx_set_cpu_mode(cpu_only)
        if result.status != 0:
            raise RuntimeError(
                f"set_cpu_mode failed ({ApiStatus(result.status).name}): " f"{_get_last_error_from_lib()}"
            )

    @staticmethod
    def get_cpu_mode() -> bool:
        """Compatibility wrapper returning effective hard CPU-only mode.

        True when ``PhysX.set_cpu_mode(True)`` has succeeded, or when
        ``OVPHYSX_DISABLE_GPU`` is active. The environment variable is read
        live before ``ovphysx_initialize`` (and again after shutdown until the
        next initialize). Initialize latches it for that interval. This is not
        a query of per-scene USD ``physxScene:enableGPUDynamics``, a CUDA
        ordinal (``active_cuda_gpus``), or attach-time resolved dynamics.

        Callable at any time. No PhysX instance is required.
        """
        out = ctypes.c_bool(False)
        result = _lib.ovphysx_get_cpu_mode(ctypes.byref(out))
        if result.status != 0:
            raise RuntimeError(
                f"get_cpu_mode failed ({ApiStatus(result.status).name}): " f"{_get_last_error_from_lib()}"
            )
        return bool(out.value)

    @staticmethod
    def _ovx_to_str(s: ovphysx_string_t) -> str:
        if not s.ptr or s.length == 0:
            return ""
        try:
            return ctypes.string_at(s.ptr, s.length).decode("utf-8", errors="replace")
        except Exception:
            return ""

    def _get_last_error(self) -> str:
        """Get the last error message from TLS."""
        err = self._lib.ovphysx_get_last_error()
        if err and err.ptr:
            try:
                return self._ovx_to_str(err) or "Unknown error"
            except Exception:
                return "Unknown error"
        return "Unknown error"

    def clone(self, source_path: str, target_paths: list[str],
              anchor_transforms: list[tuple[float, float, float, float, float, float, float]] | None = None,
              env_ids: list[int] | None = None) -> Task:
        """Clone a prim hierarchy to create multiple runtime physics copies.

        Creates physics-optimized clones in the runtime representation for high-performance
        simulation, backed by the PhysX SDK replicator so cloned articulations are real articulations.
        The source prim must exist in the loaded stage and have physics properties.
        Replication executes inline. The returned Task is already complete.
        On success subsequent operations see the clone immediately. Accepted
        execution failures are reported by ``wait_task()`` or ``wait_all()``;
        the receipt's SUCCESS status describes admission, not execution.

        This is the clone entrypoint for both standalone callers and callers that
        populate the scene through an ovstage Stage attached via
        :meth:`attach_ovstage`. Replication runs in the internal representation
        only (USD untouched).

        Cross-environment collision filtering can optionally use PhysX environment ids, controlled
        by the ``/ovphysx/clone/useEnvIds`` setting (default: on). When enabled and the scene runs
        GPU dynamics + GPU broadphase, each cloned environment gets a distinct environment id so
        copies in different environments do not collide. The source environment is included: its
        bodies are created holding environment id 0 (assigned as the attach parses them; clones
        get 1..N), so co-located clones (``anchor_transforms=None``) are collision-isolated from
        the source as well. Environment ids do not provide collision isolation in CPU mode.
        Give CPU clones spatially disjoint ``anchor_transforms``. Otherwise all copies share one
        collision space. The runtime logs a warning when env ids are requested but GPU dynamics
        or GPU broadphase is unavailable. Call :func:`ovphysx.enable_python_logging` to receive it
        on the ``ovphysx`` Python logger. Like all carbonite settings, ``useEnvIds`` is
        **per-process** (shared by every ovphysx instance in the process), so set it consistently
        before attaching.

        When one logical environment is assembled from SEVERAL clone calls (e.g. an IsaacLab
        ClonePlan cloning one source row at a time: first ``/env0/Robot`` to every environment,
        then ``/env0/Object``), pass ``env_ids`` so objects that share an environment share an
        environment id. With ``env_ids=None`` each call numbers its copies afresh, so
        ``/env1/Robot`` and ``/env1/Object`` cloned by different calls would land on different
        ids and never collide with each other::

            def clone_environment_parts(physx):
                env_ids = [0, 1]  # same ids in every call -> same environments
                physx.clone("/env0/Robot", ["/env1/Robot", "/env2/Robot"], env_ids=env_ids)
                physx.clone("/env0/Object", ["/env1/Object", "/env2/Object"], env_ids=env_ids)
                physx.wait_all()

        Args:
            source_path: USD path of the source prim hierarchy to clone (e.g., "/World/env0")
            target_paths: Runtime physics-object paths for the cloned hierarchies
                (e.g., ["/World/env1", "/World/env2"])
            anchor_transforms: Optional list of (px, py, pz, qx, qy, qz, qw) transforms
                giving the absolute world pose of each target subtree root. Entry i anchors
                the exact subtree at target_paths[i]. Position is followed by quaternion
                rotation (imaginary-first, matching tensor API convention). Identity
                rotation = (0, 0, 0, 1). Must have the same length as target_paths.
                Descendants keep their poses relative to the source subtree root:
                target_object_world = anchor_transforms[i] * inverse(source_root_world) *
                source_object_world. Pass None to co-locate every copy on the source. Co-location
                is collision-isolated only under GPU dynamics + GPU broadphase. In CPU mode,
                provide spatially disjoint transforms to avoid cross-environment collisions.
            env_ids: Optional logical environment id per target (list of int, same length
                as target_paths, each 0 <= id < 0x00FFFFFF, because PhysX supports at most
                1<<24 environments and the runtime id is env_ids[i] + 1).  Stable across calls: the
                same id always maps to the same runtime environment, so clones from
                different calls that share an id collide with each other and stay
                isolated from every other environment (engages under GPU dynamics + GPU
                broadphase, like all env-id filtering).  Pass None for automatic
                per-call numbering (each call's copies get fresh ids past every
                previous call's).

        Returns:
            Immutable Task receipt for use with :meth:`wait_task`.

        Raises:
            ValueError: If paths are invalid, ``anchor_transforms`` has the wrong
                length or contains an invalid pose, or ``env_ids`` has the wrong
                length or contains an invalid id.
            RuntimeError: If clone fails to queue, if no ovstage Stage is attached,
                or if clone() is called after :meth:`warmup` or the first
                :meth:`step` / :meth:`step_sync`. Cloning after warmup
                corrupts simulation state on GPU and is rejected in all modes
                for API consistency. To recover, call :meth:`detach_ovstage`, wait
                for it to complete, then reload the source scene or reattach
                its ovstage before cloning again.

        Preconditions:
            - An ovstage Stage is attached and source_path names physics to clone.
            - target_paths are unique and do not already exist.
            - :meth:`warmup` has not been called and no :meth:`step` /
              :meth:`step_sync` has run since the current stage was attached.
        Side effects:
            - Creates live PhysX objects keyed by the target paths. No USD or runtime-stage
              prims are authored.
        Ownership/Lifetime:
            - Clones remain valid until detach_ovstage().
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises ValueError for invalid inputs.
            - Raises RuntimeError when submission is rejected, including
              duplicate-target and after-step/after-warmup ordering violations.
            - Accepted execution failures raise TaskError when collected by a wait.
        """
        self._check_valid()
        if not source_path:
            raise ValueError("source_path must be a non-empty string")
        if not target_paths:
            raise ValueError("target_paths must be a non-empty list")
        if source_path in target_paths:
            raise ValueError(f"Target path cannot be the same as source path '{source_path}'")

        num_targets = len(target_paths)
        target_array = (ovphysx_string_t * num_targets)()
        for i, path in enumerate(target_paths):
            target_array[i] = ovphysx_string_t(path)

        # Pack the target-root world transforms into a flat float array [N*7].
        if anchor_transforms is not None:
            if len(anchor_transforms) != num_targets:
                raise ValueError(
                    f"anchor_transforms length ({len(anchor_transforms)}) "
                    f"must match target_paths length ({num_targets})"
                )
            # Each entry must be exactly 7 finite numeric values (px,py,pz,qx,qy,qz,qw): the native
            # path reads 7 per target, so a short entry would read past this buffer and a long one
            # would shift every later target's pose. Validate before allocating the ctypes array.
            xform_flat = []
            for i, entry in enumerate(anchor_transforms):
                vals = list(entry)
                if len(vals) != 7:
                    raise ValueError(
                        f"anchor_transforms[{i}] must have exactly 7 values " f"(px,py,pz,qx,qy,qz,qw), got {len(vals)}"
                    )
                try:
                    fvals = [float(v) for v in vals]
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"anchor_transforms[{i}] must contain numeric values") from exc
                if not all(math.isfinite(fv) for fv in fvals):
                    raise ValueError(f"anchor_transforms[{i}] values must be finite")
                xform_flat.extend(fvals)
            xform_array = (c_float * len(xform_flat))(*xform_flat)
            xform_ptr = ctypes.cast(xform_array, POINTER(c_float))
        else:
            xform_ptr = None

        # Pack optional logical env ids into a uint32 array [N]. The native path reads N entries
        # and maps each to a runtime environment id (env_ids[i] + 1), so validate length and range
        # before allocating the ctypes array.
        if env_ids is not None:
            if len(env_ids) != num_targets:
                raise ValueError(f"env_ids length ({len(env_ids)}) must match target_paths length ({num_targets})")
            ids = []
            for i, entry in enumerate(env_ids):
                try:
                    iv = operator.index(entry)
                except TypeError as exc:
                    raise ValueError(f"env_ids[{i}] must be an integer") from exc
                if iv < 0 or iv >= 0x00FFFFFF:
                    raise ValueError(f"env_ids[{i}] must be in [0, 0x00FFFFFF), got {iv}")
                ids.append(iv)
            env_id_array = (c_uint32 * num_targets)(*ids)
            env_id_ptr = ctypes.cast(env_id_array, POINTER(c_uint32))
        else:
            env_id_ptr = None

        result = self._lib.ovphysx_clone(
            self._omni_physx_sdk_handle.value,
            ovphysx_string_t(source_path),
            target_array,
            c_uint32(num_targets),
            xform_ptr,
            env_id_ptr,
        )
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to clone: {error_msg}")

        return Task._from_native(result)

    def get_object_type(self, prim_path: str) -> ObjectType:
        """Classify an authored USD prim by TensorAPI object type.

        .. deprecated:: 0.6.0
            Object-type lookup is deprecated.

        See :class:`ObjectType` for the taxonomy. Paths with no classified
        simulation object return ``ObjectType.INVALID`` with success. The call
        did not fail, the path is not a known simulation object. Live
        standalone, custom, and articulation joints at their authored prim paths
        must not return ``ObjectType.INVALID``.

        Raises ``RuntimeError`` for invalid input (empty path, embedded NUL
        byte) or if no stage is attached.

        Returns:
            ObjectType: One of RIGID_BODY, ARTICULATION, ARTICULATION_LINK,
            ARTICULATION_ROOT_LINK, ARTICULATION_JOINT, JOINT, CUSTOM_JOINT, or
            INVALID.
        """
        _warn_legacy_api(
            "ovphysx object-type lookup is deprecated and will be removed in a future release."
        )
        self._check_valid()
        out = c_uint32(0)
        result = _lib.ovphysx_get_object_type(
            self._omni_physx_sdk_handle.value,
            ovphysx_string_t(prim_path),
            ctypes.byref(out),
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"get_object_type failed: {self._get_last_error()}")
        return ObjectType(out.value)

    def start_recording(self, destination: "OmniPvdDestination") -> None:
        """Start an explicit OVD recording session.

        Make the shared runtime OVD recording capable before the first
        ``PhysX()`` by calling
        ``set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)``.
        The key defaults to false; capability alone does not start output.
        Attach a physics stage before calling this method; recording may start
        before the first simulation step. Creation and attachment stay dormant.
        Any ``PhysX`` instance sharing a capable runtime may start a session
        while none is active; that instance then owns it.
        A runtime created without capability rejects
        start with ``INVALID_STATE`` and an error naming the capability key.
        Unsupported platforms retain ``NOT_IMPLEMENTED``.

        A failed validation or destination-open attempt may be retried. A start
        while recording is active is invalid and does not replace its
        destination. After stop, another FILE or TCP session may be started.
        """
        from .config import OmniPvdDestination

        self._check_valid()
        if not isinstance(destination, OmniPvdDestination):
            raise TypeError("destination must be an OmniPvdDestination")

        file_path = ovphysx_string_t(destination.file_path)
        tcp_address = ovphysx_string_t(destination.tcp_address)
        c_destination = ovphysx_omnipvd_destination_t(
            0 if destination.transport == "file" else 1,
            file_path,
            tcp_address,
            destination.tcp_port,
            destination.tcp_timeout_ms,
        )
        result = self._lib.ovphysx_ovd_start_recording(
            self._omni_physx_sdk_handle.value, byref(c_destination)
        )
        if result.status != ApiStatus.SUCCESS:
            status_name = ApiStatus(result.status).name
            raise RuntimeError(f"start_recording failed ({status_name}): {self._get_last_error()}")

    def stop_recording(self) -> None:
        """Stop and finalize the OVD recording owned by this instance.

        Peer instances cannot stop another owner's session.
        """
        self._check_valid()
        result = self._lib.ovphysx_ovd_stop_recording(self._omni_physx_sdk_handle.value)
        if result.status != ApiStatus.SUCCESS:
            status_name = ApiStatus(result.status).name
            raise RuntimeError(f"stop_recording failed ({status_name}): {self._get_last_error()}")

    def is_recording(self) -> bool:
        """Return ``True`` only while this instance owns active sampling.

        A peer reports ``False`` while another instance owns a session.
        """
        self._check_valid()
        recording = ctypes.c_bool(False)
        result = self._lib.ovphysx_ovd_is_recording(
            self._omni_physx_sdk_handle.value, byref(recording)
        )
        if result.status != ApiStatus.SUCCESS:
            status_name = ApiStatus(result.status).name
            raise RuntimeError(f"is_recording failed ({status_name}): {self._get_last_error()}")
        return bool(recording.value)

    def step(self, dt: float) -> Task:
        """Initiate a physics step and return its Task receipt.

        Simulation time is tracked internally. Each step advances it by ``dt``.
        The timestep is passed through without clamping or substepping; choose
        a timestep appropriate for the simulation.

        A ``dt`` of ``0.0`` is accepted but simulates nothing: it advances no
        simulation time and leaves all state, including the contacts and
        timestep that contact reads use, exactly as the previous step left them.

        Args:
            dt: Finite, nonnegative delta time in seconds.

        Returns:
            Immutable Task receipt for use with :meth:`wait_task`.

        Example::

            def step_and_collect(physx):
                task = physx.step(0.016)
                physx.wait_task(task)

        Preconditions:
            - An ovstage Stage is attached to this instance.
        Side effects:
            - Advances simulation time and mutates physics state.
        Ownership/Lifetime:
            - The receipt owns no simulation resources. A terminal outcome is
              collected once by wait_task() or wait_all().
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises RuntimeError on failure to enqueue.
            - Accepted execution failures raise TaskError when collected by a wait.
        """
        self._check_valid()
        self._drain_pending_read_releases()  # owning-thread drain of any deferred read teardown
        result = self._lib.ovphysx_step(self._omni_physx_sdk_handle.value, c_float(dt))

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to step: {error_msg}")

        return Task._from_native(result)

    def step_sync(self, dt: float) -> None:
        """Step simulation and wait for completion in a single call.

        Faster than ``step()`` + ``wait_task()`` for performance-critical
        applications like RL training that always wait immediately.
        Simulation time is tracked internally and advanced by ``dt``.

        A ``dt`` of ``0.0`` simulates nothing, exactly as in :meth:`step`.

        Args:
            dt: Finite, nonnegative delta time [s], passed through as in :meth:`step`.

        Raises:
            RuntimeError: If the step or wait fails.
        """
        self._check_valid()
        self._drain_pending_read_releases()
        result = self._lib.ovphysx_step_sync(self._omni_physx_sdk_handle.value, c_float(dt))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"step_sync failed: {error_msg}")

    def update_articulations_kinematic(
        self,
        *,
        flags: ArticulationKinematicFlag = (
            ArticulationKinematicFlag.POSITION | ArticulationKinematicFlag.VELOCITY
        ),
        scope: ObjectScope = ObjectScope.ALL,
        prim_paths: "list[int] | None" = None,
        pattern: "str | None" = None,
    ) -> None:
        """Update selected articulation link state from current generalized state.

        The articulations updated are the members of an ``ARTICULATION`` output query: every
        articulation in ``scope``, or only those a ``prim_paths`` / ``pattern`` selection names, with the
        same selection rules as :meth:`query`. Articulations outside the selection are left unchanged.
        This waits for the query and forward-kinematics tasks and raises any execution failure. It
        returns None after updating link poses and/or velocities without running a normal simulation
        step, collision detection, or contact generation.

        In GPU mode, the first kinematic update after loading USD may perform
        the same automatic DirectGPU warmup step used by tensor reads/writes.

        Args:
            flags: POSITION, VELOCITY, or both. Zero validates the query and does
                no update.
            scope: ALL articulations or the current ACTIVE articulation set. A selection has no scope,
                so combining ``prim_paths`` or ``pattern`` with ACTIVE raises ``ValueError``.
            prim_paths: Optional native ``ovx_primpath_t`` handles naming the articulations to update, from
                the attached Stage's path dictionary as for :meth:`query`. A handle of an articulation's link
                or joint selects that articulation. Takes precedence over ``pattern``.
            pattern: Optional absolute path pattern naming the articulations to update, matched as in
                :meth:`query`; the complete rules are on the "Path patterns" page of the ovphysx
                documentation.

        Raises:
            ValueError: If a selection is combined with ``scope=ACTIVE``, or ``pattern`` is empty.
            RuntimeError: If the update fails.
        """
        self._check_valid()
        self._drain_pending_read_releases()
        handle = self._omni_physx_sdk_handle.value
        query = self._open_query(
            handle, SimObjectType.ARTICULATION, scope, prim_paths, pattern, operation="articulation update query"
        )

        def release_query() -> None:
            release = self._lib.ovphysx_release_query(handle, query)
            _wait_for_task(self, release, "articulation update query release")

        try:
            result = self._lib.ovphysx_update_articulations_kinematic(
                handle, query, c_uint32(int(flags))
            )
            _wait_for_task(self, result, "update_articulations_kinematic")
        except Exception as error:
            try:
                release_query()
            except Exception as cleanup_error:
                _chain_cleanup(error, cleanup_error)
                raise error
            raise
        release_query()

    def evaluate_sdf_shapes(
        self,
        points,
        out,
        *,
        prim_paths: "list[int] | None" = None,
        pattern: "str | None" = None,
        cuda_stream: int = 0,
        cuda_wait_event: int = 0,
    ) -> None:
        """Evaluate the signed distance fields of SDF collision shapes at the given points.

        The successor of :meth:`create_sdf_view` and :meth:`SdfView.evaluate`. The shapes are the members of an
        ``SDF_SHAPE`` output query: the collision shapes the SDF view accepts, a collider carrying
        PhysxSDFMeshCollisionAPI whose shape is a triangle mesh with an SDF, runtime clones included. Rows follow
        the selection: ``prim_paths`` in the order given, ``pattern`` in the same match order as
        :meth:`create_sdf_view`; with neither, every SDF shape, in the order the runtime stores them. A selected
        prim that is not an SDF shape is skipped, as the SDF view skipped it.

        For each row ``i`` and point ``j``, ``out[i, j]`` receives the gradient (x, y, z) and then the signed
        distance at ``points[i, j]``, negative inside. Points and gradients are in each shape's mesh-local space
        (the unscaled frame of the mesh's vertices); no transform is applied. Q, the points per shape, is read
        from the arrays on every call. A point outside the shape's SDF grid (the mesh bounds padded by the SDF
        margin) is evaluated, not refused, as by the SDF view: its distance is the SDF value at the nearest grid
        point plus the distance to the grid, a positive upper bound, and its gradient points away from the
        shape. This waits for the evaluation, so ``out`` is complete when it returns.

        Requires a GPU instance (DirectGPU). The first evaluation after loading USD may perform the automatic
        DirectGPU warmup step, as the SDF view did. After that it never steps: the scene must have simulated
        since physics objects were last created or released, as PhysX uploads shape data to the GPU only when
        it simulates, so step once after such a change.

        Args:
            points: DLPack-compatible float32 array of shape [N, Q, 3] on the CUDA device, N the number of
                selected SDF shapes.
            out: Pre-allocated DLPack-compatible float32 array of shape [N, Q, 4] on the same device.
            prim_paths: Optional native ``ovx_primpath_t`` handles naming the shapes, from the attached Stage's
                path dictionary as for :meth:`query`. Takes precedence over ``pattern``.
            pattern: Optional absolute path pattern naming the shapes, matched as in :meth:`query`; the
                complete rules are on the "Path patterns" page of the ovphysx documentation.
            cuda_stream: Producer stream of ``points`` still writing them, drained before they are read, as for
                :meth:`WriteSession.commit`. Warp arrays and CUDA DLPack producers are synchronized already.
                Stream 1 names the legacy default stream of the current CUDA context.
            cuda_wait_event: Producer completion event of ``points``, waited on before they are read. ``out``
                must have no outstanding device reads or writes when the call is made, unless that work is on
                ``cuda_stream`` or before ``cuda_wait_event``; nothing else may use ``out`` until it returns.

        Raises:
            ValueError: If ``pattern`` is empty, or an array is not C-contiguous.
            RuntimeError: If an array has the wrong dtype, shape or device, N is not the number of selected SDF
                shapes (the message states it) or exceeds 65535, N x Q exceeds 2^31 - 1, the instance is not
                DirectGPU, the scene has not simulated since physics objects last changed, the producer handoff
                fails, or the evaluation fails.
        """
        from ._dlpack_utils import acquire_dltensor

        self._check_valid()
        self._drain_pending_read_releases()
        handle = self._omni_physx_sdk_handle.value
        points_dl, points_keepalive = acquire_dltensor(points)
        out_dl, out_keepalive = acquire_dltensor(out)
        # Every argument is converted before the query opens: nothing after it may raise outside the release below.
        sync = ovstage_cuda_sync_t()
        sync.stream = int(cuda_stream)
        sync.wait_event = int(cuda_wait_event)
        query = self._open_query(
            handle, SimObjectType.SDF_SHAPE, ObjectScope.ALL, prim_paths, pattern, operation="SDF shape query"
        )

        def release_query() -> None:
            release = self._lib.ovphysx_release_query(handle, query)
            _wait_for_task(self, release, "SDF shape query release")

        try:
            result = self._lib.ovphysx_evaluate_sdf_shapes(
                handle, query, ctypes.byref(points_dl), ctypes.byref(out_dl), sync
            )
            _wait_for_task(self, result, "evaluate_sdf_shapes")
        except Exception as error:
            try:
                release_query()
            except Exception as cleanup_error:
                _chain_cleanup(error, cleanup_error)
                raise error
            raise
        finally:
            del points_keepalive, out_keepalive
        release_query()

    def wait_task(self, task: Task, *, timeout_ns: int | None = None) -> None:
        """Wait for and collect only the supplied task's terminal outcome.

        Args:
            task: Task receipt returned by this instance.
            timeout_ns: Readiness timeout in nanoseconds. None waits indefinitely;
                0 polls once. Finalization after readiness can exceed the timeout.

        Returns:
            None on success.

        Raises:
            TypeError: If task is not a Task.
            ValueError: If timeout_ns is outside 0 through 2**64 - 1.
            TaskError: If this task failed. The exception owns copied error text.
            TaskTimeoutError: If the task is unfinished at the deadline. Its
                pending_task remains available for another wait.
            RuntimeError: If the instance or receipt is invalid, the task was
                already collected, or the wait itself fails.

        Waiting for B never reports or collects A's failure. A completed outcome
        can be collected once; internal synchronization does not collect it.
        Serialize calls on each instance externally.

        Example::

            from ovphysx import TaskTimeoutError

            def poll_task(physx, task):
                try:
                    physx.wait_task(task, timeout_ns=0)
                except TaskTimeoutError:
                    return False
                return True
        """
        if not isinstance(task, Task):
            raise TypeError("task must be a Task")
        if timeout_ns is not None and (timeout_ns < 0 or timeout_ns > (1 << 64) - 1):
            raise ValueError("timeout_ns must be in range 0 through 2**64 - 1")
        self._check_valid()
        wait_result = ovphysx_wait_task_result_t()
        primary = None
        try:
            result = self._lib.ovphysx_wait_task(
                self._omni_physx_sdk_handle.value, task._as_native(),
                0xFFFFFFFFFFFFFFFF if timeout_ns is None else timeout_ns, byref(wait_result),
            )
            self._check_wait_result(result.status, wait_result, "wait_task", task=task)
        except BaseException as error:
            primary = error
            raise
        finally:
            try:
                _run_cleanups((
                    lambda: self._lib.ovphysx_destroy_wait_task_result(byref(wait_result)),
                    lambda: _drain_operation_cleanups(self),
                ), primary=primary)
            finally:
                primary = None

    def wait_all(self, *, timeout_ns: int | None = None) -> None:
        """Collect uncollected tasks accepted before this call began.

        Tasks are collected in submission order with one readiness deadline.
        Completed successes and failures are collected once. A timeout stops at
        the earliest unfinished task, leaving it and later tasks uncollected.
        Failures remain reportable even when their original receipts were discarded.

        Args:
            timeout_ns: Readiness timeout in nanoseconds, with the same semantics
                as :meth:`wait_task`. None waits indefinitely; 0 polls once.

        Returns:
            None on success, including an empty or already-collected task set.

        Raises:
            ValueError: If timeout_ns is outside 0 through 2**64 - 1.
            TaskError: If collected tasks failed. The errors property contains
                immutable (Task, message) entries with copied text.
            TaskTimeoutError: If work remains unfinished at the deadline, even
                when earlier tasks failed. The errors property contains those
                earlier failures; pending_task identifies the earliest unfinished
                task. Collected failures are not repeated by later waits.
            RuntimeError: If the instance is invalid or the wait itself fails.

        Serialize calls on each instance externally.
        """
        if timeout_ns is not None and (timeout_ns < 0 or timeout_ns > (1 << 64) - 1):
            raise ValueError("timeout_ns must be in range 0 through 2**64 - 1")
        self._check_valid()
        _run_cleanups((self._drain_pending_read_releases, lambda: self._wait_all_tasks(timeout_ns)))

    def _wait_all_tasks(self, timeout_ns=None) -> None:
        # Teardown can collect already-admitted work even if a cleanup drain rejects.
        snapshot = _registered_tasks(self)
        wait_result = ovphysx_wait_task_result_t()
        primary = None
        try:
            result = self._lib.ovphysx_wait_all(
                self._omni_physx_sdk_handle.value,
                0xFFFFFFFFFFFFFFFF if timeout_ns is None else timeout_ns, byref(wait_result),
            )
            self._check_wait_result(result.status, wait_result, "wait_all", snapshot=snapshot)
        except BaseException as error:
            primary = error
            raise
        finally:
            try:
                _run_cleanups((
                    lambda: self._lib.ovphysx_destroy_wait_task_result(byref(wait_result)),
                    lambda: _drain_operation_cleanups(self),
                ), primary=primary)
            finally:
                primary = None

    def _check_wait_result(self, status: int, wait_result: ovphysx_wait_task_result_t, operation: str,
                           *, task=None, snapshot=()) -> None:
        # Copy all native data while the result still owns its receipts and text.
        errors = tuple(
            (
                Task._from_native(wait_result.error_tasks[i]),
                self._ovx_to_str(self._lib.ovphysx_get_last_task_error(
                    self._omni_physx_sdk_handle.value, wait_result.error_tasks[i],
                )) or "Unknown error",
            )
            for i in range(wait_result.num_errors)
        )
        error_map = {(int(receipt.instance), int(receipt.op_index)): message for receipt, message in errors}
        terminal = status == ApiStatus.SUCCESS or bool(errors)
        if task is not None:
            keys = {(int(task.instance), int(task.op_index))} if terminal and status != ApiStatus.TIMEOUT else set()
        elif status == ApiStatus.TIMEOUT:
            pending = int(wait_result.pending_task.op_index)
            keys = {key for key in snapshot if key[1] < pending}
        else:
            keys = set(snapshot) if terminal else set()
        _dispatch_operations(self, keys, error_map)
        if status == ApiStatus.TIMEOUT:
            raise TaskTimeoutError(errors, Task._from_native(wait_result.pending_task))
        if errors:
            raise TaskError(errors)
        if status != ApiStatus.SUCCESS:
            raise RuntimeError(f"{operation} failed: {self._get_last_error()}")

    def subscribe_object_changes(
        self,
        *,
        on_created=None,
        on_destroyed=None,
        on_all_destroyed=None,
    ) -> int:
        """Subscribe this instance to PhysX object create/destroy notifications.

        Returns a subscription id for :meth:`unsubscribe_object_changes`.
        Callbacks fire only while this instance's own attachment is live.
        """
        self._check_valid()
        keep = []
        callbacks = ovphysx_object_change_callbacks_t()
        if on_created is not None:
            created_cb = on_created

            def _created(path, physx_type, _user_data):
                created_cb(str(path), int(physx_type))

            fn = ovphysx_object_created_fn(_created)
            callbacks.on_object_created = fn
            keep.append(fn)
        if on_destroyed is not None:
            destroyed_cb = on_destroyed

            def _destroyed(path, physx_type, _user_data):
                destroyed_cb(str(path), int(physx_type))

            fn = ovphysx_object_destroyed_fn(_destroyed)
            callbacks.on_object_destroyed = fn
            keep.append(fn)
        if on_all_destroyed is not None:
            all_cb = on_all_destroyed

            def _all(_user_data):
                all_cb()

            fn = ovphysx_all_objects_destroyed_fn(_all)
            callbacks.on_all_objects_destroyed = fn
            keep.append(fn)

        sub = c_uint64(0)
        result = self._lib.ovphysx_subscribe_object_changes(
            self._omni_physx_sdk_handle.value, byref(callbacks), byref(sub)
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"subscribe_object_changes failed: {self._get_last_error()}")
        sub_id = int(sub.value)
        self._object_change_keepalive[sub_id] = keep
        return sub_id

    def unsubscribe_object_changes(self, subscription_id: int) -> None:
        """Stop object-change notifications for a subscription this instance owns.

        Returns only after in-flight callbacks for that subscription have finished.
        Calling it from inside any object-change callback raises
        ``RuntimeError`` and leaves the subscription active. That check follows
        the Carbonite task when the callback is running as one, and the native
        thread otherwise.
        """
        self._check_valid()
        result = self._lib.ovphysx_unsubscribe_object_changes(
            self._omni_physx_sdk_handle.value, c_uint64(subscription_id)
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"unsubscribe_object_changes failed: {self._get_last_error()}")
        self._object_change_keepalive.pop(int(subscription_id), None)

    def attach_ovstage(self, stage, *, read_ordinal: int = 1) -> Task:
        """Attach an ovstage Stage and return its Task receipt.

        Attach performs the initial scene parse at ``read_ordinal``. After the
        producer authors later ovstage edits, call :meth:`update_from_ovstage`
        with only those subsequent ordinals. Tensor bindings remain available
        as a perf escape hatch.

        Execution currently runs inline. Collect the returned Task with
        :meth:`wait_task` or :meth:`wait_all` before using the attachment;
        accepted execution failures are reported by that wait.

        Args:
            stage: An owner-aware ``ovstage.Stage``; raw pointers are a low-level C integration.
            read_ordinal: Caller-owned ovstage ordinal at which selected physics
                data is sealed. Must be non-zero. 0 is reserved as the runtime
                skip-cursor sentinel. The application owns ordinal advancement.
                Defaults to 1. ``open_usd()`` / population does not seal data,
                so call ``advance_write_floor()`` first. Attachment fails if the
                initial articulation/joint schema scan cannot read that ordinal.

        Preconditions:
            - Instance must be valid.
            - Not already attached to a Stage.
            - No other instance is required to detach first. Several instances
              may be attached at once. This call does not detach them.
            - Selected physics data must be sealed at ``read_ordinal``.
            - ``read_ordinal`` must be non-zero.
            - The application registered the codeless PhysX schemas with
              ovstage before the first USD schema read, including population or
              export, in each process and worker, using both resource directories from the separately
              installed ``physx-usd-schemas`` provider (see the samples).
              Population drops attributes of schemas it cannot resolve. If
              ovstage reports that either ``physxSchema`` or
              ``omniUsdPhysicsDeformableSchema`` was missing when its USD schema
              registry was built, the native guard rejects admission and this
              method raises ``RuntimeError`` before returning a Task or taking
              Stage ownership. The refusal persists for the process: restart
              and register before populating or exporting. A procedural Stage
              attached while USD schema definitions remain unbuilt skips the
              family check, leaving it active for later attaches. Once population
              or export builds the definitions, even procedural Stages are checked.
              Registration another USD consumer made unobservable to ovstage
              cannot be detected. Nor can the guard identify schema families
              registered late before its first check: catching a late
              registration's ``OvstageError`` can let a later attach succeed
              with missing data. Always restart after a registration-ordering
              error; successful attach does not prove schema availability
              during population. The legacy setting
              ``/ovphysx/schemas/requireRegistration`` defaults to true; setting
              it to false with ``PhysXConfig(carbonite_overrides=...)`` permits
              attach with a warning but does not repair lost data.
            - Scenes that author Newton ``newton:*`` attributes need the Newton
              USD schema (``pip install newton-usd-schemas``) registered in the
              same call (``ovphysx.newton_schema_root()``); population drops
              those attributes otherwise. Before the native attach this call
              checks the installed package's registration once per process and
              emits a ``RuntimeWarning`` if ovstage reports late registration;
              a warning promoted to an error therefore leaves the instance
              detached. When the package is not
              installed it logs a warning on the ``ovphysx`` logger instead
              (nothing is wrong for scenes without ``newton:*`` attributes, and
              suites that promote warnings to errors stay unaffected). The
              check probes only once USD has built its
              schema definitions, including through export. A procedurally
              authored stage attached while those definitions remain unbuilt is
              left alone, so the check never registers a schema on the
              application's behalf. It recognizes the Newton
              schema family registered from any complete copy. The
              setting ``/ovphysx/schemas/warnMissingNewtonSchema = false``
              (``PhysXConfig(carbonite_overrides=...)``, ``"false"`` accepted)
              silences the check, for a registration ovstage cannot observe.
              This Newton check does not check either PhysX schema family;
              silencing it does not bypass the native PhysX registration guard.

        Lifetime:
            - ``stage`` must outlive the attachment because ovphysx captures and
              dereferences its native pointer until detach. This wrapper holds a
              reference to ``stage`` for the duration of the attachment, so a
              Stage created inline (``attach_ovstage(ovstage.Stage(...))``) stays
              alive. The reference is held from module scope, so garbage
              collection of an unreachable cycle containing this instance cannot
              finalize the Stage before the instance's teardown detaches it. The
              reference survives pending or failed teardown, and is dropped when
              a lifecycle task is collected and native ownership has ended, or
              by :meth:`destroy`.

        Returns:
            Immutable Task receipt. Its SUCCESS status means admission only.

        Errors:
            - Raises ``TypeError`` if ``stage`` is not an owner-aware
              ``ovstage.Stage``, including ``None`` or a raw native handle.
            - Propagates ``ovstage.OvstageError`` if ``stage`` was destroyed.
            - Raises ``ValueError`` if ``read_ordinal`` is 0.
            - Raises ``RuntimeError`` if this instance is already attached,
              the Stage's native handle is null, the native PhysX schema guard refuses the
              process, or submission is otherwise rejected. A schema refusal
              creates no Task and changes no attachment.
            - Accepted execution failures raise ``TaskError`` when collected.
              Failed rollback can leave an attachment; use
              :meth:`is_stage_attached` after collection to check ownership.
              A peer instance's attachment is left in place.
        """
        self._check_valid()
        # Drain sessions that no longer have an owner before attaching. Stage switching must go
        # through detach_ovstage(), which releases stage-derived handles while the old Stage is
        # still valid. A failed attach attempt must not invalidate metadata from the current Stage.
        self._drain_pending_read_releases()
        from ovstage import Stage
        if not isinstance(stage, Stage) or not callable(getattr(stage, "get_path_dictionary", None)):
            raise TypeError("attach_ovstage requires an owner-aware ovstage.Stage, not a raw native handle")
        stage.get_path_dictionary()
        ptr = ctypes.cast(stage._inst, ctypes.c_void_p).value
        if not ptr:
            raise RuntimeError("attach_ovstage: stage handle is null")
        if int(read_ordinal) == 0:
            raise ValueError("attach_ovstage: read_ordinal must be a caller-owned sealed ordinal; 0 is reserved")
        # Before the native attach: a RuntimeWarning promoted to an error must propagate
        # from an instance that holds no native reference to the stage.
        if self._warn_missing_newton_schema:
            _warn_if_newton_schema_unregistered()
        result = self._lib.ovphysx_attach_ovstage(
            self._omni_physx_sdk_handle.value, ctypes.c_void_p(ptr), c_uint64(read_ordinal)
        )
        task = self._admit_session_task(result, "attach_ovstage")
        # Keep the Stage alive for the duration of the attachment: ovphysx retains
        # its raw native pointer until detach. Without this, a caller that does not
        # retain its own reference would have the Stage GC'd (its __del__ destroys
        # the native instance), and later stage-dependent calls would use freed memory.
        self._attached_ovstage = stage
        self._retain_stage_until_collection(task)
        return task

    def update_from_ovstage(self, from_ordinal: int, to_ordinal: int) -> Task:
        """Submit committed edits over ``[from_ordinal, to_ordinal]`` and return a Task.

        The application that writes to ovstage owns the ordinal range and calls
        this after sealing the writes. ``population.apply_usd_changes()`` waits
        for population work but does not seal its ordinal; complete
        ``stage.advance_write_floor(ordinal).wait()`` before this call. ovphysx
        forwards the range (as ovstage's own ``ovstage_ordinal_range_t``) to the
        runtime ovstage change feed and applies the resulting deltas to the
        simulation.

        Ordinals at or below the latest successfully consumed ordinal are skipped.
        A fully consumed range succeeds as a no-op. An overlapping range applies
        only its unread suffix. :meth:`attach_ovstage` consumes its initial
        ``read_ordinal``, so replaying it does not repeat initial population
        events. Later authored and sealed population changes are applied normally.

        Execution currently runs inline. Collect the Task with :meth:`wait_task`
        or :meth:`wait_all` to observe execution failure before relying on the
        updated state. Submission errors raise ``RuntimeError`` immediately;
        accepted execution failures raise ``TaskError`` at collection. Keep the
        sealed data readable until completion.
        """
        self._check_valid()
        # The closed range must be ordered. Validate Python-side and raise ValueError
        # (the conventional "bad argument" error for this wrapper surface) before the
        # native call, which would otherwise surface the same precondition as a less
        # specific RuntimeError.
        if int(from_ordinal) > int(to_ordinal):
            raise ValueError(
                f"update_from_ovstage requires from_ordinal <= to_ordinal, got "
                f"from_ordinal={from_ordinal}, to_ordinal={to_ordinal}"
            )
        # Build ovstage's range type. has_start_ordinal=True selects the closed range [from, to].
        rng = ovstage_ordinal_range_t(
            start_ordinal=int(from_ordinal),
            end_ordinal=int(to_ordinal),
            has_start_ordinal=True,
        )
        result = self._lib.ovphysx_update_from_ovstage(self._omni_physx_sdk_handle.value, rng)
        return self._admit_session_task(result, "update_from_ovstage")

    def detach_ovstage(self) -> Task:
        """Submit Stage teardown and return its Task receipt.

        Execution currently runs inline. Earlier tasks complete before teardown,
        but their outcomes remain uncollected. Collect this Task with
        :meth:`wait_task`, or use :meth:`wait_all` to collect every pending
        outcome. Wait for successful teardown before destroying the Stage.
        Detaching an unattached instance succeeds as a no-op task. Detach does
        not restore initial poses or reset simulation time.
        Clears registered interests and output-buffer registrations, so a
        subsequent :meth:`attach_ovstage` to a different Stage starts clean.
        After detach, stage-dependent calls such as :meth:`update_from_ovstage`
        and :meth:`step` fail until a Stage is attached again. Detach invalidates
        the stage's tensor, contact, and SDF views. Do not read, write, or
        evaluate existing bindings or SDF views. Destroy them and create
        replacements after calling :meth:`attach_ovstage` and realizing a stage
        again. If this instance owns an active OVD recording, detach stops
        and finalizes it. Reattach stays dormant until an explicit recording
        start, which may occur before the next simulation step.

        Errors:
            - Raises ``RuntimeError`` if submission or Python cleanup is rejected.
            - Accepted execution failures raise ``TaskError`` when collected.
              Teardown may partially retire resources on failure. The Stage
              remains retained while native ownership remains attached.
        """
        self._check_valid()
        # Release Stage-owned path metadata before teardown; borrowed numeric
        # buffers remain valid until their array owners release them.
        self._drain_pending_read_releases()
        self._release_stage_bound_read_handles()
        # Native teardown can invoke callbacks after retiring write storage.
        # Invalidate before entering it so those callbacks cannot expose stale views.
        write_generation = getattr(self, "_write_generation", 0)
        self._write_generation = write_generation + 1
        result = self._lib.ovphysx_detach_ovstage(self._omni_physx_sdk_handle.value)
        if result.status != ApiStatus.SUCCESS:
            self._write_generation = write_generation
        task = self._admit_session_task(result, "detach_ovstage")
        self._retain_stage_until_collection(task)
        return task

    def _retain_stage_until_collection(self, task: Task) -> None:
        """Reconcile ownership after collection, even if the receipt is dropped."""
        self._stage_lifecycle_task = task
        sdk_ref = weakref.ref(self)

        def reconcile():
            sdk = sdk_ref()
            # An older completion must not clear a newer pending attach's owner.
            if sdk is not None and sdk._stage_lifecycle_task == task and not sdk.is_stage_attached():
                sdk._attached_ovstage = None

        # The existing pending-operation registry retains this cleanup through
        # collection without giving the immutable Task an instance reference.
        Operation(self, (task,), keepalive=(self._attached_ovstage,), cleanup=reconcile)

    def _release_stage_bound_read_handles(self) -> None:
        """Free every live read session's prim lists before the Stage they belong to goes away.

        A Warp array may legally outlive its ``ReadResult``, keeping its session alive past detach.
        That session still holds prim lists interned in the Stage's path dictionary,
        so freeing it later, after the Stage is gone, would destroy them through a dangling
        dictionary. Releasing the groups here (owning thread, Stage still attached) leaves the late
        teardown with only session-owned numeric buffers to free, which stay valid for the borrow.
        """
        _run_cleanups(holder.release_groups for holder in list(self._live_read_holders))

    def _enqueue_read_release(self, holder: "_ReadRelease") -> None:
        """Queue a read session whose refcount hit zero (may be called from any thread / GC)."""
        with self._pending_read_lock:
            self._pending_read_releases.append(holder)

    def _drain_pending_read_releases(self) -> None:
        """Perform any deferred read-session teardown on the CURRENT thread.

        Call only from the owning (serialized) thread: the read/step entry points and ReadResult
        ``__exit__``. This is where the actual ``ovphysx_release_*`` for a session freed on a GC
        thread happens, serialized with the owner's other ovphysx calls.
        """
        def release(holder):
            try:
                holder._free_native()
            finally:
                if holder.pending:
                    self._enqueue_read_release(holder)

        def release_reads():
            with self._pending_read_lock:
                pending = self._pending_read_releases
                self._pending_read_releases = []
            _run_cleanups(lambda holder=holder: release(holder) for holder in pending)

        _run_cleanups((
            lambda: _drain_operation_cleanups(self),
            release_reads,
            self._drain_pending_write_releases,
        ))

    def _enqueue_write_release(self, holder: "_WriteRelease") -> None:
        """Queue a write session abandoned without close() (may be called from any thread / GC)."""
        with self._pending_write_lock:
            self._pending_write_releases.append(holder)

    def _drain_pending_write_releases(self) -> None:
        """Perform any deferred write-session teardown on the CURRENT (owning) thread.

        The write mirror of :meth:`_drain_pending_read_releases`: where the native
        ``ovphysx_release_write`` / ``ovphysx_release_query`` for a session finalized on a GC thread
        actually happens, serialized with the owner's other ovphysx calls.
        """
        with self._pending_write_lock:
            pending = self._pending_write_releases
            self._pending_write_releases = []

        def release(holder):
            try:
                holder._free_native()
            finally:
                if holder.pending:
                    self._enqueue_write_release(holder)

        _run_cleanups(lambda holder=holder: release(holder) for holder in pending)

    def is_stage_attached(self, stage=None) -> bool:
        """Return native attachment state, including changes made by native callers.

        With ``stage`` supplied, also check that native code still holds that
        caller-owned Stage (or raw pointer). This is not a resource-validity check.
        The snapshot is synchronous and neither waits for nor collects tasks.
        Wait for a pending lifecycle task before interpreting its final state;
        a false snapshot does not permit destroying a pending attach's Stage.
        Output caches check their original attachment separately to detect same-stage reattachment.
        """
        self._check_valid()
        attached = ctypes.c_bool(False)
        pointer = None
        if stage is not None:
            if hasattr(stage, "handle"):
                pointer = stage.handle()
            elif hasattr(stage, "_inst"):
                pointer = ctypes.cast(stage._inst, ctypes.c_void_p).value
            else:
                pointer = int(stage)
            if not pointer:
                return False
        result = self._lib.ovphysx_is_stage_attached(
            self._omni_physx_sdk_handle.value, ctypes.c_void_p(pointer), byref(attached)
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Failed to query attachment state: {self._get_last_error()}")
        return bool(attached.value)

    def _is_query_valid(self, query: int) -> bool:
        """Check a retained query after draining execution without collecting outcomes."""
        self._check_valid()
        out = ctypes.c_bool(False)
        result = self._lib.ovphysx_internal_is_query_valid(
            self._omni_physx_sdk_handle.value, c_uint64(query), ctypes.byref(out)
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Query validity check failed: {self._get_last_error()}")
        return bool(out.value)

    def _capture_attachment_query(self) -> int:
        """Own a native query that becomes invalid when its original attachment ends."""
        self._check_valid()
        query = c_uint64(0)
        receipt = self._lib.ovphysx_query(
            self._omni_physx_sdk_handle.value, int(SimObjectType.RIGID_BODY),
            int(ObjectScope.ALL), byref(query), None,
        )
        try:
            _wait_for_task(self, receipt, "attachment query")
        except Exception:
            if query.value:
                self._release_attachment_query(query.value)
            raise
        return int(query.value)

    def _release_attachment_query(self, query: int, *, defer: bool = False) -> None:
        handle = getattr(self, "_omni_physx_sdk_handle", None)
        if query and handle is not None and handle.value:
            release = _WriteRelease(self, query, 0)
            if defer:
                self._enqueue_write_release(release)
                return
            try:
                Operation(self, release._free_native()).wait()
            finally:
                if release.pending:
                    self._enqueue_write_release(release)

    def _get_attachment_identity(self) -> int:
        """Read native session ownership without admitting or waiting for a task."""
        self._check_valid()
        identity = c_uint64(0)
        result = self._lib.ovphysx_internal_get_attach_handle(
            self._omni_physx_sdk_handle.value, byref(identity)
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Failed to query session ownership: {self._get_last_error()}")
        return int(identity.value)

    def _admit_session_task(self, receipt, operation) -> Task:
        if receipt.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"{operation} failed: {self._get_last_error()}")
        return Task._from_native(receipt)

    def _session_owner(self):
        stage = getattr(self, "_attached_ovstage", None)
        if stage is None:
            raise RuntimeError("session operations require an owner-aware attached ovstage.Stage")
        return stage, self._get_attachment_identity()

    def _check_session_owner(self, attachment):
        self._check_valid()
        if self._get_attachment_identity() != attachment:
            raise RuntimeError("operation belongs to an invalidated attachment")

    def _orphan_session_cleanup(self, kind, slot):
        holder = [None]
        sdk_ref = weakref.ref(self)
        def cleanup():
            sdk = sdk_ref()
            if sdk is None:
                return
            if holder[0] is None:
                if not slot.value:
                    return
                if kind == "read":
                    holder[0] = _ReadRelease(sdk, 0, slot.value)
                else:
                    holder[0] = _WriteRelease(sdk, slot.value if kind == "query" else 0,
                                              slot.value if kind == "write" else 0)
            holder[0]._free_native()
            if not holder[0].pending:
                slot.value = 0
        return cleanup

    @staticmethod
    def _session_attributes(attributes):
        if isinstance(attributes, (str, bytes)):
            raise TypeError("attributes must be a sequence of strings or tokens")
        values = tuple(attributes)
        array = (ovx_string_or_token_t * len(values))()
        strings = []
        for index, value in enumerate(values):
            if isinstance(value, str):
                encoded = value.encode("utf-8")
                strings.append(encoded)
                array[index].string = ovx_string_t(encoded, len(encoded))
            elif isinstance(value, int) and not isinstance(value, bool) and 0 < value <= 0xFFFFFFFFFFFFFFFF:
                array[index].token = value
            else:
                raise TypeError(f"attribute {index} must be a string or a nonzero uint64 token")
        return values, array, strings

    def query(self, object_type: SimObjectType, *, scope: ObjectScope = ObjectScope.ALL,
              prim_paths=None, pattern=None, contacts=None, metadata: bool = True) -> Operation[Query]:
        """Submit a reusable selector. Handles and metadata require successful completion.

        Explicit prim handles take precedence over a pattern. Selections use ALL
        scope and keep their resolved membership; a whole-type ACTIVE selector is
        reevaluated when a session opens. Metadata is a creation-time snapshot;
        use metadata=False to omit optional discovery work. ``contacts`` selects
        CONTACT_SENSOR rows and optional paired filters through ContactSelection;
        it cannot be combined with prim_paths, pattern, or ACTIVE scope.
        DEBUG_RENDER is this instance's target: it needs no attached stage, and
        prim_paths, pattern, and contacts do not apply. Its scope is validated
        and then ignored.
        """
        self._check_valid()
        self._drain_pending_read_releases()
        selecting = bool(prim_paths) or pattern is not None
        if int(object_type) == int(SimObjectType.DEBUG_RENDER):
            if selecting or contacts is not None:
                raise ValueError(
                    "DEBUG_RENDER is an instance target; prim_paths, pattern, and contacts do not apply"
                )
            attachment = self._get_attachment_identity()
            stage = getattr(self, "_attached_ovstage", None) if attachment else None
        else:
            stage, attachment = self._session_owner()
        if pattern == "":
            raise ValueError("pattern must be non-empty; pass pattern=None (or omit it) for the whole set")
        if selecting and int(scope) != int(ObjectScope.ALL):
            raise ValueError(
                "prim_paths/pattern select a fixed prim set and cannot be combined with scope=ACTIVE; "
                "omit prim_paths/pattern for the ACTIVE whole-set query"
            )
        output = c_uint64(0)
        summary = ovstage_query_result_t()
        instance = self._omni_physx_sdk_handle.value
        selection = paths = contact_keepalive = None
        if contacts is not None:
            receipt, contact_keepalive = self._query_contacts(
                instance, object_type, scope, prim_paths, pattern, contacts, output,
                summary if metadata else None)
        elif selecting:
            selection = ovphysx_query_selection_t()
            if prim_paths:
                paths = (c_uint64 * len(prim_paths))(*(int(path) for path in prim_paths))
                selection.prim_paths = paths
                selection.prim_paths_count = len(prim_paths)
            else:
                selection.pattern = ovphysx_string_t(pattern)
            receipt = self._lib.ovphysx_query_selection(
                instance, int(object_type), byref(selection), byref(output), byref(summary) if metadata else None)
        else:
            receipt = self._lib.ovphysx_query(
                instance, int(object_type), int(scope), byref(output), byref(summary) if metadata else None)
        task = self._admit_session_task(receipt, "query")
        def convert():
            self._check_session_owner(attachment)
            result = QueryMetadata(int(summary.total_prim_count),
                                   tuple(int(summary.attributes[i]) for i in range(summary.attribute_count))) if metadata else None
            return Query(self, output.value, object_type, result, stage, attachment)
        return Operation(self, (task,), convert=convert, cleanup=self._orphan_session_cleanup("query", output),
                         keepalive=(output, summary, selection, paths, contact_keepalive, stage))

    def _query_contacts(self, handle, object_type, scope, prim_paths, pattern, contacts, query, summary):
        """Submit a contact selector and retain the ctypes buffers until collection."""
        if prim_paths or pattern is not None:
            raise ValueError("contacts selects the sensors itself; it cannot be combined with prim_paths/pattern")
        if int(object_type) != SimObjectType.CONTACT_SENSOR:
            raise ValueError("contacts requires object_type=SimObjectType.CONTACT_SENSOR")
        if int(scope) != ObjectScope.ALL:
            raise ValueError("a contact query has no scope; contacts cannot be combined with scope=ACTIVE")
        selections = [c if isinstance(c, ContactSelection) else ContactSelection(*c) for c in contacts]
        if not selections:
            raise ValueError("contacts must name at least one ContactSelection")
        # A str is a Sequence too: taken as filters it would be one filter per character.
        if any(isinstance(c.filters, str) for c in selections):
            raise ValueError("filters must be a sequence of selections; wrap a single pattern in a list")
        filter_count = len(selections[0].filters)
        if any(len(c.filters) != filter_count for c in selections):
            raise ValueError("every ContactSelection of one query must carry the same number of filters")

        keepalive = []

        def selection_struct(spec):
            selection = ovphysx_query_selection_t()
            if isinstance(spec, str):
                if spec == "":
                    raise ValueError("a contact selection pattern must be non-empty")
                text = ovphysx_string_t(spec)
                keepalive.append(text)
                selection.pattern = text
                selection.prim_paths = None
                selection.prim_paths_count = 0
            else:
                try:
                    handles = [operator.index(p) for p in spec]
                except TypeError:
                    raise ValueError(
                        "a contact selection is a glob str or a list of prim handles (int); "
                        "this list holds a non-integer"
                    ) from None
                if not handles:
                    raise ValueError("an explicit contact selection must name at least one prim handle")
                array = (c_uint64 * len(handles))(*handles)
                keepalive.append(array)
                selection.pattern = ovphysx_string_t()
                selection.prim_paths = cast(array, POINTER(c_uint64))
                selection.prim_paths_count = len(handles)
            return selection

        sensor_array = (ovphysx_contact_sensor_selection_t * len(selections))()
        keepalive.append(sensor_array)
        for i, contact in enumerate(selections):
            sensor_array[i].sensors = selection_struct(contact.sensors)
            if filter_count:
                filter_array = (ovphysx_query_selection_t * filter_count)(
                    *[selection_struct(f) for f in contact.filters]
                )
                keepalive.append(filter_array)
                sensor_array[i].filters = cast(filter_array, POINTER(ovphysx_query_selection_t))
            else:
                sensor_array[i].filters = None
        receipt = self._lib.ovphysx_query_contacts(
            handle, sensor_array, len(selections), filter_count, byref(query),
            byref(summary) if summary is not None else None)
        return receipt, keepalive

    def _open_query(self, handle, object_type, scope, prim_paths=None, pattern=None, operation="query"):
        # Internal synchronous convenience used by the separate articulation API.
        query = self.query(object_type, scope=scope, prim_paths=prim_paths, pattern=pattern, metadata=False).wait()
        handle = c_uint64(query._query)
        query._query = 0
        return handle

    def read(self, query: Query, attributes, *, metadata: bool = False) -> Operation[ReadResult]:
        """Submit a read-session open using a completed Query, without fetching.

        Attributes are names or tokens from the owning Stage dictionary. Optional
        metadata describes availability at open; None never causes a later fetch.
        Use session.fetch_next().wait() to materialize a group on the current Warp
        stream. Native host completion does not synchronize unrelated CUDA streams.

        Availability is per attribute and scene. Simulated DirectGPU columns need
        the first simulation step. ``isSleeping`` is a CPU uint8 snapshot (1 asleep,
        0 awake); links report their articulation's state. DirectGPU disables
        sleeping, so emitted values are 0 and its existing step-first and disabled
        body omission rules apply. Use an ALL-scope query to include sleeping
        bodies on modes that support sleep.
        """
        return self._open_session(query, attributes, metadata=metadata, write=False)

    def write(self, query: Query, attributes, *, metadata: bool = False) -> Operation[WriteSession]:
        """Submit one multi-attribute write-session open using a completed Query.

        Open validates every requested writable attribute before exposing groups.
        It publishes no values. Fetch indices refer to the original request order;
        each commit has its own task and may fail after earlier commits applied.
        """
        return self._open_session(query, attributes, metadata=metadata, write=True)

    def _open_session(self, query, attributes, *, metadata, write):
        if not isinstance(query, Query):
            raise TypeError("read/write requires a completed Query; use read_sync/write_sync for type-based convenience")
        if query._sdk is not self:
            raise ValueError("query belongs to another PhysX instance")
        query._validate()
        self._drain_pending_read_releases()
        values, array, strings = self._session_attributes(attributes)
        output = c_uint64(0)
        summary = ovphysx_write_result_t() if write else ovphysx_read_result_t()
        stage, attachment, object_type = query._stage, query._attachment, query.object_type
        write_generation = getattr(self, "_write_generation", 0)
        native = self._lib.ovphysx_write if write else self._lib.ovphysx_read
        receipt = native(self._omni_physx_sdk_handle.value, query._query, array, len(values),
                         byref(output), byref(summary) if metadata else None)
        kind = "write" if write else "read"
        task = self._admit_session_task(receipt, kind + " session open")
        def convert():
            self._check_session_owner(attachment)
            if write and write_generation != getattr(self, "_write_generation", 0):
                raise RuntimeError("write session was retired by attachment teardown")
            result = None
            if metadata:
                if write:
                    rows = tuple(WriteAttributeResult(int(summary.attributes[i].attribute),
                                                     int(summary.attributes[i].group_count))
                                 for i in range(summary.attribute_count))
                    result = WriteMetadata(int(summary.total_prim_count), rows)
                else:
                    rows = tuple(ReadAttributeResult(int(summary.attributes[i].attribute),
                                                    int(summary.attributes[i].group_count),
                                                    ReadOmission(summary.attributes[i].omission_reasons))
                                 for i in range(summary.attribute_count))
                    result = ReadMetadata(int(summary.total_prim_count), rows)
            if write:
                return WriteSession(self, 0, output.value, metadata=result, attributes=values,
                                    stage=stage, attachment=attachment, object_type=object_type)
            return ReadResult(self, 0, output.value, metadata=result, object_type=object_type,
                              stage=stage, attachment=attachment)
        return Operation(self, (task,), convert=convert, cleanup=self._orphan_session_cleanup(kind, output),
                         keepalive=(array, strings, output, summary, stage), owners=(query,))

    def read_sync(self, object_type: SimObjectType, attribute_names, *, scope=ObjectScope.ALL,
                  prim_paths=None, pattern=None, contacts=None, metadata: bool = False) -> ReadResult:
        """Explicit waiting convenience: open an internal query and fetch every group."""
        return self._session_sync(object_type, attribute_names, scope, prim_paths, pattern, metadata, False, contacts)

    def read_tokens_sync(self, object_type: SimObjectType, attribute_tokens, *, scope=ObjectScope.ALL,
                         prim_paths=None, pattern=None, contacts=None, metadata: bool = False) -> ReadResult:
        """Explicit waiting convenience for attributes from the owning Stage dictionary."""
        return self.read_sync(object_type, attribute_tokens, scope=scope, prim_paths=prim_paths,
                              pattern=pattern, contacts=contacts, metadata=metadata)

    def write_sync(self, object_type: SimObjectType, attributes, *, scope=ObjectScope.ALL,
                   prim_paths=None, pattern=None, metadata: bool = False, values=None) -> WriteSession:
        """Explicit waiting convenience; a single name or an attribute sequence is accepted.

        ``values`` fills a debug-render ``scope`` or ``cullingBox`` buffer before return.
        """
        if isinstance(attributes, (str, int)):
            attributes = [attributes]
        if values is not None and (len(attributes) != 1 or attributes[0] not in ("scope", "cullingBox")):
            raise ValueError("values is only supported for debug-render scope and cullingBox")
        session = self._session_sync(object_type, attributes, scope, prim_paths, pattern, metadata, True)
        if values is not None:
            if not session.groups or not session.groups[0].tensors:
                session.close_sync()
                raise RuntimeError(f"debug-render {attributes[0]} write has no buffer")
            self._fill_debug_render_values(attributes[0], session.groups[0].tensors[0], values)
        return session

    def _session_sync(self, object_type, attributes, scope, prim_paths, pattern, metadata, write, contacts=None):
        query = self.query(object_type, scope=scope, prim_paths=prim_paths, pattern=pattern,
                           contacts=contacts, metadata=False).wait()
        session = None
        try:
            session = (self.write if write else self.read)(query, attributes, metadata=metadata).wait()
            # Only the explicit synchronous convenience owns its internal selector.
            session._query = session._release._query = query._query
            query._query = 0
            session._sync = True
            while session.fetch_next().wait() is not None:
                pass
            return session
        except Exception as error:
            try:
                if session is not None:
                    session.groups.clear()
                    session.close_sync()
                query.close_sync()
            except Exception as cleanup_error:
                _chain_cleanup(error, cleanup_error)
                raise error
            raise

    def _materialize_read_group(self, group, object_type, release):
        import warp as wp
        _warn_if_warp_build_breaks_cpu_mode(wp)
        self._order_warp_stream_after_group(group, wp)
        tensors = []
        index_map = prim_index_map = None
        try:
            if group.data.tensors:
                for i in range(int(group.data.tensor_count)):
                    tensors.append(self._dltensor_to_warp_array(group.data.tensors[i], wp, release))
            index_map = self._u32_warp_array(group.data.index_map, int(group.data.count), wp, release)
            prim_index_map = self._u32_warp_array(group.prims.index_map, int(group.prims.count), wp, release)
        except Exception:
            # Cached conversion errors retain tracebacks. Do not let their frames
            # silently retain an array that was never exposed to the caller.
            tensors.clear()
            index_map = prim_index_map = None
            raise
        return ReadGroup(
            attribute=int(group.attribute), object_type=SimObjectType(object_type), ordinal=int(group.ordinal),
            is_array=bool(group.is_array), is_delete=bool(group.is_delete), semantic=int(group.semantic),
            prim_list=int(group.prims.list), prim_offset=int(group.prims.offset), prim_count=int(group.prims.count),
            prim_index_map=prim_index_map, index_map=index_map, layout_generation=int(group.meta.layout_generation),
            write_floor_ordinal=int(group.meta.attribute_write_floor_ordinal), tensors=tensors,
            cuda_stream=int(group.data.cuda_sync.stream), cuda_wait_event=int(group.data.cuda_sync.wait_event))

    # DLPack dtype.code/bits -> Warp scalar dtype name shared by read and write.
    # (6, 8) is kDLBool, an encoding the native decoder emits (OvstagePopulator.setBool
    # writes {kDLBool, 8, 1}), so it must decode rather than raise.
    _DL_WARP_DTYPE = {
        (2, 32): "float32",
        (2, 64): "float64",
        (0, 64): "int64",
        (0, 32): "int32",
        (0, 8): "int8",
        (1, 64): "uint64",
        (1, 32): "uint32",
        (1, 8): "uint8",
        (6, 8): "bool",
    }

    @staticmethod
    def _retain_warp_array(wp, *, ptr: int, dtype, shape, device: str, on_release) -> "object":
        """Wrap one non-empty native allocation and tie it to a read-session borrow.

        Warp calls the deleter exactly once, from ``array.__del__``, on whatever thread collects
        the array. ``release_borrow`` is written for that.
        """
        on_release.retain()
        # An array retains the SDK, while queue-owned holders use a weak owner.
        # This avoids SDK -> pending holder -> SDK cycles after an operation is dropped.
        def release(*args, sdk=getattr(on_release, "_sdk", None)):
            on_release.release_borrow(*args)
        try:
            return wp.array(
                ptr=ptr,
                dtype=dtype,
                shape=shape,
                device=device,
                deleter=release,
            )
        except Exception:
            on_release.release_borrow()
            raise

    @classmethod
    def _u32_warp_array(cls, ptr, n, wp, on_release) -> "object":
        """Wrap a borrowed uint32 index map as a CPU Warp array, or return None."""
        if not ptr or n <= 0:
            return None
        address = ctypes.cast(ptr, ctypes.c_void_p).value
        return cls._retain_warp_array(
            wp,
            ptr=int(address),
            dtype=wp.uint32,
            shape=(n,),
            device="cpu",
            on_release=on_release,
        )

    def _order_warp_stream_after_group(self, g, wp) -> None:
        """Make Warp's current stream wait on a CUDA group's producer completion event.

        A group hands over its device columns before the producing work has necessarily
        finished, and a raw pointer carries no readiness, so the wait has to be issued here.
        No-op for a host group or a group with no event.
        """
        cuda_wait_event = int(g.data.cuda_sync.wait_event)
        if not cuda_wait_event or not g.data.tensor_count or not g.data.tensors:
            return
        # Tensor zero speaks for the whole group: a group is device-uniform by construction on the
        # native side (one GroupStore carries a single device ordinal and CUDA context for all of
        # its tensors). Re-checking every tensor here would cost O(tensor_count) per read on array
        # groups, one tensor per prim, to verify something the producer guarantees structurally.
        device = g.data.tensors[0].device
        if _dl_int(device.device_type) != DLDeviceType.kDLCUDA:
            return
        # The private wait uses the calling thread's current CUDA context. ScopedDevice
        # selects it, and get_stream preserves the caller's wp.ScopedStream override.
        warp_device = _warp_device(device)
        with wp.ScopedDevice(warp_device):
            # Warp's null stream is the CUDA default stream and reports cuda_stream as None.
            # This bridge takes CUDA driver handles (0 = default), unlike ovstage_cuda_sync_t
            # on write commit (1 = default).
            cuda_stream = wp.get_stream(warp_device).cuda_stream
            stream_handle = 0 if cuda_stream is None else int(cuda_stream)
            result = self._lib.ovphysx_internal_cuda_stream_wait_event(
                ctypes.c_void_p(stream_handle), ctypes.c_void_p(cuda_wait_event)
            )
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(
                    f"ovphysx read could not order Warp stream {stream_handle} after the "
                    f"producer completion event: {self._get_last_error()}"
                )

    @staticmethod
    def _fill_debug_render_values(attribute_name: str, tensor, values) -> None:
        """Copy a scope token list or a culling box into the producer write buffer.

        The native shape is already the capacity. This writes the count or the
        store/clear flag. It does not assign tensor descriptor fields.
        """
        pointer = getattr(tensor, "ptr", None)
        if pointer is None:
            pointer = getattr(tensor, "data", None)
        if not pointer:
            raise RuntimeError(f"debug-render {attribute_name} write has no buffer")
        if attribute_name == "scope":
            tokens = [int(token) for token in values]
            slots = cast(c_void_p(pointer), POINTER(c_uint64))
            slots[0] = len(tokens)
            for index, token in enumerate(tokens[:4096]):
                slots[index + 1] = token
            return
        numbers = [float(value) for value in values]
        if len(numbers) not in (0, 6):
            raise ValueError("cullingBox values must be six floats, or empty to clear")
        row = cast(c_void_p(pointer), POINTER(c_float))
        for index in range(7):
            row[index] = 0.0
        if len(numbers) == 6:
            row[0] = 1.0
            for index, value in enumerate(numbers):
                row[index + 1] = value

    def _dltensor_to_warp_array(self, t, wp, on_release) -> "object":
        """Wrap a DLTensor as a CPU or CUDA Warp array for read or write.

        Tuple width in ``dtype.lanes`` becomes a trailing scalar dimension, so a
        flat native vec3 column becomes shape ``[N, 3]``. A non-empty read array
        retains its session through a Warp deleter. A write array has no deleter
        or lease because its mapped storage remains session-owned.
        """
        code, bits, lanes = _dl_int(t.dtype.code), _dl_int(t.dtype.bits), _dl_int(t.dtype.lanes) or 1
        dtype_name = self._DL_WARP_DTYPE.get((code, bits))
        if dtype_name is None:
            # Never silently reinterpret an unmapped dtype as float32: that returns wrong
            # values (and only half the buffer for 64-bit elements) with no error. Fail
            # loudly so a native dtype-encoding change is caught instead of corrupting data.
            raise TypeError(
                f"ovphysx column has unsupported DLPack dtype "
                f"(code={code}, bits={bits}, lanes={lanes}); cannot expose it as a Warp array"
            )
        warp_dtype = getattr(wp, dtype_name)
        ndim = int(t.ndim)
        shape = [int(t.shape[i]) for i in range(ndim)]
        total = math.prod(shape) * lanes
        out_shape = tuple(shape + ([lanes] if lanes > 1 else []))

        device = _warp_device(t.device)

        if total == 0 or not t.data:
            return wp.empty(shape=out_shape, dtype=warp_dtype, device=device)
        # byte_offset is part of the address, not of the shape: a column that starts
        # partway into its allocation would otherwise alias from the wrong element.
        address = _dltensor_data_ptr(t)

        if on_release is None:
            return wp.array(ptr=address, dtype=warp_dtype, shape=out_shape, device=device)
        return self._retain_warp_array(
            wp, ptr=address, dtype=warp_dtype, shape=out_shape, device=device, on_release=on_release
        )

    def set_config(self, key: ConfigKey, value: bool | int | float | str) -> None:
        """Set one value retained by this instance."""
        self._check_valid()
        native_value = _make_scoped_config_value(value)
        result = self._lib.ovphysx_set_config(self.handle, int(key), native_value)
        if result.status != ApiStatus.SUCCESS:
            _raise_config_error(result, "PhysX.set_config")

    def get_config(self, key: ConfigKey) -> bool | int | float | str | None:
        """Return one value retained by this instance, or ``None`` when unset."""
        self._check_valid()
        enum_key = ConfigKey(key)
        expected_type = str if enum_key == ConfigKey.ACTIVE_CUDA_GPUS else float
        return _get_scoped_config(
            self._lib.ovphysx_get_config,
            self.handle,
            int(enum_key),
            expected_type,
        )

    def destroy(self) -> None:
        """Destroy this PhysX instance.

        Preconditions:
            - No other thread is using this instance. Calling ``destroy()``
              again after terminal destruction is a valid no-op.
        Side effects:
            - Releases native resources and unregisters the instance.
            - Collects all outstanding raw tasks and session operations, including
              step and clone receipts. They cannot be collected again afterward.
        Ownership/Lifetime:
            - All tensor bindings and contact bindings created by this instance are
              automatically released.
            - Object-change subscriptions and their Python callback captures are
              released after native subscription cleanup.
            - Drop every output-read Warp array and downstream view before destruction.
              Live aliases cause RuntimeError before teardown; the instance and arrays
              stay valid so destruction can be retried after dropping the aliases.
            - The instance becomes unusable after destruction.
        Threading:
            - Do not call concurrently with other operations on this instance.
        Errors:
            - Reports collected task and cleanup failures after attempting teardown;
              a TaskError can therefore be raised after the instance is destroyed.
            - Raises ``RuntimeError`` without changing the instance if called
              from a native log callback. Retry after the callback returns.
            - Raises ``RuntimeError`` and preserves ownership when native destruction
              returns ``INVALID_STATE`` because companion leases or a runtime/recording
              transition remain active. Close leases or finish the transition, then retry.
            - Other native destruction failures or process shutdown failures raise
              ``RuntimeError`` after terminal destruction; a later call is a no-op.
              If both fail, the process-shutdown error is reported with the
              native status included.
            - A Python or ctypes exception raised while invoking native destruction
              leaves ownership intact so the call can be retried.
            - Because this is a checked operation, a cleanup failure raised from
              a ``finally`` block becomes the active exception. Python preserves
              any in-flight exception as chained context; applications that need
              different precedence must catch and log cleanup failures explicitly.
        """
        self._destroy()

    def _destroy(self, *, finalizing: bool = False) -> None:
        # A terminal finalizer may collect an unreachable SDK/array cycle or run at
        # process exit. Neither leaves a caller that can drop aliases and retry.
        handle = getattr(self, "_omni_physx_sdk_handle", None)
        owns_native_lifecycle = (handle is not None and handle.value != _INVALID_HANDLE) or getattr(
            self, "_lifecycle_acquired", False
        )
        if owns_native_lifecycle and getattr(_python_log_callback_context, "active", False):
            raise RuntimeError(
                "ovphysx_shutdown cannot be called from within a log callback; "
                "retry PhysX.destroy() after the callback returns"
            )

        collection_error = None
        try:
            destroy_status = ApiStatus.SUCCESS
            if handle is not None and handle.value != _INVALID_HANDLE:
                if not finalizing and any(
                    holder.has_live_borrows for holder in list(getattr(self, "_live_read_holders", ()))
                ):
                    raise RuntimeError(
                        "Cannot destroy PhysX while output-read arrays or downstream views are alive; "
                        "drop all read arrays and views, then retry destroy()"
                    )

                # Collection may discover orphan outputs, whose release can queue a
                # physical session release. Collect those bounded follow-up waves and
                # stage-bound group releases before retiring their native owner. Continue
                # collection after a cleanup rejection so accepted outcomes are not lost.
                try:
                    _run_cleanups((
                        self._drain_pending_read_releases,
                        self._wait_all_tasks,
                        self._release_stage_bound_read_handles,
                        self._drain_pending_read_releases,
                        self._wait_all_tasks,
                        self._drain_pending_read_releases,
                        self._wait_all_tasks,
                    ))
                except Exception as error:
                    collection_error = error
                # Do not catch Python/ctypes exceptions here. If the call did not
                # return a native status, ownership may still be live and a retry is
                # required. If it completed before raising, the retry converges on
                # the terminal already-absent status handled below.
                result = self._lib.ovphysx_destroy_instance(handle.value)
                destroy_status = result.status
                if destroy_status == ApiStatus.INVALID_STATE:
                    # Admission refused before native teardown. Keep the owner,
                    # Stage and lifecycle token intact for a deferred retry.
                    raise _DestroyAdmissionBusy("Cannot destroy PhysX while leases or concurrent runtime/recording operations are active; close leases, finish operations and retry") from collection_error

            # omni_sdk_physx_destroy() reports failure only when the handle is
            # already absent from the native instance registry. All paths for a live
            # handle erase it and return success. Commit the terminal state for both
            # statuses so a stale handle cannot leak the process-lifecycle token.
            # The native instance also detaches any attached ovstage on destroy.
            self._omni_physx_sdk_handle = None
            self._attached_ovstage = None
            keepalive = getattr(self, "_object_change_keepalive", None)
            if keepalive is not None:
                keepalive.clear()
            self._released = True
            _untrack_process_instance(self)

            destroy_error = None
            if destroy_status != ApiStatus.SUCCESS:
                status_value = int(destroy_status)
                try:
                    status_name = ApiStatus(status_value).name
                except ValueError:
                    status_name = "UNKNOWN"
                destroy_error = (
                    f"ovphysx_destroy_instance() failed with status {status_name} "
                    f"({status_value}). The instance is no longer registered and "
                    "has been marked destroyed. This call does not provide an "
                    "error string; consult the ovphysx log."
                )

            if getattr(self, "_lifecycle_acquired", False):
                # Clear ownership before the potentially-raising shutdown. Native
                # instance destruction has already completed, so retrying this
                # process-lifecycle release would double-decrement the refcount.
                self._lifecycle_acquired = False
                try:
                    _release_process_lifecycle()
                except Exception as exc:
                    message = f"PhysX instance was destroyed, but process shutdown failed: {exc}"
                    if destroy_error is not None:
                        message += f" Native destruction also reported: {destroy_error}"
                    raise RuntimeError(message) from exc

            if destroy_error is not None:
                raise RuntimeError(destroy_error)
            if handle is not None and handle.value != _INVALID_HANDLE and collection_error is not None:
                raise collection_error
        finally:
            collection_error = None


    # -------------------------------------------------------------------------
    # Tensor Binding API - efficient bulk access to physics simulation data
    # -------------------------------------------------------------------------

    def create_tensor_binding(
        self,
        pattern: str = None,
        prim_paths: list[str] = None,
        tensor_type: int = TensorType.RIGID_BODY_POSE,
        *,
        raise_if_empty: bool = False,
    ) -> TensorBinding:
        """Create tensor binding for bulk physics data access (synchronous).

        .. deprecated:: 0.6.0
            The tensor-binding API is deprecated. Use :meth:`PhysX.read` for
            reads and :meth:`PhysX.write` for writes.

        A tensor binding connects physics objects (by path pattern or explicit
        paths) to a tensor type, including authored USD objects and runtime-only
        clones.

        :param pattern: Physics-object path pattern
            (e.g., "/World/robot*", "/World/env[0-9]/robot"; ``[N]`` is not an
            index placeholder, it matches the letter N). The matching rules
            are on the "Path patterns" page of the ovphysx documentation. A
            single path component (the text between two slashes; a parenthesized group
            counts as one component even if it contains a slash) may be at
            most 4096 characters long; a longer component is rejected with
            ``RuntimeError``. Mutually exclusive with ``prim_paths``.
        :param prim_paths: Explicit list of physics-object paths. Mutually
            exclusive with ``pattern``.
        :param tensor_type: Tensor type enum value (``TensorType.*``).
        :param raise_if_empty: If ``True``, raise ``ValueError`` when the
            binding matches zero physics objects. The default keeps empty bindings valid.
            Prefer it for optional or broad queries and check ``binding.count``.
        :returns: TensorBinding object for reading/writing tensor data.
        :raises ValueError: If neither ``pattern`` nor ``prim_paths`` is provided, both are,
            or ``raise_if_empty`` is true and no physics objects match.
        :raises RuntimeError: If binding creation fails.

        Examples::

            import numpy as np
            from ovphysx import TensorType

            def use_tensor_bindings(physx):
                # Optional broad queries can be empty.
                with physx.create_tensor_binding(
                    "/World/robot*", tensor_type=TensorType.RIGID_BODY_POSE
                ) as binding:
                    if binding.count:
                        poses = np.zeros(
                            binding.shape, dtype=np.dtype(str(binding.dtype))
                        )
                        binding.read(poses)

                binding = physx.create_tensor_binding(
                    prim_paths=["/World/env1/robot", "/World/env2/robot"],
                    tensor_type=TensorType.ARTICULATION_DOF_POSITION_TARGET,
                )
                targets = np.zeros(
                    binding.shape, dtype=np.dtype(str(binding.dtype))
                )
                binding.write(targets)
                binding.destroy()

        Preconditions:
            - Exactly one of ``pattern`` or ``prim_paths`` must be provided.
            - A USD stage is loaded.
        Side effects:
            - Allocates native binding resources.
        Ownership/Lifetime:
            - Returned TensorBinding retains native resources until ``destroy()``.
              Each call returns an independent handle; destroying one binding
              does not invalidate a sibling selecting the same objects.
            - A new binding selects the currently eligible realized objects,
              even while an older binding for the same pattern remains alive.
            - Use ``binding.shape`` and ``binding.dtype`` (or ``binding.spec``)
              for layout and ``binding.native_device`` for no-staging placement.
              Most tensor types are float32, but some types such as
              ``RIGID_BODY_DISABLE_SIMULATION`` are not.
            - The binding is tied to the current stage topology. Reuse it across
              steps, but do not keep it across ``detach_ovstage()``, removing USD data
              that contains bound objects, or replacing/reparsing the stage so
              bound objects are destroyed and recreated. Destroy cached bindings
              before those lifecycle operations when practical. If a stale
              binding survives, only destroy it. Create replacements after the
              operation completes.
        Diagnostics:
            - Pattern bindings can intentionally match zero physics objects, so expected
              TensorAPI no-match diagnostics are quieted on the simulation view
              used to create that binding.
            - Explicit ``prim_paths`` keep the default error-level no-match
              diagnostics for typo detection. To detect partial misses
              programmatically, compare the requested ``prim_paths`` with the
              resolved ``binding.prim_paths`` returned after creation.
        Threading:
            - Do not create bindings concurrently with stage mutation.
        Errors:
            - Raises ``ValueError`` for invalid arguments.
            - Raises ``RuntimeError`` on creation failure.
        """
        warnings.warn(
            "ovphysx tensor bindings are deprecated and will be removed in a future "
            "release; use PhysX.read for reads and PhysX.write for writes.",
            DeprecationWarning,
            stacklevel=2,
        )
        self._check_valid()
        if pattern is None and prim_paths is None:
            raise ValueError("Either 'pattern' or 'prim_paths' must be provided")
        if pattern is not None and prim_paths is not None:
            raise ValueError("Cannot specify both 'pattern' and 'prim_paths'")
        if prim_paths is not None and len(prim_paths) == 0:
            raise ValueError("prim_paths must not be empty; pass at least one prim path.")

        desc = ovphysx_tensor_binding_desc_t()
        desc.tensor_type = tensor_type

        # Keep references to prevent garbage collection during C call
        c_paths_array = None
        c_paths_refs = []

        if prim_paths is not None:
            desc.pattern = ovphysx_string_t()  # Empty pattern
            desc.prim_paths_count = len(prim_paths)
            c_paths_array = (ovphysx_string_t * len(prim_paths))()
            for i, path in enumerate(prim_paths):
                c_paths_array[i] = ovphysx_string_t(path)
                c_paths_refs.append(c_paths_array[i])
            desc.prim_paths = cast(c_paths_array, POINTER(ovphysx_string_t))
        else:
            desc.pattern = ovphysx_string_t(pattern)
            desc.prim_paths = None
            desc.prim_paths_count = 0

        handle = c_uint64(0)
        result = self._lib.ovphysx_create_tensor_binding(self._omni_physx_sdk_handle.value, byref(desc), byref(handle))

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to create tensor binding: {error_msg}")

        spec = ovphysx_tensor_spec_t()
        spec_result = self._lib.ovphysx_get_tensor_binding_spec(
            self._omni_physx_sdk_handle.value, handle.value, byref(spec)
        )

        if spec_result.status != ApiStatus.SUCCESS:
            self._lib.ovphysx_destroy_tensor_binding(self._omni_physx_sdk_handle.value, handle.value)
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to get tensor binding spec: {error_msg}")

        ndim = spec.ndim
        shape = tuple(spec.shape[i] for i in range(ndim))

        if raise_if_empty and (not shape or shape[0] == 0):
            cleanup_result = self._lib.ovphysx_destroy_tensor_binding(self._omni_physx_sdk_handle.value, handle.value)
            cleanup_error = None
            if cleanup_result.status != ApiStatus.SUCCESS:
                cleanup_error = self._get_last_error() or "unknown error"
            if prim_paths is not None:
                source = f"prim_paths ({len(prim_paths)} entries)"
            else:
                source = f"pattern {pattern!r}"
            if cleanup_error is not None:
                warnings.warn(
                    f"Failed to destroy empty tensor binding after {source} matched 0 prims: {cleanup_error}",
                    RuntimeWarning,
                )
            raise ValueError(
                f"create_tensor_binding matched 0 prims for {source}. "
                "Check the path input and ensure USDs are loaded before creating the binding. "
                "For optional readback or broad scene queries, leave raise_if_empty=False "
                "and check binding.count before reading."
            )

        # The factory already emitted the caller-precise deprecation warning above. _from_factory
        # skips the constructor's duplicate so the caller sees exactly one, without touching the
        # process-global warnings filter.
        return TensorBinding(self, handle.value, tensor_type, ndim, shape, spec.dtype, _from_factory=True)

    def warmup(self) -> None:
        """Explicitly run the warmup step (synchronous).

        On first call, runs a minimal simulation step (~1ns) to initialize PhysX
        structures. Works in both CPU and GPU mode (in GPU mode this also populates
        DirectGPU buffers).

        Normally done automatically on the first tensor read, but calling it
        explicitly lets you control when the latency occurs.

        This function is idempotent. Calling it multiple times has no effect after
        the first successful call. Warmup state resets after detach_ovstage() or
        attaching a new USD stage.

        Raises:
            RuntimeError: If warmup fails.

        Side effects:
            - Advances simulation by a minimal timestep on first call.
        Threading:
            - Do not call concurrently with other operations on this instance.
        """
        self._check_valid()
        result = self._lib.ovphysx_warmup(self._omni_physx_sdk_handle.value)

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to warmup: {error_msg}")

    # ------------------------------------------------------------------
    # Scene queries
    # ------------------------------------------------------------------

    @staticmethod
    def _make_float3(values: list | tuple) -> "ctypes.Array[ctypes.c_float]":
        """Convert a 3-element sequence to a ctypes float[3]."""
        arr = (ctypes.c_float * 3)()
        arr[0], arr[1], arr[2] = float(values[0]), float(values[1]), float(values[2])
        return arr

    def _parse_scene_query_hits(self, hits_ptr, count: int) -> list[dict]:
        """Convert the internal hit buffer to a list of dicts."""
        results = []
        for i in range(count):
            h = hits_ptr[i]
            results.append(
                {
                    "collision": h.collision,
                    "rigid_body": h.rigid_body,
                    "proto_index": h.proto_index,
                    "normal": (h.normal[0], h.normal[1], h.normal[2]),
                    "position": (h.position[0], h.position[1], h.position[2]),
                    "distance": h.distance,
                    "face_index": h.face_index,
                    "material": h.material,
                }
            )
        return results

    def raycast(
        self,
        origin: tuple | list,
        direction: tuple | list,
        distance: float,
        mode: SceneQueryMode = SceneQueryMode.CLOSEST,
        both_sides: bool = False,
    ) -> list[dict]:
        """Cast a ray and return hits.

        .. deprecated:: 0.6.0
            Scene queries are deprecated.

        Args:
            origin: Ray origin [x, y, z].
            direction: Normalized ray direction [x, y, z].
            distance: Maximum ray length (>= 0).
            mode: :class:`~ovphysx.SceneQueryMode` (CLOSEST, ANY, or ALL).
            both_sides: If True, test both sides of mesh triangles.

        Returns:
            List of hit dicts. Each dict contains ``collision``, ``rigid_body``,
            ``proto_index``, ``normal``, ``position``, ``distance``,
            ``face_index``, ``material``. For ANY mode, hit fields are zeroed.
        """
        _warn_legacy_api(
            "ovphysx scene queries are deprecated and will be removed in a future release."
        )
        o = self._make_float3(origin)
        d = self._make_float3(direction)
        hits_ptr = ctypes.POINTER(ovphysx_scene_query_hit_t)()
        count = ctypes.c_uint32(0)
        result = _lib.ovphysx_raycast(
            self._omni_physx_sdk_handle.value,
            o,
            d,
            ctypes.c_float(distance),
            ctypes.c_bool(both_sides),
            ctypes.c_int(int(mode)),
            ctypes.byref(hits_ptr),
            ctypes.byref(count),
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Raycast failed: {self._get_last_error()}")
        return self._parse_scene_query_hits(hits_ptr, count.value)

    _GEOMETRY_PARAMS = {
        SceneQueryGeometryType.SPHERE: {"radius", "position"},
        SceneQueryGeometryType.BOX: {"half_extent", "position", "rotation"},
        SceneQueryGeometryType.SHAPE: {"prim_path"},
    }

    def _make_geometry_desc(
        self, geometry_type: SceneQueryGeometryType, **kwargs
    ) -> "ovphysx_scene_query_geometry_desc_t":
        """Build an ovphysx_scene_query_geometry_desc_t from keyword args."""
        expected = self._GEOMETRY_PARAMS.get(geometry_type)
        if expected is not None:
            unexpected = set(kwargs) - expected
            if unexpected:
                raise TypeError(f"Unexpected kwargs for {SceneQueryGeometryType(geometry_type).name}: {unexpected}")
        desc = ovphysx_scene_query_geometry_desc_t()
        desc.type = int(geometry_type)
        if geometry_type == SceneQueryGeometryType.SPHERE:
            desc._geom.sphere.radius = float(kwargs["radius"])
            pos = kwargs.get("position", (0, 0, 0))
            desc._geom.sphere.position[0] = float(pos[0])
            desc._geom.sphere.position[1] = float(pos[1])
            desc._geom.sphere.position[2] = float(pos[2])
        elif geometry_type == SceneQueryGeometryType.BOX:
            he = kwargs["half_extent"]
            desc._geom.box.half_extent[0] = float(he[0])
            desc._geom.box.half_extent[1] = float(he[1])
            desc._geom.box.half_extent[2] = float(he[2])
            pos = kwargs.get("position", (0, 0, 0))
            desc._geom.box.position[0] = float(pos[0])
            desc._geom.box.position[1] = float(pos[1])
            desc._geom.box.position[2] = float(pos[2])
            rot = kwargs.get("rotation", (0, 0, 0, 1))
            desc._geom.box.rotation[0] = float(rot[0])
            desc._geom.box.rotation[1] = float(rot[1])
            desc._geom.box.rotation[2] = float(rot[2])
            desc._geom.box.rotation[3] = float(rot[3])
        elif geometry_type == SceneQueryGeometryType.SHAPE:
            prim_path = kwargs["prim_path"]
            if isinstance(prim_path, (bytes, bytearray)):
                prim_path = bytes(prim_path).decode("utf-8")
            # ovphysx_string_t is length-prefixed so embedded NULs reach the C
            # API, which rejects them instead of silently truncating the path.
            path_str = ovphysx_string_t(prim_path)
            desc._geom.shape.prim_path = path_str
            desc._keepalive = path_str  # keeps path_str._bytes buffer alive
        else:
            raise ValueError(f"Unknown geometry type: {geometry_type}")
        return desc

    def sweep(
        self,
        geometry_type: SceneQueryGeometryType,
        direction: tuple | list,
        distance: float,
        mode: SceneQueryMode = SceneQueryMode.CLOSEST,
        both_sides: bool = False,
        **kwargs,
    ) -> list[dict]:
        """Sweep a geometry shape along a direction and return hits.

        .. deprecated:: 0.6.0
            Scene queries are deprecated.

        Args:
            geometry_type: :class:`~ovphysx.SceneQueryGeometryType`.
            direction: Normalized sweep direction [x, y, z].
            distance: Maximum sweep distance (>= 0).
            mode: :class:`~ovphysx.SceneQueryMode`.
            both_sides: If True, test both sides of mesh triangles.
            **kwargs: Geometry parameters:

                - SPHERE: ``radius``, ``position``
                - BOX: ``half_extent``, ``position``, ``rotation`` (xyzw quaternion)
                - SHAPE: ``prim_path`` (USD prim path string)

        Returns:
            List of hit dicts (same format as :meth:`raycast`).
        """
        _warn_legacy_api(
            "ovphysx scene queries are deprecated and will be removed in a future release."
        )
        desc = self._make_geometry_desc(geometry_type, **kwargs)
        d = self._make_float3(direction)
        hits_ptr = ctypes.POINTER(ovphysx_scene_query_hit_t)()
        count = ctypes.c_uint32(0)
        result = _lib.ovphysx_sweep(
            self._omni_physx_sdk_handle.value,
            ctypes.byref(desc),
            d,
            ctypes.c_float(distance),
            ctypes.c_bool(both_sides),
            ctypes.c_int(int(mode)),
            ctypes.byref(hits_ptr),
            ctypes.byref(count),
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Sweep failed: {self._get_last_error()}")
        return self._parse_scene_query_hits(hits_ptr, count.value)

    def overlap(
        self,
        geometry_type: SceneQueryGeometryType,
        mode: SceneQueryMode = SceneQueryMode.ALL,
        **kwargs,
    ) -> list[dict]:
        """Test geometry overlap against objects in the scene.

        .. deprecated:: 0.6.0
            Scene queries are deprecated.

        For overlap queries, location fields (normal, position, distance,
        face_index, material) are zeroed. Only object identity is populated.

        Args:
            geometry_type: :class:`~ovphysx.SceneQueryGeometryType`.
            mode: :class:`~ovphysx.SceneQueryMode` (ANY or ALL).
            **kwargs: Geometry parameters (same as :meth:`sweep`).

        Returns:
            List of hit dicts (same format as :meth:`raycast`).

        Raises:
            ValueError: If ``mode`` is :attr:`~ovphysx.SceneQueryMode.CLOSEST`.
        """
        _warn_legacy_api(
            "ovphysx scene queries are deprecated and will be removed in a future release."
        )
        if mode == SceneQueryMode.CLOSEST:
            raise ValueError(
                "overlap() does not support SceneQueryMode.CLOSEST -- "
                "CLOSEST has no meaning for overlap queries (no direction/ray). "
                "Use SceneQueryMode.ALL or SceneQueryMode.ANY."
            )
        desc = self._make_geometry_desc(geometry_type, **kwargs)
        hits_ptr = ctypes.POINTER(ovphysx_scene_query_hit_t)()
        count = ctypes.c_uint32(0)
        result = _lib.ovphysx_overlap(
            self._omni_physx_sdk_handle.value,
            ctypes.byref(desc),
            ctypes.c_int(int(mode)),
            ctypes.byref(hits_ptr),
            ctypes.byref(count),
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Overlap failed: {self._get_last_error()}")
        return self._parse_scene_query_hits(hits_ptr, count.value)

    def get_scene_query_paths_from_ids(self, ids: "tuple | list") -> list[str]:
        """Resolve scene-query hit identity fields to physics-object paths.

        .. deprecated:: 0.6.0
            Scene queries are deprecated.

        ``ids`` holds opaque identity handles taken directly from the
        ``collision``, ``rigid_body``, or ``material`` entries of hit dicts
        returned by :meth:`raycast`, :meth:`sweep`, or :meth:`overlap`. IDs
        that cannot be resolved (a zero id, an id from an object removed
        since the query, or no active attach) yield empty strings.

        Args:
            ids: Sequence of ``int`` identity handles.

        Returns:
            list[str]: Physics-object paths in the same order as ``ids``.
        """
        _warn_legacy_api(
            "ovphysx scene queries are deprecated and will be removed in a future release."
        )
        n = len(ids)
        if n == 0:
            return []
        ids_buf = (ctypes.c_uint64 * n)(*ids)
        paths_buf = (ovphysx_string_t * n)()
        count = ctypes.c_uint32(0)
        result = _lib.ovphysx_scene_query_get_paths_from_ids(
            self._omni_physx_sdk_handle.value,
            ids_buf,
            ctypes.c_uint32(n),
            paths_buf,
            ctypes.c_uint32(n),
            ctypes.byref(count),
        )
        if result.status not in (ApiStatus.SUCCESS, ApiStatus.BUFFER_TOO_SMALL):
            raise RuntimeError(f"Failed to resolve scene-query IDs to paths: {self._get_last_error()}")
        return _decode_truncated_paths(
            paths_buf,
            int(count.value),
            n,
            "ovphysx_scene_query_get_paths_from_ids",
        )

    # ------------------------------------------------------------------
    # Contact bindings
    # ------------------------------------------------------------------

    def create_contact_binding(
        self,
        sensor_patterns: list[str],
        filter_patterns: list[str] | None = None,
        filters_per_sensor: int = 0,
        max_contact_data_count: int = 0,
    ) -> ContactBinding:
        """Create a contact binding for reading aggregate and detailed contact tensors.

        .. deprecated:: 0.6.0
            Contact bindings are deprecated.

        Returns DLPack-compatible tensors of net forces ``[S, 3]`` or force
        matrices ``[S, F, 3]``. Detailed contact and friction data are exposed
        as flat ``[C, ...]`` buffers plus ``[S, F]`` count/start-index tensors
        via :meth:`ContactBinding.read_normal_contact_data` and
        :meth:`ContactBinding.read_friction_contact_data`.

        A **sensor** is a set of rigid bodies matched by a physics-object path
        pattern. A **filter** is a second set of bodies whose contacts with each
        sensor you want to measure. Patterns include authored USD objects and
        runtime-only clones.

        Sensor selection is opt-in: every authored USD prim matched by
        ``sensor_patterns`` must have ``PhysxContactReportAPI`` applied, on the
        prim named as the sensor itself (not a parent body or child collider). A
        matched prim without the schema is dropped from the binding, and if that
        leaves no sensors this call raises ``RuntimeError``. Filter prims need no
        extra schema. Runtime-only clones use the schema metadata copied from
        the corresponding source object: a cloned collider whose source collider
        lacks the schema is rejected even if its parent body has it.

        On CPU simulation and GPU dynamics with CPU readback, a separate child collider's schema enables
        sensor selection, not reporting for its owning body. Also enable reporting on the body, owning
        articulation link, or articulation root to avoid depending on reports from the other actor in each
        pair. If neither actor reports matching contacts, the child sensor reads zero. Configure reporting
        settings on the reporting actor or articulation root; a standalone static collider is its own actor.

        In these modes, enable owner reporting but select only the child as a binding sensor. If the owner
        is also selected as a sensor in this or another live contact binding on the same attachment, the
        owner takes precedence: contacts are credited to the owner and the child reads zero. Check wildcard
        patterns for unintended owner matches, and establish bindings before stepping. Session reads support
        selecting both the owner and child in these modes.

        On DirectGPU, a separate child collider carrying ``PhysxContactReportAPI`` is accepted by the
        deprecated contact binding, but the runtime logs an unsupported collider warning and the child
        reads zero force. DirectGPU session queries omit that child row; select the owning body or
        articulation link instead.

        The binding must be created *before* the first simulation step whose
        contacts you want to observe. Call
        :meth:`ContactBinding.read_net_normal_forces` or
        :meth:`ContactBinding.read_normal_force_matrix` after a successful
        :meth:`PhysX.step` or :meth:`PhysX.step_sync` call. Before the first step, both return
        all-zeros tensors.

        Result tensor shapes after step:
          - net forces:    ``[S, 3]``   where S = matched sensor count
          - force matrix:  ``[S, F, 3]`` where F = matched filter count per sensor
          - detailed data: flat ``[C, 1]`` or ``[C, 3]`` buffers indexed by
            ``counts`` and ``start_indices`` with shape ``[S, F]``

        Use :attr:`ContactBinding.sensor_paths` and
        :attr:`ContactBinding.filter_paths` to map rows and columns back to
        resolved physics-object paths.

        Example::

            import torch

            def read_contact_forces(physx):
                with physx.create_contact_binding(
                    sensor_patterns=["/World/robot_0/ee"],
                    filter_patterns=["/World/obstacles/box"],
                    filters_per_sensor=1,
                    max_contact_data_count=256,
                ) as binding:
                    # Call this after a successful simulation step.
                    forces = torch.zeros(
                        (binding.sensor_count, 3), device="cuda"
                    )
                    binding.read_net_normal_forces(forces)
                    return forces

        Args:
            sensor_patterns: Physics-object path patterns for sensor bodies. A single
                path component may be at most 4096 characters long; a longer one
                raises ``RuntimeError`` (also for ``filter_patterns``). The matching
                rules are on the "Path patterns" page of the ovphysx documentation.
            filter_patterns: Flat list of physics-object path patterns for filters.
                Total length must equal ``len(sensor_patterns) * filters_per_sensor``.
                Pass ``None`` with ``filters_per_sensor=0`` to get contacts with all bodies.
            filters_per_sensor: Number of filter patterns per sensor (same for all sensors).
            max_contact_data_count: Max raw contact pairs to track in the native
                backend. Also caps the detailed contact/friction flat-buffer reads.
                Detailed reads require this value and ``filters_per_sensor`` to
                be positive.
        """
        _warn_legacy_api(
            "ovphysx contact bindings are deprecated and will be removed in a future release."
        )
        self._check_valid()
        n_sensors = len(sensor_patterns)
        if n_sensors == 0:
            raise ValueError("sensor_patterns must be non-empty")
        if filters_per_sensor < 0:
            raise ValueError("filters_per_sensor must be >= 0")
        c_sensors = (ovphysx_string_t * n_sensors)(*[ovphysx_string_t(p) for p in sensor_patterns])

        if filter_patterns:
            expected = n_sensors * filters_per_sensor
            if filters_per_sensor == 0:
                raise ValueError("filters_per_sensor must be > 0 when filter_patterns is provided")
            if len(filter_patterns) != expected:
                raise ValueError(
                    f"filter_patterns length {len(filter_patterns)} != "
                    f"n_sensors ({n_sensors}) * filters_per_sensor ({filters_per_sensor}) = {expected}"
                )
            n_filters = len(filter_patterns)
            c_filters = (ovphysx_string_t * n_filters)(*[ovphysx_string_t(p) for p in filter_patterns])
            c_filters_ptr = cast(c_filters, POINTER(ovphysx_string_t))
        else:
            c_filters = None
            c_filters_ptr = None
            filters_per_sensor = 0

        out_handle = c_uint64(0)
        result = self._lib.ovphysx_create_contact_binding(
            self._omni_physx_sdk_handle.value,
            cast(c_sensors, POINTER(ovphysx_string_t)),
            c_uint32(n_sensors),
            c_filters_ptr,
            c_uint32(filters_per_sensor),
            c_uint32(max_contact_data_count),
            byref(out_handle),
        )

        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Failed to create contact binding: {self._get_last_error()}")

        from ctypes import c_int32 as _c_int32

        sensor_count = _c_int32(0)
        filter_count = _c_int32(0)
        spec_result = self._lib.ovphysx_get_contact_binding_spec(
            self._omni_physx_sdk_handle.value, out_handle.value, byref(sensor_count), byref(filter_count)
        )

        if spec_result.status != ApiStatus.SUCCESS:
            self._lib.ovphysx_destroy_contact_binding(self._omni_physx_sdk_handle.value, out_handle.value)
            raise RuntimeError(f"Failed to get contact spec: {self._get_last_error()}")

        capacity = c_uint32(0)
        capacity_result = self._lib.ovphysx_get_contact_binding_capacity(
            self._omni_physx_sdk_handle.value, out_handle.value, byref(capacity)
        )
        if capacity_result.status != ApiStatus.SUCCESS:
            self._lib.ovphysx_destroy_contact_binding(self._omni_physx_sdk_handle.value, out_handle.value)
            raise RuntimeError(f"Failed to get contact capacity: {self._get_last_error()}")

        return ContactBinding(self, out_handle.value, sensor_count.value, filter_count.value, capacity.value)

    def create_sdf_view(self, pattern: str, max_query_points: int) -> SdfView:
        """Create an SDF shape view for evaluating signed distance fields.

        .. deprecated:: 0.6.0
            SDF views are deprecated. Use :meth:`evaluate_sdf_shapes`, which evaluates the same shapes through
            an ``SDF_SHAPE`` output query and takes Q from the arrays on every call.

        Requires a GPU instance. CPU SDF evaluation is not implemented.

        Args:
            pattern: Physics-object path pattern matching SDF collision shapes,
                including runtime-only clones. A single path component may be at
                most 4096 characters long; a longer one raises ``RuntimeError``. The
                matching rules are on the "Path patterns" page of the ovphysx documentation.
            max_query_points: Number of query points per shape per call. Query
                tensors passed to ``SdfView.evaluate`` must have Q equal to this.

        Returns:
            SdfView with count == number of matched shapes.

        Example::

            sdf = physx.create_sdf_view("/World/Mesh*", max_query_points=64)
            # Query/output tensors must be on the CUDA device (SDF eval is GPU-only).
            pts = torch.zeros((sdf.count, 64, 3), dtype=torch.float32, device="cuda")
            out = torch.zeros((sdf.count, 64, 4), dtype=torch.float32, device="cuda")
            sdf.evaluate(pts, out)
            sdf.destroy()
        """
        _warn_legacy_api(
            "ovphysx SDF views are deprecated and will be removed in a future release."
        )
        self._check_valid()
        handle = self._omni_physx_sdk_handle.value
        pat = ovphysx_string_t(pattern)
        out_handle = c_uint64(0)
        result = _lib.ovphysx_create_sdf_view(handle, pat, c_uint32(max_query_points), byref(out_handle))
        if result.status != 0:
            raise RuntimeError(f"create_sdf_view failed: {self._get_last_error()}")
        count_out = c_uint32(0)
        count_result = _lib.ovphysx_sdf_view_get_count(handle, out_handle.value, byref(count_out))
        if count_result.status != 0:
            _lib.ovphysx_destroy_sdf_view(handle, out_handle.value)
            raise RuntimeError(f"create_sdf_view failed querying shape count: {self._get_last_error()}")
        return SdfView(self, out_handle.value, count_out.value, max_query_points)

    def _finalize_unreleased(self, *, retry_busy: bool = False) -> None:
        """Run the finalizer's non-throwing destroy/retry policy."""
        # No caller remains to perform destroy()'s documented retry after a
        # pre-status invocation exception, so make one immediate best-effort
        # attempt. Returned statuses other than busy, and shutdown failures,
        # already discharged ownership and are terminal, so are not retried.
        attempts = 0
        while attempts < 2:
            try:
                if any(holder.has_live_borrows for holder in list(getattr(self, "_live_read_holders", ()))):
                    # Unreachable cycles and interpreter exit end array lifetime.
                    self._destroy(finalizing=True)
                else:
                    self.destroy()
                return
            except _DestroyAdmissionBusy:
                # A returned busy status proves the instance is still live.
                # Keep self, its Stage and lifecycle token reachable until the
                # unrelated cook/recording operation releases admission.
                if retry_busy:
                    if _PROCESS_EXIT_CLEANUP_STARTED.is_set():
                        # Exit cleanup joins this worker, then takes over the
                        # still-live owner; never act on it concurrently. Its
                        # finalizer has already run once, so pin it strongly:
                        # dropping this worker's reference would otherwise free
                        # it silently with the native instance still live.
                        with _PROCESS_LIFECYCLE_INSTANCES_LOCK:
                            _PROCESS_LIFECYCLE_INSTANCES[id(self)] = lambda: self
                        return
                    threading.Event().wait(0.02)
                    continue
                if getattr(self, "_busy_finalizer_pending", False):
                    return
                self._busy_finalizer_pending = True
                try:
                    started = _start_deferred_finalizer(self, retry_busy=True)
                except Exception:
                    started = None
                if started:
                    return
                self._busy_finalizer_pending = False
                # Reuse the existing exit registry with a strong callable. A
                # weak entry would orphan the known-live instance as soon as
                # __del__ returns.
                with _PROCESS_LIFECYCLE_INSTANCES_LOCK:
                    _PROCESS_LIFECYCLE_INSTANCES[id(self)] = lambda: self
                if started is None:
                    warnings.warn("PhysX could not start deferred cleanup after a busy native status; ownership was retained for process-exit cleanup",
                                  RuntimeWarning, stacklevel=2)
                else:
                    self._retained_busy_at_exit = True
                    warnings.warn("PhysX was still busy at process exit; its native instance, Stage and lifecycle token were retained",
                                  RuntimeWarning, stacklevel=2)
                return
            except Exception:
                if not getattr(self, "_lifecycle_acquired", False):
                    return
                attempts += 1

        # Both invocations raised before a status was returned. Native liveness
        # is unknowable, and this object is about to be freed. Retaining the
        # token would pin it to a dead object and prevent Python from ever
        # calling process shutdown. Abandon the handle and emit a default-visible
        # warning because a native instance may remain registered.
        try:
            warnings.warn(
                "PhysX cleanup failed twice before native destruction returned "
                "a status. The native instance may still be registered and "
                "cannot be reached again; releasing the process-lifecycle "
                "token. Consult the ovphysx log.",
                RuntimeWarning,
                stacklevel=2,
            )
        except Exception:
            pass
        self._omni_physx_sdk_handle = None
        self._attached_ovstage = None
        self._released = True
        self._lifecycle_acquired = False
        _untrack_process_instance(self)
        try:
            _release_process_lifecycle()
        except Exception:
            pass

    def __del__(self) -> None:
        """Destructor - ensures cleanup on garbage collection.

        Emits a :class:`ResourceWarning` when the instance is garbage-collected
        without an explicit :meth:`destroy`. Mirrors Python file-object
        semantics. The warning is silent by default
        (filtered out unless ``python -W default::ResourceWarning`` or a test
        suite captures it), so existing code is not surprised, but a missing
        destroy is surfaced to anyone looking for resource hygiene issues. A
        native invocation exception gets one immediate best-effort retry.
        A returned busy status retains ownership in a deferred worker until
        admission succeeds. Persistent invocation errors are suppressed. If both invocations fail,
        native state is unknowable, so the finalizer abandons the Python handle
        with a default-visible ``RuntimeWarning`` and discharges process-lifecycle
        ownership rather than letting an unreachable object prevent Python from
        calling process shutdown.

        Note: During interpreter shutdown, calling C functions may fail.
        The destructor checks sys.is_finalizing() to avoid spurious errors.
        """
        try:
            import sys as _sys  # local re-bind: module-level sys may be None at shutdown

            if _sys.is_finalizing():
                # Skip cleanup during interpreter shutdown. The native library
                # may already be unloaded or in an inconsistent state.
                return
        except Exception:
            # During interpreter shutdown, importing sys itself can fail with
            # "import of sys halted; None in sys.modules". In that case, do not
            # attempt cleanup.
            return
        if not getattr(self, "_released", True):
            try:
                import warnings as _warnings  # local re-bind for shutdown safety

                _warnings.warn(
                    "PhysX instance garbage-collected without explicit "
                    "destroy(). Call physx.destroy() in a finally block to "
                    "ensure deterministic cleanup. "
                    "Process-exit cleanup is best-effort and cannot run "
                    "after abrupt termination.",
                    ResourceWarning,
                    stacklevel=2,
                )
            except Exception:
                pass

        owns_native_lifecycle = getattr(self, "_lifecycle_acquired", False)
        handle = getattr(self, "_omni_physx_sdk_handle", None)
        owns_native_lifecycle = owns_native_lifecycle or (handle is not None and handle.value != _INVALID_HANDLE)
        if owns_native_lifecycle and getattr(_python_log_callback_context, "active", False):
            # Explicit destroy remains fail-fast in callback context. A GC
            # finalizer has no caller that can retry, so retain self through a
            # daemon worker and let native shutdown drain this callback after
            # it returns. Never join the worker from the callback thread.
            # Exit cleanup joins the worker before touching this instance.
            try:
                started = _start_deferred_finalizer(self)
            except Exception:
                started = False
            if not started:
                try:
                    warnings.warn(
                        "PhysX finalizer could not defer cleanup outside the "
                        "native log callback. The native instance and its "
                        "process-lifecycle token may remain registered.",
                        RuntimeWarning,
                        stacklevel=2,
                    )
                except Exception:
                    pass
            return

        self._finalize_unreleased()
