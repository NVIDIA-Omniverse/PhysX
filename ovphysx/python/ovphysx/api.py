# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-CAPI-LOG-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
# @implements REQ-PYTHON-KEYWORD-001
# @covers AC-1 AC-2
# @implements REQ-PYTHON-OMNIPVD-LATE-001
# @covers AC-3 AC-4 AC-5 AC-6 AC-7 AC-8
# @implements REQ-CAPI-WRITE-001
# @covers AC-9 AC-11 AC-12
# @implements REQ-PYTHON-READPOOL-001
# @covers AC-3
# @implements REQ-PYTHON-BINDING-DEVICE-001
# @covers AC-1 AC-2 AC-3 AC-4
# @implements REQ-PYTHON-CLONE-001
# @covers AC-2

"""High-level Python API for the ovphysx library.

Stream-Ordered Execution Model
------------------------------
All operations in this API are stream-ordered, meaning they execute in submission
order as if on a single queue. This provides sequential consistency:

- Operations appear to complete in submission order
- Writes from operation N are visible to operation N+1
- You don't need explicit synchronization between dependent operations
- Independent operations may execute concurrently internally for performance

Example (no explicit waits needed between dependent operations):

.. code-block:: python
   :caption: Stream-ordered operation sequence

    from ovphysx.types import ObjectScope, SimObjectType

    def step_and_read(physx, stage, initial_ordinal, from_ordinal, to_ordinal, dt):
        physx.attach_ovstage(stage, read_ordinal=initial_ordinal)
        # After the application authors later ovstage edits:
        physx.update_from_ovstage(from_ordinal, to_ordinal)
        physx.step(dt)  # Sees the drained stage edits
        with physx.read(
            SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL
        ) as result:  # Reads current state
            for group in result.groups:
                ...  # use group.tensors (native CPU/CUDA columns)

Use wait_op() when:

- Before accessing results outside the stream (e.g., reading data on CPU/GPU)
- To ensure operations complete before program exit
- For explicit synchronization points in your application

Thread Safety
-------------
- PhysX instances share the underlying omni.physx runtime. Serialize simulation,
  stage mutation, and binding creation across instances.
- Only one instance may own a live ovstage attach in a process. A peer attach
  attempt raises ``RuntimeError`` and leaves the owner's stage and bindings
  unchanged. Detach the owner before attaching another instance.
- A single instance is NOT thread-safe. Use external synchronization if calling
  from multiple threads.
- ctypes releases the GIL during native calls, so concurrent ``step()`` and
  ``PhysX.read()`` / ``PhysX.write()`` from different threads is a data race.
  See the developer guide threading section for the recommended pattern.

"""

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8
# @implements REQ-CAPI-CACHE-001
# @covers AC-5
# @implements REQ-PYTHON-READ-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6

import atexit
import ctypes
import gc
import math
import operator
import threading
import warnings
import weakref
from ctypes import (
    POINTER,
    byref,
    c_char_p,
    c_float,
    c_int32,
    c_uint32,
    c_uint64,
    c_void_p,
    cast,
)
from typing import TYPE_CHECKING, Any, NamedTuple

from packaging.version import Version

if TYPE_CHECKING:
    from .config import OmniPvdDestination, PhysXConfig

from ._bindings import (
    OP_INDEX_ALL,
)
from ._bindings import OVPHYSX_INVALID_HANDLE as _INVALID_HANDLE
from ._bindings import (
    ContactEventHeader,
    ContactPoint,
    FrictionAnchor,
    _lib,
    ovphysx_articulation_metadata_t,
    ovphysx_create_args,
    ovphysx_log_callback_t,
    ovphysx_omnipvd_destination_t,
    ovphysx_op_wait_result_t,
    ovphysx_string_t,
    ovphysx_tensor_binding_desc_t,
    ovphysx_tensor_spec_t,
    ovphysx_config_entry_t,
    ovstage_read_group_t,
    ovstage_map_group_t,
    ovstage_cuda_sync_t,
    ovstage_query_result_t,
    ovstage_ordinal_range_t,
    ovx_string_or_token_t,
    ovx_string_t,
    ovphysx_scene_query_geometry_desc_t,
    ovphysx_scene_query_hit_t,
)
from . import __version__ as _python_version
from .types import (
    ApiStatus,
    LogLevel,
    ObjectType,
    SimObjectType,
    ObjectScope,
    SceneQueryGeometryType,
    SceneQueryMode,
    TensorType,
)

# Set of tensor types for which articulation metadata (dof_count, body_names, etc.) is valid.
# Derived from the enum so it stays in sync automatically when new ARTICULATION_ types are added.
_ARTICULATION_TENSOR_TYPES: frozenset[int] = frozenset(t for t in TensorType if t.name.startswith("ARTICULATION_"))


class WriteGroup(NamedTuple):
    """One WRITABLE group from :meth:`PhysX.write`, the mirror of :class:`ReadGroup`.

    Every tensor is a ``warp.array`` on the native CPU or CUDA device. Non-empty tensors
    are MUTABLE VIEWS onto storage the runtime owns, not copies: fill them in place, then
    hand the group to :meth:`WriteSession.commit`. Empty tensors are Warp-owned empty
    arrays. Using a non-empty tensor after its group is committed or its session closes
    is undefined. The mapped storage is no longer caller-owned.

    Two fields ReadGroup has are absent, and their absence is the contract:

    * no ``attribute``: a session carries exactly one, named when it is opened, because
      the native group has nowhere to record it.
    * no group id: a committed group is identified by the native pointer it came from,
      which is why :meth:`WriteSession.commit` takes the group itself.
    """

    prim_list: int
    prim_offset: int
    prim_count: int
    tensors: "list"


class _WriteRelease:
    """Deferred, owning-thread teardown for a write session (ADR-0012).

    A WriteSession's group tensors are mutable aliases valid only inside the ``with`` block, so,
    unlike a read, nothing borrows the session past close and there is no refcount to keep. The
    one hazard this guards is the thread: ``WriteSession.__del__`` can fire on whatever thread the
    collector is on, and a single ovphysx instance is NOT thread-safe. So the native
    ``ovphysx_release_write`` / ``ovphysx_release_query`` is never issued from ``__del__``. It only
    enqueues, and the owning thread performs the release on its next SDK-call drain
    (``PhysX._drain_pending_write_releases``). ``_free_native`` is idempotent, so an owning-thread
    ``close()`` and a later drained ``__del__`` cannot double-free.
    """

    def __init__(self, sdk, query: int, write: int):
        self._sdk = sdk
        self._query = int(query)
        self._write = int(write)
        self._native_freed = False
        self._lock = threading.Lock()

    def _free_native(self) -> None:
        # MUST run on the owning thread (serialized with other ovphysx calls on this instance).
        # Invoked only by WriteSession.close (owner) and PhysX._drain_pending_write_releases.
        with self._lock:
            if self._native_freed:
                return
            self._native_freed = True
        lib = getattr(getattr(self._sdk, "_omni_physx_sdk_handle", None), "value", None)
        if lib is None:
            return  # parent SDK already destroyed, and the instance's handles went with it
        if self._write:
            self._sdk._lib.ovphysx_release_write(lib, self._write)
        if self._query:
            self._sdk._lib.ovphysx_release_query(lib, self._query)


class WriteSession:
    """A context-managed app -> physics write session (ADR-0012).

    The return direction of :class:`ReadResult`, and deliberately its mirror: the groups
    cover the same prims in the same order. Each tensor exposes the native residency of
    the write path. This can differ from the corresponding read when a write uses host
    staging on a GPU scene. Inspect ``tensor.device`` instead of inferring placement from
    the scene or read result.

    Anything not committed when the block exits is DISCARDED, so abandoning a fill midway
    publishes nothing rather than leaking a half-filled column into the solver::

        import warp as wp

        row = 0
        with physx.write(SimObjectType.RIGID_BODY, "position") as w:
            for g in w.groups:
                tensor = g.tensors[0]
                tensor.assign(new_positions[row : row + g.prim_count])
                if tensor.device.is_cuda:
                    stream = wp.get_stream(tensor.device)
                    w.commit(g, cuda_stream=int(stream.cuda_stream or 1))
                else:
                    w.commit(g)
                row += g.prim_count
    """

    def __init__(self, sdk, query: int, write: int, groups: "list[WriteGroup]", native: "list"):
        self._sdk = sdk
        self._query = int(query)
        self._write = int(write)
        self.groups = groups
        # The native pointers, parallel to `groups`. commit() looks a group up here because the
        # ADDRESS is the commit identity. A copy of the struct would be a pointer the runtime
        # cannot recognise.
        self._native = native
        self._closed = False
        self._release = _WriteRelease(sdk, self._query, self._write)

    def commit(self, group: "WriteGroup", cuda_stream: int = 0, cuda_wait_event: int = 0) -> None:
        """Publish one filled group to the simulation.

        Commits exactly once per group. A second commit of the same group raises, because
        commit IS the mutation and reporting success twice would tell the caller state was
        published when it was not.

        ``cuda_stream`` / ``cuda_wait_event`` hand over a GPU fill still in flight: the
        runtime orders the scatter behind them before reading the column. For
        ``cuda_stream``, ``0`` means no synchronization and ``1`` means CUDA's default
        stream.
        """
        if self._closed:
            raise RuntimeError("write session is closed")
        try:
            i = self.groups.index(group)
        except ValueError:
            raise ValueError("group does not belong to this write session") from None
        sync = ovstage_cuda_sync_t()
        sync.stream = int(cuda_stream)
        sync.wait_event = int(cuda_wait_event)
        result = self._sdk._lib.ovphysx_commit_group(
            self._sdk._omni_physx_sdk_handle.value, self._write, self._native[i], sync
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"commit failed: {self._sdk._get_last_error()}")

    def close(self) -> None:
        """Release the write session and its query. OWNING THREAD ONLY.

        Issues native ``ovphysx_release_*``, so it must run on the thread that owns the ovphysx
        instance. A single instance is not thread-safe. ``__del__`` never calls this. It only
        queues the teardown for the owning thread to drain. Group tensors are invalid once this
        returns: the runtime storage they aliased is gone.
        """
        if self._closed:
            return
        self._closed = True
        self._release._free_native()

    def __enter__(self) -> "WriteSession":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self):
        # May run on a GC thread, so never issue the native release here. A single ovphysx
        # instance is not thread-safe. Queue it for the owning thread's next drain (mirrors
        # ReadResult.__del__, which likewise only hands the teardown off).
        try:
            if not self._closed:
                self._closed = True
                self._sdk._enqueue_write_release(self._release)
        except Exception:
            pass


class ReadGroup(NamedTuple):
    """One physics-output column group returned by :meth:`PhysX.read`.

    A flattened view of the native ``ovstage_read_group_t`` the read returns (its
    ``prims`` / ``data`` / ``meta`` sub-structs unpacked into these fields). The
    interned identifiers (``attribute`` token and ``prim_list`` handle) resolve
    through the *same* process-shared ovstage path dictionary the attached Stage uses,
    so an ``ovstage.PathDictionary(stage)`` resolves them, and ``prim_list`` feeds
    straight into ``stage.query_from_path_list`` for a no-repack write-back.
    ``object_type`` is NOT part of the native group (the read is opened over one
    type). It is stamped here from the ``object_type`` passed to :meth:`PhysX.read`.

    Constant on the physics-output path (so callers can rely on them): ``is_delete``
    is always ``False`` (this read never emits tombstones). ``prim_offset`` is ``0``
    and ``prim_index_map`` is ``None`` (each group carries its own full ``prim_list``).
    ``ordinal`` is ``0`` (groups are not ordinal-stamped here).

    Attributes:
        attribute: Interned EMITTED attribute token (resolve via the path
            dictionary). May differ from the requested name: a "position" request on
            a point-instancer is emitted as "positions" (instancer-local). Write back
            using this token, not the requested string.
        object_type: The queried :class:`SimObjectType` (stamped from the read call,
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
            current during ``read()``. It does not block the host, and no other
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

    `ovphysx_set_cpu_mode(True)` keeps ovphysx's own code from loading the CUDA driver,
    but `read()` and `write()` expose `warp.array`, and building one initializes the Warp
    runtime, which opens the driver on a CUDA-enabled Warp build. A CPU-only Warp keeps
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
        cpu_only = PhysX.get_cpu_mode()
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
        "warp.array that read() or write() exposes initializes the Warp runtime, which opens the CUDA "
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
        f"ovphysx column has unsupported DLPack device type {device_type}; "
        "the Python frontend supports CPU and CUDA"
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
        self._sdk = sdk
        self._query = int(query)
        self._read = int(read)
        self._group_ids: "list[int]" = []
        self._refs = 1  # the owning ReadResult's ref, plus one per outstanding Warp array
        # Outstanding array borrows, counted apart from _refs: closing the ReadResult and dropping a
        # tensor both decrement _refs, so one live borrow leaves _refs == 1, indistinguishable from
        # an open ReadResult with none.
        self._borrows = 0
        self._groups_released = False
        self._freed = False
        self._native_freed = False
        self._lock = threading.Lock()

    def add_group(self, gid: int) -> None:
        self._group_ids.append(int(gid))  # only called during the single-threaded read() build

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
        reference, which it does whenever an array outlives its PhysX instance, a case
        ``_free_native`` deliberately supports, releasing it inline runs the whole of
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

    def release_groups(self) -> None:
        """Free the session's stage-derived prim lists. OWNING THREAD ONLY, stage still attached.

        Split from the refcounted teardown because these handles belong to the ovstage path
        dictionary, not to the read session: a borrowed array defers ``_free_native`` past
        detach/reset/release, and destroying a prim list through a dictionary whose Stage is gone is
        a use-after-free. Idempotent, because the native side clears each list it frees.

        Gated on ``_native_freed`` rather than ``_freed``, so a holder queued for teardown that has
        not run yet still retires its prim lists while the Stage is alive.
        """
        with self._lock:
            if self._groups_released or self._native_freed:
                return
            self._groups_released = True
            group_ids = list(self._group_ids)
        lib = getattr(getattr(self._sdk, "_omni_physx_sdk_handle", None), "value", None)
        if lib is None or not self._read:
            return
        for gid in group_ids:
            self._sdk._lib.ovphysx_release_group(lib, c_uint64(self._read), c_uint64(gid))

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
        self._sdk._enqueue_read_release(self)

    def _free(self) -> None:
        # Force immediate teardown on the OWNING thread, bypassing the refcount. Used only by the
        # read-build error path, which must drop every array it built FIRST, because anything
        # still holding a borrow is left aliasing freed storage. Marking _freed makes any Warp
        # deleter that fires later a no-op in release() (no enqueue, no double free).
        with self._lock:
            if self._freed:
                return
            self._freed = True
        self._free_native()

    def _free_native(self) -> None:
        # MUST run on the owning thread (serialized with other ovphysx calls on this instance).
        # Invoked only by PhysX._drain_pending_read_releases and _free above.
        with self._lock:
            if self._native_freed:
                return
            self._native_freed = True
            groups_released = self._groups_released
            self._groups_released = True
        lib = getattr(getattr(self._sdk, "_omni_physx_sdk_handle", None), "value", None)
        if lib is None:
            # Parent SDK already released. The read session is freed by its read id, not via the
            # instance, but with no owning thread left and a possible live borrow it cannot be freed
            # safely here. Its buffers stay allocated until ovphysx_shutdown reclaims the CUDA
            # context, which every borrow holds off (see retain()), so a caller's array stays readable.
            return
        if self._read:
            if not groups_released:
                # Fallback for sessions freed without ever closing their ReadResult. close() and
                # stage teardown normally retire the groups first.
                for gid in self._group_ids:
                    self._sdk._lib.ovphysx_release_group(lib, c_uint64(self._read), c_uint64(gid))
            self._sdk._lib.ovphysx_release_read(lib, c_uint64(self._read))
        if self._query:
            self._sdk._lib.ovphysx_release_query(lib, c_uint64(self._query))


class ReadResult:
    """Context-managed result of :meth:`PhysX.read` (ADR-0007).

    Holds the query + read session open so each group's interned ``prim_list`` /
    ``attribute`` handles stay valid for the lifetime of the ``with`` block. Feed
    them straight back into the ovstage write path (``stage.query_from_path_list(
    group.prim_list)``) for a no-repack write-back. Exiting releases every group's
    stage-derived path metadata and drops the result's owner reference. The read
    session and query are released immediately when no array aliases their numeric
    storage, or after the last alias is dropped and an owning-thread SDK call drains
    deferred cleanup.

    CPU and CUDA columns are borrowed ``warp.array`` snapshots. Each keeps its
    native read-session buffer alive for as long as the array, a Warp view, or a
    downstream framework view imported from it is referenced. Arrays are safe to
    keep past the ``with`` block. The buffer is freed after the last reference drops.

    Usage::

        with physx.read(SimObjectType.RIGID_BODY, ["position"]) as result:
            for g in result.groups:
                print(g.prim_list, g.attribute)
    """

    def __init__(self, sdk, query: int, read: int, groups: "list[ReadGroup]", group_ids: "list[int]",
                 release=None):
        self._sdk = sdk
        self._query = int(query)
        self._read = int(read)
        self._group_ids = list(group_ids)
        self.groups = groups
        self._closed = False
        # _iterate_read_groups builds the holder up front so aliased Warp arrays can retain it.
        # Callers with nothing to alias pass none, so synthesize one that owns the group ids.
        if release is None:
            release = _ReadRelease(sdk, query, read)
            for gid in group_ids:
                release.add_group(gid)
        self._release = release

    @property
    def dictionary(self) -> int:
        """Opaque process-shared path-dictionary pointer (resolves tokens / prim lists)."""
        return self._sdk.query_shared_dictionary(self._query) if self._query else 0

    def close(self) -> None:
        """Release the result's groups and drop its session reference. OWNING THREAD ONLY.

        Unlike dropping a returned array, this issues native calls (the stage-derived prim lists
        have to be retired while the Stage is attached), so it must run on the thread that owns the
        ovphysx instance. ``__del__`` only drops the reference, never calls this. Aliased Warp arrays
        retain the session-owned numeric storage independently and stay valid after this returns.
        """
        if self._closed:
            return
        self._closed = True
        self._release.release_groups()
        self._release.release()
        self._sdk._drain_pending_read_releases()

    def __enter__(self) -> "ReadResult":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self):
        # May run on a GC thread, so only drop the result ref. Group/native teardown is drained by
        # the next owning-thread SDK call or by the SDK's destroy path.
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

    if python_base != native_base:
        raise RuntimeError(
            "ovphysx Python package version does not match the native library. "
            f"Python: '{_python_version}' (base {python_base}), "
            f"native: '{native_version}' (base {native_base}). "
            "Reinstall the wheel or set OVPHYSX_LIB to the matching library. "
            "To bypass this check, pass ignore_version_mismatch=True."
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


def _contact_header_to_dict(h: "ContactEventHeader") -> dict:
    """Materialize a ContactEventHeader ctypes struct into a Python-owned dict.

    Used by ``get_contact_report(copy=True)`` so callers can retain contact
    data across simulation steps without zero-copy lifetime hazards.
    """
    return {
        "type": int(h.type),
        "attachHandle": int(h.attachHandle),
        "actor0": int(h.actor0),
        "actor1": int(h.actor1),
        "collider0": int(h.collider0),
        "collider1": int(h.collider1),
        "contactDataOffset": int(h.contactDataOffset),
        "numContactData": int(h.numContactData),
        "frictionAnchorsDataOffset": int(h.frictionAnchorsDataOffset),
        "numfrictionAnchorsData": int(h.numfrictionAnchorsData),
        "protoIndex0": int(h.protoIndex0),
        "protoIndex1": int(h.protoIndex1),
    }


def _contact_point_to_dict(p: "ContactPoint") -> dict:
    """Materialize a ContactPoint ctypes struct into a Python-owned dict."""
    return {
        "position": (float(p.position[0]), float(p.position[1]), float(p.position[2])),
        "normal": (float(p.normal[0]), float(p.normal[1]), float(p.normal[2])),
        "impulse": (float(p.impulse[0]), float(p.impulse[1]), float(p.impulse[2])),
        "separation": float(p.separation),
        "faceIndex0": int(p.faceIndex0),
        "faceIndex1": int(p.faceIndex1),
        "material0": int(p.material0),
        "material1": int(p.material1),
    }


def _friction_anchor_to_dict(a: "FrictionAnchor") -> dict:
    """Materialize a FrictionAnchor ctypes struct into a Python-owned dict."""
    return {
        "position": (float(a.position[0]), float(a.position[1]), float(a.position[2])),
        "impulse": (float(a.impulse[0]), float(a.impulse[1]), float(a.impulse[2])),
    }


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
    across simulation steps, but do not keep them across reset_stage(), removing USD
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
                    out_count.value, count,
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
        ``DLTensor`` buffer object, an internal cache skips DLPack acquisition
        and attribute chain lookups, giving near-raw-C-call overhead. The numpy
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
                    result = cached.c_func(cached.sdk_handle, cached.bind_handle, cached.dl_ptr)
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
        indices/mask, an internal cache skips DLPack acquisition and attribute
        chain lookups, giving near-raw-C-call overhead. Callers that want this
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
                        result = cached.c_func(cached.sdk_handle, cached.bind_handle, cached.dl_ptr, None)
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
    same CUDA device as the instance (e.g. torch/cupy tensors).

    SdfView handles are tied to the attached USD stage. After
    :meth:`PhysX.reset_stage` or :meth:`PhysX.detach_ovstage`, existing views
    are invalid. Destroy them and create replacements after re-attaching a stage.

    Example::

        with physx.create_sdf_view(pattern="/World/Mesh*", max_query_points=100) as sdf:
            pts = torch.zeros((sdf.count, 100, 3), dtype=torch.float32, device="cuda")
            out = torch.zeros((sdf.count, 100, 4), dtype=torch.float32, device="cuda")
            sdf.evaluate(pts.__dlpack__(), out.__dlpack__())
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
        reset_stage() or detach_ovstage() cleanup removed the native view.
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


class ContactBinding:
    """Contact tensor binding backed by IRigidContactView.

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
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to get contact sensor paths: {self._sdk._get_last_error()}")
            if out_count.value > self._sensor_count:
                # Same defensive log as TensorBinding.prim_paths. A native
                # out_count exceeding the buffer size is a C-library bug.
                import logging as _logging
                _logging.getLogger(__name__).warning(
                    "ovphysx_contact_binding_get_sensor_paths: native out_count=%d > buffer size=%d; truncating",
                    out_count.value, self._sensor_count,
                )
            return [str(paths_arr[i]) for i in range(min(out_count.value, self._sensor_count))]

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
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to get contact filter paths: {self._sdk._get_last_error()}")
            if out_count.value != total:
                raise RuntimeError(f"Expected {total} contact filter paths, got {out_count.value}")
            flat = [str(paths_arr[i]) for i in range(out_count.value)]
            return [
                flat[sensor_idx * self._filter_count:(sensor_idx + 1) * self._filter_count]
                for sensor_idx in range(self._sensor_count)
            ]

    def read_net_forces(self, output) -> None:
        """Read net contact forces into output. Expected shape: [sensor_count, 3].

        The dt for impulse-to-force conversion is taken automatically from the
        last successful :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            from ._dlpack_utils import acquire_dltensor

            dl_tensor, keepalive = acquire_dltensor(output)
            result = _lib.ovphysx_read_contact_net_forces(
                self._sdk._omni_physx_sdk_handle.value, self._handle, ctypes.byref(dl_tensor)
            )
            _ = keepalive
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to read net contact forces: {self._sdk._get_last_error()}")

    def read_force_matrix(self, output) -> None:
        """Read contact force matrix into output. Expected shape: [sensor_count, filter_count, 3].

        The dt for impulse-to-force conversion is taken automatically from the
        last successful :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            from ._dlpack_utils import acquire_dltensor

            dl_tensor, keepalive = acquire_dltensor(output)
            result = _lib.ovphysx_read_contact_force_matrix(
                self._sdk._omni_physx_sdk_handle.value, self._handle, ctypes.byref(dl_tensor)
            )
            _ = keepalive
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to read contact force matrix: {self._sdk._get_last_error()}")

    def read_contact_data(self, contact_forces, positions, normals, separations, counts, start_indices) -> None:
        """Read detailed contact data into flat buffers.

        Expected shapes are ``[C, 1]`` for ``contact_forces`` and ``separations``,
        ``[C, 3]`` for ``positions`` and ``normals``, and ``[sensor_count,
        filter_count]`` for ``counts`` and ``start_indices``. ``C`` is
        :attr:`max_contact_data_count`. Both ``C`` and ``filter_count`` must
        be positive. Count and start-index tensors may be int32 or uint32.
        Contact force magnitudes use the timestep from the last successful
        :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            if self._max_contact_data_count <= 0:
                raise RuntimeError(
                    "Detailed contact reads require max_contact_data_count > 0 "
                    "when creating the contact binding."
                )
            if self._filter_count <= 0:
                raise RuntimeError(
                    "Detailed contact reads require filters_per_sensor > 0 "
                    "when creating the contact binding."
                )
            from ._dlpack_utils import acquire_dltensor

            force_dl, force_keepalive = acquire_dltensor(contact_forces)
            pos_dl, pos_keepalive = acquire_dltensor(positions)
            normal_dl, normal_keepalive = acquire_dltensor(normals)
            sep_dl, sep_keepalive = acquire_dltensor(separations)
            count_dl, count_keepalive = acquire_dltensor(counts)
            start_dl, start_keepalive = acquire_dltensor(start_indices)

            result = _lib.ovphysx_read_contact_data(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(force_dl),
                ctypes.byref(pos_dl),
                ctypes.byref(normal_dl),
                ctypes.byref(sep_dl),
                ctypes.byref(count_dl),
                ctypes.byref(start_dl),
            )
            _ = (force_keepalive, pos_keepalive, normal_keepalive, sep_keepalive, count_keepalive, start_keepalive)
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to read detailed contact data: {self._sdk._get_last_error()}")

    def read_raw_contact_data(
        self,
        contact_forces,
        positions,
        normals,
        separations,
        sensor_layout,
        actor_ids,
    ) -> None:
        """Read raw (unfiltered) contact data into flat buffers.

        Filter-less variant of :meth:`read_contact_data`. Returns every
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
        :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call.

        **Truncation**: when the total contact count for a step exceeds
        ``max_contact_data_count``, the runtime fills the buffers with as many
        contacts as fit and emits a logged warning. A sensor's count reports
        only the contacts actually written, and its start index is clamped to
        ``max_contact_data_count``, so ``[start, start + count)`` is always an
        in-range (possibly empty) slice. Increase
        ``max_contact_data_count`` at binding creation if truncation occurs.

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
                    "Raw contact reads require max_contact_data_count > 0 "
                    "when creating the contact binding."
                )
            from ._dlpack_utils import acquire_dltensor

            force_dl, force_keepalive = acquire_dltensor(contact_forces)
            pos_dl, pos_keepalive = acquire_dltensor(positions)
            normal_dl, normal_keepalive = acquire_dltensor(normals)
            sep_dl, sep_keepalive = acquire_dltensor(separations)
            layout_dl, layout_keepalive = acquire_dltensor(sensor_layout)
            ids_dl, ids_keepalive = acquire_dltensor(actor_ids)

            result = _lib.ovphysx_read_raw_contact_data(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(force_dl),
                ctypes.byref(pos_dl),
                ctypes.byref(normal_dl),
                ctypes.byref(sep_dl),
                ctypes.byref(layout_dl),
                ctypes.byref(ids_dl),
            )
            _ = (force_keepalive, pos_keepalive, normal_keepalive, sep_keepalive,
                 layout_keepalive, ids_keepalive)
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to read raw contact data: {self._sdk._get_last_error()}")

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
                raise ValueError(
                    f"ids_array must be a 1D int64/uint64 tensor; got ndim={ids_dl.ndim}"
                )
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
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(
                    f"Failed to resolve actor IDs to paths: {self._sdk._get_last_error()}"
                )
            # Clamp to the provided buffer. The engine should never write
            # more than `n`, but never index past `buf` if it does.
            written = min(int(count.value), n)
            # Decode by explicit length even though returned strings guarantee
            # ptr[length] == NUL. This preserves the view contract, and direct
            # c_char_p slicing would truncate at the first NUL.
            return [str(buf[i]) for i in range(written)]

    def read_friction_data(self, friction_forces, friction_points, counts, start_indices) -> None:
        """Read detailed friction data into flat buffers.

        Expected shapes are ``[C, 3]`` for ``friction_forces`` and
        ``friction_points``, and ``[sensor_count, filter_count]`` for ``counts``
        and ``start_indices``. ``C`` is :attr:`max_contact_data_count` and must
        be positive, and ``filter_count`` must also be positive. Count and
        start-index tensors may be int32 or uint32.
        Friction entries are per-anchor. Sum each flat slice to build a
        pair-level ``[sensor_count, filter_count, 3]`` force tensor.
        Friction forces use the timestep from the last successful
        :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call.
        """
        with self._lock:
            if self._destroyed:
                raise RuntimeError("ContactBinding has been destroyed")
            self._check_sdk_valid()
            if self._max_contact_data_count <= 0:
                raise RuntimeError(
                    "Detailed friction reads require max_contact_data_count > 0 "
                    "when creating the contact binding."
                )
            if self._filter_count <= 0:
                raise RuntimeError(
                    "Detailed friction reads require filters_per_sensor > 0 "
                    "when creating the contact binding."
                )
            from ._dlpack_utils import acquire_dltensor

            force_dl, force_keepalive = acquire_dltensor(friction_forces)
            point_dl, point_keepalive = acquire_dltensor(friction_points)
            count_dl, count_keepalive = acquire_dltensor(counts)
            start_dl, start_keepalive = acquire_dltensor(start_indices)

            result = _lib.ovphysx_read_friction_data(
                self._sdk._omni_physx_sdk_handle.value,
                self._handle,
                ctypes.byref(force_dl),
                ctypes.byref(point_dl),
                ctypes.byref(count_dl),
                ctypes.byref(start_dl),
            )
            _ = (force_keepalive, point_keepalive, count_keepalive, start_keepalive)
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(f"Failed to read detailed friction data: {self._sdk._get_last_error()}")

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
# Module-level logging configuration
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

        result = _lib.ovphysx_set_log_callback(
            min_severity, filter_pointer, ctypes.cast(_callback, c_void_p), None
        )
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
_PROCESS_LIFECYCLE_INSTANCES: dict[int, weakref.ref] = {}
_PROCESS_LIFECYCLE_ATEXIT_REGISTERED = False


def _destroy_process_instances_at_exit() -> None:
    """Destroy live Python instances before interpreter finalization."""
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
    for instance in instances:
        # _finalize_unreleased() is the non-throwing finalizer policy, but at this
        # exit boundary no caller is left to recover. Isolate each instance so one
        # failure cannot strand the remaining lifecycle tokens or skip the
        # last-loader cache removal.
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

                sys.stderr.write(
                    f"PhysX process-exit cleanup failed for one instance: {exc}\n"
                )
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


class PhysX:
    """High-level wrapper around the C API using ctypes."""

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

        To force process-wide CPU-only mode, call PhysX.set_cpu_mode(True) before
        creating any PhysX instances.

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

        if not ignore_version_mismatch:
            _check_version_match()

        args = ovphysx_create_args()

        args.active_cuda_gpus = ovphysx_string_t(active_cuda_gpus or "")

        # Current wheel/source layouts use runtime discovery from lib/ and
        # plugins/, so the Python wrapper leaves bundled_deps_path empty.
        args.bundled_deps_path = ovphysx_string_t()

        all_entries = []
        if config is not None:
            from .config import _to_c_config
            all_entries.extend(_to_c_config(config))

        if all_entries:
            config_arr = (ovphysx_config_entry_t * len(all_entries))(*all_entries)
            args.config_entries = cast(config_arr, POINTER(ovphysx_config_entry_t))
            args.config_entry_count = len(all_entries)
        else:
            args.config_entries = None
            args.config_entry_count = 0

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
        # Weakly-held set of live read holders, so stage teardown can release path metadata and
        # destroy() can warn when an aliased CPU or CUDA array still owns native storage.
        self._live_read_holders: "weakref.WeakSet[_ReadRelease]" = weakref.WeakSet()
        try:
            _acquire_process_lifecycle(self)
        except BaseException:
            if self._lifecycle_acquired:
                self._lifecycle_acquired = False
                _release_process_lifecycle()
            raise

        try:
            result = self._lib.ovphysx_create_instance(byref(args), byref(self._omni_physx_sdk_handle))
            if result.status != ApiStatus.SUCCESS:
                error_msg = self._get_last_error()
                raise RuntimeError(f"ovphysx_create_instance() failed: {error_msg}")
            if self._omni_physx_sdk_handle.value == _INVALID_HANDLE:
                self._omni_physx_sdk_handle = None
                raise RuntimeError("ovphysx_create_instance() returned invalid handle")
        except Exception:
            if self._lifecycle_acquired:
                self._lifecycle_acquired = False
                _release_process_lifecycle()
            raise

        # Track explicit-destruction state for the ResourceWarning finalizer.
        self._released = False
        _track_process_instance(self)

    def _check_valid(self) -> None:
        if self._omni_physx_sdk_handle is None:
            raise RuntimeError(
                "PhysX instance has been destroyed. Create a new PhysX() instance."
            )

    @property
    def handle(self) -> int:
        """The raw ``ovphysx_handle_t`` for this instance (read-only).

        Use this when passing the handle to C/C++ code that calls the
        ovphysx C API directly.

        Raises:
            RuntimeError: If the instance has been destroyed.
        """
        self._check_valid()
        return self._omni_physx_sdk_handle.value

    @staticmethod
    def set_cpu_mode(cpu_only: bool) -> None:
        """Force process-wide CPU-only mode.

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
                f"set_cpu_mode failed ({ApiStatus(result.status).name}): "
                f"{_get_last_error_from_lib()}"
            )

    @staticmethod
    def get_cpu_mode() -> bool:
        """Return whether process-wide hard CPU-only mode is in effect.

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
                f"get_cpu_mode failed ({ApiStatus(result.status).name}): "
                f"{_get_last_error_from_lib()}"
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


    def reset_stage(self) -> int:
        """Reset stage to empty (async).

        Returns:
            op_index (can be used with wait_op() for explicit synchronization)

        Example:
            # Simple usage (stream-ordered)
            physx.reset_stage()
            physx.wait_all()

        Preconditions:
            - Instance must be valid.
        Side effects:
            - Clears the runtime stage.
            - Detaches any attached ovstage Stage (the C runtime calls
              detach_ovstage internally). Callers must re-attach with
              attach_ovstage() before any further update_from_ovstage().
        Ownership/Lifetime:
            - All TensorBinding, ContactBinding, and SdfView objects for the previous
              stage become invalid. Destroy cached bindings and SDF views before reset
              when practical. If a stale handle survives, only destroy it. Create
              replacement bindings and SDF views after the reset completes.
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises RuntimeError on failure.
        """
        self._check_valid()
        # reset_stage detaches internally and invalidates read sessions. Free any pending teardown
        # first, plus the stage-derived handles of sessions still borrowed by a live Warp array
        # (those are not pending and would otherwise be freed after the Stage is gone).
        self._drain_pending_read_releases()
        self._release_stage_bound_read_handles()
        result = self._lib.ovphysx_reset_stage(self._omni_physx_sdk_handle.value)

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to reset_stage: {error_msg}")

        # reset_stage detaches any attached ovstage in the C runtime. Drop the
        # Python keepalive to match (see attach_ovstage / detach_ovstage).
        self._attached_ovstage = None
        return result.op_index

    def clone(self, source_path: str, target_paths: list[str],
              anchor_transforms: list[tuple[float, float, float, float, float, float, float]] | None = None,
              env_ids: list[int] | None = None) -> int:
        """Clone a prim hierarchy to create multiple runtime physics copies.

        Creates physics-optimized clones in the runtime representation for high-performance
        simulation, backed by the PhysX SDK replicator so cloned articulations are real articulations.
        The source prim must exist in the loaded stage and have physics properties.
        Replication executes inline. The returned operation index is already complete, so
        subsequent operations see the clone immediately and ``wait_op()`` returns immediately.

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

            env_ids = [0, 1]  # same ids in every call -> same logical environments
            physx.clone("/env0/Robot", ["/env1/Robot", "/env2/Robot"], env_ids=env_ids)
            physx.clone("/env0/Object", ["/env1/Object", "/env2/Object"], env_ids=env_ids)

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
            op_index (can be used with wait_op() for explicit synchronization)

        Raises:
            ValueError: If paths are invalid, ``anchor_transforms`` has the wrong
                length or contains an invalid pose, or ``env_ids`` has the wrong
                length or contains an invalid id.
            RuntimeError: If clone fails to queue, if no USD scene is loaded,
                or if clone() is called after :meth:`warmup` or the first
                :meth:`step` / :meth:`step_sync`. Cloning after warmup
                corrupts simulation state on GPU and is rejected in all modes
                for API consistency. To recover, call :meth:`reset_stage`, wait
                for it to complete, then reload the source scene or reattach
                its ovstage before cloning again.

        Preconditions:
            - A USD stage is loaded and source_path exists.
            - target_paths are unique and do not already exist.
            - :meth:`warmup` has not been called and no :meth:`step` /
              :meth:`step_sync` has run since the current stage was attached.
        Side effects:
            - Creates live PhysX objects keyed by the target paths. No USD or runtime-stage
              prims are authored.
        Ownership/Lifetime:
            - Clones remain valid until reset_stage().
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises ValueError for invalid inputs.
            - Raises RuntimeError on internal failure, including
              duplicate-target and after-step/after-warmup ordering violations.
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
                        f"anchor_transforms[{i}] must have exactly 7 values "
                        f"(px,py,pz,qx,qy,qz,qw), got {len(vals)}"
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
                raise ValueError(
                    f"env_ids length ({len(env_ids)}) must match target_paths length ({num_targets})"
                )
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
            self._omni_physx_sdk_handle.value, ovphysx_string_t(source_path),
            target_array, c_uint32(num_targets), xform_ptr, env_id_ptr
        )
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to clone: {error_msg}")

        return result.op_index

    def get_object_type(self, prim_path: str) -> ObjectType:
        """Classify an authored USD prim by TensorAPI object type.

        See :class:`ObjectType` for the taxonomy. Paths with no classified
        simulation object return ``ObjectType.INVALID`` with success -- the call
        did not fail, the path just isn't a known simulation object. Live
        standalone, custom, and articulation joints at their authored prim paths
        must not return ``ObjectType.INVALID``.

        Raises ``RuntimeError`` for invalid input (empty path, embedded NUL
        byte) or if no stage is attached.

        Returns:
            ObjectType: One of RIGID_BODY, ARTICULATION, ARTICULATION_LINK,
            ARTICULATION_ROOT_LINK, ARTICULATION_JOINT, JOINT, CUSTOM_JOINT, or
            INVALID.
        """
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
        """Start a late OmniPVD recording session.

        The shared runtime must have recording capability selected before the
        first instance is created, either explicitly with
        ``PhysXConfig(omnipvd_recording_capable=True)`` or implicitly with
        ``omnipvd_output_enabled=True``. Once an inactive capable runtime is
        established, an unconfigured peer may start a late session. A runtime
        created without capability deliberately rejects late start with
        ``INVALID_STATE`` and an error naming ``omnipvd_recording_capable``.
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
        result = self._lib.ovphysx_start_recording(
            self._omni_physx_sdk_handle.value, byref(c_destination)
        )
        if result.status != ApiStatus.SUCCESS:
            status_name = ApiStatus(result.status).name
            raise RuntimeError(f"start_recording failed ({status_name}): {self._get_last_error()}")

    def stop_recording(self) -> None:
        """Stop and finalize the OmniPVD recording owned by this instance.

        This also stops a startup session owned by the instance that created
        the shared runtime. Peer instances cannot stop that owner's session.
        """
        self._check_valid()
        result = self._lib.ovphysx_stop_recording(self._omni_physx_sdk_handle.value)
        if result.status != ApiStatus.SUCCESS:
            status_name = ApiStatus(result.status).name
            raise RuntimeError(f"stop_recording failed ({status_name}): {self._get_last_error()}")

    def is_recording(self) -> bool:
        """Return ``True`` only while this instance owns active sampling.

        A peer reports ``False`` while another instance owns a startup or late
        session.
        """
        self._check_valid()
        recording = ctypes.c_bool(False)
        result = self._lib.ovphysx_is_recording(
            self._omni_physx_sdk_handle.value, byref(recording)
        )
        if result.status != ApiStatus.SUCCESS:
            status_name = ApiStatus(result.status).name
            raise RuntimeError(f"is_recording failed ({status_name}): {self._get_last_error()}")
        return bool(recording.value)

    def step(self, dt: float) -> int:
        """Initiate physics step (async, returns op_index).

        Simulation time is tracked internally. Each step advances it by ``dt``.

        Args:
            dt: Delta time for this step [s].

        Returns:
            op_index (can be used with wait_op() for explicit synchronization)

        Examples:
            # Simple usage (stream-ordered)
            physx.step(0.016)
            binding.read(output)  # Automatically waits for step

            # Explicit wait (if accessing results outside stream)
            op = physx.step(0.016)
            physx.wait_op(op)  # Ensure step completes before external GPU work

        Preconditions:
            - A USD stage is loaded if physics content is expected.
        Side effects:
            - Advances simulation time and mutates physics state.
        Ownership/Lifetime:
            - Returned op_index is single-use and must be waited once if needed.
        Threading:
            - Do not call concurrently on the same instance without external sync.
        Errors:
            - Raises RuntimeError on failure to enqueue.
        """
        self._check_valid()
        self._drain_pending_read_releases()  # owning-thread drain of any deferred read teardown
        result = self._lib.ovphysx_step(self._omni_physx_sdk_handle.value, c_float(dt))

        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to step: {error_msg}")

        return result.op_index

    def step_sync(self, dt: float) -> None:
        """Step simulation and wait for completion in a single call.

        Faster than ``step()`` + ``wait_op()`` for performance-critical
        applications like RL training that always wait immediately.
        Simulation time is tracked internally and advanced by ``dt``.

        Args:
            dt: Delta time [s] for this step.

        Raises:
            RuntimeError: If the step or wait fails.
        """
        self._check_valid()
        self._drain_pending_read_releases()
        result = self._lib.ovphysx_step_sync(self._omni_physx_sdk_handle.value, c_float(dt))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"step_sync failed: {error_msg}")

    def step_n_sync(self, n: int, dt: float) -> None:
        """Run N steps in a single C call, saving (N-1) ctypes round-trips.

        Equivalent to calling ``step_sync(dt)`` n times, but with only one
        Python-to-C transition. Simulation time is tracked internally and
        advanced by ``n * dt``.

        Args:
            n: Number of steps to run (must be >= 1).
            dt: Duration of each step [s].

        Raises:
            RuntimeError: If any step fails.
        """
        self._check_valid()
        self._drain_pending_read_releases()
        result = self._lib.ovphysx_step_n_sync(
            self._omni_physx_sdk_handle.value, c_int32(n), c_float(dt)
        )
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"step_n_sync failed: {error_msg}")

    def update_articulations_kinematic(self) -> None:
        """Update articulation link poses from current joint positions.

        This performs a synchronous articulation forward-kinematics update
        without running a normal simulation step, collision detection, or
        contact generation. Call it after writing articulation DOF positions
        and before reading articulation link pose tensors when fresh link poses
        are needed in the same frame.

        In GPU mode, the first kinematic update after loading USD may perform
        the same automatic DirectGPU warmup step used by tensor reads/writes.

        Raises:
            RuntimeError: If the update fails.
        """
        self._check_valid()
        result = self._lib.ovphysx_update_articulations_kinematic(
            self._omni_physx_sdk_handle.value
        )
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"update_articulations_kinematic failed: {error_msg}")

    def wait_op(self, op_index: int, *, timeout_ns: int | None = None) -> None:
        """Wait for operation(s) to complete.

        Args:
            op_index: Operation index to wait for, or OP_INDEX_ALL for all ops
            timeout_ns: Readiness timeout in nanoseconds. None waits
                indefinitely, and 0 performs one non-blocking readiness poll.
                A positive value bounds only the wait for readiness; once an
                operation is ready, synchronous result finalization may make
                the total call duration exceed this timeout.

        Raises:
            RuntimeError: If an operation failed or op_index is invalid or already consumed.
            TimeoutError: If timeout expired (e.g., when polling with timeout_ns=0 and the operation is not ready)

        Preconditions:
            - op_index must be valid and not previously consumed.
        Side effects:
            - Consumes each completed or failed operation reached up to op_index.
            - An operation still pending when the wait times out is not consumed.
            - An index completed by internal stream synchronization may be
              acknowledged once; that acknowledgement consumes it.
        Ownership/Lifetime:
            - Wait-result storage is released internally.
            - Error strings are borrowed and remain valid until the next API
              call on the same thread.
        Threading:
            - Serialize calls on one PhysX instance externally.
            - Do not wait on the same op_index from multiple threads.

        Examples::

            def wait_for_operation(physx, op_index):
                # Blocking wait (default)
                physx.wait_op(op_index)

            def poll_operation(physx, op_index):
                # Non-blocking poll
                try:
                    physx.wait_op(op_index, timeout_ns=0)
                except TimeoutError:
                    return False
                return True
        """
        self._check_valid()
        if timeout_ns is None:
            timeout_ns = 0xFFFFFFFFFFFFFFFF  # Infinite wait

        wait_result = ovphysx_op_wait_result_t()
        result = self._lib.ovphysx_wait_op(self._omni_physx_sdk_handle.value, op_index, timeout_ns, byref(wait_result))

        try:
            if wait_result.num_errors > 0:
                errors = []
                for i in range(wait_result.num_errors):
                    failed_op = wait_result.error_op_indices[i]
                    err_str = self._lib.ovphysx_get_last_op_error(failed_op)
                    error_msg = str(err_str) if err_str.ptr else "Unknown error"
                    errors.append(f"op {failed_op}: {error_msg}")

                raise RuntimeError("Operation(s) failed:\n  " + "\n  ".join(errors))

            if result.status == ApiStatus.TIMEOUT:
                raise TimeoutError(f"Operation {op_index} timed out")

            if result.status != ApiStatus.SUCCESS:
                error_msg = self._get_last_error()
                raise RuntimeError(f"wait_op failed: {error_msg}")
        finally:
            self._lib.ovphysx_destroy_wait_result(byref(wait_result))

    def wait_all(self, *, timeout_ns: int | None = None) -> None:
        """Wait for all pending operations (convenience wrapper for wait_op(ALL)).

        Args:
            timeout_ns: Readiness timeout in nanoseconds, with the same
                semantics as :meth:`wait_op`.

        Preconditions:
            - Instance must be valid.
        Side effects:
            - Consumes each completed or failed operation reached before
              success or timeout.
        Threading:
            - Serialize all calls on the same instance externally.
        Errors:
            - Raises RuntimeError on failure.
            - Raises TimeoutError if timeout expired (e.g., when polling with timeout_ns=0 and operations are not ready).
        """
        # _check_valid() is called inside wait_op()
        self.wait_op(OP_INDEX_ALL, timeout_ns=timeout_ns)

    def attach_ovstage(self, stage, *, read_ordinal: int = 1) -> None:
        """Attach an ovstage Stage as the orchestration data surface.

        Attach performs the initial scene parse at ``read_ordinal``. After the
        producer authors later ovstage edits, call :meth:`update_from_ovstage`
        with only those subsequent ordinals. Tensor bindings remain available
        as a perf escape hatch.

        Args:
            stage: An ``ovstage.Stage`` or a raw ``ovstage_instance_t*`` handle.
            read_ordinal: Caller-owned ovstage ordinal at which selected physics
                data is sealed. Must be non-zero. 0 is reserved as the runtime
                skip-cursor sentinel. The application owns ordinal advancement.
                Defaults to 1. ``open_usd()`` / population does not seal data,
                so call ``advance_write_floor()`` first. Attachment fails if the
                initial articulation/joint schema scan cannot read that ordinal.

        Preconditions:
            - Instance must be valid.
            - Not already attached to a Stage.
            - No other instance owns the process-wide live ovstage attach.
            - Selected physics data must be sealed at ``read_ordinal``.
            - ``read_ordinal`` must be non-zero.
            - The application registered ovphysx's codeless PhysX schemas with
              ovstage before the first population in the process
              (``ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])``).
              Population drops every Physx* API it cannot resolve, so an
              unregistered stage carries none of the asset's PhysX settings.
              This call verifies the registration and raises ``RuntimeError``
              naming the missing call when population ran without it (for the
              rest of the process; the Carbonite setting
              ``/ovphysx/schemas/requireRegistration = false`` downgrades this
              to a warning).

        Lifetime:
            - ``stage`` must outlive the attachment because ovphysx captures and
              dereferences its native pointer until detach. This wrapper holds a
              reference to ``stage`` for the duration of the attachment, so a
              Stage created inline (``attach_ovstage(ovstage.Stage(...))``) stays
              alive. The reference is dropped by :meth:`detach_ovstage` and
              :meth:`destroy`.

        Errors:
            - Raises ``ValueError`` if ``read_ordinal`` is 0.
            - Raises ``RuntimeError`` if already attached, another instance owns
              the live process-wide attach, ``stage`` is null, or the runtime
              attach fails. Instance remains unattached on failure.
        """
        self._check_valid()
        # Drain sessions that no longer have an owner before attaching. Stage switching must go
        # through detach_ovstage(), which releases stage-derived handles while the old Stage is
        # still valid. A failed attach attempt must not invalidate metadata from the current Stage.
        self._drain_pending_read_releases()
        if stage is None:
            raise RuntimeError("attach_ovstage: stage is None")
        if hasattr(stage, "handle"):
            ptr = stage.handle()
        elif hasattr(stage, "_inst"):
            ptr = ctypes.cast(stage._inst, ctypes.c_void_p).value
        else:
            ptr = int(stage)
        if not ptr:
            raise RuntimeError("attach_ovstage: stage handle is null")
        if int(read_ordinal) == 0:
            raise ValueError(
                "attach_ovstage: read_ordinal must be a caller-owned sealed ordinal; 0 is reserved")
        result = self._lib.ovphysx_attach_ovstage(
            self._omni_physx_sdk_handle.value, ctypes.c_void_p(ptr), c_uint64(read_ordinal))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Failed to attach ovstage: {self._get_last_error()}")
        # Keep the Stage alive for the duration of the attachment: ovphysx retains
        # its raw native pointer until detach. Without this, a caller that does not
        # retain its own reference would have the Stage GC'd (its __del__ destroys
        # the native instance), and later stage-dependent calls would use freed memory.
        self._attached_ovstage = stage

    def update_from_ovstage(self, from_ordinal: int, to_ordinal: int) -> None:
        """Apply committed ovstage edits over the closed range ``[from_ordinal, to_ordinal]``.

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
        result = self._lib.ovphysx_update_from_ovstage(
            self._omni_physx_sdk_handle.value, rng)
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"update_from_ovstage failed: {self._get_last_error()}")

    def detach_ovstage(self) -> None:
        """Detach the currently-attached ovstage Stage.

        Idempotent: calling on an unattached instance is a no-op success.
        Clears registered interests and output-buffer registrations, so a
        subsequent :meth:`attach_ovstage` to a different Stage starts clean.
        After detach, stage-dependent calls such as :meth:`update_from_ovstage`
        and :meth:`step` fail until a Stage is attached again. Detach invalidates
        the stage's tensor, contact, and SDF views. Do not read, write, or
        evaluate existing bindings or SDF views. Destroy them and create
        replacements after calling :meth:`attach_ovstage` and realizing a stage
        again. If this instance owns an active OmniPVD recording, detach stops
        and finalizes it. On reattach, capability-only recording is dormant and
        can start immediately. Configured startup output instead starts a new
        startup session owned by the reattaching instance. Stop it before
        starting a late destination.

        Errors:
            - Raises ``RuntimeError`` on internal failures.
        """
        self._check_valid()
        # Free any read session pending teardown before detaching: detach invalidates a session's
        # stage-derived data, so draining a stale handle afterward would crash. Sessions still
        # borrowed by a live Warp array are not pending, so drop their stage-derived handles
        # explicitly as well. Their numeric buffers stay valid for the borrow.
        self._drain_pending_read_releases()
        self._release_stage_bound_read_handles()
        result = self._lib.ovphysx_detach_ovstage(self._omni_physx_sdk_handle.value)
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Failed to detach ovstage: {self._get_last_error()}")
        # Native no longer references the Stage. Drop the keepalive.
        self._attached_ovstage = None

    def _release_stage_bound_read_handles(self) -> None:
        """Free every live read session's prim lists before the Stage they belong to goes away.

        A Warp array may legally outlive its ``ReadResult``, keeping its session alive past detach /
        reset / release. That session still holds prim lists interned in the Stage's path dictionary,
        so freeing it later, after the Stage is gone, would destroy them through a dangling
        dictionary. Releasing the groups here (owning thread, Stage still attached) leaves the late
        teardown with only session-owned numeric buffers to free, which stay valid for the borrow.
        """
        for holder in list(self._live_read_holders):
            try:
                holder.release_groups()
            except Exception:
                pass

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
        with self._pending_read_lock:
            pending = self._pending_read_releases
            self._pending_read_releases = []
        for holder in pending:
            holder._free_native()
        # Write sessions defer to the same owning thread, so retire them on the same drain. Every
        # entry point that drains reads then also retires any GC-orphaned write session.
        self._drain_pending_write_releases()

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
        for holder in pending:
            holder._free_native()

    def get_attach_handle(self) -> int:
        """Return the handle identifying this instance's current attach.

        An attach handle is an attach *identity*, not a USD stage id: it is
        nonzero for every live attach (including an ovstage attach whose source
        has no backing USD stage), and a fresh handle is minted per attach. A
        caller that stores it when it binds can tell "still the attach I bound
        to" apart from "detached" and from "a different attach that happens to
        reuse the same stage id". See ADR-0016.

        Returns:
            The current attach handle, or ``0`` when nothing is attached.

        Errors:
            - Raises ``RuntimeError`` if the instance is invalid.
        """
        self._check_valid()
        out_handle = c_uint64(0)
        result = self._lib.ovphysx_get_attach_handle(
            self._omni_physx_sdk_handle.value, byref(out_handle))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Failed to get attach handle: {self._get_last_error()}")
        return int(out_handle.value)

    def read(
        self,
        object_type: SimObjectType,
        attribute_names: "list[str]",
        *,
        scope: ObjectScope = ObjectScope.ALL,
    ) -> "ReadResult":
        """Read physics output (ADR-0007) for one simulated type as column groups.

        Mirrors the ovstage read idiom: open a query over ``object_type`` in
        ``scope``, read the named ``attribute_names`` (e.g. ``["position",
        "orientation"]``), and return a context-managed :class:`ReadResult` whose
        ``groups`` is one :class:`ReadGroup` per typed column. The read is
        ovstage-native. Attach an ovstage Stage first.

        Use it as a context manager: the query + read session stay open for the
        ``with`` block so each group's interned ``prim_list`` / ``attribute``
        handles are valid. Feed them straight into the ovstage write path
        (``stage.query_from_path_list(group.prim_list)``) for a no-repack
        write-back. Group ``tensors`` are ``warp.array`` snapshots on the native
        CPU or CUDA device. They keep their read-session storage alive until the
        array, its Warp views, and any downstream framework views are dropped, so
        they are safe to keep past the block.

        This is the *physics -> app* direction. To avoid physics consuming its own
        output, write the data back into ovstage at ordinals that are never passed
        to :meth:`update_from_ovstage`. See the ovstage Integration guide for the
        ordinal-coupling principle.

        **Step at least once before reading on DirectGPU.** On a DirectGPU scene, simulated state
        columns for ``RIGID_BODY``, ``ARTICULATION_LINK``, ``ARTICULATION``, and
        ``ARTICULATION_JOINT`` come from PhysX's direct-GPU API, which sizes its
        structures during the first simulation step and refuses reads until that step
        has run. Whole-articulation shape/material columns remain on the CPU, so one
        result can mix devices. Reading before the first step returns **no groups** for
        those types even though the objects exist and a query reports them. Step once,
        then read. CPU scenes can report authored initial state once buffered scene
        insertion is complete; a still-pending articulation root or joint partition is
        omitted normally rather than reported as an error.

        Args:
            object_type: Simulated type to read (:class:`SimObjectType`).
            attribute_names: Semantic attribute names to read.
            scope: ``ALL`` or ``ACTIVE`` (active is single-frame).

        Returns:
            A :class:`ReadResult` context manager. ``result.groups`` is empty if no
            objects matched, and, on a DirectGPU scene before the first step, for the
            direct-GPU-sourced types described above. Readiness is evaluated per scene,
            so a multi-scene result can contain ready partitions while omitting an
            unready DirectGPU scene's partition.

        Raises:
            RuntimeError: on a native error (e.g. no ovstage attached).
            TypeError: if a returned column carries a device or DLPack dtype this
                Warp frontend does not support.
        """
        self._check_valid()
        # Owning-thread drain: free any read session whose last borrow was dropped on a GC thread.
        self._drain_pending_read_releases()
        handle = self._omni_physx_sdk_handle.value

        query = c_uint64(0)
        result = self._lib.ovphysx_query(
            handle, int(object_type), int(scope), ctypes.byref(query))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"read query failed: {self._get_last_error()}")
        if query.value == 0:
            # Defensive: ovphysx_query reports a nonzero handle on success (an empty
            # match included) and surfaces 0 as an error above. Treat a stray 0 as
            # an empty result rather than dereferencing it.
            return ReadResult(self, 0, 0, [], [])

        # Build the ovx_string_or_token_t array from string names (token = 0).
        names = list(attribute_names)
        name_bytes = [n.encode("utf-8") for n in names]  # keepalive across the call
        arr = (ovx_string_or_token_t * len(names))()
        for i, b in enumerate(name_bytes):
            arr[i].token = 0
            arr[i].string = ovx_string_t(b, len(b))
        read = self._open_read_session(handle, query, arr, len(names))
        return self._iterate_read_groups(handle, query, read, object_type)

    def read_tokens(
        self,
        object_type: SimObjectType,
        attribute_tokens: "list[int]",
        *,
        scope: ObjectScope = ObjectScope.ALL,
    ) -> "ReadResult":
        """Token form of :meth:`read`.

        Identical to :meth:`read` but the attributes are given as interned attribute
        tokens (e.g. an emitted :attr:`ReadGroup.attribute`, or a token obtained
        through the C query API) instead of strings, so a token can be fed straight
        back in with no token-to-string-to-name round-trip. Both forms build the same
        ``ovx_string_or_token_t`` array under the hood.

        Args:
            object_type: Simulated type to read (:class:`SimObjectType`).
            attribute_tokens: Interned attribute tokens to read.
            scope: ``ALL`` or ``ACTIVE`` (active is single-frame).

        Returns:
            A :class:`ReadResult` context manager (see :meth:`read`).

        Raises:
            RuntimeError: on a native error (e.g. no ovstage attached).
            TypeError: if a returned column carries a device or DLPack dtype this
                Warp frontend does not support.
        """
        self._check_valid()
        self._drain_pending_read_releases()  # owning-thread drain of any deferred read teardown
        handle = self._omni_physx_sdk_handle.value

        query = c_uint64(0)
        result = self._lib.ovphysx_query(
            handle, int(object_type), int(scope), ctypes.byref(query))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"read query failed: {self._get_last_error()}")
        if query.value == 0:
            return ReadResult(self, 0, 0, [], [])

        toks = [int(t) for t in attribute_tokens]
        arr = (ovx_string_or_token_t * len(toks))()
        for i, t in enumerate(toks):
            arr[i].token = t
            arr[i].string = ovx_string_t(None, 0)
        read = self._open_read_session(handle, query, arr, len(toks))
        return self._iterate_read_groups(handle, query, read, object_type)

    def _open_read_session(self, handle, query, attrs_array, count) -> "object":
        """Open a read session over an ovx_string_or_token_t array (shared by read / read_tokens)."""
        read = c_uint64(0)
        result = self._lib.ovphysx_read(handle, query, attrs_array, count, ctypes.byref(read))
        if result.status != ApiStatus.SUCCESS:
            self._lib.ovphysx_release_query(handle, query)
            raise RuntimeError(f"read session open failed: {self._get_last_error()}")
        return read

    def _iterate_read_groups(self, handle, query, read, object_type) -> "ReadResult":
        """Drain an open read session into a :class:`ReadResult` (shared by read / read_tokens).

        The native group is ovstage's ``ovstage_read_group_t`` (no ovphysx mirror). The
        queried ``object_type`` is stamped on each :class:`ReadGroup` from the caller's
        request, since the native group does not carry it.
        """
        groups: list[ReadGroup] = []
        group_ids: list[int] = []
        # Refcount holder: every non-empty Warp array retains the native read session so its
        # aliased CPU or CUDA buffer outlives ReadResult.close() until every borrow is dropped.
        release = _ReadRelease(self, int(query.value), int(read.value))
        self._live_read_holders.add(release)  # tracked for stage teardown and destroy() preconditions
        wp = None
        try:
            while True:
                # Producer-owned group: fetch hands back a borrowed ovstage_read_group_t*
                # (valid until the group/read is released), not a caller-filled struct.
                gp = ctypes.POINTER(ovstage_read_group_t)()
                result = self._lib.ovphysx_fetch_read_next(handle, read, ctypes.byref(gp))
                if result.status == ApiStatus.END_OF_ITERATION:
                    break
                if result.status != ApiStatus.SUCCESS:
                    raise RuntimeError(f"read fetch failed: {self._get_last_error()}")
                if not gp:
                    break
                g = gp.contents
                read_group_id = int(g.read_group_id)
                # Track the group before any fallible metadata conversion so the error path can
                # release this fetched group as well as the enclosing read and query handles.
                group_ids.append(read_group_id)
                release.add_group(read_group_id)

                tensor_count = int(g.data.tensor_count)
                # Imported here rather than before the loop: building the first array initializes
                # the Warp runtime, and a CUDA-enabled Warp build loads the CUDA driver there,
                # outside set_cpu_mode's ovphysx-only guarantee. Importing Warp is itself
                # driverless. CONSTRUCTING the first array is the boundary.
                #
                # Deferred, not moved out of the try: a missing or broken Warp installation must
                # still release the already-open read and query rather than leak both handles.
                if wp is None:
                    import warp as wp
                    _warn_if_warp_build_breaks_cpu_mode(wp)

                # One event per group covers every column it produced, and wrapping a pointer
                # carries no readiness, so order Warp's current stream against it once here,
                # before any of the group's arrays reaches the caller.
                self._order_warp_stream_after_group(g, wp)
                tensors = [
                    self._dltensor_to_warp_array(g.data.tensors[i], wp, release)
                    for i in range(tensor_count)
                ] if g.data.tensors else []
                index_map = self._u32_warp_array(g.data.index_map, int(g.data.count), wp, release)
                prim_index_map = self._u32_warp_array(g.prims.index_map, int(g.prims.count), wp, release)
                groups.append(
                    ReadGroup(
                        attribute=int(g.attribute),
                        object_type=SimObjectType(int(object_type)),
                        ordinal=int(g.ordinal),
                        is_array=bool(g.is_array),
                        is_delete=bool(g.is_delete),
                        semantic=int(g.semantic),
                        prim_list=int(g.prims.list),
                        prim_offset=int(g.prims.offset),
                        prim_count=int(g.prims.count),
                        prim_index_map=prim_index_map,
                        index_map=index_map,
                        layout_generation=int(g.meta.layout_generation),
                        write_floor_ordinal=int(g.meta.attribute_write_floor_ordinal),
                        tensors=tensors,
                        cuda_stream=int(g.data.cuda_sync.stream),
                        cuda_wait_event=int(g.data.cuda_sync.wait_event),
                    )
                )
        except Exception:
            # Drop every array this call already built BEFORE the native teardown. `_free()`
            # bypasses the refcount, and `groups` holds fully-built warp.arrays from earlier
            # iterations that alias storage this session owns. `tensors`/`index_map`/
            # `prim_index_map` may still be bound to the previous iteration's arrays when the
            # failure lands before this iteration reassigns them, so they are cleared as well.
            # The Warp deleter is `release_borrow`, which drops a borrow without triggering
            # teardown, so `_free()` below still performs the immediate native release.
            groups.clear()
            tensors = index_map = prim_index_map = None  # noqa: F841 -- drop borrows, not values
            release._free()
            raise

        return ReadResult(self, int(query.value), int(read.value), groups, group_ids, release=release)

    def write(
        self,
        object_type: SimObjectType,
        attribute_name: str,
        *,
        scope: ObjectScope = ObjectScope.ALL,
    ) -> "WriteSession":
        """Open an app -> physics write session for ONE attribute (ADR-0012).

        The return direction of :meth:`read`, and its mirror: the groups cover the same
        prims in the same order. Each tensor exposes the native residency of the write
        path. This can differ from the corresponding read when a write uses host staging
        on a GPU scene. Inspect ``tensor.device`` instead of inferring placement from the
        scene or read result.

        One attribute per session, unlike :meth:`read`'s list: the native group carries
        no attribute field, so a session that mixed attributes could not label its
        groups. Writing position and orientation is two sessions.

        Every group tensor is a ``warp.array`` on its native CPU or CUDA device.
        Non-empty tensors are MUTABLE VIEWS onto runtime-owned storage. Empty tensors are
        Warp-owned empty arrays. Fill a non-empty tensor, then
        :meth:`WriteSession.commit` its group. Anything left uncommitted when the block
        exits is discarded rather than published.

        An attribute the type does not accept raises here rather than silently writing
        nothing.

        Raises RuntimeError if no ovstage Stage is attached, or if the attribute is not
        writable for ``object_type``.
        """
        self._check_valid()
        self._drain_pending_read_releases()  # owning-thread drain of any deferred read teardown
        handle = self._omni_physx_sdk_handle.value

        query = c_uint64(0)
        result = self._lib.ovphysx_query(handle, int(object_type), int(scope), ctypes.byref(query))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"write query failed: {self._get_last_error()}")

        name = attribute_name.encode("utf-8")
        attr = ovx_string_or_token_t()
        attr.token = 0
        attr.string = ovx_string_t(name, len(name))

        write = c_uint64(0)
        result = self._lib.ovphysx_write(handle, query, ctypes.byref(attr), ctypes.byref(write))
        if result.status != ApiStatus.SUCCESS:
            self._lib.ovphysx_release_query(handle, query)
            raise RuntimeError(f"write session open failed: {self._get_last_error()}")

        groups: list[WriteGroup] = []
        native: list = []
        wp = None
        try:
            while True:
                gp = ctypes.POINTER(ovstage_map_group_t)()
                result = self._lib.ovphysx_fetch_write_next(handle, write, ctypes.byref(gp))
                if result.status == ApiStatus.END_OF_ITERATION:
                    break
                if result.status != ApiStatus.SUCCESS:
                    raise RuntimeError(f"write fetch failed: {self._get_last_error()}")
                if not gp:
                    break
                g = gp.contents
                # Deferred exactly as the read defers it: importing Warp is driverless, but
                # CONSTRUCTING the first array opens the CUDA driver on a CUDA-enabled build,
                # outside set_cpu_mode's ovphysx-only guarantee. Kept inside the try so a
                # missing or broken Warp still releases the open write and query.
                if wp is None:
                    import warp as wp
                    _warn_if_warp_build_breaks_cpu_mode(wp)
                tensors = [
                    self._dltensor_to_warp_array(g.data.tensors[i], wp, None)
                    for i in range(int(g.data.tensor_count))
                ] if g.data.tensors else []
                groups.append(
                    WriteGroup(
                        prim_list=int(g.prims.list),
                        prim_offset=int(g.prims.offset),
                        prim_count=int(g.prims.count),
                        tensors=tensors,
                    )
                )
                # The POINTER, not a copy of the struct: the address is the commit identity.
                native.append(gp)
        except Exception:
            # Drop every array and native group pointer before releasing their backing
            # session. A later-group conversion failure can leave successful earlier
            # groups reachable through both `groups` and the previous `tensors` local.
            groups.clear()
            native.clear()
            tensors = gp = g = None  # noqa: F841 -- drop aliases before native teardown
            self._lib.ovphysx_release_write(handle, write)
            self._lib.ovphysx_release_query(handle, query)
            raise

        return WriteSession(self, int(query.value), int(write.value), groups, native)

    def query_shared_dictionary(self, query: int) -> int:
        """Return the opaque pointer to the shared ovstage path dictionary backing a query.

        This is NOT an ovphysx-private dictionary: it is the process-shared ovstage
        dictionary the attached Stage uses, the same one that interned the query's
        tokens / prim lists, so a group's ``attribute`` token / ``prim_list`` handle
        resolve through an ``ovstage.PathDictionary(stage)`` as well. Returns 0 if
        unavailable.
        """
        self._check_valid()
        out = c_void_p(0)
        result = self._lib.ovphysx_query_shared_dictionary(
            self._omni_physx_sdk_handle.value, c_uint64(query), ctypes.byref(out))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"query_shared_dictionary failed: {self._get_last_error()}")
        return int(out.value or 0)

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
    def _retain_warp_array(
        wp, *, ptr: int, dtype, shape, device: str, on_release
    ) -> "object":
        """Wrap one non-empty native allocation and tie it to a read-session borrow.

        Warp calls the deleter exactly once, from ``array.__del__``, on whatever thread collects
        the array. ``release_borrow`` is written for that.
        """
        on_release.retain()
        try:
            return wp.array(
                ptr=ptr,
                dtype=dtype,
                shape=shape,
                device=device,
                deleter=on_release.release_borrow,
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
        # ScopedDevice pushes the device context: ovphysx_cuda_stream_wait_event issues the wait
        # in whatever CUDA context is current on the calling thread. get_stream is asked for that
        # device explicitly, so a caller's wp.ScopedStream override is the stream that is ordered.
        warp_device = _warp_device(device)
        with wp.ScopedDevice(warp_device):
            # Warp's null stream is the CUDA default stream and reports cuda_stream as None.
            # This bridge takes CUDA driver handles (0 = default), unlike ovstage_cuda_sync_t
            # on write commit (1 = default).
            cuda_stream = wp.get_stream(warp_device).cuda_stream
            stream_handle = 0 if cuda_stream is None else int(cuda_stream)
            result = self._lib.ovphysx_cuda_stream_wait_event(
                ctypes.c_void_p(stream_handle), ctypes.c_void_p(cuda_wait_event)
            )
            if result.status != ApiStatus.SUCCESS:
                raise RuntimeError(
                    f"ovphysx read could not order Warp stream {stream_handle} after the "
                    f"producer completion event: {self._get_last_error()}"
                )

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

    def set_config(self, entry: ovphysx_config_entry_t) -> None:
        """Set a typed global config entry at runtime (process-global).

        Prefer the typed setters (:meth:`set_config_bool`, :meth:`set_config_int32`,
        :meth:`set_config_float`) for a cleaner API.

        Args:
            entry: Typed config entry (``ovphysx_config_entry_t``).
        """
        result = self._lib.ovphysx_set_global_config(entry)
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to set config: {error_msg}")

    def set_config_bool(self, key: int, value: bool) -> None:
        """Set a boolean config value at runtime (process-global).

        Args:
            key: Boolean config key (e.g., ``ConfigBool.DISABLE_CONTACT_PROCESSING``).
            value: Boolean value.
        """
        self._check_valid()
        from .config import _make_bool_entry
        self.set_config(_make_bool_entry(key, value))

    def set_config_int32(self, key: int, value: int) -> None:
        """Set an int32 config value at runtime (process-global).

        Args:
            key: Int32 config key (e.g., ``ConfigInt32.NUM_THREADS``).
            value: Int32 value.
        """
        self._check_valid()
        from .config import _make_int32_entry
        self.set_config(_make_int32_entry(key, value))

    def set_config_float(self, key: int, value: float) -> None:
        """Set a float config value at runtime (process-global).

        Args:
            key: Float config key.
            value: Float value.
        """
        self._check_valid()
        from .config import _make_float_entry
        self.set_config(_make_float_entry(key, value))

    def get_config_bool(self, key: int) -> bool:
        """Get a boolean config value.

        Args:
            key: Boolean config key (e.g., ``ovphysx.ConfigBool.DISABLE_CONTACT_PROCESSING``).

        Returns:
            Current boolean value.
        """
        self._check_valid()
        out = ctypes.c_bool(False)
        result = self._lib.ovphysx_get_global_config_bool(key, byref(out))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to get config bool: {error_msg}")
        return bool(out.value)

    def get_config_int32(self, key: int) -> int:
        """Get an int32 config value.

        Args:
            key: Int32 config key (e.g., ``ovphysx.ConfigInt32.NUM_THREADS``).

        Returns:
            Current int32 value.
        """
        self._check_valid()
        out = c_int32(0)
        result = self._lib.ovphysx_get_global_config_int32(key, byref(out))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to get config int32: {error_msg}")
        return int(out.value)

    def get_config_float(self, key: int) -> float:
        """Get a float config value.

        Args:
            key: Float config key.

        Returns:
            Current float value.
        """
        self._check_valid()
        out = c_float(0.0)
        result = self._lib.ovphysx_get_global_config_float(key, byref(out))
        if result.status != ApiStatus.SUCCESS:
            error_msg = self._get_last_error()
            raise RuntimeError(f"Failed to get config float: {error_msg}")
        return float(out.value)

    def get_config_string(self, key: int) -> str | None:
        """Get a string config value.

        Args:
            key: String config key from ``ovphysx.ConfigString``.

        Returns:
            Current string value, or None if not found.
        """
        self._check_valid()
        buffer_size = 256
        buffer = ctypes.create_string_buffer(buffer_size)
        value_out = ovphysx_string_t()
        value_out.ptr = ctypes.cast(buffer, c_char_p)
        value_out.length = buffer_size

        required_size = ctypes.c_size_t(0)
        result = self._lib.ovphysx_get_global_config_string(key, byref(value_out), byref(required_size))

        if result.status == ApiStatus.BUFFER_TOO_SMALL:
            buffer_size = required_size.value
            buffer = ctypes.create_string_buffer(buffer_size)
            value_out = ovphysx_string_t()
            value_out.ptr = ctypes.cast(buffer, c_char_p)
            value_out.length = buffer_size
            required_size = ctypes.c_size_t(0)
            result = self._lib.ovphysx_get_global_config_string(key, byref(value_out), byref(required_size))
        if result.status != ApiStatus.SUCCESS:
            self._get_last_error()
            return None
        return str(value_out)

    def destroy(self) -> None:
        """Destroy this PhysX instance.

        Preconditions:
            - No other thread is using this instance. Calling ``destroy()``
              again after terminal destruction is a valid no-op.
        Side effects:
            - Releases native resources and unregisters the instance.
        Ownership/Lifetime:
            - All tensor bindings and contact bindings created by this instance are
              automatically released.
            - Drop every output-read Warp array and downstream view before destruction.
              Destroying with live aliases warns and leaves their read-session resources
              allocated so their pointers do not dangle.
            - The instance becomes unusable after destruction.
        Threading:
            - Do not call concurrently with other operations on this instance.
        Errors:
            - Raises ``RuntimeError`` without changing the instance if called
              from a native log callback. Retry after the callback returns.
            - Raises ``RuntimeError`` if native destruction reports a failure or
              process shutdown fails. The instance is already destroyed when
              either failure is reported, so a later call is an idempotent no-op.
              If both fail, the process-shutdown error is reported with the
              native status included.
            - A Python or ctypes exception raised while invoking native destruction
              leaves ownership intact so the call can be retried.
            - Because this is a checked operation, a cleanup failure raised from
              a ``finally`` block becomes the active exception. Python preserves
              any in-flight exception as chained context; applications that need
              different precedence must catch and log cleanup failures explicitly.
        """
        handle = getattr(self, "_omni_physx_sdk_handle", None)
        owns_native_lifecycle = (handle is not None and handle.value != _INVALID_HANDLE) or getattr(
            self, "_lifecycle_acquired", False
        )
        if owns_native_lifecycle and getattr(_python_log_callback_context, "active", False):
            raise RuntimeError(
                "ovphysx_shutdown cannot be called from within a log callback; "
                "retry PhysX.destroy() after the callback returns"
            )

        destroy_status = ApiStatus.SUCCESS
        if handle is not None and handle.value != _INVALID_HANDLE:
            # An aliased Warp array cannot be freed here without dangling the user's pointer, so
            # warn and keep the read session pinned until the alias is dropped. Borrows are counted
            # explicitly rather than inferred from _refs, which sits at 1 in the case being warned
            # about (closing the ReadResult and dropping an array both decrement it).
            try:
                borrowed = sum(1 for holder in list(self._live_read_holders) if holder.has_live_borrows)
                if borrowed:
                    warnings.warn(
                        f"{borrowed} output-read session(s) still have aliased Warp arrays or "
                        "downstream views at PhysX.destroy(); their native buffers stay allocated "
                        "and the process-wide CUDA context is held open until every alias is "
                        "dropped. Drop all read arrays (and any views of them) first.",
                        ResourceWarning,
                        stacklevel=2,
                    )
            except Exception:
                pass

            # Drain sessions already ready for teardown and release stage-derived handles while
            # the attached Stage is still valid. Borrowed numeric buffers remain session-owned.
            try:
                self._release_stage_bound_read_handles()
                self._drain_pending_read_releases()
            except Exception:
                pass
            # Do not catch Python/ctypes exceptions here. If the call did not
            # return a native status, ownership may still be live and a retry is
            # required. If it completed before raising, the retry converges on
            # the terminal already-absent status handled below.
            result = self._lib.ovphysx_destroy_instance(handle.value)
            destroy_status = result.status

        # omni_sdk_physx_destroy() reports failure only when the handle is
        # already absent from the native instance registry. All paths for a live
        # handle erase it and return success. Commit the terminal state for both
        # statuses so a stale handle cannot leak the process-lifecycle token.
        # The native instance also detaches any attached ovstage on destroy.
        self._omni_physx_sdk_handle = None
        self._attached_ovstage = None
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

        :param pattern: Physics-object path glob pattern
            (e.g., "/World/robot*", "/World/env[N]/robot"). A single path
            component (the text between two slashes; a parenthesized group
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
            - Returned TensorBinding owns native resources until ``destroy()``.
            - Use ``binding.shape`` and ``binding.dtype`` (or ``binding.spec``)
              for layout and ``binding.native_device`` for no-staging placement.
              Most tensor types are float32, but some types such as
              ``RIGID_BODY_DISABLE_SIMULATION`` are not.
            - The binding is tied to the current stage topology. Reuse it across
              steps, but do not keep it across ``reset_stage()``, removing USD data
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
        the first successful call. Warmup state resets after reset_stage() or
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
    # Contact report
    # ------------------------------------------------------------------

    def get_contact_report(self, *, include_friction_anchors: bool = False, copy: bool = False) -> dict:
        """Get per-contact-point event data for the current simulation step.

        Use this for custom contact sensors, collision debugging, or per-point
        force analysis. For **aggregate force tensors** (net forces or force
        matrices between sensor/filter body sets), use
        :meth:`create_contact_binding` instead.

        .. warning::
            With the default ``copy=False``, the returned ``headers``,
            ``points``, and ``anchors`` are zero-copy ctypes views into
            internal C buffers that are valid **only until the next**
            :meth:`step` or :meth:`step_sync` call. After the next step the
            buffers may be reallocated or reused. Accessing the views is
            undefined behavior (silent data corruption or segfault). Python
            cannot detect this dangling state.

            Pass ``copy=True`` to get Python-owned lists of dicts that are
            safe to retain across simulation steps. This is the recommended
            mode for RL training loops or any code that holds contact data
            beyond a single step.

        Args:
            include_friction_anchors: If True, also return friction anchor data
                (position and impulse at each friction anchor point).
            copy: If True, return Python-owned ``list[dict]`` for each section
                (safe to hold across steps). If False (default), return
                zero-copy ctypes array views (faster but valid only until the
                next ``step()``/``step_sync()``).

        Returns a dict with:
            - ``headers``: contact event headers describing each contact pair
              (actors, colliders, event type). When ``copy=False``, a ctypes
              array of :class:`ContactEventHeader`; when ``copy=True``, a
              ``list[dict]`` with the same field names. Length is
              ``num_headers``.
            - ``num_headers`` (int): Number of contact event headers.
            - ``points``: per-contact-point data (position, normal, impulse,
              separation). When ``copy=False``, a ctypes array of
              :class:`ContactPoint`; when ``copy=True``, a ``list[dict]``.
              Length is ``num_points``.
            - ``num_points`` (int): Number of contact point entries.
            - ``anchors`` (only if ``include_friction_anchors=True``): friction
              anchor data. When ``copy=False``, a ctypes array of
              :class:`FrictionAnchor`; when ``copy=True``, a ``list[dict]``.
              Length is ``num_anchors``.
            - ``num_anchors`` (int, only if ``include_friction_anchors=True``):
              Number of friction anchors.

        Example (safe across steps, ``copy=True``)::

            report = physx.get_contact_report(copy=True)
            physx.step_sync(dt)  # next step, report still valid
            for h in report["headers"]:
                print(h["actor0"], h["numContactData"])
            for p in report["points"]:
                print(p["position"], p["normal"], p["impulse"])

        Example (zero-copy, ``copy=False``)::

            report = physx.get_contact_report()
            for i in range(report["num_headers"]):
                h = report["headers"][i]
                print(h.actor0, h.numContactData)
            # Do NOT call step() before finishing access to report.

        Prims must have ``PhysxContactReportAPI`` applied in the USD stage
        for contacts to be reported.

        Raises:
            RuntimeError: If the call fails.
        """
        headers_ptr = ctypes.POINTER(ContactEventHeader)()
        num_headers = ctypes.c_uint32(0)
        data_ptr = ctypes.POINTER(ContactPoint)()
        num_data = ctypes.c_uint32(0)
        anchors_ptr = ctypes.POINTER(FrictionAnchor)() if include_friction_anchors else None
        num_anchors = ctypes.c_uint32(0) if include_friction_anchors else None
        result = _lib.ovphysx_get_contact_report(
            self._omni_physx_sdk_handle.value,
            ctypes.byref(headers_ptr),
            ctypes.byref(num_headers),
            ctypes.byref(data_ptr),
            ctypes.byref(num_data),
            ctypes.byref(anchors_ptr) if anchors_ptr is not None else None,
            ctypes.byref(num_anchors) if num_anchors is not None else None,
        )
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(f"Failed to get contact report: {self._get_last_error()}")

        nh = num_headers.value
        np_ = num_data.value
        if copy:
            headers = [_contact_header_to_dict(headers_ptr[i]) for i in range(nh)]
            points = [_contact_point_to_dict(data_ptr[i]) for i in range(np_)]
        else:
            headers = (
                ctypes.cast(headers_ptr, ctypes.POINTER(ContactEventHeader * nh)).contents
                if nh
                else (ContactEventHeader * 0)()
            )
            points = (
                ctypes.cast(data_ptr, ctypes.POINTER(ContactPoint * np_)).contents
                if np_
                else (ContactPoint * 0)()
            )
        out = {
            "headers": headers,
            "num_headers": nh,
            "points": points,
            "num_points": np_,
        }
        if include_friction_anchors:
            na = num_anchors.value
            if copy:
                anchors = [_friction_anchor_to_dict(anchors_ptr[i]) for i in range(na)]
            else:
                anchors = (
                    ctypes.cast(anchors_ptr, ctypes.POINTER(FrictionAnchor * na)).contents
                    if na
                    else (FrictionAnchor * 0)()
                )
            out["anchors"] = anchors
            out["num_anchors"] = na
        return out

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
            results.append({
                "collision": h.collision,
                "rigid_body": h.rigid_body,
                "proto_index": h.proto_index,
                "normal": (h.normal[0], h.normal[1], h.normal[2]),
                "position": (h.position[0], h.position[1], h.position[2]),
                "distance": h.distance,
                "face_index": h.face_index,
                "material": h.material,
            })
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
        o = self._make_float3(origin)
        d = self._make_float3(direction)
        hits_ptr = ctypes.POINTER(ovphysx_scene_query_hit_t)()
        count = ctypes.c_uint32(0)
        result = _lib.ovphysx_raycast(
            self._omni_physx_sdk_handle.value,
            o, d,
            ctypes.c_float(distance),
            ctypes.c_bool(both_sides),
            ctypes.c_int(int(mode)),
            ctypes.byref(hits_ptr),
            ctypes.byref(count))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(
                f"Raycast failed: {self._get_last_error()}")
        return self._parse_scene_query_hits(hits_ptr, count.value)

    _GEOMETRY_PARAMS = {
        SceneQueryGeometryType.SPHERE: {"radius", "position"},
        SceneQueryGeometryType.BOX: {"half_extent", "position", "rotation"},
        SceneQueryGeometryType.SHAPE: {"prim_path"},
    }

    def _make_geometry_desc(self, geometry_type: SceneQueryGeometryType, **kwargs) -> "ovphysx_scene_query_geometry_desc_t":
        """Build an ovphysx_scene_query_geometry_desc_t from keyword args."""
        expected = self._GEOMETRY_PARAMS.get(geometry_type)
        if expected is not None:
            unexpected = set(kwargs) - expected
            if unexpected:
                raise TypeError(
                    f"Unexpected kwargs for {SceneQueryGeometryType(geometry_type).name}: {unexpected}")
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
            ctypes.byref(count))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(
                f"Sweep failed: {self._get_last_error()}")
        return self._parse_scene_query_hits(hits_ptr, count.value)

    def overlap(
        self,
        geometry_type: SceneQueryGeometryType,
        mode: SceneQueryMode = SceneQueryMode.ALL,
        **kwargs,
    ) -> list[dict]:
        """Test geometry overlap against objects in the scene.

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
        if mode == SceneQueryMode.CLOSEST:
            raise ValueError(
                "overlap() does not support SceneQueryMode.CLOSEST -- "
                "CLOSEST has no meaning for overlap queries (no direction/ray). "
                "Use SceneQueryMode.ALL or SceneQueryMode.ANY.")
        desc = self._make_geometry_desc(geometry_type, **kwargs)
        hits_ptr = ctypes.POINTER(ovphysx_scene_query_hit_t)()
        count = ctypes.c_uint32(0)
        result = _lib.ovphysx_overlap(
            self._omni_physx_sdk_handle.value,
            ctypes.byref(desc),
            ctypes.c_int(int(mode)),
            ctypes.byref(hits_ptr),
            ctypes.byref(count))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(
                f"Overlap failed: {self._get_last_error()}")
        return self._parse_scene_query_hits(hits_ptr, count.value)

    def get_scene_query_paths_from_ids(self, ids: "tuple | list") -> list[str]:
        """Resolve scene-query hit identity fields to physics-object paths.

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
            ctypes.byref(count))
        if result.status != ApiStatus.SUCCESS:
            raise RuntimeError(
                f"Failed to resolve scene-query IDs to paths: {self._get_last_error()}")
        written = min(int(count.value), n)
        return [str(paths_buf[i]) for i in range(written)]

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

        Returns DLPack-compatible tensors of net forces ``[S, 3]`` or force
        matrices ``[S, F, 3]``. Detailed contact and friction data are exposed
        as flat ``[C, ...]`` buffers plus ``[S, F]`` count/start-index tensors
        via :meth:`ContactBinding.read_contact_data` and
        :meth:`ContactBinding.read_friction_data`.

        A **sensor** is a set of rigid bodies matched by a physics-object path
        pattern. A **filter** is a second set of bodies whose contacts with each
        sensor you want to measure. Patterns include authored USD objects and
        runtime-only clones.

        Contact reporting is opt-in: every authored USD prim matched by
        ``sensor_patterns`` must have ``PhysxContactReportAPI`` applied, on the
        prim named as the sensor itself (not a parent body or child collider). A
        matched prim without the schema is dropped from the binding, and if that
        leaves no sensors this call raises ``RuntimeError``. Filter prims need no
        extra schema, and runtime-only clones inherit contact reporting from the
        source actor.

        The binding must be created *before* the first simulation step whose
        contacts you want to observe. Call
        :meth:`ContactBinding.read_net_forces` or
        :meth:`ContactBinding.read_force_matrix` after a successful
        :meth:`PhysX.step`, :meth:`PhysX.step_sync`, or
        :meth:`PhysX.step_n_sync` call. Before the first step, both return
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
                    binding.read_net_forces(forces)
                    return forces

        Args:
            sensor_patterns: Physics-object path patterns for sensor bodies. A single
                path component may be at most 4096 characters long; a longer one
                raises ``RuntimeError`` (also for ``filter_patterns``).
            filter_patterns: Flat list of physics-object path patterns for filters.
                Total length must equal ``len(sensor_patterns) * filters_per_sensor``.
                Pass ``None`` with ``filters_per_sensor=0`` to get contacts with all bodies.
            filters_per_sensor: Number of filter patterns per sensor (same for all sensors).
            max_contact_data_count: Max raw contact pairs to track in the native
                backend. Also caps the detailed contact/friction flat-buffer reads.
                Detailed reads require this value and ``filters_per_sensor`` to
                be positive.
        """
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

        Requires a GPU instance. CPU SDF evaluation is not implemented.

        Args:
            pattern: USD-style object-path glob matching SDF collision shapes,
                including runtime-only clones. A single path component may be at
                most 4096 characters long; a longer one raises ``RuntimeError``.
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

    def _finalize_unreleased(self) -> None:
        """Run the finalizer's non-throwing destroy/retry policy."""
        # No caller remains to perform destroy()'s documented retry after a
        # pre-status invocation exception, so make one immediate best-effort
        # attempt. Returned-status and shutdown failures already discharged
        # ownership inside destroy() and are terminal, so they are not retried.
        for _ in range(2):
            try:
                self.destroy()
                return
            except Exception:
                if not getattr(self, "_lifecycle_acquired", False):
                    return

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
        Persistent cleanup errors are suppressed. If both invocations fail,
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
        owns_native_lifecycle = owns_native_lifecycle or (
            handle is not None and handle.value != _INVALID_HANDLE
        )
        if owns_native_lifecycle and getattr(_python_log_callback_context, "active", False):
            # Explicit destroy remains fail-fast in callback context. A GC
            # finalizer has no caller that can retry, so retain self through a
            # daemon worker and let native shutdown drain this callback after
            # it returns. Never join the worker from the callback thread.
            try:
                threading.Thread(
                    target=self._finalize_unreleased,
                    name="ovphysx-finalizer",
                    daemon=True,
                ).start()
                return
            except Exception:
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
