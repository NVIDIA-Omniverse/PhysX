# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Lifetime tests for zero-copy CPU and CUDA Warp output-read arrays.

Every non-empty array borrows session-owned storage. Closing ReadResult releases
Stage-bound identity immediately but native numeric storage remains alive until
all Warp arrays and downstream views are gone. Deleters may run on arbitrary
threads, so they only enqueue teardown. An owning-thread API call drains it.
"""

# @implements REQ-PYTHON-READ-001
# @covers AC-3 AC-4 AC-5
# @maps_to TEST-PYTHON-READ-001

import ctypes
import gc
import threading
from types import SimpleNamespace

import pytest
import warp as wp
from ovphysx.types import ObjectScope, SimObjectType
from test_utils import data_path, load_usd_with_ovstage


class _FakeLib:
    def __init__(self):
        self.calls = []

    def ovphysx_release_group(self, *args):
        self.calls.append("group")

    def ovphysx_release_read(self, *args):
        self.calls.append("read")

    def ovphysx_release_query(self, *args):
        self.calls.append("query")


class _FakeSdk:
    """Mirrors the PhysX owning-thread drain contract used by _ReadRelease."""

    class _Handle:
        value = 0x1234

    def __init__(self):
        self._lib = _FakeLib()
        self._omni_physx_sdk_handle = _FakeSdk._Handle()
        self._pending_read_releases = []
        self._pending_read_lock = threading.Lock()

    def _enqueue_read_release(self, holder):
        with self._pending_read_lock:
            self._pending_read_releases.append(holder)

    def _drain_pending_read_releases(self):
        with self._pending_read_lock:
            pending = self._pending_read_releases
            self._pending_read_releases = []
        for holder in pending:
            holder._free_native()


def _lifecycle_spies(monkeypatch):
    from ovphysx import api

    acquired = []
    released = []
    monkeypatch.setattr(api, "_acquire_process_lifecycle", lambda: acquired.append(threading.get_ident()))
    monkeypatch.setattr(api, "_release_process_lifecycle", lambda: released.append(threading.get_ident()))
    return acquired, released


def _borrow_cpu_array(holder):
    from ovphysx.api import PhysX

    storage = (ctypes.c_float * 4)(1.0, 2.0, 3.0, 4.0)
    array = PhysX._retain_warp_array(
        wp,
        ptr=ctypes.addressof(storage),
        dtype=wp.float32,
        shape=(4,),
        device="cpu",
        on_release=holder,
    )
    return array, storage


def test_read_release_defers_native_teardown_to_owner_drain(monkeypatch):
    """Close plus the final array deleter queues one native teardown."""
    from ovphysx.api import _ReadRelease

    acquired, released = _lifecycle_spies(monkeypatch)
    owner_thread = threading.get_ident()
    sdk = _FakeSdk()
    holder = _ReadRelease(sdk, query=7, read=9)
    holder.add_group(1)
    holder.add_group(2)
    holder.retain()
    holder.retain()

    assert acquired == [owner_thread, owner_thread], "every borrow takes its own process-lifecycle reference"
    holder.release_groups()
    holder.release()
    holder.release_borrow()
    assert sdk._lib.calls == ["group", "group"]
    assert sdk._pending_read_releases == []
    assert released == [owner_thread]

    holder.release_borrow()
    assert sdk._lib.calls == ["group", "group"]
    assert sdk._pending_read_releases == [holder]
    assert released == [owner_thread, owner_thread], "every borrow returns its reference"

    sdk._drain_pending_read_releases()
    assert sdk._lib.calls == ["group", "group", "read", "query"]

    # Idempotent: a stray extra release plus a re-drain must not double-free.
    holder.release()
    sdk._drain_pending_read_releases()
    assert sdk._lib.calls == ["group", "group", "read", "query"]
    assert released == [owner_thread, owner_thread]


def test_pending_zero_ref_release_retires_groups_before_native_session(monkeypatch):
    """A pending holder still retires Stage metadata exactly once before native teardown."""
    from ovphysx.api import _ReadRelease

    acquired, released = _lifecycle_spies(monkeypatch)
    sdk = _FakeSdk()
    holder = _ReadRelease(sdk, query=7, read=9)
    holder.add_group(1)

    holder.release()
    assert sdk._pending_read_releases == [holder]
    assert sdk._lib.calls == []

    holder.release_groups()
    holder.release_groups()
    assert sdk._lib.calls == ["group"]

    holder._free_native()
    holder._free_native()
    assert sdk._lib.calls == ["group", "read", "query"]
    assert acquired == released == []


def test_read_release_refcount_is_thread_safe_under_concurrent_drops(monkeypatch):
    """Concurrent deleter drops cannot lose a decrement.

    A bare ``-=`` is not atomic even under the GIL, so a lost decrement would leak the
    session forever. N simultaneous releases must reach zero: queued once, freed once.
    """
    from ovphysx.api import _ReadRelease

    acquired, released = _lifecycle_spies(monkeypatch)
    n = 64
    sdk = _FakeSdk()
    holder = _ReadRelease(sdk, query=7, read=9)
    holder.add_group(1)
    for _ in range(n):
        holder.retain()
    holder.release()  # the owning ReadResult's ref, so only the borrows remain

    start = threading.Barrier(n)

    def drop():
        start.wait()  # maximize contention on the refcount
        holder.release_borrow()

    threads = [threading.Thread(target=drop) for _ in range(n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert len(acquired) == n and len(released) == n, "borrow/lifecycle accounting must balance"
    assert sdk._lib.calls == [], "native teardown must not run from a release/GC thread"
    assert len(sdk._pending_read_releases) == 1, "a lost decrement would leave refs > 0 (session leak)"
    sdk._drain_pending_read_releases()
    assert sdk._lib.calls == ["group", "read", "query"]


def test_every_borrow_returns_its_lifecycle_reference():
    """N concurrent borrows must return exactly N references to the REAL process refcount.

    The last release runs ``ovphysx_shutdown()``, which reclaims the CUDA context the
    borrowed buffer lives in. Too few returned and the process can never shut down; too
    many and shutdown runs early, destroying the context under a live array.
    """
    import ovphysx.api as api

    n = 32
    sdk = _FakeSdk()
    holder = api._ReadRelease(sdk, query=1, read=2)
    before = api._PROCESS_LIFECYCLE_REFCOUNT

    for _ in range(n):
        holder.retain()
    assert api._PROCESS_LIFECYCLE_REFCOUNT == before + n, "a borrow must pin the process lifecycle"

    threads = [threading.Thread(target=holder.release_borrow) for _ in range(n)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    # The LAST release is the one that would be final shutdown, and a deleter queues that rather
    # than running native teardown on a GC thread. Queued is not lost: draining retires it, and the
    # accounting balances as before. Asserting without the drain would be asserting the bug.
    api._drain_deferred_lifecycle_releases()
    assert api._PROCESS_LIFECYCLE_REFCOUNT == before, "borrow/lifecycle accounting must balance"


def test_warp_slice_keeps_read_session_alive_after_source_and_result_drop(monkeypatch):
    """A Warp slice retains its source lease until the slice itself is released."""
    from ovphysx.api import _ReadRelease

    # Collect BEFORE the spies go in. They patch module-level lifecycle hooks, so a borrowed
    # Warp array an earlier test left uncollected would fire its deleter into `released`
    # during this test's own gc.collect() below. Twice, because the first pass can make a
    # cycle collectable that only the second reclaims.
    gc.collect()
    gc.collect()

    acquired, released = _lifecycle_spies(monkeypatch)
    owner_thread = threading.get_ident()
    sdk = _FakeSdk()
    holder = _ReadRelease(sdk, query=7, read=9)
    holder.add_group(1)
    array, storage = _borrow_cpu_array(holder)
    view = array[1:3]

    holder.release_groups()
    holder.release()
    del array
    gc.collect()

    assert sdk._pending_read_releases == []
    assert sdk._lib.calls == ["group"]
    assert view.numpy().tolist() == list(storage)[1:3]

    del view
    gc.collect()
    assert sdk._pending_read_releases == [holder]
    sdk._drain_pending_read_releases()

    assert sdk._lib.calls == ["group", "read", "query"]
    assert acquired == released == [owner_thread]


class _StubWarp:
    """Just enough of the warp module for the stream-ordering call path, off a GPU."""

    class _Stream:
        cuda_stream = 0x1000

    class ScopedDevice:
        def __init__(self, device):
            self.device = device

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

    @staticmethod
    def get_stream(device=None):
        return _StubWarp._Stream()


def _cuda_group(wait_event):
    """A minimal stand-in for a CUDA ovstage_read_group_t carrying a producer event."""
    from ovphysx.dlpack import DLDeviceType, DLTensor

    tensor = DLTensor()
    tensor.device.device_type = DLDeviceType.kDLCUDA
    tensor.device.device_id = 0
    return SimpleNamespace(
        data=SimpleNamespace(
            cuda_sync=SimpleNamespace(wait_event=wait_event),
            tensor_count=1,
            tensors=[tensor],
        )
    )


def test_failed_stream_wait_raises_before_any_array_is_returned(monkeypatch):
    """A refused producer-event wait must raise, not hand back an unordered column."""
    from ovphysx import _bindings
    from ovphysx.api import PhysX

    class _FailedWait:
        status = 1

    monkeypatch.setattr(
        _bindings._lib, "ovphysx_cuda_stream_wait_event", lambda *args: _FailedWait()
    )

    sdk = object.__new__(PhysX)
    sdk._lib = _bindings._lib
    sdk._get_last_error = lambda: "forced wait failure"
    # Suppress the real object's ResourceWarning/finalizer path in a pure unit test.
    sdk._released = True

    with pytest.raises(RuntimeError, match="could not order Warp stream"):
        sdk._order_warp_stream_after_group(_cuda_group(0xBEEF), _StubWarp)


def test_a_borrow_dropped_during_a_lifecycle_transition_still_releases_the_session(monkeypatch):
    """A raising lifecycle release must not cost the session reference.

    ``release_borrow`` is a Warp deleter: it runs on whatever thread the collector is on, so it
    can land while another thread owns a lifecycle transition. ``_release_process_lifecycle``
    raises on both INITIALIZING and SHUTTING_DOWN, and from a deleter Python can only print
    "exception ignored". Without the guard the ``self.release()`` that follows it never runs and
    the native read refcount stays pinned for good.

    The real ``_release_process_lifecycle`` is used here, raising for its own reasons off the real
    flags, so the test cannot pass against a stub that no longer raises.
    """
    from ovphysx import api
    from ovphysx.api import PhysX, _ReadRelease

    # retain() is not under test and would raise off the same flags.
    monkeypatch.setattr(api, "_acquire_process_lifecycle", lambda: None)
    monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_DEFERRED_RELEASES", 0)

    for flag in ("_PROCESS_LIFECYCLE_INITIALIZING", "_PROCESS_LIFECYCLE_SHUTTING_DOWN"):
        monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
        monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
        monkeypatch.setattr(api, flag, True)

        # The premise: this is the call that raises out of a deleter.
        with pytest.raises(RuntimeError):
            api._release_process_lifecycle()

        sdk = object.__new__(PhysX)
        sdk._lib = _FakeLib()
        sdk._omni_physx_sdk_handle = _FakeSdk._Handle()
        sdk._live_read_holders = set()
        sdk._released = True

        holder = _ReadRelease(sdk, 0, 0)
        # __init__ seeds one reference for the owning ReadResult, so the oracle is "back to where
        # it started", not zero. A borrow returns its own reference and nothing else.
        owner_refs = holder._refs
        holder.retain()
        assert holder.has_live_borrows
        assert holder._refs == owner_refs + 1

        # Must not propagate: a deleter has nowhere to report it.
        holder.release_borrow()

        assert not holder.has_live_borrows, f"borrow not dropped while {flag} was active"
        assert holder._refs == owner_refs, f"session reference stranded while {flag} was active"

    # The references the deleter could not hand back are queued, not lost.
    assert api._PROCESS_LIFECYCLE_DEFERRED_RELEASES == 2

    # A later entry point, once no transition is in flight, retires them.
    monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    retired = []
    monkeypatch.setattr(api, "_release_process_lifecycle", lambda: retired.append(1))
    api._drain_deferred_lifecycle_releases()
    assert retired == [1, 1]
    assert api._PROCESS_LIFECYCLE_DEFERRED_RELEASES == 0


def test_a_later_group_failure_drops_earlier_arrays_before_native_teardown(monkeypatch):
    """A multi-group read whose SECOND group fails must not free the session under the first.

    `_ReadRelease._free()` bypasses the refcount, so any array still reachable when it runs is
    left aliasing freed CPU or device storage. Anything that formats frame locals while the
    exception propagates (a debugger, a crash reporter, logging with locals) dereferences it.

    The single-group test above cannot see this: it fails before any group has succeeded, so
    nothing is accumulated. The defect needs a LATER group to fail, which is the ordinary shape of
    a multi-attribute read.

    The oracle is the borrow count at the moment native teardown runs. Every array retains the
    holder, so a surviving array means a non-zero count.
    """
    from ovphysx import _bindings
    from ovphysx.api import PhysX, _ReadRelease
    from ovphysx.dlpack import DLTensor
    from ovphysx.types import ApiStatus

    _lifecycle_spies(monkeypatch)

    # One real zeroed native group, reused for both fetches: every field the loop reads is either
    # zero (index maps empty, no CUDA event to wait on) or set below.
    tensor = DLTensor()
    group = _bindings.ovstage_read_group_t()
    group.data.tensor_count = 1
    group.data.tensors = ctypes.pointer(tensor)

    fetches = {"n": 0}

    def fake_fetch(handle, read, out_pp):
        if fetches["n"] >= 2:
            return SimpleNamespace(status=ApiStatus.END_OF_ITERATION)
        fetches["n"] += 1
        # `byref(gp)._obj` is the caller's POINTER instance. Pointing it at the fake struct is how
        # a pure-Python stand-in returns a producer-owned group.
        out_pp._obj.contents = group
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    sdk = object.__new__(PhysX)
    sdk._lib = _FakeLib()
    sdk._lib.ovphysx_fetch_read_next = fake_fetch
    sdk._omni_physx_sdk_handle = _FakeSdk._Handle()
    sdk._live_read_holders = set()
    sdk._get_last_error = lambda: "unused"
    sdk._released = True

    kept_storage = []
    converted = {"n": 0}

    def fake_convert(t, wp_mod, on_release):
        converted["n"] += 1
        if converted["n"] == 1:
            array, storage = _borrow_cpu_array(on_release)
            kept_storage.append(storage)  # the array aliases it; it must outlive the call
            return array
        raise RuntimeError("forced conversion failure on the second group")

    sdk._dltensor_to_warp_array = fake_convert

    observed = {}
    original_free_native = _ReadRelease._free_native

    def spy_free_native(self):
        observed["borrows"] = self._borrows
        return original_free_native(self)

    monkeypatch.setattr(_ReadRelease, "_free_native", spy_free_native)

    with pytest.raises(RuntimeError, match="forced conversion failure"):
        sdk._iterate_read_groups(
            SimpleNamespace(value=0x1234),
            SimpleNamespace(value=1),
            SimpleNamespace(value=2),
            SimObjectType.RIGID_BODY,
        )

    assert converted["n"] == 2, "the second group must have been reached"
    # Zero, not one. A first-group array still held by `groups` (or by the previous iteration's
    # `tensors` local) when _free() runs shows up here as a surviving borrow.
    assert observed["borrows"] == 0


def test_null_warp_stream_orders_against_the_default_stream(monkeypatch):
    """A caller inside wp.ScopedStream(device.null_stream) must still get its wait.

    Warp's null stream reports cuda_stream as None rather than 0, so reading the handle
    without normalizing raises TypeError before the wait is ever enqueued.
    """
    from ovphysx import _bindings
    from ovphysx.api import PhysX

    class _NullStreamWarp(_StubWarp):
        class _Stream:
            cuda_stream = None

        @staticmethod
        def get_stream(device=None):
            return _NullStreamWarp._Stream()

    class _Ok:
        status = 0

    calls = []
    monkeypatch.setattr(
        _bindings._lib,
        "ovphysx_cuda_stream_wait_event",
        lambda stream, event: (calls.append((stream.value or 0, event.value or 0)), _Ok())[1],
    )

    sdk = object.__new__(PhysX)
    sdk._lib = _bindings._lib
    sdk._released = True

    sdk._order_warp_stream_after_group(_cuda_group(0xBEEF), _NullStreamWarp)
    assert calls == [(0, 0xBEEF)]


def test_device_column_release_is_deferred_to_the_owning_thread(physx_sdk, monkeypatch):
    """End-to-end on a real GPU scene: a borrowed CUDA column keeps the native read session
    alive past the ``with`` block, and dropping the borrow defers the teardown
    (ovphysx_release_read, and with it the cuMemFree) to the owning thread's next drain."""
    wp.init()
    if wp.get_cuda_device_count() == 0:
        pytest.skip("no CUDA device available")

    load_usd_with_ovstage(physx_sdk, data_path("boxes_falling_on_groundplane.usda"))
    for _ in range(5):
        physx_sdk.step(1.0 / 60.0)
    physx_sdk.wait_all()

    calls = []
    orig_release_read = physx_sdk._lib.ovphysx_release_read

    def _spy_release_read(*args):
        calls.append(1)
        return orig_release_read(*args)

    monkeypatch.setattr(physx_sdk._lib, "ovphysx_release_read", _spy_release_read)

    with physx_sdk.read(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
        assert result.groups and result.groups[0].tensors, "expected a device read column"
        held = [t for g in result.groups for t in g.tensors]
        assert held[0].device.is_cuda, "precondition: GPU-resident column"
    del result  # drop the ReadResult, so only the held Warp arrays still borrow the session

    # A device column is still borrowed, so the native read must NOT be released.
    assert calls == [], "ovphysx_release_read must be deferred while a device column is borrowed"

    # Drop the last borrow. The deleter only queues the teardown and must NOT run it on this GC path.
    held = None
    gc.collect()
    assert calls == [], "teardown must be deferred to the owning thread, not run from the deleter"

    # The owning thread's next ovphysx call drains the queued teardown exactly once.
    physx_sdk.step(1.0 / 60.0)
    assert len(calls) == 1, f"expected one deferred ovphysx_release_read on the owning drain, got {len(calls)}"


def test_a_borrow_deleted_on_a_worker_thread_never_runs_native_shutdown_there(monkeypatch):
    """The last array borrow must not carry native shutdown onto a foreign thread.

    ``release_borrow`` is a Warp deleter: it runs on whatever thread the collector is on. When an
    array outlives its PhysX instance, a case ``_free_native`` deliberately supports, keeping the
    buffer readable rather than dangling, that borrow holds the LAST process reference. Handing it
    back inline, before releasing the session, would run the whole of ``ovphysx_shutdown()`` on a
    GC thread inside a ``__del__`` and only then queue the read holder, against an SDK whose native
    handle is already gone.

    Spied at ``_finish_process_shutdown_transition`` rather than at ``_release_process_lifecycle``:
    the veto lives INSIDE the latter, so stubbing it would remove the very thing under test. The
    real release runs here, and what is asserted is that it stops short of native teardown.
    """
    from ovphysx import api
    from ovphysx.api import PhysX, _ReadRelease

    shutdown_threads = []
    monkeypatch.setattr(
        api, "_finish_process_shutdown_transition", lambda: shutdown_threads.append(threading.get_ident())
    )
    monkeypatch.setattr(api, "_acquire_process_lifecycle", lambda: None)
    monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_DEFERRED_RELEASES", 0)
    monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    # Exactly one reference, so this borrow's release IS the final one. With more than one the
    # release is only a decrement and the test would prove nothing.
    monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_REFCOUNT", 1)

    sdk = object.__new__(PhysX)
    sdk._lib = _FakeLib()
    sdk._omni_physx_sdk_handle = _FakeSdk._Handle()
    sdk._live_read_holders = set()
    sdk._released = True
    # The real queue, so the enqueue genuinely succeeds. Without these the deleter's
    # _enqueue_read_release raises inside the worker and the queuing never happens.
    sdk._pending_read_lock = threading.Lock()
    sdk._pending_read_releases = []

    holder = _ReadRelease(sdk, 0, 0)
    holder.release()  # the owning ReadResult closes; the borrow is now the only reference

    worker = threading.Thread(target=holder.release_borrow)
    worker.start()
    worker.join()

    # THE THREAD. Native shutdown must not have run at all, least of all on the worker.
    assert shutdown_threads == [], f"native shutdown ran on a foreign thread: {shutdown_threads}"

    # Deferred, not dropped: queued is only correct if something can still retire it.
    assert api._PROCESS_LIFECYCLE_DEFERRED_RELEASES == 1, (
        "the final release must be queued when it comes from a deleter"
    )
    assert api._PROCESS_LIFECYCLE_REFCOUNT == 1, "the reference is not given back until it is retired"

    # THE ORDER. The session teardown is queued by the deleter itself, before the process reference
    # is handed back. The reverse order could complete shutdown first and queue against a dead handle.
    assert sdk._pending_read_releases == [holder], "the session was not queued for the owner"

    # And a real entry point retires it, on a thread that may take it.
    api._drain_deferred_lifecycle_releases()
    assert shutdown_threads == [threading.get_ident()], "the drain must run shutdown on its own thread"
    assert api._PROCESS_LIFECYCLE_DEFERRED_RELEASES == 0
