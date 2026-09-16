# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Failure-injection tests for the public PhysX destruction contract."""

# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6

import gc
import threading
import warnings
import weakref
from ctypes import c_uint64
from types import SimpleNamespace

import ovphysx.api as api
import pytest
from ovphysx.types import ApiStatus

from ovphysx import PhysX


class _FakeLib:
    def __init__(self, outcomes):
        self._outcomes = list(outcomes)
        self.destroy_calls = []

    def ovphysx_destroy_instance(self, handle):
        self.destroy_calls.append(handle)
        outcome = self._outcomes.pop(0)
        if isinstance(outcome, BaseException):
            raise outcome
        return SimpleNamespace(status=outcome)


def _make_physx(fake_lib, *, handle=17, lifecycle_acquired=True):
    physx = PhysX.__new__(PhysX)
    physx._lib = fake_lib
    physx._omni_physx_sdk_handle = None if handle is None else c_uint64(handle)
    physx._attached_ovstage = object()
    physx._lifecycle_acquired = lifecycle_acquired
    physx._released = False
    return physx


def test_native_status_failure_commits_terminal_state(monkeypatch):
    fake_lib = _FakeLib([ApiStatus.ERROR])
    physx = _make_physx(fake_lib)
    lifecycle_releases = []
    monkeypatch.setattr(api, "_release_process_lifecycle", lambda: lifecycle_releases.append(True))

    with pytest.raises(RuntimeError, match=r"status ERROR \(1\).+no longer registered"):
        physx.destroy()

    assert physx._omni_physx_sdk_handle is None
    assert physx._attached_ovstage is None
    assert physx._lifecycle_acquired is False
    assert physx._released is True
    assert fake_lib.destroy_calls == [17]
    assert lifecycle_releases == [True]

    physx.destroy()
    assert fake_lib.destroy_calls == [17]
    assert lifecycle_releases == [True]


def test_native_call_exception_retains_ownership_until_retry(monkeypatch):
    fake_lib = _FakeLib([OSError("ctypes call failed"), ApiStatus.ERROR])
    physx = _make_physx(fake_lib)
    attached_stage = physx._attached_ovstage
    lifecycle_releases = []
    monkeypatch.setattr(api, "_release_process_lifecycle", lambda: lifecycle_releases.append(True))

    with pytest.raises(OSError, match="ctypes call failed"):
        physx.destroy()

    assert physx._omni_physx_sdk_handle.value == 17
    assert physx._attached_ovstage is attached_stage
    assert physx._lifecycle_acquired is True
    assert physx._released is False
    assert lifecycle_releases == []

    with pytest.raises(RuntimeError, match=r"status ERROR \(1\)"):
        physx.destroy()

    assert physx._omni_physx_sdk_handle is None
    assert physx._attached_ovstage is None
    assert physx._lifecycle_acquired is False
    assert physx._released is True
    assert fake_lib.destroy_calls == [17, 17]
    assert lifecycle_releases == [True]


def test_shutdown_failure_is_terminal_and_not_retried(monkeypatch):
    fake_lib = _FakeLib([ApiStatus.SUCCESS])
    physx = _make_physx(fake_lib)
    lifecycle_releases = []

    def fail_shutdown():
        lifecycle_releases.append(True)
        raise RuntimeError("shutdown boom")

    monkeypatch.setattr(api, "_release_process_lifecycle", fail_shutdown)

    with pytest.raises(
        RuntimeError,
        match="instance was destroyed, but process shutdown failed: shutdown boom",
    ):
        physx.destroy()

    assert physx._omni_physx_sdk_handle is None
    assert physx._attached_ovstage is None
    assert physx._lifecycle_acquired is False
    assert physx._released is True

    physx.destroy()
    physx.__del__()
    assert fake_lib.destroy_calls == [17]
    assert lifecycle_releases == [True]


def test_shutdown_failure_includes_returned_native_error(monkeypatch):
    fake_lib = _FakeLib([ApiStatus.ERROR])
    physx = _make_physx(fake_lib)
    lifecycle_releases = []

    def fail_shutdown():
        lifecycle_releases.append(True)
        raise RuntimeError("shutdown boom")

    monkeypatch.setattr(api, "_release_process_lifecycle", fail_shutdown)

    with pytest.raises(
        RuntimeError,
        match="instance was destroyed, but process shutdown failed: shutdown boom",
    ) as exc_info:
        physx.destroy()

    assert "status ERROR (1)" in str(exc_info.value)
    assert fake_lib.destroy_calls == [17]
    assert lifecycle_releases == [True]
    assert physx._omni_physx_sdk_handle is None
    assert physx._attached_ovstage is None
    assert physx._lifecycle_acquired is False
    assert physx._released is True

    physx.destroy()
    assert fake_lib.destroy_calls == [17]
    assert lifecycle_releases == [True]


def test_removed_legacy_lifecycle_protocols_are_absent():
    assert not hasattr(PhysX, "release")
    assert not hasattr(PhysX, "__enter__")
    assert not hasattr(PhysX, "__exit__")


def test_finalizer_warning_behavior(monkeypatch):
    fake_lib = _FakeLib([])
    monkeypatch.setattr(api, "_release_process_lifecycle", lambda: None)
    unreleased = _make_physx(fake_lib, handle=0, lifecycle_acquired=False)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        unreleased.__del__()

    assert any(issubclass(item.category, ResourceWarning) for item in caught)
    assert unreleased._released is True


@pytest.mark.parametrize("retry_status", [ApiStatus.SUCCESS, ApiStatus.ERROR])
def test_finalizer_retries_native_invocation_exception_once(monkeypatch, retry_status):
    fake_lib = _FakeLib([OSError("ctypes call failed"), retry_status])
    physx = _make_physx(fake_lib)
    lifecycle_releases = []
    monkeypatch.setattr(api, "_release_process_lifecycle", lambda: lifecycle_releases.append(True))

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        physx.__del__()

    assert fake_lib.destroy_calls == [17, 17]
    assert lifecycle_releases == [True]
    assert physx._omni_physx_sdk_handle is None
    assert physx._attached_ovstage is None
    assert physx._lifecycle_acquired is False
    assert physx._released is True
    assert sum(issubclass(item.category, ResourceWarning) for item in caught) == 1
    assert not any(issubclass(item.category, RuntimeWarning) for item in caught)


def test_finalizer_does_not_retry_terminal_status_failure(monkeypatch):
    fake_lib = _FakeLib([ApiStatus.ERROR])
    physx = _make_physx(fake_lib)
    lifecycle_releases = []
    monkeypatch.setattr(api, "_release_process_lifecycle", lambda: lifecycle_releases.append(True))

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        physx.__del__()

    assert fake_lib.destroy_calls == [17]
    assert lifecycle_releases == [True]
    assert physx._omni_physx_sdk_handle is None
    assert physx._attached_ovstage is None
    assert physx._lifecycle_acquired is False
    assert physx._released is True
    assert sum(issubclass(item.category, ResourceWarning) for item in caught) == 1
    assert not any(issubclass(item.category, RuntimeWarning) for item in caught)


def test_finalizer_abandons_ownership_after_persistent_invocation_exceptions(monkeypatch):
    fake_lib = _FakeLib([OSError("first failure"), OSError("second failure")])
    physx = _make_physx(fake_lib)
    lifecycle_releases = []
    monkeypatch.setattr(api, "_release_process_lifecycle", lambda: lifecycle_releases.append(True))

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        physx.__del__()

    assert fake_lib.destroy_calls == [17, 17]
    assert lifecycle_releases == [True]
    assert physx._omni_physx_sdk_handle is None
    assert physx._attached_ovstage is None
    assert physx._lifecycle_acquired is False
    assert physx._released is True
    assert sum(issubclass(item.category, ResourceWarning) for item in caught) == 1
    runtime_warnings = [item for item in caught if issubclass(item.category, RuntimeWarning)]
    assert len(runtime_warnings) == 1
    assert "native instance may still be registered" in str(runtime_warnings[0].message)


@pytest.mark.parametrize("initial_refcount", [1, 2])
def test_callback_context_finalizer_defers_cleanup_without_joining(monkeypatch, initial_refcount):
    """GC cleanup must retain self and finish off the native callback thread."""
    fake_lib = _FakeLib([ApiStatus.SUCCESS])
    physx = _make_physx(fake_lib)
    worker_entered = threading.Event()
    release_worker = threading.Event()
    worker_observations = []
    workers = []
    shutdown_calls = []
    original_thread = threading.Thread

    monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_REFCOUNT", initial_refcount)
    monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_INITIALIZING", False)
    monkeypatch.setattr(api, "_PROCESS_LIFECYCLE_SHUTTING_DOWN", False)
    monkeypatch.setattr(api, "_python_log_callback", None)
    monkeypatch.setattr(api, "_python_log_logger_name", None)
    monkeypatch.setattr(api, "_python_log_retained_callbacks", ())
    monkeypatch.setattr(api, "_python_log_callback_transition", False)

    def destroy_instance(handle):
        worker_observations.append(
            (
                threading.get_ident(),
                getattr(api._python_log_callback_context, "active", False),
            )
        )
        worker_entered.set()
        release_worker.wait(timeout=5)
        fake_lib.destroy_calls.append(handle)
        return SimpleNamespace(status=ApiStatus.SUCCESS)

    fake_lib.ovphysx_destroy_instance = destroy_instance
    monkeypatch.setattr(
        api._lib,
        "ovphysx_shutdown",
        lambda: shutdown_calls.append(True) or SimpleNamespace(status=ApiStatus.SUCCESS),
    )

    def capture_thread(*args, **kwargs):
        worker = original_thread(*args, **kwargs)
        workers.append(worker)
        return worker

    monkeypatch.setattr(api.threading, "Thread", capture_thread)

    callback_thread = threading.get_ident()
    physx_ref = weakref.ref(physx)
    api._python_log_callback_context.active = True
    try:
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            del physx
            gc.collect()
        assert worker_entered.wait(timeout=5), "Deferred finalizer worker did not start"
        assert len(workers) == 1
        assert workers[0].daemon
        assert workers[0].is_alive(), "Finalizer joined its worker from the callback"
        assert physx_ref() is not None, "Deferred worker did not retain the unreachable instance"
    finally:
        api._python_log_callback_context.active = False
        release_worker.set()
        for worker in workers:
            worker.join(timeout=5)

    assert all(not worker.is_alive() for worker in workers)
    gc.collect()
    assert physx_ref() is None
    assert worker_observations == [(workers[0].ident, False)]
    assert worker_observations[0][0] != callback_thread
    assert fake_lib.destroy_calls == [17]
    assert api._PROCESS_LIFECYCLE_REFCOUNT == initial_refcount - 1
    assert shutdown_calls == ([True] if initial_refcount == 1 else [])
    assert sum(issubclass(item.category, ResourceWarning) for item in caught) == 1


def test_partial_construction_destroy_needs_no_initialized_attributes():
    physx = PhysX.__new__(PhysX)

    physx.destroy()

    assert physx._omni_physx_sdk_handle is None
    assert physx._attached_ovstage is None
    assert physx._released is True


def test_invalid_partial_handle_releases_lifecycle_once(monkeypatch):
    fake_lib = _FakeLib([])
    physx = _make_physx(fake_lib, handle=0, lifecycle_acquired=True)
    lifecycle_releases = []
    monkeypatch.setattr(api, "_release_process_lifecycle", lambda: lifecycle_releases.append(True))

    physx.destroy()
    physx.destroy()

    assert fake_lib.destroy_calls == []
    assert lifecycle_releases == [True]
    assert physx._omni_physx_sdk_handle is None
    assert physx._attached_ovstage is None
    assert physx._lifecycle_acquired is False
    assert physx._released is True
