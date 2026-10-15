# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-TASK-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5

"""Deterministic receipt, timeout precedence, and native-result ownership tests."""

import ctypes
import threading
from dataclasses import FrozenInstanceError
from types import SimpleNamespace

import pytest

from ovphysx import ApiStatus, PhysX, Task, TaskError, TaskTimeoutError
from ovphysx._bindings import (
    ovphysx_enqueue_result_t,
    ovphysx_result_t,
    ovphysx_string_t,
    ovphysx_wait_task_result_t,
)


def _fake_sdk(lib):
    sdk = object.__new__(PhysX)
    sdk._lib = lib
    sdk._omni_physx_sdk_handle = ctypes.c_uint64(17)
    sdk._released = True
    sdk._finalize_unreleased = lambda: None
    sdk._pending_read_lock = threading.Lock()
    sdk._pending_read_releases = []
    sdk._pending_write_lock = threading.Lock()
    sdk._pending_write_releases = []
    return sdk


def test_task_is_an_immutable_native_receipt_copy():
    receipt = ovphysx_enqueue_result_t(ApiStatus.SUCCESS, 4, 17)
    task = Task._from_native(receipt)
    receipt.op_index = 99
    receipt.instance = 88
    assert task == Task(ApiStatus.SUCCESS, 4, 17)
    with pytest.raises(FrozenInstanceError):
        task.op_index = 42
    native_copy = task._as_native()
    native_copy.op_index = 55
    assert task.op_index == 4


@pytest.mark.parametrize("value", [None, 1, "1", (ApiStatus.SUCCESS, 1, 17), object()])
def test_wait_task_rejects_non_task_before_calling_native(value):
    sdk = _fake_sdk(SimpleNamespace())
    with pytest.raises(TypeError, match="must be a Task"):
        sdk.wait_task(value)


@pytest.mark.parametrize("operation", ["wait_task", "wait_all"])
@pytest.mark.parametrize("timeout_ns", [-1, -(1 << 64), 1 << 64, (1 << 64) + 1])
def test_wait_rejects_out_of_range_timeout_before_calling_native(operation, timeout_ns):
    sdk = _fake_sdk(SimpleNamespace())
    args = (Task(ApiStatus.SUCCESS, 4, 17),) if operation == "wait_task" else ()
    with pytest.raises(ValueError, match="timeout_ns must be in range"):
        getattr(sdk, operation)(*args, timeout_ns=timeout_ns)


@pytest.mark.parametrize("operation", ["wait_task", "wait_all"])
@pytest.mark.parametrize("timeout_ns", [None, 0, 1, (1 << 64) - 1])
def test_wait_preserves_timeout_boundaries(operation, timeout_ns):
    timeouts = []

    def wait(*args):
        timeouts.append(args[-2])
        return ovphysx_result_t(ApiStatus.SUCCESS)

    sdk = _fake_sdk(SimpleNamespace(
        ovphysx_wait_task=wait,
        ovphysx_wait_all=wait,
        ovphysx_destroy_wait_task_result=lambda result: None,
    ))
    args = (Task(ApiStatus.SUCCESS, 4, 17),) if operation == "wait_task" else ()
    assert getattr(sdk, operation)(*args, timeout_ns=timeout_ns) is None
    assert timeouts == [(1 << 64) - 1 if timeout_ns is None else timeout_ns]


@pytest.mark.parametrize("operation", ["wait_task", "wait_all"])
def test_partial_timeout_copies_errors_and_pending_before_cleanup(operation):
    failed = Task(ApiStatus.SUCCESS, 4, 17)
    pending = Task(ApiStatus.SUCCESS, 5, 17)
    native_errors = (ovphysx_enqueue_result_t * 1)(failed._as_native())
    native_text = ctypes.create_string_buffer(b"earlier failure")
    destroyed = []
    timeouts = []

    def wait(*args):
        timeouts.append(args[-2])
        result = ctypes.cast(args[-1], ctypes.POINTER(ovphysx_wait_task_result_t)).contents
        result.error_tasks = native_errors
        result.num_errors = 1
        result.pending_task = pending._as_native()
        return ovphysx_result_t(ApiStatus.TIMEOUT)

    def error(instance, task):
        assert instance == 17
        assert Task._from_native(task) == failed
        text = ovphysx_string_t()
        text.ptr = ctypes.cast(native_text, ctypes.c_char_p)
        text.length = len(b"earlier failure")
        return text

    def destroy(result_ptr):
        destroyed.append(True)
        native_text.value = b"overwritten"
        native_errors[0].op_index = 999
        ctypes.memset(result_ptr, 0, ctypes.sizeof(ovphysx_wait_task_result_t))

    sdk = _fake_sdk(SimpleNamespace(
        ovphysx_wait_task=wait,
        ovphysx_wait_all=wait,
        ovphysx_get_last_task_error=error,
        ovphysx_destroy_wait_task_result=destroy,
    ))
    with pytest.raises(TaskTimeoutError) as caught:
        if operation == "wait_task":
            sdk.wait_task(pending, timeout_ns=0)
        else:
            sdk.wait_all(timeout_ns=0)

    exception = caught.value
    assert isinstance(exception, TimeoutError)
    assert exception.errors == ((failed, "earlier failure"),)
    assert exception.pending_task == pending
    assert destroyed == [True]
    assert timeouts == [0]
    with pytest.raises(AttributeError):
        exception.errors = ()
    with pytest.raises(AttributeError):
        exception.pending_task = failed
    with pytest.raises(TypeError):
        exception.errors[0] = (failed, "modified")

    def complete(instance, receipt, timeout, result_ptr):
        assert Task._from_native(receipt) == pending
        assert timeout == 0xFFFFFFFFFFFFFFFF
        return ovphysx_result_t(ApiStatus.SUCCESS)

    sdk._lib.ovphysx_wait_task = complete
    assert sdk.wait_task(exception.pending_task) is None
    sdk._omni_physx_sdk_handle = None
    assert exception.errors == ((failed, "earlier failure"),)
    assert exception.pending_task == pending
    assert destroyed == [True, True]


@pytest.mark.parametrize("status", [ApiStatus.ERROR, ApiStatus.INVALID_ARGUMENT, ApiStatus.NOT_FOUND])
@pytest.mark.parametrize("operation", ["wait_task", "wait_all"])
def test_wait_validation_and_internal_failures_are_runtime_errors(status, operation):
    destroyed = []
    message = ovphysx_string_t("native wait validation failed")
    lib = SimpleNamespace(
        ovphysx_wait_task=lambda *args: ovphysx_result_t(status),
        ovphysx_wait_all=lambda *args: ovphysx_result_t(status),
        ovphysx_get_last_error=lambda: message,
        ovphysx_destroy_wait_task_result=lambda result: destroyed.append(True),
    )
    sdk = _fake_sdk(lib)
    with pytest.raises(RuntimeError, match="native wait validation failed") as caught:
        if operation == "wait_task":
            sdk.wait_task(Task(ApiStatus.SUCCESS, 4, 17))
        else:
            sdk.wait_all()
    assert type(caught.value) is RuntimeError
    assert destroyed == [True]


def test_error_copy_failure_still_destroys_native_result():
    errors = (ovphysx_enqueue_result_t * 1)(ovphysx_enqueue_result_t(ApiStatus.SUCCESS, 4, 17))
    destroyed = []

    def wait(instance, timeout, result_ptr):
        result = ctypes.cast(result_ptr, ctypes.POINTER(ovphysx_wait_task_result_t)).contents
        result.error_tasks = errors
        result.num_errors = 1
        return ovphysx_result_t(ApiStatus.ERROR)

    def lookup(instance, receipt):
        raise MemoryError("copy failed")

    sdk = _fake_sdk(SimpleNamespace(
        ovphysx_wait_all=wait,
        ovphysx_get_last_error=lambda: ovphysx_string_t("task failure"),
        ovphysx_get_last_task_error=lookup,
        ovphysx_destroy_wait_task_result=lambda result: destroyed.append(True),
    ))
    with pytest.raises(MemoryError, match="copy failed"):
        sdk.wait_all()
    assert destroyed == [True]


def test_task_error_entries_are_immutable():
    task = Task(ApiStatus.SUCCESS, 4, 17)
    source_entries = [[task, "failure"]]
    exception = TaskError(source_entries)
    source_entries[0][1] = "changed"
    assert exception.errors == ((task, "failure"),)
    with pytest.raises(AttributeError):
        exception.errors = ()
    with pytest.raises(TypeError):
        exception.errors[0][1] = "changed"


@pytest.mark.parametrize(
    "query_status,attached,keeps_stage",
    [(ApiStatus.SUCCESS, False, False), (ApiStatus.SUCCESS, True, True), (ApiStatus.ERROR, False, True)],
)
def test_detach_retains_stage_until_collected_and_native_detach_is_confirmed(query_status, attached, keeps_stage):
    stage = object()

    def query(instance, stage, out_handle):
        ctypes.cast(out_handle, ctypes.POINTER(ctypes.c_bool)).contents.value = attached
        return ovphysx_result_t(query_status)

    sdk = _fake_sdk(SimpleNamespace(
        ovphysx_detach_ovstage=lambda instance: ovphysx_enqueue_result_t(ApiStatus.SUCCESS, 4, instance),
        ovphysx_is_stage_attached=query,
        ovphysx_wait_task=lambda *args: ovphysx_result_t(ApiStatus.SUCCESS),
        ovphysx_destroy_wait_task_result=lambda result: None,
        ovphysx_get_last_error=lambda: ovphysx_string_t("query failed"),
    ))
    sdk._attached_ovstage = stage
    sdk._drain_pending_read_releases = lambda: None
    sdk._release_stage_bound_read_handles = lambda: None
    task = sdk.detach_ovstage()
    assert task == Task(ApiStatus.SUCCESS, 4, 17)
    assert sdk._attached_ovstage is stage
    if query_status == ApiStatus.SUCCESS:
        sdk.wait_task(task)
    else:
        with pytest.raises(RuntimeError, match="query attachment state"):
            sdk.wait_task(task)
    assert sdk._attached_ovstage is (stage if keeps_stage else None)
