# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-TASK-001
# @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6

"""Real CPU runtime coverage for single-task collection and retained failures."""

from dataclasses import FrozenInstanceError

import pytest

import os
import sys
import time

from ovphysx import ApiStatus, GlobalConfigKey, PhysX, Task, TaskError, TaskStatus, get_global_config, set_global_config
from test_utils import data_path, load_usd_with_ovstage


def _load(sdk):
    load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
    sdk.wait_all()


def _fail_clone(sdk, name):
    return sdk.clone(f"/World/missing_{name}", [f"/World/clone_{name}"])


def test_failed_clone_returns_task_and_survives_waiting_for_step(physx_sdk):
    sdk = physx_sdk
    _load(sdk)
    failed = _fail_clone(sdk, "a")
    assert failed.status == ApiStatus.SUCCESS
    assert failed.instance == sdk.handle
    succeeding = sdk.step(1.0 / 60.0)
    assert sdk.wait_task(succeeding) is None

    with pytest.raises(TaskError) as caught:
        sdk.wait_all()
    assert len(caught.value.errors) == 1
    assert caught.value.errors[0][0] == failed
    assert "missing_a" in caught.value.errors[0][1]
    assert sdk.wait_all() is None
    with pytest.raises(RuntimeError, match="already collected"):
        sdk.wait_task(failed)


def test_discarded_receipts_and_multiple_errors_remain_in_submission_order(physx_sdk):
    sdk = physx_sdk
    _load(sdk)
    _fail_clone(sdk, "first")
    _fail_clone(sdk, "second")
    succeeded = sdk.clone("/World/envs/env0", ["/World/success"])
    # The clones are queued: wait for the successful one instead of polling it.
    assert sdk.wait_task(succeeded) is None
    with pytest.raises(TaskError) as caught:
        sdk.wait_all()

    errors = caught.value.errors
    assert len(errors) == 2
    assert errors[0][0].op_index < errors[1][0].op_index < succeeded.op_index
    assert all(task.instance == sdk.handle and task.status == ApiStatus.SUCCESS for task, _ in errors)
    assert "missing_first" in errors[0][1]
    assert "missing_second" in errors[1][1]
    sdk.wait_task(sdk.step(1.0 / 60.0))
    assert caught.value.errors == errors
    assert sdk.wait_all() is None


def test_direct_wait_collects_only_its_failure(physx_sdk):
    sdk = physx_sdk
    _load(sdk)
    first = _fail_clone(sdk, "first")
    second = _fail_clone(sdk, "second")
    with pytest.raises(TaskError) as caught_second:
        sdk.wait_task(second)
    assert caught_second.value.errors[0][0] == second
    with pytest.raises(TaskError) as caught_first:
        sdk.wait_task(first)
    assert caught_first.value.errors[0][0] == first
    assert "missing_second" in caught_second.value.errors[0][1]
    assert sdk.wait_all() is None


def test_successful_clone_returns_immutable_task(physx_sdk):
    sdk = physx_sdk
    _load(sdk)
    task = sdk.clone("/World/envs/env0", ["/World/clone"])
    assert isinstance(task, Task)
    with pytest.raises(FrozenInstanceError):
        task.instance = 0
    assert sdk.wait_task(task) is None
    with pytest.raises(RuntimeError, match="already collected"):
        sdk.wait_task(task)


def test_wrong_instance_does_not_collect_task(physx_sdk):
    sdk = physx_sdk
    _load(sdk)
    task = sdk.step(1.0 / 60.0)
    other = PhysX()
    try:
        with pytest.raises(RuntimeError) as caught:
            other.wait_task(task)
        assert type(caught.value) is RuntimeError
        assert sdk.wait_task(task) is None
    finally:
        other.destroy()


@pytest.mark.parametrize("status,index", [(ApiStatus.ERROR, 0), (ApiStatus.SUCCESS, 0), (ApiStatus.SUCCESS, 999999)])
def test_invalid_receipts_are_runtime_errors(physx_sdk, status, index):
    with pytest.raises(RuntimeError) as caught:
        physx_sdk.wait_task(Task(status, index, physx_sdk.handle))
    assert type(caught.value) is RuntimeError


def test_rejected_step_does_not_create_a_deferred_failure(physx_sdk):
    sdk = physx_sdk
    _load(sdk)
    with pytest.raises(RuntimeError) as caught:
        sdk.step(-0.5)
    assert type(caught.value) is RuntimeError
    assert sdk.wait_all() is None


def test_sync_step_retires_execution_without_collecting_async_receipts(physx_sdk):
    sdk = physx_sdk
    _load(sdk)
    first = sdk.step(1.0 / 60.0)
    sdk.step_sync(1.0 / 60.0)
    second = sdk.step(1.0 / 60.0)
    assert sdk.wait_task(second) is None
    assert sdk.wait_task(first, timeout_ns=0) is None
    assert sdk.wait_all() is None


def test_execution_failure_cause_survives_disabled_logging(physx_sdk):
    from ovphysx import LogLevel, get_log_level, set_log_level

    sdk = physx_sdk
    _load(sdk)
    original_level = get_log_level()
    try:
        set_log_level(LogLevel.NONE)
        task = _fail_clone(sdk, "without_logging")
        with pytest.raises(TaskError) as caught:
            sdk.wait_task(task)
        assert caught.value.errors[0][0] == task
        assert "missing_without_logging" in caught.value.errors[0][1]
    finally:
        set_log_level(original_level)


def test_waiting_session_calls_collect_only_their_own_tasks(physx_sdk, monkeypatch):
    """Explicit synchronous adapters leave an earlier failure available to its caller."""
    import gc
    from ovphysx.types import SimObjectType

    sdk = physx_sdk
    _load(sdk)
    earlier = _fail_clone(sdk, "before_sessions")
    collected = []
    wait = sdk.wait_task

    def record_wait(task, **kwargs):
        result = wait(task, **kwargs)
        collected.append(task)
        return result

    monkeypatch.setattr(sdk, "wait_task", record_wait)
    with sdk.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        assert result.groups
        token = result.groups[0].attribute
    del result
    gc.collect()
    with sdk.read_tokens_sync(SimObjectType.RIGID_BODY, [token]) as result:
        assert result.groups
    del result
    gc.collect()
    with sdk.write_sync(SimObjectType.RIGID_BODY, "position") as session:
        assert session.groups
    assert sdk.update_articulations_kinematic() is None
    sdk._drain_pending_read_releases()
    assert collected
    for task in collected:
        with pytest.raises(RuntimeError, match="already collected"):
            wait(task)
    with pytest.raises(TaskError) as caught:
        sdk.wait_all()
    assert [task for task, message in caught.value.errors] == [earlier]
    assert sdk.wait_all() is None


def _status_once_finished(sdk, task, deadline_s=30.0):
    deadline = time.monotonic() + deadline_s
    status = sdk.task_status(task)
    while status in (TaskStatus.PENDING, TaskStatus.RUNNING) and time.monotonic() < deadline:
        time.sleep(0.001)
        status = sdk.task_status(task)
    return status


def test_instance_without_a_worker_thread_finishes_each_call_before_it_returns(physx_sdk):
    # physx_sdk keeps the runtime up, so creating an instance starts no thread but its worker.
    threads = len(os.listdir("/proc/self/task")) if sys.platform.startswith("linux") else None
    previous = get_global_config(GlobalConfigKey.ASYNC_EXECUTION)
    set_global_config(GlobalConfigKey.ASYNC_EXECUTION, False)
    try:
        assert get_global_config(GlobalConfigKey.ASYNC_EXECUTION) is False
        sdk = PhysX()
    finally:
        set_global_config(GlobalConfigKey.ASYNC_EXECUTION, previous)
    try:
        if threads is not None:
            assert len(os.listdir("/proc/self/task")) == threads
        _load(sdk)
        step = sdk.step(1.0 / 60.0)
        assert sdk.task_status(step) == TaskStatus.SUCCEEDED
        assert sdk.wait_task(step) is None
    finally:
        sdk.destroy()


def test_task_status_reports_without_collecting(physx_sdk):
    sdk = physx_sdk
    _load(sdk)
    # Clones must precede the first step, so the failing clone comes first.
    failed = _fail_clone(sdk, "status")
    assert _status_once_finished(sdk, failed) == TaskStatus.FAILED
    assert sdk.task_status(failed) == TaskStatus.FAILED, "a status query never collects"
    with pytest.raises(TaskError):
        sdk.wait_task(failed)
    assert sdk.task_status(failed) == TaskStatus.NOT_FOUND

    step = sdk.step(1.0 / 60.0)
    assert sdk.task_status(step) in (TaskStatus.PENDING, TaskStatus.RUNNING, TaskStatus.SUCCEEDED)
    assert sdk.wait_task(step) is None
    assert sdk.task_status(step) == TaskStatus.NOT_FOUND
    with pytest.raises(TypeError, match="task must be a Task"):
        sdk.task_status(object())
    with pytest.raises(RuntimeError, match="task_status failed"):
        sdk.task_status(Task(ApiStatus.SUCCESS, 0, sdk.handle))
