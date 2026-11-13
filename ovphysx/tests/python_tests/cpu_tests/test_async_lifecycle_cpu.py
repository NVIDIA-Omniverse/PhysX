# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-TASK-001
# @covers AC-5 AC-6 AC-7 AC-10 AC-11
# @maps_to TEST-PYTHON-TASK-002
# @implements REQ-PYTHON-LIFECYCLE-001
# @covers AC-5 AC-7

"""Python ownership while long calls really stay pending on the instance worker.

A private native test gate holds the worker, so attach, detach, clone and step
jobs stay queued while the test inspects Stage ownership, query identity, task
status and finalizers. Release the hold before any call that waits for the
worker: such a call would otherwise never return.
"""

import ctypes
import gc
import os
import threading
import time
import warnings
import weakref
from contextlib import contextmanager

import numpy as np
import ovphysx.api as api
import ovstage
from ovphysx.population import physics_population_descs
import pytest
from ovphysx import ApiStatus, PhysX, TaskError, TaskStatus
from ovphysx import _bindings
from ovphysx.types import SimObjectType, TensorType
from test_utils import data_path, load_usd_with_ovstage, read_rigid_body_poses, register_physx_schemas_with_ovstage


def _set_worker_hold(sdk, hold):
    # Test-only binding of the private gate, bound here and never in ovphysx._bindings.
    hold_worker = _bindings._lib["ovphysx_set_worker_hold_for_test_internal"]
    hold_worker.argtypes = [ctypes.c_uint64, ctypes.c_bool]
    hold_worker.restype = ctypes.c_bool
    assert hold_worker(sdk.handle, hold)


# Under OVPHYSX_TEST_EXECUTION=noworker instances have no worker thread: each call runs
# before it returns, callbacks included.
_NO_WORKER = os.environ.get("OVPHYSX_TEST_EXECUTION") == "noworker"


@contextmanager
def _worker_held(sdk):
    if _NO_WORKER:
        pytest.skip("the instance has no worker thread to hold")
    _set_worker_hold(sdk, True)
    try:
        yield
    finally:
        _set_worker_hold(sdk, False)


def _stage(name, scene="basic_simulation.usda", *, sealed=True):
    register_physx_schemas_with_ovstage()
    stage = ovstage.Stage(name)
    ovstage.population.open_usd_with_desc(stage, data_path(scene), ordinal=1, time=ovstage.population.DEFAULT_TIME, descs=physics_population_descs(rendering=True))
    if sealed:
        stage.advance_write_floor(ordinal=1).wait()
    return stage


def _finished_status(sdk, task, deadline_s=30.0):
    deadline = time.monotonic() + deadline_s
    status = sdk.task_status(task)
    while status in (TaskStatus.PENDING, TaskStatus.RUNNING) and time.monotonic() < deadline:
        time.sleep(0.001)
        status = sdk.task_status(task)
    return status


def _attach_a_then_b(sdk, *, reverse):
    stage_a, stage_b = _stage("async-attach-a"), _stage("async-attach-b")
    a, b = weakref.ref(stage_a), weakref.ref(stage_b)
    with _worker_held(sdk):
        task_a = sdk.attach_ovstage(stage_a)
        task_b = sdk.attach_ovstage(stage_b)
        assert sdk.task_status(task_a) == TaskStatus.PENDING
        assert sdk.task_status(task_b) == TaskStatus.PENDING
        del stage_a, stage_b
        gc.collect()
        assert a() is not None and b() is not None, "pending attach candidates must stay pinned"
    # A attaches first; B then finds this instance attached.
    assert _finished_status(sdk, task_b) == TaskStatus.FAILED

    def collect(task):
        if task == task_b:
            with pytest.raises(TaskError, match="already attached"):
                sdk.wait_task(task)
        else:
            sdk.wait_task(task)

    first, second = (task_b, task_a) if reverse else (task_a, task_b)
    collect(first)
    gc.collect()
    # Ownership is known only once both receipts are collected.
    assert a() is not None and b() is not None
    assert sdk.is_stage_attached(a())
    collect(second)
    gc.collect()
    assert a() is not None, "garbage collection freed the attached Stage"
    assert sdk.is_stage_attached(a()) and sdk._attached_ovstage is a()
    assert b() is None, "the failed attach's Stage outlived the known ownership"
    sdk.wait_task(sdk.detach_ovstage())
    gc.collect()
    assert a() is None


def test_attach_a_then_b_collect_in_order(physx_sdk):
    _attach_a_then_b(physx_sdk, reverse=False)


def test_attach_a_then_b_collect_in_reverse(physx_sdk):
    _attach_a_then_b(physx_sdk, reverse=True)


def test_failed_attach_rollback_keeps_stage_until_ownership_known(physx_sdk):
    """An attach that fails when it runs keeps its Stage until collection shows it unattached.

    The articulation schema scan reads the unsealed ordinal and fails the accepted
    task without leaving an attachment behind.
    """
    sdk = physx_sdk
    stage = _stage("async-attach-unsealed", "CartPole.usda", sealed=False)
    ref = weakref.ref(stage)
    with _worker_held(sdk):
        task = sdk.attach_ovstage(stage)
        del stage
        gc.collect()
        assert ref() is not None
    assert _finished_status(sdk, task) == TaskStatus.FAILED
    gc.collect()
    assert ref() is not None, "an uncollected failed attach dropped its Stage"
    with pytest.raises(TaskError, match="schema scan"):
        sdk.wait_task(task)
    assert not sdk.is_stage_attached()
    assert sdk._attached_ovstage is None
    gc.collect()
    assert ref() is None


def test_detach_keeps_stage_until_detach_completes(physx_sdk):
    """A detach keeps the Stage until its success is collected, dropped references or not.

    A detach that fails when it runs keeps the Stage committed; the Python double in
    test_session_operations.py covers it, since no Python call can make it fail here.
    """
    sdk = physx_sdk
    stage = _stage("async-detach")
    ref = weakref.ref(stage)
    sdk.wait_task(sdk.attach_ovstage(stage))
    del stage
    gc.collect()
    assert ref() is not None and sdk._attached_ovstage is ref()
    with _worker_held(sdk):
        task = sdk.detach_ovstage()
        gc.collect()
        assert sdk.task_status(task) == TaskStatus.PENDING
        assert ref() is not None and sdk.is_stage_attached(ref())
    assert _finished_status(sdk, task) == TaskStatus.SUCCEEDED
    gc.collect()
    assert ref() is not None, "the Stage was released before its detach was collected"
    sdk.wait_task(task)
    gc.collect()
    assert ref() is None and sdk._attached_ovstage is None


def test_query_after_unwaited_attach_succeeds(physx_sdk):
    sdk = physx_sdk
    stage = _stage("async-query-after-attach")
    task = sdk.attach_ovstage(stage)
    # No wait in between: the query runs after the queued attach and belongs to it.
    query = sdk.query(SimObjectType.RIGID_BODY).wait()
    try:
        assert query._stage is stage
        assert query._attachment != 0 and query._attachment == sdk._get_attachment_identity()
        assert query.metadata.total_prim_count == 1
        sdk.wait_task(task)
        query._validate()
        with _worker_held(sdk):
            detach = sdk.detach_ovstage()
            # Not run yet: the query still belongs to the live attachment.
            query._validate()
        sdk.wait_task(detach)
        with pytest.raises(RuntimeError, match="invalidated attachment"):
            query._validate()
    finally:
        query.close().wait()


def test_read_sync_right_after_an_unwaited_attach_reads_that_attachment(physx_sdk):
    sdk = physx_sdk
    stage = _stage("async-read-sync-after-attach")
    task = sdk.attach_ovstage(stage)
    with sdk.read_sync(SimObjectType.RIGID_BODY, ["position"]) as result:
        assert result._stage is stage
        assert [group.prim_count for group in result.groups] == [1]
    sdk.wait_task(task)


def test_query_and_read_queued_behind_a_step_materialize_after_it(physx_sdk):
    sdk = physx_sdk
    load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
    with _worker_held(sdk):
        step = sdk.step(1.0 / 60.0)
        pending_query = sdk.query(SimObjectType.RIGID_BODY)
        assert sdk.task_status(pending_query.task) == TaskStatus.PENDING
    query = pending_query.wait()
    sdk.wait_task(step)
    with query:
        with _worker_held(sdk):
            step = sdk.step(1.0 / 60.0)
            pending_read = sdk.read(query, ["position"])
            assert sdk.task_status(pending_read.task) == TaskStatus.PENDING
        with pending_read.wait() as result:
            group = result.fetch_next().wait()
            assert group.prim_count == 1
            assert group.tensors[0].numpy()[0].tolist() == read_rigid_body_poses(sdk)[0, :3].tolist()
    sdk.wait_task(step)


def test_detach_before_materialization_invalidates_the_query(physx_sdk):
    sdk = physx_sdk
    load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
    with _worker_held(sdk):
        pending_query = sdk.query(SimObjectType.RIGID_BODY)
        detach = sdk.detach_ovstage()
    sdk.wait_task(detach)
    with pytest.raises(RuntimeError, match="invalidated attachment"):
        pending_query.wait()


@pytest.mark.parametrize("venue", ["calling thread", "queued"])
def test_query_that_finds_no_stage_fails_its_task_on_both_venues(physx_sdk, venue):
    sdk = physx_sdk
    load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
    if venue == "queued":
        with _worker_held(sdk):
            detach = sdk.detach_ovstage()
            pending_query = sdk.query(SimObjectType.RIGID_BODY)
            assert sdk.task_status(pending_query.task) == TaskStatus.PENDING
    else:
        # The detach ran but is uncollected, so Python still submits the query.
        detach = sdk.detach_ovstage()
        assert _finished_status(sdk, detach) == TaskStatus.SUCCEEDED
        pending_query = sdk.query(SimObjectType.RIGID_BODY)
    with pytest.raises(TaskError, match="attach an ovstage Stage first"):
        pending_query.wait()
    sdk.wait_task(detach)


def test_clone_inputs_may_change_after_submit(physx_sdk):
    """Native code copies every clone input: the caller's lists may change at once."""
    sdk = physx_sdk
    load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
    targets = ["/World/envs/env1", "/World/envs/env2"]
    anchors = [(5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), (10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)]
    env_ids = [3, 4]
    with _worker_held(sdk):
        task = sdk.clone("/World/envs/env0", targets, anchor_transforms=anchors, env_ids=env_ids)
        assert sdk.task_status(task) == TaskStatus.PENDING
        # The job has not run: overwrite every input, drop it, and reuse freed memory.
        targets[:] = ["/World/envs/env7", "/World/envs/env8"]
        anchors[:] = [(70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)] * 2
        env_ids[:] = [0, 0]
        del targets, anchors, env_ids
        gc.collect()
        churn = [bytearray(4096) for _ in range(256)]
        del churn
    sdk.wait_task(task)
    sdk.warmup()
    # The table sits at x = 0 in env0; the clones keep it at their anchors' x.
    poses = read_rigid_body_poses(sdk)
    assert sorted(np.round(poses[:, 0], 3).tolist()) == [0.0, 5.0, 10.0]


def test_query_task_status_does_not_collect(physx_sdk):
    sdk = physx_sdk
    stage = _stage("async-task-status")
    with _worker_held(sdk):
        task = sdk.attach_ovstage(stage)
        assert sdk.task_status(task) == TaskStatus.PENDING
        assert sdk.task_status(task) == TaskStatus.PENDING
    assert _finished_status(sdk, task) == TaskStatus.SUCCEEDED
    for _ in range(3):
        assert sdk.task_status(task) == TaskStatus.SUCCEEDED
    # A status query collects nothing, so ownership is not committed yet either.
    assert sdk._attached_ovstage is None
    sdk.wait_task(task)
    assert sdk.task_status(task) == TaskStatus.NOT_FOUND
    assert sdk._attached_ovstage is stage
    sdk.wait_task(sdk.detach_ovstage())


def test_step_returns_pending_while_worker_held(physx_sdk):
    sdk = physx_sdk
    load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
    with _worker_held(sdk):
        step = sdk.step(1.0 / 60.0)
        assert step.status == ApiStatus.SUCCESS
        assert sdk.task_status(step) == TaskStatus.PENDING
        assert sdk.task_status(step) == TaskStatus.PENDING
    assert sdk.wait_task(step) is None
    assert sdk.task_status(step) == TaskStatus.NOT_FOUND


@pytest.mark.parametrize("active_cuda_gpus", [None, "0"])
def test_constructor_in_object_change_callback_rejects_before_native_ownership(physx_sdk, monkeypatch, active_cuda_gpus):
    """A real object-change callback cannot acquire a process token or create an instance."""
    sdk = physx_sdk
    load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
    refcount = api._PROCESS_LIFECYCLE_REFCOUNT
    attempted_creates = []
    refusals = []

    def unexpected_create(*_args):
        attempted_creates.append(api._PROCESS_LIFECYCLE_REFCOUNT)
        # Keep a failing regression from leaking the instance it wrongly creates.
        raise RuntimeError("native creation reached from an object-change callback")

    def callback(*_args):
        try:
            PhysX(active_cuda_gpus=active_cuda_gpus)
        except RuntimeError as error:
            refusals.append(str(error))

    subscription = sdk.subscribe_object_changes(on_destroyed=callback, on_all_destroyed=callback)
    try:
        with monkeypatch.context() as patch:
            patch.setattr(api._lib, "ovphysx_create_instance", unexpected_create)
            sdk.wait_task(sdk.detach_ovstage())
    finally:
        sdk.unsubscribe_object_changes(subscription)

    assert refusals and all("PhysX() cannot be called" in error and "callback" in error for error in refusals)
    assert not attempted_creates
    assert api._PROCESS_LIFECYCLE_REFCOUNT == refcount
    assert not api._in_native_callback()


@pytest.mark.skipif(_NO_WORKER, reason="the instance has no worker thread to run the callback")
def test_finalizer_on_worker_thread_only_defers(physx_sdk, monkeypatch):
    """Objects dropped in a callback on the worker are cleaned up later, elsewhere.

    The detach job delivers the object-change callback on the instance worker. Inside
    it a PhysX call refuses instead of waiting, a dropped Query only queues its
    release for the owning thread, and an unreachable PhysX instance is destroyed by
    a deferred finalizer thread after the callback returns.
    """
    sdk = physx_sdk
    load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
    other = PhysX()
    other_handle = other.handle
    held = {"query": sdk.query(SimObjectType.RIGID_BODY).wait(), "other": other}
    del other
    main_thread = threading.get_ident()
    observed = {}
    deferred_from = []
    destroyed_on = []
    start_deferred = api._start_deferred_finalizer
    destroy = PhysX._destroy

    def recording_start(instance, **kwargs):
        deferred_from.append(threading.get_ident())
        return start_deferred(instance, **kwargs)

    def recording_destroy(self, **kwargs):
        destroyed_on.append(threading.get_ident())
        return destroy(self, **kwargs)

    monkeypatch.setattr(api, "_start_deferred_finalizer", recording_start)
    monkeypatch.setattr(PhysX, "_destroy", recording_destroy)

    def callback(*_args):
        if "thread" in observed:
            return
        observed["thread"] = threading.get_ident()
        try:
            sdk.step(1.0 / 60.0)
        except RuntimeError as error:
            observed["step"] = str(error)
        # Drops the Query and the last reference to the other instance here.
        held.clear()
        observed["pending_releases"] = len(sdk._pending_write_releases)

    subscription = sdk.subscribe_object_changes(
        on_destroyed=callback, on_all_destroyed=callback)
    try:
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            # An infinite wait may run an unstarted detach on this thread. A finite
            # wait leaves the job on the worker whose finalization this test checks.
            sdk.wait_task(sdk.detach_ovstage(), timeout_ns=60_000_000_000)
    finally:
        sdk.unsubscribe_object_changes(subscription)
        with api._PROCESS_LIFECYCLE_INSTANCES_LOCK:
            finalizers = list(api._DEFERRED_FINALIZER_THREADS)
        for worker in finalizers:
            worker.join(timeout=30)

    assert observed.get("thread") not in (None, main_thread), "no callback ran on the worker"
    assert "callback" in observed.get("step", ""), observed
    assert observed["pending_releases"] == 1, "the dropped Query's release was not just queued"
    assert sdk._pending_write_releases, "the Query's release ran before the owning thread's drain"
    sdk.wait_all()
    assert sdk._pending_write_releases, "a wait submitted the deferred release"
    sdk.query(SimObjectType.DEBUG_RENDER).wait().close().wait()
    assert not sdk._pending_write_releases
    assert deferred_from == [observed["thread"]]
    assert destroyed_on and observed["thread"] not in destroyed_on and main_thread not in destroyed_on
    assert all(not worker.is_alive() for worker in finalizers)
    attached = ctypes.c_bool(False)
    gone = _bindings._lib.ovphysx_is_stage_attached(other_handle, None, ctypes.byref(attached))
    assert gone.status == ApiStatus.INVALID_ARGUMENT, "the other instance was not destroyed"
    assert any(issubclass(item.category, ResourceWarning) for item in caught)


@pytest.mark.filterwarnings("ignore:ovphysx tensor bindings are deprecated:DeprecationWarning")
def test_binding_call_in_a_worker_callback_raises_instead_of_deadlocking(physx_sdk):
    """A callback on the worker touches a binding that this thread holds while it waits for the worker."""
    sdk = physx_sdk
    load_usd_with_ovstage(sdk, data_path("boxes_falling_on_groundplane.usda"))
    binding = sdk.create_tensor_binding(pattern="/World/Cube*", tensor_type=TensorType.RIGID_BODY_POSE)
    poses = np.zeros(binding.shape, dtype=np.float32)
    refusals = []

    def touch_binding(*_args):
        if not refusals:
            try:
                binding.prim_paths
            except RuntimeError as error:
                refusals.append(str(error))

    subscription = sdk.subscribe_object_changes(on_destroyed=touch_binding, on_all_destroyed=touch_binding)
    try:
        _set_worker_hold(sdk, True)
        detach = sdk.detach_ovstage()
        # The read takes the binding, then waits for the held detach, whose callbacks run once this fires.
        threading.Timer(0.5, _set_worker_hold, (sdk, False)).start()
        with pytest.raises(RuntimeError, match="binding invalidated"):
            binding.read(poses)
        sdk.wait_task(detach)
    finally:
        _set_worker_hold(sdk, False)
        sdk.unsubscribe_object_changes(subscription)
        binding.destroy()
    assert refusals and "callback" in refusals[0], refusals


@pytest.mark.filterwarnings("ignore:ovphysx tensor bindings are deprecated:DeprecationWarning")
def test_binding_dropped_in_a_worker_callback_is_destroyed_by_its_owner(physx_sdk, monkeypatch):
    """A binding finalized in a callback of another instance cannot take its lock there.
    Its destruction waits for the owner's earlier work, so an async submission, a finite
    wait or a poll of the owner leaves it queued, and the owner's next wait without a
    deadline or step_sync() destroys it on that thread: a step-and-wait loop never
    accumulates them. Destroying the owner drops one still queued."""
    sdk = physx_sdk
    other = PhysX()
    try:
        load_usd_with_ovstage(other, data_path("boxes_falling_on_groundplane.usda"))
        destroyed_on = []
        destroy = api.TensorBinding.destroy

        def recording_destroy(self):
            destroy(self)
            destroyed_on.append(threading.get_ident())

        monkeypatch.setattr(api.TensorBinding, "destroy", recording_destroy)

        def drop_binding_in_a_callback():
            # Detaching sdk runs its object-change callback, which drops the last
            # reference to a binding of other.
            load_usd_with_ovstage(sdk, data_path("basic_simulation.usda"))
            held = [other.create_tensor_binding(pattern="/World/Cube*", tensor_type=TensorType.RIGID_BODY_POSE)]
            dropped = weakref.ref(held[0])
            subscription = sdk.subscribe_object_changes(on_all_destroyed=lambda *_args: held.clear())
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore", ResourceWarning)
                    sdk.wait_task(sdk.detach_ovstage())
            finally:
                sdk.unsubscribe_object_changes(subscription)
            assert not held, "the callback did not run"
            return dropped

        for dropped_so_far in range(3):
            drop_binding_in_a_callback()
            assert len(destroyed_on) == dropped_so_far
            other.wait_task(other.step(1.0 / 60.0), timeout_ns=60_000_000_000)
            assert len(destroyed_on) == dropped_so_far
            other.wait_task(other.step(1.0 / 60.0))
            assert destroyed_on == [threading.get_ident()] * (dropped_so_far + 1)
        drop_binding_in_a_callback()
        other.step_sync(1.0 / 60.0)
        assert len(destroyed_on) == 4

        dropped = drop_binding_in_a_callback()
        other.destroy()
        gc.collect()
        assert dropped() is None, "the destroyed owner kept the finalized binding alive"
        assert len(destroyed_on) == 4
    finally:
        other.destroy()
