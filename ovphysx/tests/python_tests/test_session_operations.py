# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-TASK-001
# @covers AC-6 AC-7 AC-9 AC-10
# @maps_to TEST-PYTHON-TASK-001
# @implements REQ-CAPI-READ-CONTACT-002
# @covers AC-3
# @maps_to TEST-CAPI-READ-CONTACT-003
# @implements REQ-CAPI-DETACH-001
# @covers AC-6

"""Delayed binding doubles verify ownership and collection, not a native scheduler."""

import ctypes
import gc
import threading
import sys
from types import SimpleNamespace
import weakref
from collections import defaultdict

import pytest

from ovphysx import _bindings as b
from ovphysx.api import Operation, PhysX, Query, ReadResult, Task, TaskError, TaskTimeoutError, WriteGroup, WriteSession
from ovphysx.types import ApiStatus, ContactSelection, SimObjectType


class Native:
    def __init__(self):
        self.calls = []
        self.waits = []
        self.pending = {}
        self.failures = defaultdict(list)
        self.collected_errors = {}
        self.storage = []
        self.blocked = None
        self.next_id = 1
        self.groups = []
        self.write_groups = []
        self.last_error = ""
        self.summary_requests = []

    def submit(self, name, publish=lambda: None):
        failure = self.failures[name].pop(0) if self.failures[name] else None
        self.calls.append(name)
        if failure == "reject":
            self.last_error = name + " rejected"
            return b.ovphysx_enqueue_result_t(ApiStatus.ERROR, 0, 0)
        index = self.next_id
        self.next_id += 1
        self.pending[index] = (name, publish, failure)
        return b.ovphysx_enqueue_result_t(ApiStatus.SUCCESS, index, 17)

    def _collect(self, index):
        name, publish, failure = self.pending.pop(index)
        self.waits.append(name)
        if failure == "execute":
            self.collected_errors[index] = (name + " failed").encode()
            return Task(ApiStatus.SUCCESS, index, 17)
        publish()
        return None

    def _errors(self, output, errors):
        native = (b.ovphysx_enqueue_result_t * len(errors))(*(task._as_native() for task in errors))
        self.storage.append(native)
        output._obj.error_tasks = native
        output._obj.num_errors = len(errors)

    def ovphysx_wait_task(self, instance, receipt, timeout, output):
        if receipt.op_index == self.blocked:
            output._obj.pending_task = receipt
            return b.ovphysx_result_t(ApiStatus.TIMEOUT)
        if receipt.op_index not in self.pending:
            self.last_error = "already collected"
            return b.ovphysx_result_t(ApiStatus.ERROR)
        error = self._collect(receipt.op_index)
        self._errors(output, [error] if error else [])
        return b.ovphysx_result_t(ApiStatus.SUCCESS)

    def ovphysx_wait_all(self, instance, timeout, output):
        errors = []
        for index in sorted(list(self.pending)):
            if index == self.blocked:
                self._errors(output, errors)
                output._obj.pending_task = b.ovphysx_enqueue_result_t(ApiStatus.SUCCESS, index, 17)
                return b.ovphysx_result_t(ApiStatus.TIMEOUT)
            error = self._collect(index)
            if error:
                errors.append(error)
        self._errors(output, errors)
        return b.ovphysx_result_t(ApiStatus.SUCCESS)

    def ovphysx_destroy_wait_task_result(self, output):
        self.collected_errors.clear()

    def ovphysx_get_last_task_error(self, instance, receipt):
        value = self.collected_errors[receipt.op_index]
        return b.ovx_string_t(value, len(value))

    def ovphysx_query(self, instance, kind, scope, output, metadata):
        self.summary_requests.append(("query", bool(metadata)))
        return self.submit("query", lambda: setattr(output._obj, "value", 7))

    def ovphysx_query_contacts(self, instance, selections, count, filters_per_sensor, output, metadata):
        self.summary_requests.append(("query_contacts", bool(metadata)))
        def publish():
            self.contact_selections = [
                (
                    ctypes.string_at(selections[i].sensors.pattern.ptr, selections[i].sensors.pattern.length),
                    [int(selections[i].filters[j].prim_paths[0]) for j in range(filters_per_sensor)],
                )
                for i in range(count)
            ]
            output._obj.value = 7
            if metadata:
                attributes = (ctypes.c_uint64 * 2)(40, 41)
                self.storage.append(attributes)
                metadata._obj.total_prim_count = count
                metadata._obj.attributes = attributes
                metadata._obj.attribute_count = 2
        return self.submit("query_contacts", publish)

    def ovphysx_read(self, instance, query, attrs, count, output, metadata):
        assert query == 7
        self.summary_requests.append(("read", bool(metadata)))
        def publish():
            output._obj.value = 9
            if metadata:
                rows = (b.ovphysx_read_attribute_result_t * 2)(
                    b.ovphysx_read_attribute_result_t(40, 1, 0), b.ovphysx_read_attribute_result_t(0, 0, 1))
                self.storage.append(rows)
                metadata._obj.total_prim_count = 3
                metadata._obj.attribute_count = 2
                metadata._obj.attributes = rows
        return self.submit("read", publish)

    def ovphysx_write(self, instance, query, attrs, count, output, metadata):
        assert query == 7
        self.summary_requests.append(("write", bool(metadata)))
        return self.submit("write", lambda: setattr(output._obj, "value", 10))

    def ovphysx_fetch_write_next(self, instance, write, output):
        item = self.write_groups.pop(0) if self.write_groups else None
        def publish():
            if item is not None:
                group, index = item
                output._obj.group = ctypes.pointer(group)
                output._obj.attribute_index = index
        return self.submit("fetch_write", publish)

    def ovphysx_fetch_read_next(self, instance, read, output):
        group = self.groups.pop(0) if self.groups else None
        def publish():
            if group is not None:
                ctypes.cast(output, ctypes.POINTER(ctypes.POINTER(b.ovstage_read_group_t)))[0] = ctypes.pointer(group)
        return self.submit("fetch", publish)

    def ovphysx_release_query(self, *args):
        return self.submit("release_query")

    def ovphysx_release_read(self, *args):
        return self.submit("release_read")

    def ovphysx_release_write(self, *args):
        return self.submit("release_write")

    def ovphysx_release_group(self, *args):
        return self.submit("release_group")

    def ovphysx_commit_group(self, instance, write, pointer, sync):
        self.commit_pointer = pointer
        return self.submit("commit")

    def ovphysx_destroy_instance(self, instance):
        self.calls.append("destroy")
        assert not self.pending
        return b.ovphysx_result_t(ApiStatus.SUCCESS)

    def ovphysx_attach_ovstage(self, instance, stage, ordinal):
        return self.submit("attach", self.on_attach)

    def ovphysx_update_from_ovstage(self, instance, ordinal_range):
        return self.submit("update")

    def ovphysx_detach_ovstage(self, instance):
        return self.submit("detach", self.on_detach)

    def ovphysx_is_stage_attached(self, instance, stage, output):
        output._obj.value = bool(self.attachment())
        return b.ovphysx_result_t(ApiStatus.SUCCESS)

    def ovphysx_internal_get_attach_handle(self, instance, output):
        output._obj.value = self.attachment()
        return b.ovphysx_result_t(ApiStatus.SUCCESS)


def make_sdk(native_type=Native):
    result = object.__new__(PhysX)
    result._released = True
    result._finalize_unreleased = lambda: None
    result._check_valid = lambda: None
    result._lib = native_type()
    owner = weakref.proxy(result)
    result._get_last_error = lambda: owner._lib.last_error
    result._omni_physx_sdk_handle = ctypes.c_uint64(17)
    result._attached_ovstage = object()
    result._attachment_identity = 44
    result._lib.attachment = lambda: owner._attachment_identity
    result._lib.on_attach = lambda: setattr(owner, "_attachment_identity", 44)
    result._lib.on_detach = lambda: setattr(owner, "_attachment_identity", 0)
    result._warn_missing_newton_schema = False
    result._live_read_holders = weakref.WeakSet()
    result._pending_read_lock = threading.Lock()
    result._pending_write_lock = threading.Lock()
    result._pending_read_releases = []
    result._pending_write_releases = []
    return result


@pytest.fixture
def sdk():
    # Native is a test double; finalizers only enqueue bookkeeping here.
    return make_sdk()


def query(sdk):
    return Query(sdk, 7, SimObjectType.RIGID_BODY, None, sdk._attached_ovstage, 44)


@pytest.mark.parametrize("failure", [None, "reject", "execute", "rollback"])
def test_attach_retains_stage_until_collected_ownership_is_known(sdk, monkeypatch, failure):
    class Stage:
        _inst = ctypes.c_void_p(1)

        def get_path_dictionary(self):
            return object()

    monkeypatch.setitem(sys.modules, "ovstage", SimpleNamespace(Stage=Stage))
    sdk._attached_ovstage = None
    sdk._attachment_identity = 0
    stage = Stage()
    owner = weakref.ref(stage)
    if failure:
        sdk._lib.failures["attach"] = ["execute" if failure == "rollback" else failure]
    if failure == "reject":
        with pytest.raises(RuntimeError, match="attach rejected"):
            sdk.attach_ovstage(stage)
        del stage
        assert owner() is None
        assert not sdk._lib.pending
        return

    receipt = sdk.attach_ovstage(stage)
    del stage
    assert isinstance(receipt, Task)
    assert sdk._lib.waits == []
    assert owner() is not None
    # A pending attach can still report false; the Stage must stay alive.
    assert not sdk.is_stage_attached()
    if failure == "rollback":
        # Failed attachment rollback can leave ownership published.
        sdk._attachment_identity = 44
        with pytest.raises(TaskError, match="attach failed"):
            sdk.wait_task(receipt)
        assert owner() is sdk._attached_ovstage
        sdk.wait_task(sdk.detach_ovstage())
        assert owner() is None
    elif failure == "execute":
        with pytest.raises(TaskError, match="attach failed"):
            sdk.wait_task(receipt)
        assert owner() is None
    else:
        sdk.wait_task(receipt)
        assert sdk.is_stage_attached()
        assert owner() is sdk._attached_ovstage
        sdk.wait_task(sdk.detach_ovstage())
        assert owner() is None


@pytest.mark.parametrize("operation", ["update", "detach"])
@pytest.mark.parametrize("failure", [None, "reject", "execute"])
def test_lifecycle_receipts_separate_admission_from_execution(sdk, operation, failure):
    stage = sdk._attached_ovstage
    if failure:
        sdk._lib.failures[operation] = [failure]

    def submit():
        return sdk.update_from_ovstage(2, 2) if operation == "update" else sdk.detach_ovstage()

    if failure == "reject":
        with pytest.raises(RuntimeError, match=operation + " rejected"):
            submit()
        assert not sdk._lib.pending
        assert sdk._attached_ovstage is stage
        return

    receipt = submit()
    assert isinstance(receipt, Task)
    assert receipt.status == ApiStatus.SUCCESS
    assert sdk._lib.waits == []
    assert sdk._attached_ovstage is stage
    if failure == "execute":
        with pytest.raises(TaskError, match=operation + " failed"):
            sdk.wait_task(receipt)
        assert sdk._attached_ovstage is stage
    else:
        sdk.wait_task(receipt)
        assert sdk._attached_ovstage is (None if operation == "detach" else stage)


def test_dropped_detach_receipt_retains_stage_through_timeout(sdk):
    class Stage:
        pass

    stage = Stage()
    owner = weakref.ref(stage)
    sdk._attached_ovstage = stage
    sdk._lib.blocked = sdk.detach_ovstage().op_index
    del stage
    with pytest.raises(TaskTimeoutError):
        sdk.wait_all(timeout_ns=0)
    assert owner() is sdk._attached_ovstage
    sdk._lib.blocked = None
    sdk.wait_all()
    assert owner() is None


def test_older_detach_collection_does_not_drop_a_later_pending_stage_owner(sdk):
    stage = sdk._attached_ovstage
    older = sdk.detach_ovstage()
    newer = Task._from_native(sdk._lib.submit("attach", sdk._lib.on_attach))
    sdk._retain_stage_until_collection(newer)
    sdk.wait_task(older)
    assert not sdk.is_stage_attached()
    assert sdk._attached_ovstage is stage
    sdk.wait_task(newer)
    assert sdk.is_stage_attached()
    assert sdk._attached_ovstage is stage


def test_query_dictionary_accessor_is_not_public():
    assert not hasattr(PhysX, "query_shared_dictionary")
    assert not hasattr(b._lib, "ovphysx_query_shared_dictionary")


def test_default_query_does_not_wait_or_expose_uncollected_output(sdk):
    operation = sdk.query(SimObjectType.RIGID_BODY)
    assert sdk._lib.waits == []
    with pytest.raises(RuntimeError, match="not collected"):
        _ = operation.result
    assert operation.wait().handle == 7
    assert operation.result is operation.result
    assert len(sdk._lib.waits) == 1
    assert sdk._lib.summary_requests == [("query", True)]


@pytest.mark.parametrize("family", ["query", "read", "write"])
def test_native_same_stage_reattach_invalidates_completed_sessions(sdk, family):
    selector = query(sdk)
    session = selector if family == "query" else getattr(sdk, family)(selector, ["position"]).wait()
    stage = sdk._attached_ovstage
    calls, waits = list(sdk._lib.calls), list(sdk._lib.waits)

    sdk._attachment_identity += 1  # Native detach/reattach bypasses the Python wrapper.

    assert sdk._attached_ovstage is stage
    with pytest.raises(RuntimeError, match="invalidated attachment"):
        session._validate()
    assert sdk._lib.calls == calls and sdk._lib.waits == waits
    assert not hasattr(sdk, "get_attach_handle")


@pytest.mark.parametrize("failure", [None, "execute"])
def test_attachment_query_release_collects_its_outcome_once(sdk, failure):
    if failure:
        sdk._lib.failures["release_query"] = [failure]
        with pytest.raises(TaskError, match="release_query failed"):
            sdk._release_attachment_query(7)
    else:
        sdk._release_attachment_query(7)
    assert sdk._lib.calls == ["release_query"]
    assert sdk._lib.waits == ["release_query"]
    assert sdk._lib.pending == {}
    assert sdk._pending_write_releases == []


@pytest.mark.parametrize("metadata", [False, True])
def test_contact_query_collects_output_and_optional_metadata(sdk, metadata):
    operation = sdk.query(
        SimObjectType.CONTACT_SENSOR,
        contacts=[ContactSelection("/World/Sensor*", [[101], [102]])],
        metadata=metadata,
    )
    assert sdk._lib.waits == []
    with pytest.raises(RuntimeError, match="not collected"):
        _ = operation.result
    gc.collect()
    selector = operation.wait()
    assert selector.handle == 7
    assert selector.object_type == SimObjectType.CONTACT_SENSOR
    assert sdk._lib.contact_selections == [(b"/World/Sensor*", [101, 102])]
    assert sdk._lib.summary_requests == [("query_contacts", metadata)]
    if metadata:
        assert selector.metadata.total_prim_count == 1
        assert selector.metadata.attributes == (40, 41)
    else:
        assert selector.metadata is None
    selector.close_sync()
    assert sdk._lib.pending == {}


@pytest.mark.parametrize("method,attributes", [("read_sync", ["netNormalContactForce"]), ("read_tokens_sync", [40])])
def test_contact_read_sync_owns_and_collects_its_contact_query(sdk, method, attributes):
    with getattr(sdk, method)(
        SimObjectType.CONTACT_SENSOR, attributes,
        contacts=[ContactSelection("/World/Sensor*", [[101]])],
    ) as result:
        assert result.groups == []
        assert sdk._lib.waits == ["query_contacts", "read", "fetch"]
        assert sdk._lib.summary_requests == [("query_contacts", False), ("read", False)]
    assert sdk._lib.pending == {}
    assert sdk._lib.waits[-2:] == ["release_read", "release_query"]


@pytest.mark.parametrize("family", ["read", "write"])
def test_default_session_open_neither_fetches_nor_collects(family, sdk):
    selector = query(sdk)
    operation = getattr(sdk, family)(selector, ["position", "linearVelocity"])
    assert sdk._lib.calls == [family]
    assert sdk._lib.waits == []
    session = operation.wait()
    assert session.groups == []
    assert session.metadata is None
    assert session._release._query == 0
    assert selector.handle == 7
    assert sdk._lib.summary_requests == [(family, False)]


def test_wait_all_records_without_materializing_payload(sdk):
    converted = []
    receipt = sdk._lib.submit("work")
    operation = Operation(sdk, (Task._from_native(receipt),), convert=lambda: converted.append("now") or 42)
    sdk.wait_all()
    assert operation.done
    assert converted == []
    assert operation.result == operation.wait() == 42
    assert converted == ["now"]


def test_single_task_wait_does_not_collect_other_task(sdk):
    first = Operation(sdk, (Task._from_native(sdk._lib.submit("first")),))
    second = Operation(sdk, (Task._from_native(sdk._lib.submit("second")),))
    second.wait()
    assert second.done and not first.done
    assert sdk._lib.waits == ["second"]
    first.wait()


def test_cached_failure_survives_wait_result_destruction_and_repeats(sdk):
    sdk._lib.failures["work"] = ["execute"]
    operation = Operation(sdk, (Task._from_native(sdk._lib.submit("work")),))
    with pytest.raises(TaskError, match="work failed"):
        sdk.wait_all()
    assert sdk._lib.collected_errors == {}
    for read in (lambda: operation.result, operation.wait):
        with pytest.raises(TaskError, match="work failed"):
            read()
    assert sdk._lib.waits == ["work"]


def test_cached_conversion_failure_does_not_retain_caller_frames(sdk):
    references = []
    depths = []

    class Borrow:
        pass

    def fail_conversion():
        unexposed = Borrow()
        references.append(weakref.ref(unexposed))
        raise ValueError("conversion failed") from RuntimeError("conversion cause")

    operation = Operation(sdk, convert=fail_conversion)

    def access():
        caller_borrow = Borrow()
        references.append(weakref.ref(caller_borrow))
        try:
            _ = operation.result
        except ValueError as error:
            assert str(error) == "conversion failed"
            assert str(error.__cause__) == "conversion cause"
            depth = 0
            traceback = error.__traceback__
            while traceback is not None:
                depth += 1
                traceback = traceback.tb_next
            depths.append(depth)

    for _ in range(4):
        access()
        gc.collect()
        assert all(reference() is None for reference in references)
    assert len(set(depths[1:])) == 1


@pytest.mark.parametrize("wait_kind", ["wait_task", "wait_all", "operation"])
@pytest.mark.parametrize("failure", ["execute", "timeout"])
def test_wait_keeps_primary_error_and_all_rejected_cleanup_causes(sdk, wait_kind, failure):
    if failure == "execute":
        sdk._lib.failures["step"] = ["execute"]
    operation = Operation(sdk, (Task._from_native(sdk._lib.submit("step")),))
    if failure == "timeout":
        sdk._lib.blocked = operation.task.op_index
    attempts = []

    def cleanup(name):
        attempts.append(name)
        if attempts.count(name) == 1:
            raise RuntimeError(name + " rejected")

    pending = [lambda: cleanup("query release"), lambda: cleanup("read release")]
    native_name = "ovphysx_" + ("wait_task" if wait_kind == "operation" else wait_kind)
    native_wait = getattr(sdk._lib, native_name)

    def wait_then_queue_cleanup(*args):
        result = native_wait(*args)
        sdk._session_cleanups.extend(pending)
        return result

    setattr(sdk._lib, native_name, wait_then_queue_cleanup)
    expected = TaskError if failure == "execute" else TaskTimeoutError
    with pytest.raises(expected) as caught:
        if wait_kind == "operation":
            operation.wait(timeout_ns=0)
        elif wait_kind == "wait_task":
            sdk.wait_task(operation.task, timeout_ns=0)
        else:
            sdk.wait_all(timeout_ns=0)
    assert attempts == ["query release", "read release"]
    messages = []
    error = caught.value.__cause__
    while error is not None:
        messages.append(str(error))
        error = error.__cause__
    assert "query release rejected" in messages
    assert "read release rejected" in messages
    assert sdk._session_cleanups == pending
    sdk._drain_pending_read_releases()
    assert attempts == ["query release", "read release"] * 2
    assert not sdk._session_cleanups


def test_rejected_orphan_cleanup_does_not_skip_read_and_write_holders(sdk):
    from ovphysx.api import _ReadRelease, _WriteRelease

    Operation(sdk)
    attempts = []

    def orphan():
        attempts.append("orphan")
        if len(attempts) == 1:
            raise RuntimeError("orphan release rejected")

    sdk._session_cleanups.append(orphan)
    read = _ReadRelease(sdk, 0, 9)
    write = _WriteRelease(sdk, 0, 10)
    sdk._enqueue_read_release(read)
    sdk._enqueue_write_release(write)
    sdk._lib.failures["release_read"] = ["reject"]
    sdk._lib.failures["release_write"] = ["reject"]
    with pytest.raises(RuntimeError) as caught:
        sdk._drain_pending_read_releases()
    messages = []
    error = caught.value
    while error is not None:
        messages.append(str(error))
        error = error.__cause__
    assert any("orphan release rejected" in message for message in messages)
    assert any("release_read rejected" in message for message in messages)
    assert any("release_write rejected" in message for message in messages)
    assert sdk._pending_read_releases == [read]
    assert sdk._pending_write_releases == [write]
    assert sdk._session_cleanups == [orphan]
    sdk._drain_pending_read_releases()
    assert not read.pending and not write.pending
    assert not sdk._session_cleanups
    sdk.wait_all()


@pytest.mark.parametrize("wait_kind", ["wait_task", "wait_all", "operation"])
def test_successful_wait_inside_except_does_not_capture_callers_error(sdk, wait_kind):
    operation = Operation(sdk, (Task._from_native(sdk._lib.submit("work")),))
    native_name = "ovphysx_wait_all" if wait_kind == "wait_all" else "ovphysx_wait_task"
    native_wait = getattr(sdk._lib, native_name)

    def rejected():
        raise RuntimeError("cleanup rejected")

    def wait_then_queue(*args):
        result = native_wait(*args)
        sdk._session_cleanups.append(rejected)
        return result

    setattr(sdk._lib, native_name, wait_then_queue)
    outer = KeyError("unrelated caller error")
    try:
        raise outer
    except KeyError:
        with pytest.raises(RuntimeError, match="cleanup rejected") as caught:
            if wait_kind == "operation":
                operation.wait()
            elif wait_kind == "wait_task":
                sdk.wait_task(operation.task)
            else:
                sdk.wait_all()
    assert caught.value is not outer
    assert outer.__cause__ is None
    assert operation.done
    sdk._session_cleanups.clear()


def test_cleanup_chaining_terminates_and_preserves_distinct_errors_in_a_cycle():
    from ovphysx.api import _run_cleanups

    primary = ValueError("body")
    primary.__cause__ = primary
    rejected = RuntimeError("cleanup rejected")
    rejected.__cause__ = rejected

    def cleanup():
        raise rejected

    with pytest.raises(ValueError, match="body") as caught:
        _run_cleanups((cleanup,), primary=primary)
    assert caught.value.__cause__ is rejected
    assert rejected.__cause__ is None


def test_wait_all_collects_even_if_queued_cleanup_keeps_rejecting(sdk):
    operation = Operation(sdk, (Task._from_native(sdk._lib.submit("work")),))

    def rejected():
        raise RuntimeError("cleanup rejected")

    sdk._session_cleanups.append(rejected)
    with pytest.raises(RuntimeError, match="cleanup rejected"):
        sdk.wait_all()
    assert operation.done and operation.result is None
    assert sdk._session_cleanups == [rejected]
    sdk._session_cleanups.clear()


def test_detach_leaves_earlier_failure_for_explicit_collection(sdk):
    sdk._lib.failures["step"] = ["execute"]
    receipt = Task._from_native(sdk._lib.submit("step"))
    detach = sdk.detach_ovstage()
    assert sdk._attached_ovstage is not None
    assert sdk._lib.waits == []
    sdk.wait_task(detach)
    assert sdk._attached_ovstage is None
    assert sdk._get_attachment_identity() == 0
    with pytest.raises(TaskError, match="step failed"):
        sdk.wait_task(receipt)


@pytest.mark.parametrize("teardown", ["close", "detach"])
def test_read_fetch_rejects_released_descriptor_after_failed_teardown(sdk, teardown):
    session = sdk.read(query(sdk), ["position"]).wait()
    operations = []
    for group_id in (61, 62):
        group = b.ovstage_read_group_t()
        group.read_group_id = group_id
        sdk._lib.groups.append(group)
        operation = session.fetch_next()
        sdk.wait_task(operation.task)
        operations.append(operation)
    sdk._materialize_read_group = lambda *args: pytest.fail("dereferenced a retired descriptor")
    if teardown == "close":
        sdk._lib.failures["release_group"] = [None, "reject"]
        with pytest.raises(RuntimeError, match="rejected"):
            session.close()
        assert not session._closed
    else:
        sdk._lib.failures["detach"] = ["reject"]
        with pytest.raises(RuntimeError, match="detach rejected"):
            sdk.detach_ovstage()
    assert sdk._get_attachment_identity() == 44
    with pytest.raises(RuntimeError, match="released before materialization"):
        _ = operations[0].result


@pytest.mark.parametrize("output_kind", ["open", "fetch"])
def test_write_output_rejects_retired_storage_after_failed_teardown(sdk, output_kind):
    if output_kind == "open":
        native_write = sdk._lib.ovphysx_write

        def write_with_invalidated_summary(*args):
            receipt = native_write(*args)
            summary = args[-1]._obj
            # An unavailable row would raise on access; teardown must reject first.
            summary.attribute_count = 1
            return receipt

        sdk._lib.ovphysx_write = write_with_invalidated_summary
        operation = sdk.write(query(sdk), ["position"], metadata=True)
    else:
        session = sdk.write(query(sdk), ["position"]).wait()
        native = b.ovstage_map_group_t()
        sdk._lib.write_groups.append((native, 0))
        operation = session.fetch_next()
    sdk.wait_task(operation.task)
    sdk._lib.failures["detach"] = ["execute"]
    receipt = sdk.detach_ovstage()
    with pytest.raises(TaskError, match="detach failed"):
        sdk.wait_task(receipt)
    assert sdk._get_attachment_identity() == 44
    assert sdk._attached_ovstage is not None
    with pytest.raises(RuntimeError, match="retired by attachment teardown"):
        _ = operation.result


def test_missing_stage_owner_never_constructs_a_private_dictionary(sdk, monkeypatch):
    monkeypatch.setitem(sys.modules, "ovstage", SimpleNamespace(
        PathDictionary=lambda stage: pytest.fail("constructed a private dictionary")))
    selector = Query(sdk, 7, SimObjectType.RIGID_BODY, None, None, 44)
    read = ReadResult(sdk, 0, 9)
    for item in (selector, read):
        with pytest.raises(RuntimeError, match="no owning ovstage.Stage"):
            _ = item.dictionary
    write = WriteSession(sdk, 0, 10, attributes=["position"])
    sdk._lib.write_groups.append((b.ovstage_map_group_t(), 0))
    with pytest.raises(RuntimeError, match="no owning ovstage.Stage"):
        write.fetch_next().wait()


def test_cached_conversion_notes_are_not_mutated_by_an_earlier_handler(sdk):
    def convert():
        error = ValueError("conversion failed")
        error.add_note("original note")
        raise error

    operation = Operation(sdk, convert=convert)
    for _ in range(3):
        with pytest.raises(ValueError) as caught:
            _ = operation.result
        assert caught.value.__notes__ == ["original note"]
        caught.value.add_note("handler note")


def test_wait_all_timeout_caches_only_completed_prefix(sdk):
    sdk._lib.failures["first"] = ["execute"]
    operations = [Operation(sdk, (Task._from_native(sdk._lib.submit(name)),)) for name in ("first", "second", "third")]
    sdk._lib.blocked = operations[1].task.op_index
    with pytest.raises(TaskTimeoutError) as caught:
        sdk.wait_all(timeout_ns=0)
    assert caught.value.errors[0][0] == operations[0].task
    assert [operation.done for operation in operations] == [True, False, False]
    sdk._lib.blocked = None
    sdk.wait_all()
    assert all(operation.done for operation in operations)
    with pytest.raises(TaskError):
        operations[0].wait()


def test_zero_one_and_many_real_receipts(sdk):
    local = Operation(sdk)
    assert local.tasks == () and local.done and local.wait() is None
    with pytest.raises(ValueError):
        _ = local.task
    tasks = tuple(Task._from_native(sdk._lib.submit(name)) for name in ("a", "b"))
    operation = Operation(sdk, tasks)
    assert operation.tasks == tasks
    with pytest.raises(ValueError):
        _ = operation.task
    operation.wait()
    assert sdk._lib.waits == ["a", "b"]


def test_dropped_creator_releases_orphan_without_materialization(sdk):
    operation = sdk.read(query(sdk), ["position"])
    reference = weakref.ref(operation)
    del operation
    gc.collect()
    assert reference() is None
    assert sdk._session_pending
    sdk.wait_all()
    assert "release_read" in sdk._lib.calls
    assert sdk._lib.waits == ["read", "release_query"]
    assert not sdk._session_pending
    sdk.wait_all()
    assert "release_read" in sdk._lib.waits


def test_read_fetch_materializes_once_at_first_result_access(sdk):
    session = sdk.read(query(sdk), ["position"]).wait()
    native = b.ovstage_read_group_t()
    native.read_group_id = 91
    sdk._lib.groups.append(native)
    current = ["stream at wait"]
    converted = []
    sdk._materialize_read_group = lambda group, kind, holder: converted.append(current[0]) or object()
    operation = session.fetch_next()
    sdk.wait_all()
    assert converted == []
    current[0] = "stream at materialization"
    assert operation.result is operation.wait()
    assert converted == ["stream at materialization"]
    assert session._release._group_ids == [91]


def test_fetch_materialization_after_parent_close_fails_and_retires_group(sdk):
    session = sdk.read(query(sdk), ["position"]).wait()
    native = b.ovstage_read_group_t()
    native.read_group_id = 93
    sdk._lib.groups.append(native)
    operation = session.fetch_next()
    session.close()
    assert "release_read" not in sdk._lib.calls
    with pytest.raises(RuntimeError, match="closed"):
        operation.wait()
    assert "release_group" in sdk._lib.calls
    sdk._drain_pending_read_releases()
    assert "release_read" in sdk._lib.calls


def test_close_rejected_group_retains_owner_for_retry(sdk):
    session = ReadResult(sdk, 0, 9, group_ids=[3])
    sdk._lib.failures["release_group"] = ["reject"]
    with pytest.raises(RuntimeError, match="rejected"):
        session.close()
    assert not session._closed and session._release._refs == 1
    operation = session.close()
    assert session._closed
    assert session._release._refs == 0
    assert sdk._lib.waits == []
    assert operation.task.op_index > 0


def test_deferred_cleanup_drain_submits_without_waiting(sdk):
    session = ReadResult(sdk, 0, 9, group_ids=[5])
    session.close()
    sdk._drain_pending_read_releases()
    assert sdk._lib.waits == []
    assert sdk._lib.calls == ["release_group", "release_read"]
    sdk.wait_all()
    assert sdk._lib.waits == sdk._lib.calls


@pytest.mark.parametrize("family", ["query", "write"])
def test_deferred_write_release_failure_survives_dropped_receipt(family, sdk):
    session = query(sdk) if family == "query" else WriteSession(sdk, 0, 10)
    release_name = "release_" + family
    sdk._lib.failures[release_name] = ["execute"]
    reference = weakref.ref(session)
    del session
    gc.collect()
    assert reference() is None
    assert sdk._lib.calls == []
    with pytest.raises(TaskError, match=release_name + " failed"):
        sdk.wait_all()
    assert sdk._lib.calls == [release_name]
    assert sdk._lib.waits == [release_name]
    assert not sdk._lib.pending
    sdk.wait_all()
    assert sdk._lib.calls == [release_name]


def test_write_commit_spends_exact_group_on_admission_even_if_execution_fails(sdk):
    group = WriteGroup(3, 0, 1, [], 1, 42)
    native = ctypes.pointer(b.ovstage_map_group_t())
    session = WriteSession(sdk, 0, 10, [group], [native])
    sdk._lib.failures["commit"] = ["execute"]
    operation = session.commit(group)
    assert sdk._lib.commit_pointer is native
    assert sdk._lib.waits == []
    with pytest.raises(RuntimeError, match="already been committed"):
        session.commit(group)
    with pytest.raises(TaskError):
        operation.wait()


def test_write_release_has_only_real_receipts_and_no_wait(sdk):
    session = WriteSession(sdk, 7, 10)
    operation = session.close()
    assert len(operation.tasks) == 2
    assert session._closed
    assert sdk._lib.waits == []
    operation.wait()
    assert sdk._lib.waits == ["release_write", "release_query"]


def test_operation_keeps_inputs_until_collection(sdk):
    array = (ctypes.c_uint64 * 2)(1, 2)
    reference = weakref.ref(array)
    operation = Operation(sdk, (Task._from_native(sdk._lib.submit("input")),), keepalive=(array,))
    del array
    gc.collect()
    assert reference() is not None
    sdk.wait_all()
    gc.collect()
    assert reference() is None
    assert operation.done


def test_dropped_multi_task_retains_inputs_until_all_outcomes_collected(sdk):
    array = (ctypes.c_uint64 * 1)(1)
    reference = weakref.ref(array)
    tasks = tuple(Task._from_native(sdk._lib.submit(name)) for name in ("one", "two"))
    cleaned = []
    operation = Operation(sdk, tasks, cleanup=lambda: cleaned.append(True), keepalive=(array,))
    del array, operation
    gc.collect()
    sdk.wait_task(tasks[0])
    assert reference() is not None and cleaned == []
    sdk.wait_task(tasks[1])
    gc.collect()
    assert reference() is None and cleaned == [True]


def test_sync_close_collects_own_cleanup_without_consuming_unrelated_failure(sdk):
    sdk._lib.failures["unrelated"] = ["execute"]
    unrelated = Task._from_native(sdk._lib.submit("unrelated"))
    session = ReadResult(sdk, 0, 9, group_ids=[5])
    session.close_sync()
    assert unrelated.op_index in sdk._lib.pending
    assert sdk._lib.waits == ["release_group", "release_read"]
    with pytest.raises(TaskError, match="unrelated"):
        sdk.wait_task(unrelated)


@pytest.mark.parametrize("family", ["read", "write"])
def test_sync_convenience_omits_internal_query_summary(family, sdk):
    session = getattr(sdk, family + "_sync")(SimObjectType.RIGID_BODY, ["position"])
    assert sdk._lib.summary_requests == [("query", False), (family, False)]
    assert session._release._query == 7
    assert session.groups == []
    session.close_sync()


def test_opt_in_metadata_is_copied_immutable_snapshot(sdk):
    from dataclasses import FrozenInstanceError
    from ovphysx.api import ReadOmission
    session = sdk.read(query(sdk), ["position", "unknown"], metadata=True).wait()
    assert session.metadata.total_prim_count == 3
    assert session.metadata.attributes[0].attribute == 40
    assert session.metadata.attributes[1].omission_reasons == ReadOmission.UNSUPPORTED_ATTRIBUTE
    with pytest.raises(FrozenInstanceError):
        session.metadata.total_prim_count = 99
    rows = next(value for value in sdk._lib.storage if value._type_ is b.ovphysx_read_attribute_result_t)
    rows[0].attribute = 81
    assert session.metadata.attributes[0].attribute == 40


@pytest.mark.parametrize("family", ["read", "write"])
def test_partial_release_admission_preserves_prior_execution_failure(family, sdk):
    if family == "read":
        session = ReadResult(sdk, 0, 9, group_ids=[1, 2])
        sdk._lib.failures["release_group"] = ["execute", "reject"]
    else:
        session = WriteSession(sdk, 7, 10)
        sdk._lib.failures["release_write"] = ["execute"]
        sdk._lib.failures["release_query"] = ["reject"]
    with pytest.raises(RuntimeError, match="rejected"):
        session.close()
    with pytest.raises(TaskError, match="failed"):
        session.close_sync()
    assert not sdk._lib.pending


def test_materialization_error_requeues_rejected_cleanup(sdk):
    session = sdk.read(query(sdk), ["position"]).wait()
    native = b.ovstage_read_group_t()
    native.read_group_id = 99
    sdk._lib.groups.append(native)
    operation = session.fetch_next()
    session.close()
    sdk._lib.failures["release_group"] = ["reject"]
    with pytest.raises(RuntimeError, match="closed"):
        operation.wait()
    assert sdk._session_cleanups and session._release._refs == 1
    sdk._drain_pending_read_releases()
    assert not sdk._session_cleanups and session._release._refs == 0
    sdk.wait_all()


def test_fetch_cleanup_failure_survives_dropped_group_receipt(sdk):
    session = ReadResult(sdk, 0, 9)
    native = b.ovstage_read_group_t()
    native.read_group_id = 99
    sdk._lib.groups.append(native)
    operation = session.fetch_next()
    session.close()
    sdk._lib.failures["release_group"] = ["execute"]
    with pytest.raises(RuntimeError, match="closed"):
        operation.wait()
    assert session._release._group_ids == []
    with pytest.raises(TaskError, match="release_group failed"):
        sdk.wait_all()
    assert not sdk._lib.pending
    sdk.wait_all()
    assert sdk._lib.calls.count("release_group") == 1


def test_destroy_reports_last_stage_group_release_failure(sdk):
    session = ReadResult(sdk, 0, 9, group_ids=[77])
    sdk._lib.failures["release_group"] = ["execute"]
    with pytest.raises(TaskError, match="release_group failed"):
        sdk.destroy()
    assert sdk._omni_physx_sdk_handle is None
    assert sdk._lib.calls[-1] == "destroy"
    assert session._release._group_ids == []


def test_default_detach_does_not_collect_and_invalidates_pending_payload(sdk):
    operation = sdk.read(query(sdk), ["position"])
    detach = sdk.detach_ovstage()
    assert sdk._lib.waits == []
    assert not operation.done and sdk._session_pending
    sdk.wait_all()
    assert detach.op_index not in sdk._lib.pending
    with pytest.raises(RuntimeError, match="invalidated attachment"):
        _ = operation.result
    sdk.wait_all()


def test_conversion_error_does_not_keep_unexposed_array_in_cached_traceback(sdk, monkeypatch):
    from ovphysx import api
    monkeypatch.setitem(sys.modules, "warp", SimpleNamespace())
    monkeypatch.setattr(api, "_warn_if_warp_build_breaks_cpu_mode", lambda wp: None)
    sdk._order_warp_stream_after_group = lambda *args: None
    borrows = []
    class Borrow:
        def __init__(self):
            borrows.append(True)
        def __del__(self):
            borrows.pop()
    sdk._dltensor_to_warp_array = lambda *args: Borrow()
    def bad_map(*args):
        raise ValueError("index map conversion failed")
    sdk._u32_warp_array = bad_map
    native = b.ovstage_read_group_t()
    native.read_group_id = 108
    tensors = (b.DLTensor * 1)()
    native.data.tensors = tensors
    native.data.tensor_count = 1
    sdk._lib.groups.append(native)
    session = sdk.read(query(sdk), ["position"]).wait()
    operation = session.fetch_next()
    with pytest.raises(ValueError, match="index map"):
        operation.wait()
    assert not borrows
    with pytest.raises(ValueError, match="index map"):
        _ = operation.result
    assert not borrows


def test_read_close_drain_rejection_preserves_group_receipt(sdk):
    session = ReadResult(sdk, 0, 9, group_ids=[71])
    sdk._lib.failures["release_group"] = ["execute"]
    sdk._lib.failures["release_read"] = ["reject"]
    with pytest.raises(RuntimeError, match="release read|release_read"):
        session.close()
    assert session._closed
    with pytest.raises(TaskError, match="release_group failed"):
        session.close_sync()
    assert not sdk._lib.pending


@pytest.mark.parametrize("family", ["query", "read", "write", "read_fetch", "write_fetch", "commit"])
def test_discarded_pending_operations_do_not_create_an_sdk_gc_cycle(family):
    sdk = make_sdk()
    owner = weakref.ref(sdk)
    if family == "query":
        operation = sdk.query(SimObjectType.RIGID_BODY)
    elif family in ("read", "write"):
        operation = getattr(sdk, family)(query(sdk), ["position"])
    elif family == "commit":
        group = WriteGroup(0, 0, 0, [], 0, 40)
        session = WriteSession(sdk, 0, 10, groups=[group], native=[ctypes.pointer(b.ovstage_map_group_t())])
        operation = session.commit(group)
        del session
    else:
        session = getattr(sdk, family.split("_")[0])(query(sdk), ["position"]).wait()
        operation = session.fetch_next()
        del session
    del sdk
    assert owner() is not None, "a live public Operation retains its SDK"
    del operation
    assert owner() is None, "discarded operations must not require cyclic garbage collection"


def test_pending_read_orphan_gc_only_enqueues_until_owner_collection(sdk):
    session = sdk.read(query(sdk), ["position"]).wait()
    group = b.ovstage_read_group_t()
    group.read_group_id = 201
    sdk._lib.groups.append(group)
    operation = session.fetch_next()
    sdk.wait_task(operation.task)
    cycle = [operation, session]
    cycle.append(cycle)
    del operation, session, cycle
    calls_before = list(sdk._lib.calls)
    collector = threading.Thread(target=gc.collect)
    collector.start()
    collector.join()
    assert sdk._lib.calls == calls_before
    sdk.wait_all()
    assert "release_group" in sdk._lib.calls
    assert "release_read" in sdk._lib.calls
    sdk.wait_all()
    assert not sdk._lib.pending


@pytest.mark.parametrize("family", ["read", "write"])
def test_rejected_queued_release_does_not_create_an_sdk_gc_cycle(family):
    sdk = make_sdk()
    owner = weakref.ref(sdk)
    if family == "read":
        session = ReadResult(sdk, 7, 9, group_ids=[1])
        sdk._lib.failures["release_read"] = ["reject"]
    else:
        session = WriteSession(sdk, 7, 10)
        sdk._lib.failures["release_query"] = ["reject"]
    try:
        session.close()
    except RuntimeError:
        pass
    del session, sdk
    assert owner() is None


def test_array_borrow_retains_sdk_without_a_queue_owned_strong_owner(monkeypatch):
    from ovphysx import api

    monkeypatch.setattr(api, "_acquire_process_lifecycle", lambda: None)
    monkeypatch.setattr(api, "_defer_process_lifecycle_release", lambda: None)
    sdk = make_sdk()
    owner = weakref.ref(sdk)
    holder = api._ReadRelease(sdk, 0, 9)
    array = PhysX._retain_warp_array(SimpleNamespace(array=lambda **kwargs: SimpleNamespace(**kwargs)),
                                   ptr=1, dtype=None, shape=(1,), device="cpu", on_release=holder)
    holder.release()
    del sdk
    assert owner() is not None
    array.deleter()
    del array
    assert owner() is None


def test_rejected_detach_does_not_retire_live_write_session(sdk):
    session = sdk.write(query(sdk), ["position"]).wait()
    sdk._lib.failures["detach"] = ["reject"]
    with pytest.raises(RuntimeError, match="detach rejected"):
        sdk.detach_ovstage()
    session._validate()


def test_detach_callback_cannot_materialize_retired_write_buffers(sdk):
    session = sdk.write(query(sdk), ["position"]).wait()
    sdk._lib.write_groups.append((b.ovstage_map_group_t(), 0))
    operation = session.fetch_next()
    sdk.wait_task(operation.task)
    native_detach = sdk._lib.ovphysx_detach_ovstage
    callbacks = []

    def detach_with_object_destroyed_callback(instance):
        with pytest.raises(RuntimeError, match="retired by attachment teardown"):
            _ = operation.result
        callbacks.append(True)
        return native_detach(instance)

    sdk._lib.ovphysx_detach_ovstage = detach_with_object_destroyed_callback
    sdk.wait_task(sdk.detach_ovstage())
    assert callbacks == [True]


def test_boolean_is_not_an_attribute_token(sdk):
    with pytest.raises(TypeError, match="nonzero uint64"):
        sdk.read(query(sdk), [True])


@pytest.mark.parametrize("kind", ["wait_task", "wait_all", "operation", "cleanup", "wait_and_cleanup", "destroy",
                                  "query_context", "read_context", "write_context"])
def test_discarded_failure_does_not_retain_sdk_until_cyclic_gc(kind):
    sdk = make_sdk()
    owner = weakref.ref(sdk)
    sdk._lib.failures["step"] = ["execute"]
    operation = Operation(sdk, (Task._from_native(sdk._lib.submit("step")),))
    caught = False
    if kind == "cleanup":
        session = WriteSession(sdk, 7, 10)
        sdk._lib.failures["release_query"] = ["reject"]
        try:
            session.close()
        except RuntimeError:
            caught = True
        del session
    elif kind.endswith("_context"):
        family = kind.split("_")[0]
        if family == "query":
            session = query(sdk)
        elif family == "read":
            session = ReadResult(sdk, 0, 9)
        else:
            session = WriteSession(sdk, 0, 10)
        sdk._lib.failures["release_" + family] = ["reject"]
        try:
            with session:
                raise ValueError("body failed")
        except ValueError:
            caught = True
        del session
    else:
        if kind == "wait_and_cleanup":
            def rejected():
                raise RuntimeError("orphan release rejected")
            sdk._session_cleanups.append(rejected)
        try:
            if kind in ("wait_task", "wait_and_cleanup"):
                sdk.wait_task(operation.task)
            elif kind == "wait_all":
                sdk.wait_all()
            elif kind == "destroy":
                sdk.destroy()
            else:
                operation.wait()
        except TaskError:
            caught = True
    assert caught
    del operation, sdk
    assert owner() is None, "discarding an error must not leave a traceback-local ownership cycle"
