# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-TASK-001
# @covers AC-6 AC-7
# @maps_to TEST-PYTHON-TASK-001

"""Waiting conveniences must collect outcomes before exposing native session outputs."""

import ctypes

import pytest

from ovphysx import ApiStatus, PhysX, Task, TaskError
from ovphysx.types import SimObjectType
from ovphysx import _bindings
from ovphysx.api import ReadResult, WriteGroup, WriteSession, _ReadRelease, _WriteRelease


from test_session_operations import Native, make_sdk


class _NativeSessions(Native):
    """Reuse the delayed native double and the real Python completion dispatcher."""

    @property
    def submitted(self):
        return self.calls

    @property
    def waited(self):
        return self.waits

    def _collect(self, index):
        result = super()._collect(index)
        if result is not None:
            self.collected_errors[index] = self.collected_errors[index].replace(b" failed", b" execution failed")
        return result

    def ovphysx_fetch_read_next(self, instance, read, output):
        return self.submit("fetch_read")

    def ovphysx_update_articulations_kinematic(self, *args):
        return self.submit("kinematic")


def _sdk():
    return make_sdk(_NativeSessions)


def _open(sdk, operation):
    attributes = "position" if operation == "write" else [1] if operation == "read_tokens" else ["position"]
    return getattr(sdk, operation + "_sync")(SimObjectType.RIGID_BODY, attributes)


@pytest.mark.parametrize("operation", ["read", "read_tokens", "write"])
def test_session_waits_before_using_outputs_and_collects_eof(operation):
    sdk = _sdk()
    native = "write" if operation == "write" else "read"
    result = _open(sdk, operation)
    assert result.groups == []
    assert sdk._lib.waited == ["query", native, "fetch_" + native]
    result.close_sync()
    assert sdk._lib.waited == ["query", native, "fetch_" + native, "release_" + native, "release_query"]
    assert sdk._lib.pending == {}


@pytest.mark.parametrize("operation,failed", [
    ("read", "query"), ("read", "read"), ("read", "fetch_read"),
    ("read_tokens", "read"), ("write", "query"), ("write", "write"), ("write", "fetch_write"),
])
def test_session_execution_failure_is_raised_and_hidden_cleanup_is_collected(operation, failed):
    sdk = _sdk()
    sdk._lib.failures[failed] = ["execute"]
    with pytest.raises(TaskError, match=failed + " execution failed"):
        _open(sdk, operation)
    assert sdk._lib.pending == {}
    assert sdk._lib.waited == sdk._lib.submitted


@pytest.mark.parametrize("operation", ["read", "write"])
def test_open_rejection_preserves_its_cause_when_query_cleanup_also_fails(operation):
    sdk = _sdk()
    sdk._lib.failures[operation] = ["reject"]
    sdk._lib.failures["release_query"] = ["execute"]
    with pytest.raises(RuntimeError, match=operation + " rejected") as caught:
        _open(sdk, operation)
    assert isinstance(caught.value.__cause__, TaskError)
    assert "release_query execution failed" in str(caught.value.__cause__)
    assert sdk._lib.pending == {}


@pytest.mark.parametrize("failure", [None, "execute", "reject"])
def test_kinematic_update_waits_and_returns_none(failure):
    sdk = _sdk()
    sdk._lib.failures["kinematic"] = [failure]
    if failure:
        with pytest.raises(RuntimeError, match="kinematic"):
            sdk.update_articulations_kinematic()
    else:
        assert sdk.update_articulations_kinematic() is None
    expected = ["query"]
    if failure != "reject":
        expected.append("kinematic")
    expected.append("release_query")
    assert sdk._lib.waited == expected
    assert sdk._lib.pending == {}


@pytest.mark.parametrize("kind", ["read", "write"])
def test_kinematic_update_drains_deferred_session_cleanup(kind):
    sdk = _sdk()
    holder = _ReadRelease(sdk, 7, 9) if kind == "read" else _WriteRelease(sdk, 7, 9)
    getattr(sdk, "_enqueue_" + kind + "_release")(holder)

    sdk.update_articulations_kinematic()

    assert sdk._lib.waited == ["query", "kinematic", "release_query"]
    assert len(sdk._lib.pending) == 2
    sdk.wait_all()
    assert sdk._lib.waited[-2:] == ["release_" + kind, "release_query"]
    assert sdk._lib.pending == {}


def test_commit_waits_for_execution_failure():
    sdk = _sdk()
    group = WriteGroup(0, 0, 0, [], 0, 1)
    session = WriteSession(sdk, 7, 9, [group], [ctypes.pointer(_bindings.ovstage_map_group_t())])
    sdk._lib.failures["commit"] = ["execute"]
    with pytest.raises(TaskError, match="commit execution failed"):
        session.commit_sync(group)
    session.close_sync()
    assert sdk._lib.waited == ["commit", "release_write", "release_query"]
    assert sdk._lib.pending == {}


@pytest.mark.parametrize("kind", ["read", "write", "query", "group"])
@pytest.mark.parametrize("failure", ["reject", "execute"])
def test_cleanup_retries_rejection_but_never_reissues_accepted_failure(kind, failure):
    sdk = _sdk()
    if kind == "write":
        holder = _WriteRelease(sdk, 0, 9)
        cleanup = holder._free_native
        owns_handle = lambda: bool(holder._write)
    else:
        holder = _ReadRelease(sdk, 7 if kind == "query" else 0, 9 if kind != "query" else 0)
        if kind == "group":
            holder.add_group(11)
            cleanup = holder.release_groups
            owns_handle = lambda: bool(holder._group_ids)
        else:
            cleanup = holder._free_native
            owns_handle = lambda: bool(holder._query if kind == "query" else holder._read)
    name = "release_" + kind
    sdk._lib.failures[name] = [failure]
    if failure == "reject":
        with pytest.raises(RuntimeError, match=kind):
            cleanup()
    else:
        cleanup()
    assert owns_handle() is (failure == "reject")
    cleanup()
    assert not owns_handle()
    assert sdk._lib.waited == []
    if failure == "execute":
        with pytest.raises(TaskError, match=name):
            sdk.wait_all()
    else:
        sdk.wait_all()
    # A rejection never created a task; terminal failure must not produce a second release task.
    assert sdk._lib.submitted.count(name) == (2 if failure == "reject" else 1)
    assert sdk._lib.waited.count(name) == 1
    assert sdk._lib.pending == {}


@pytest.mark.parametrize("kind", ["read", "write"])
def test_deferred_cleanup_keeps_rejected_handles_and_attempts_remaining_holders(kind):
    sdk = _sdk()
    holder_type = _ReadRelease if kind == "read" else _WriteRelease
    first = holder_type(sdk, 7, 9)
    second = holder_type(sdk, 8, 10)
    enqueue = getattr(sdk, "_enqueue_" + kind + "_release")
    enqueue(first)
    enqueue(second)
    sdk._lib.failures["release_" + kind] = ["reject"]
    drain = getattr(sdk, "_drain_pending_" + kind + "_releases")
    with pytest.raises(RuntimeError, match="rejected"):
        drain()
    assert first.pending
    assert not second.pending
    assert getattr(sdk, "_pending_" + kind + "_releases") == [first]
    drain()
    assert not first.pending
    assert sdk._lib.waited == []
    sdk.wait_all()
    assert sdk._lib.pending == {}


@pytest.mark.parametrize("kind", ["read", "write"])
def test_context_exit_preserves_body_exception_and_chains_cleanup_failure(kind):
    sdk = _sdk()
    result = ReadResult(sdk, 7, 9, [], []) if kind == "read" else WriteSession(sdk, 7, 9, [], [])
    result._sync = True
    sdk._lib.failures["release_" + kind] = ["execute"]
    with pytest.raises(ValueError, match="body failure") as caught:
        with result:
            raise ValueError("body failure")
    assert isinstance(caught.value.__cause__, TaskError)
    assert sdk._lib.pending == {}


def test_read_close_keeps_rejected_group_retryable_and_collects_other_groups():
    sdk = _sdk()
    result = ReadResult(sdk, 7, 9, [], [11, 12])
    sdk._lib.failures["release_group"] = ["reject", None]
    with pytest.raises(RuntimeError, match="rejected"):
        result.close_sync()
    assert not result._closed
    assert result._release._group_ids == [11]
    assert sdk._lib.waited == ["release_group"]
    result.close_sync()
    assert result._closed
    assert sdk._lib.waited == ["release_group", "release_group", "release_read", "release_query"]


def test_write_close_remains_retryable_after_rejected_release():
    sdk = _sdk()
    session = WriteSession(sdk, 7, 9, [], [])
    sdk._lib.failures["release_write"] = ["reject"]
    with pytest.raises(RuntimeError, match="rejected"):
        session.close_sync()
    assert not session._closed
    session.close_sync()
    assert session._closed
    assert sdk._lib.pending == {}


@pytest.mark.parametrize("name", [
    "query", "query_contacts", "read", "fetch_read_next", "release_group", "release_read", "release_query",
    "write", "fetch_write_next", "commit_group", "release_write", "update_articulations_kinematic",
    "evaluate_sdf_shapes",
])
def test_low_level_session_bindings_return_full_receipts(name):
    assert getattr(_bindings._lib, "ovphysx_" + name).restype is _bindings.ovphysx_enqueue_result_t


def test_context_exit_keeps_all_nested_cleanup_causes():
    sdk = _sdk()
    result = ReadResult(sdk, 7, 9, [], [11])
    result._sync = True
    for name in ("release_group", "release_read", "release_query"):
        sdk._lib.failures[name] = ["execute"]
    with pytest.raises(ValueError, match="body failure") as caught:
        with result:
            raise ValueError("body failure")
    causes = []
    error = caught.value.__cause__
    while error is not None:
        assert isinstance(error, TaskError)
        causes.extend(error.errors)
        error = error.__cause__
    # One TaskError can aggregate several receipts; each failed release must still appear exactly once.
    assert [message for _, message in sorted(causes, key=lambda cause: cause[0].op_index)] == [
        name + " execution failed" for name in ("release_group", "release_read", "release_query")
    ]
    assert sdk._lib.pending == {}
