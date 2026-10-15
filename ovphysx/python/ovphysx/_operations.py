# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-TASK-001
# @covers AC-6 AC-7 AC-9 AC-10

"""Typed Python results over the native, one-shot task collection contract."""

import copy
import threading
import time
import weakref
from typing import Generic, TypeVar

T = TypeVar("T")
_UNSET = object()


def _copy_error(error, copies=None):
    """Copy diagnostic values without retaining traceback frames or their arrays."""
    from .api import TaskError, TaskTimeoutError

    if error is None:
        return None
    copies = {} if copies is None else copies
    if id(error) in copies:
        return copies[id(error)]
    if isinstance(error, TaskError):
        result = TaskError(error.errors)
    elif isinstance(error, TaskTimeoutError):
        result = TaskTimeoutError(error.errors, error.pending_task)
    else:
        result = copy.copy(error)
    if hasattr(error, "__notes__"):
        result.__notes__ = list(error.__notes__)
    copies[id(error)] = result
    result.__traceback__ = None
    result.__cause__ = _copy_error(error.__cause__, copies)
    result.__context__ = _copy_error(error.__context__, copies)
    result.__suppress_context__ = error.__suppress_context__
    return result


class _Output:
    def __init__(self, cleanup, keepalive, on_complete):
        self.cleanup = cleanup
        self.keepalive = keepalive
        self.on_complete = on_complete
        self.operation = None
        self.collected = False
        self.remaining = set()
        self.failed = False


class Operation(Generic[T]):
    """A typed result of zero, one, or several real native tasks.

    ``wait()`` collects completion explicitly. ``result`` never waits and raises
    while an outcome is uncollected. Outcomes are cached after either operation
    waits or the owning PhysX instance's waits. Payload conversion runs once on
    first result access, on the submitting thread. In particular, read arrays use
    the Warp stream current at that access, not the stream used by ``wait_all``.

    Inputs and output slots survive until collection, even if the operation is
    dropped. Periodically call ``wait_all()`` for fire-and-forget submissions.
    Collecting Python-owned receipts directly through the C API is unsupported.
    """

    def __init__(self, sdk, tasks=(), *, convert=lambda: None, cleanup=None, keepalive=(), on_complete=None,
                 owners=()):
        self._sdk = sdk
        self._tasks = tuple(tasks)
        self._outcomes = {}
        self._value = _UNSET
        self._error = None
        self._thread = threading.get_ident()
        # Materializers may own a session or SDK. Only the live Operation keeps
        # them; the SDK registry owns raw slots and weak-owner cleanup callbacks.
        self._convert = convert
        self._owners = owners
        self._output = _Output(cleanup, keepalive, on_complete)
        self._output.operation = weakref.ref(self)
        self._output.remaining = {_key(task) for task in self.tasks}
        _ensure_registry(sdk)
        for task in self.tasks:
            key = _key(task)
            sdk._session_operations[key] = self
            if keepalive or cleanup is not None:
                sdk._session_pending[key] = self._output
        if not self.tasks:
            self._output.collected = True

    @property
    def tasks(self):
        """Immutable tuple of the operation's real native receipts."""
        return self._tasks

    @property
    def task(self):
        """The native receipt, available only for a single-task operation."""
        if len(self.tasks) != 1:
            raise ValueError("operation does not have exactly one native task; use tasks")
        return self.tasks[0]

    @property
    def done(self) -> bool:
        """Whether every native outcome has been collected; does not poll."""
        return len(self._outcomes) == len(self.tasks)

    @property
    def result(self) -> T:
        """Return the cached typed result without waiting."""
        from .api import TaskError, _chain_cleanup

        if threading.get_ident() != self._thread:
            raise RuntimeError("operation results must be materialized on the submitting thread")
        if not self.done:
            raise RuntimeError("operation outcome is not collected; call wait() or PhysX.wait_all()")
        errors = tuple((task, self._outcomes[_key(task)]) for task in self.tasks if self._outcomes[_key(task)])
        if errors:
            raise TaskError(errors)
        if self._error is not None:
            raise _copy_error(self._error)
        if self._value is _UNSET:
            try:
                self._value = self._convert()
            except Exception as error:
                cleanup, self._output.cleanup = self._output.cleanup, None
                try:
                    if cleanup is not None:
                        cleanup()
                except Exception as cleanup_error:
                    # Rejected admission leaves native ownership in the closure.
                    with self._sdk._session_cleanup_lock:
                        self._sdk._session_cleanups.append(cleanup)
                    _chain_cleanup(error, cleanup_error)
                self._error = _copy_error(error)
                raise
            finally:
                self._convert = None
                self._owners = ()
                self._output.keepalive = ()
            self._output.cleanup = None
        return self._value

    def wait(self, *, timeout_ns: int | None = None) -> T:
        """Collect this operation's tasks and return its typed result.

        A finite timeout is one readiness budget across all constituent tasks.
        Completed failures are cached, including when a later task times out.
        """
        from .api import TaskError

        if timeout_ns is not None and not 0 <= timeout_ns <= 0xFFFFFFFFFFFFFFFF:
            raise ValueError("timeout_ns must be in range 0 through 2**64 - 1")
        deadline = None if timeout_ns is None else time.monotonic_ns() + timeout_ns
        for task in self.tasks:
            if _key(task) in self._outcomes:
                continue
            remaining = None if deadline is None else max(0, deadline - time.monotonic_ns())
            try:
                self._sdk.wait_task(task, timeout_ns=remaining)
            except TaskError as error:
                # Dispatch already cached the failure. Collect the other real tasks too.
                # A chained cleanup failure must remain observable on this wait.
                if _key(task) not in self._outcomes or error.__cause__ is not None:
                    raise
        return self.result

    def _complete(self, task, error):
        self._outcomes[_key(task)] = error
        if self.done:
            self._output.collected = True
            self._output.keepalive = ()
            self._owners = ()
            if not any(self._outcomes.values()) and self._output.on_complete is not None:
                self._output.on_complete()
            self._output.on_complete = None
            if any(self._outcomes.values()) and self._output.cleanup is not None:
                cleanup, self._output.cleanup = self._output.cleanup, None
                with self._sdk._session_cleanup_lock:
                    self._sdk._session_cleanups.append(cleanup)

    def __del__(self):
        # A GC thread may only enqueue cleanup. Native calls run in the SDK drain.
        try:
            output = self._output
            if output.collected and output.cleanup is not None:
                cleanup, output.cleanup = output.cleanup, None
                with self._sdk._session_cleanup_lock:
                    self._sdk._session_cleanups.append(cleanup)
        except Exception:
            pass


def _key(task):
    return (int(task.instance), int(task.op_index))


def _ensure_registry(sdk):
    if not hasattr(sdk, "_session_operations"):
        sdk._session_operations = weakref.WeakValueDictionary()
        sdk._session_pending = {}
        sdk._session_cleanups = []
        sdk._session_cleanup_lock = threading.Lock()


def registered_tasks(sdk):
    _ensure_registry(sdk)
    return set(sdk._session_operations) | set(sdk._session_pending)


def dispatch(sdk, keys, errors):
    """Record copied native outcomes, never construct typed payloads here."""
    _ensure_registry(sdk)
    for key in keys:
        operation = sdk._session_operations.pop(key, None)
        output = sdk._session_pending.pop(key, None)
        error = errors.get(key)
        if output is not None:
            output.remaining.discard(key)
            output.failed = output.failed or error is not None
        if operation is not None:
            task = next(task for task in operation.tasks if _key(task) == key)
            operation._complete(task, error)
        elif output is not None and not output.remaining:
            output.keepalive = ()
            output.collected = True
            if not output.failed and output.on_complete is not None:
                output.on_complete()
            output.on_complete = None
            cleanup, output.cleanup = output.cleanup, None
            if cleanup is not None:
                with sdk._session_cleanup_lock:
                    sdk._session_cleanups.append(cleanup)


def drain_cleanups(sdk):
    """Submit orphan-output cleanup on the owning thread, without waiting."""
    from .api import _run_cleanups

    _ensure_registry(sdk)
    with sdk._session_cleanup_lock:
        pending, sdk._session_cleanups = sdk._session_cleanups, []

    def run(cleanup):
        try:
            cleanup()
        except Exception:
            # Cleanup closures retain any rejected native handle for retry.
            with sdk._session_cleanup_lock:
                sdk._session_cleanups.append(cleanup)
            raise

    _run_cleanups(lambda cleanup=cleanup: run(cleanup) for cleanup in pending)
