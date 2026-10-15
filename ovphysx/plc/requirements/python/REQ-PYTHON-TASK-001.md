<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-TASK-001
title: Python task receipts, typed operations and copied failures
status: implemented
owner: ovphysx
---

## Description

Python exposes native task completion through immutable value receipts and
exceptions that retain copied failures after native wait-result cleanup. Session
operations return typed `Operation` objects over their real receipts without
collecting before return; only explicitly named synchronous adapters and instance
destruction wait. Operation outcomes are recorded when any Python wait collects them, and typed
payloads are converted once, on first result access.

## Acceptance Criteria

- **AC-1: Value receipt.** `Task` is immutable and stores a copy of native admission
  status, operation index and owning instance. It owns no simulation resource and
  has no destructor, wait method, future, cancellation or awaitability.
- **AC-2: Explicit waits.** `PhysX.wait_task(task, *, timeout_ns=None)` and
  `PhysX.wait_all(*, timeout_ns=None)` return `None` on success. Non-Task arguments
  raise `TypeError`; receipt validation and internal wait errors raise ordinary
  runtime errors. Timeout values preserve native polling/finite/infinite semantics.
  A timeout outside 0 through `2**64 - 1` raises `ValueError` before native calls.
- **AC-3: Failure values.** `TaskError(RuntimeError)` exposes immutable `(Task,
  message)` entries. `TaskTimeoutError(TimeoutError)` exposes partial errors and
  the pending task. Timeout takes precedence over completed execution failures.
- **AC-4: Cleanup.** Python copies all result receipts and messages before native
  cleanup in `finally`. Retained exception messages survive later waits and
  instance destruction, including when translation itself raises.
- **AC-5: Producer semantics.** `step`, `clone`, `attach_ovstage`,
  `update_from_ovstage` and `detach_ovstage` return `Task` on
  successful admission. Accepted inline execution failures raise only when
  collected by a wait; rejection raises immediately without a Task.

- **AC-6: Typed session operations.** `PhysX.query`, `PhysX.read`, `PhysX.write`,
  `Query.close`, `ReadResult.fetch_next`, `ReadResult.close`,
  `WriteSession.fetch_next`, `WriteSession.commit` and `WriteSession.close` return an
  `Operation` after admission without waiting; admission rejection raises at once.
  `Operation.tasks` is the immutable tuple of its real native receipts, of any length;
  `Operation.task` exists only for exactly one receipt; no receipt is fabricated for
  Python-only work, so a purely local operation is complete with no tasks. `result`
  never waits and raises while any outcome is uncollected. `wait(timeout_ns=None)`
  collects the operation's own tasks within one readiness budget and returns the typed
  result. `read` and `write` require a completed `Query` of the same instance. The
  explicit waiting adapters are `read_sync`, `read_tokens_sync`, `write_sync`,
  `commit_sync` and `close_sync`. ctypes exposes complete native receipts for every
  operation and mirrors the native session-result layouts. CUDA producer/consumer
  stream dependencies remain separate from host task waits.
- **AC-7: Cleanup ownership.** Default close and release, default context exit, finalizers and
  owning-thread deferred drains only submit releases; they never call `wait_task` or
  `wait_all`. Accepted admission transfers cleanup to the SDK and is never reissued, and
  implicit cleanup may drop its receipt because the uncollected native outcome remains
  for `wait_all`. Rejected admission keeps or requeues the holder for a later drain.
  Deferred cleanup runs on the owning thread, attempts every queued holder and preserves
  rejected handles. An original operation or context exception remains primary, with
  cleanup failures chained to it. `destroy()` collects outstanding operations
  and the cleanup it triggers before retiring native state, and reports earlier
  release failures instead of erasing them. `detach_ovstage()` submits deferred
  releases and retires Stage-bound read metadata without collecting earlier task
  outcomes; payloads still pending across teardown fail materialization as
  invalidated. Accepted detach retires the Python write generation; rejected
  native admission does not. Attach/detach keep the Stage alive through outcome
  collection, even when their receipts are dropped. A timeout retains it; after
  collection, including failure, native ownership must confirm detachment before
  releasing the keepalive. `reset_stage()` is removed.
- **AC-8: Articulation convenience.** `update_articulations_kinematic` opens and
  waits an `ARTICULATION` query for the requested scope, or a selected one when
  `prim_paths=` or `pattern=` is given (the selection rules of
  [REQ-CAPI-SELECT-001](../capi/REQ-CAPI-SELECT-001.md) AC-6), applies the
  requested POSITION/VELOCITY flags, waits the update, and releases the query.
  The default updates both position and velocity for all articulations.
- **AC-9: Outcome dispatch.** `wait_task` records the outcome of the one task it
  collects; `wait_all` records every registered operation it collects and includes their
  failures in its aggregate `TaskError`. A timed-out `wait_all` records only the
  collected prefix and leaves later operations pending. A later `wait()` or `result` on
  an operation with a recorded failure raises that cached failure again without another
  native wait. Recording converts no payload: typed conversion runs once, on the first
  successful `result` or `wait()` on the submitting thread, and a conversion failure is a
  Python error that cleans up the unexposed output rather than a native task failure. A
  cached failure is re-raised as a copy that retains no traceback frames, so it keeps
  neither caller locals nor unexposed arrays alive.
- **AC-10: Retention and orphan outputs.** Borrowed input bytes and arrays, output
  slots, selection buffers and commit event owners stay alive until their operation's
  outcomes are collected or the instance is destroyed, even when the `Operation` is
  dropped. A dropped operation whose creation succeeded has its query, read or write
  output released submit-only on the owning thread after collection, without
  materializing Warp arrays; an unexposed read group is retired through its parent. No
  native call runs on a garbage-collection thread. Callers that submit without waiting
  call `wait_all` periodically to bound retained storage and observe failures.

## Test References

- [TEST-PYTHON-TASK-001](../../tests/python/TEST-PYTHON-TASK-001.md)

## Code References

- `ovphysx/python/ovphysx/api.py`: Task, errors, producers, wait translation,
  session operations and cleanup queues.
- `ovphysx/python/ovphysx/_operations.py`: `Operation`, outcome dispatch, retention and
  orphan cleanup.
- `ovphysx/python/ovphysx/_bindings.py`: native receipt, result and session-result
  layouts and functions.
- `ovphysx/python/ovphysx/api.pyi`: typed public contract.
- `ovphysx/python/ovphysx/types.py`: articulation kinematic flags.
- `ovphysx/python/ovphysx/__init__.py`: public exports.

## Dependencies

- [REQ-CAPI-TASK-001](../capi/REQ-CAPI-TASK-001.md).
- [REQ-CAPI-ASYNC-002](../capi/REQ-CAPI-ASYNC-002.md).
- [REQ-CAPI-SESSION-001](../capi/REQ-CAPI-SESSION-001.md).
