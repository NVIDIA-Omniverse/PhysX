<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-ASYNC-003
title: Long operations are queued on the instance worker
status: implemented
owner: ovphysx
---

## Description

`ovphysx_attach_ovstage`, `ovphysx_update_from_ovstage`, `ovphysx_detach_ovstage`,
`ovphysx_clone` and `ovphysx_step` are the long operations. Each one checks its
arguments and queues one job on an asynchronous instance
([REQ-CAPI-EXECUTOR-001](REQ-CAPI-EXECUTOR-001.md)), returning its task receipt
without executing or waiting for that job. A workerless instance instead completes
accepted work on the caller before returning. The worker runs jobs one at a time in
submission order, so the caller can submit a chain of operations and wait once.

A task-returning call may complete before it returns: the worker may start or even
finish the job before the submitting call returns. Callers depend on neither.

Checks that depend on what earlier work did run in the job, so they see the state
that earlier queued work left behind. They report through the task, with the same
message text the former rejection used.

## Acceptance Criteria

- **AC-1: Queued.** With asynchronous execution enabled for the instance, each
  long operation queues exactly one job without executing it or waiting for its
  lane, including a step with zero PhysX solver threads. Workerless instances
  complete the job on the submitting thread. While a recording is active or
  starting, attach and detach are synchronous exceptions
  ([REQ-CAPI-OMNIPVD-LATE-001](REQ-CAPI-OMNIPVD-LATE-001.md) AC-12).
- **AC-2: Rejected at submission.** Only checks of the call itself and of process
  state reject at submission, with a zero receipt, an immediate error and no change:
  an invalid instance; null, empty or NUL-containing strings; zero counts; an
  invalid ordinal or range; a negative or non-finite timestep; a clone target equal
  to the source or repeated in the call; env ids out of range; an unavailable runtime
  interface; a schema-registration refusal (attach); a conflicting companion
  operation or native-access lease (attach, detach); callback context
  ([REQ-CAPI-REENTRY-001](REQ-CAPI-REENTRY-001.md)); a stopped instance or a full
  queue.
- **AC-3: State checks are task failures.** These checks run in the job and fail the
  task with its API/task-specific cause: no stage attached (step,
  update, clone), already attached (attach), clone after warmup or the first step,
  target already cloned on this attachment, and simulation-time overflow (step). A
  failed job retains its cause, and later valid jobs still run. Lifecycle-generation
  failure propagation follows REQ-CAPI-ASYNC-002 AC-14.
- **AC-4: Inputs.** Clone copies its source path, target paths, anchor transforms and
  env ids at submission; the caller may free or overwrite them once the call returns.
  Attach and update borrow the Stage and the sealed data their ordinals name until the
  task completes; an attached Stage stays borrowed until a successful detach
  ([REQ-CAPI-OVSTAGE-ATTACH-001](REQ-CAPI-OVSTAGE-ATTACH-001.md) AC-5).
- **AC-5: Step job.** A step job performs lazy attachment, `simulate` and
  `fetchResults` before publication. It progresses without an application wait;
  a native infinite wait may take an unstarted job under REQ-CAPI-EXECUTOR-001 AC-6.
  Post-step callbacks run on the thread executing the job. Simulation time, the
  last positive timestep, and warmup/first-step state advance when execution
  reaches their existing update points, not merely when the call is submitted. `ovphysx_step_sync` and warmup run on
  the calling thread after earlier jobs ([REQ-CAPI-LANE-001](REQ-CAPI-LANE-001.md))
  and return only after completion. Their internal execution uses no worker
  handover. A failed step reports its own outcome through its receipt. When
  simulation dispatch fails before `fetchResults` delivers callbacks, that first
  failure remains the task cause even if a callback changes thread-local error state.
- **AC-6: Submission order.** Results are as if the calls ran in submission order:
  attach, update, clone, step and a following query, submitted without waits in
  between, all succeed; detach followed by attach without a wait succeed in that
  order.
- **AC-7: Lifecycle lock.** Attach and detach jobs hold one process-wide lifecycle
  lock while they run and never wait for a lane while holding it. Lifecycle work
  on different instances is serialized; submitting it does not wait for a peer's
  lifecycle job. An accepted attach or detach reserves companion transition
  admission until its lifecycle work finishes and releases it before publishing
  the task outcome, so a later companion operation reports busy rather than
  racing the lifecycle task, while an operation after a successful wait is
  admitted. Successful attach/detach publishes its attachment generation.
- **AC-8: CUDA.** A queued attach job of a GPU instance runs in the CUDA context
  that was current on the submitting thread at submission and leaves no context
  bound on the worker; the caller's current context is unchanged. A GPU
  instance's worker sets its thread's stream-capture mode to relaxed once, before
  queued CUDA work. A CPU-only instance's worker makes no CUDA call. A native
  waiter executing a job temporarily relaxes and restores its capture mode.
  Workerless execution keeps the caller's capture mode and follows synchronous
  attach context behavior: device-ordinal setup may leave its newly-created CUDA
  context current when the caller had none. Applications collect pending tasks
  before beginning capture.

## Test References

- [TEST-CAPI-ASYNC-003](../../tests/capi/TEST-CAPI-ASYNC-003.md)
- [TEST-CAPI-ASYNC-004](../../tests/capi/TEST-CAPI-ASYNC-004.md)
- [TEST-CAPI-ASYNC-005](../../tests/capi/TEST-CAPI-ASYNC-005.md)

## Code References

- `ovphysx/src/ovphysx/ovphysx.cpp`: submission and job bodies of `ovphysx_step`,
  `ovphysx_clone`, `ovphysx_update_from_ovstage`, `ovphysx_attach_ovstage` and
  `ovphysx_detach_ovstage`; lifecycle lock; attach CUDA context (AC-1 to AC-8).
- `ovphysx/src/include/internal/sdk/ovphysxSDK.hpp`: job record and lock order (AC-1, AC-7).
- `ovphysx/src/ovphysx/ovphysxTensorBinding.cpp` (`ovphysx_warmup_if_needed`):
  warmup on the calling thread (AC-5).
- `ovphysx/include/ovphysx/ovphysx.h`: per-function submission and task-error
  contract (AC-1 to AC-4).

## Dependencies

- [REQ-CAPI-EXECUTOR-001](REQ-CAPI-EXECUTOR-001.md)
- [REQ-CAPI-TASK-001](REQ-CAPI-TASK-001.md) (receipts, waits and collection)
- [REQ-CAPI-LANE-001](REQ-CAPI-LANE-001.md) (calls that run on the calling thread)
