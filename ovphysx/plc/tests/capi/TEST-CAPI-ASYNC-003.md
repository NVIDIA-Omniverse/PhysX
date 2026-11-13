<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-ASYNC-003
maps_to: REQ-CAPI-ASYNC-003
type: integration
---

## Scenario

A step returns before its work runs, progresses independently, and preserves order.

## Given

- `WorkerStep` in `tests/c_unittests/test_worker_step.cpp`, a CPU scene and worker hold.
- `GpuStepErrorGpuTest.AsyncFailureSurvivesDisabledLogging` in
  `tests/c_unittests/test_step_errors.cpp`, in GPU readback and DirectGPU modes.

## When

- Submit a step behind the hold, then release it without waiting.
- Submit consecutive steps, poll and use bounded waits; collect outcomes afterward.
- Step a newly attached scene with zero PhysX solver threads.
- Compare a synchronous drain and an asynchronous step; inject lazy-attach failure.
- Plant a dispatch error in a pre-step callback, then clear or replace thread-local
  error state in the matching post-step callback.
- Abort GPU simulation with logging disabled, then collect the asynchronous
  step failure after its required `fetchResults`.

## Then

- Submission performs no simulation or initial parse on the caller, including
  with zero solver threads (REQ AC-1, AC-5).
- The job reaches terminal state without a wait; collection reports a failed
  lazy attach through the step's receipt (REQ AC-3, AC-5).
- Steps execute in order and synchronous drains leave earlier outcomes collectible
  (REQ AC-5, AC-6).
- The asynchronous GPU step retains the CUDA dispatch diagnostic through fetch
  completion and reports it from the task (REQ AC-5).
- Successful and rejected callback-time API calls cannot replace the dispatch
  failure that the step task reports (REQ AC-5).

## Automated coverage

- `WorkerStep.ReturnsWhilePending`, `ProgressesWithoutWait`, `OrderedChain`,
  `IdleStepRunsNothingInTheCallWithZeroSolverThreads`,
  `InitialSceneParseRunsOnTheWorkerAfterTheCallReturns`,
  `FailedLazyStageAttachFailsTheStep`, `SyncDrainDoesNotCollect`,
  `WorkerStep.DispatchErrorSurvivesFetchCallbacks`,
  `GpuStepErrorGpuTest.AsyncFailureSurvivesDisabledLogging`.
