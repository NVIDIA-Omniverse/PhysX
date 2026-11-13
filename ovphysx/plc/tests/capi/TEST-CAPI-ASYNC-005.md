<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-ASYNC-005
maps_to: REQ-CAPI-ASYNC-003
type: integration
---

## Scenario

Queued attachment lifecycle preserves order, caller CUDA context and capture mode.

## Given

- `WorkerLifecycle` in `tests/c_unittests/test_worker_lifecycle.cpp`,
  `CompanionBridge` in `tests/c_unittests/test_companion.cpp`, sealed caller-owned
  stages, CPU and CUDA variants, and a deterministic worker hold.

## When

- Submit attach, update, clone, step and output without intermediate waits.
- Submit detach followed immediately by another attach; run peer lifecycle calls.
- Hold an accepted detach, then attempt a companion operation before the worker
  runs the lifecycle task.
- Create the first device-ordinal GPU context manager during attach takeover.
- Submit attach without a current CUDA context, then install nested contexts on
  the waiting thread and take over the held worker job with an infinite wait.
- Attach with a caller-selected CUDA context, run an infinite wait during capture,
  and compare asynchronous and workerless instances.

## Then

- Asynchronous submissions return pending and chains run in order; the workerless
  instance finishes each job before return (REQ AC-1, AC-3, AC-6).
- Peer lifecycle execution is serialized without making submission wait for the
  peer's job (REQ AC-7).
- A companion operation submitted after accepted lifecycle work reports busy;
  the lifecycle task succeeds when the worker is released, and companion
  admission is open by the time its wait returns (REQ AC-7).
- An active-context attach submitted without a context still falls back to CPU when the waiter
  takes it over; the waiter retains its complete nested context stack. Any context
  created by device-ordinal setup is removed before that stack is restored (REQ AC-8).
- Attach uses the caller's context without changing it; CPU-only execution makes
  no CUDA call. Capturing callers either leave jobs to the worker or temporarily
  relax and restore their capture mode as permitted. Workerless active-context
  attach leaves that context current without worker-style push/pop, while
  device-ordinal setup may leave a new context current (REQ AC-8).

## Automated coverage

- `WorkerLifecycle.AttachReturnsWhilePending`, `OrderedChainWithoutWaits`,
  `AcceptedAttachKeepsLaterCompanionOperationBusy`, `DetachThenAttachWithoutWait`,
  `ConcurrentLifecycleOnTwoInstances`,
  `InstanceWithoutAWorkerThreadFinishesEachCallBeforeItReturns`, `CpuOnlyWorkerMakesNoCudaCall`.
- `WorkerLifecycle.NoContextOrdinalAttachRestoresCallerGpuTest` runs alone to cover
  the first device-ordinal CUDA manager creation; `scripts/test_cpp.cmake` gives
  it a separate GPU process.
- `WorkerLifecycle.ActiveContextAttachGpuTest`, `ActiveContextAttachWithoutWorkerGpuTest`,
  `WaitInsideACaptureLeavesTheJobToTheWorkerGpuTest`,
  `WaitInsideANonBlockingCaptureRunsTheJobRelaxedGpuTest`, `TwoInstancesOneContextCaptureGpuTest`.
- `CompanionBridge.AcceptedDetachKeepsLaterCompanionOperationBusy`.
