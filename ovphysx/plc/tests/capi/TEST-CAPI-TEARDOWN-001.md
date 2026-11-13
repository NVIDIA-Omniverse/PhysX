<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-TEARDOWN-001
maps_to: REQ-CAPI-TEARDOWN-001
type: integration
---

## Scenario

Destruction cancels unstarted work only after it can commit to teardown.

## Given

- `WorkerTeardown` in `tests/c_unittests/test_worker_teardown.cpp`, queued and
  running jobs, live output handles, companion leases and a saturated Carbonite
  task pool.

## When

- Destroy during a retryable companion condition, then release that condition.
- Destroy with a running job and queued successors; attempt destroy inside a callback.
- Destroy an instance with more live output handles than one cleanup batch.
- Hold the lifecycle lock while destruction starts; shut down with live instances.

## Then

- Callback destruction is refused without mutation (REQ AC-1).
- Retryable destruction preserves queued work and it resumes normally (REQ AC-2).
- Successful destruction waits for running work, cancels successors and leaves
  receipts inaccessible after handle retirement (REQ AC-3).
- Destruction completes after batched output-backend cleanup (REQ AC-3).
- Lock ordering and task-aware waits permit the saturated pool to progress
  (REQ AC-4).
- Shutdown stops workers and later execution on stopped handles is refused
  (REQ AC-5).

## Automated coverage

- `WorkerTeardown.RetryableDestroyKeepsQueuedWork`, `DestroyCancelsQueuedLifecycleJobs`,
  `DestroyWaitsForRunningJobOnly`, `DestroyCleansMoreThanOneBatchOfOutputHandles`,
  `DestroyTakesTheLifecycleLock`,
  `DestroyFromOwnWorkerCallbackRefused`, `SaturatedPoolLaneAndDestroy`,
  `ShutdownJoinsWorkersWithLiveInstances`.
