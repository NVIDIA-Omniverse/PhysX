<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-TASK-002
maps_to: REQ-CAPI-TASK-001
type: unit
---

## Scenario

Wait-all timeout returns completed-prefix failures without consuming pending work.

## Given

- A failed task followed by a deterministically held job and later submitted work.
- Poll, finite and infinite wait modes.

## When

- Poll or finitely wait for all, retain the partial result, release the hold and retry.
- Wait on a later completed task without collecting an earlier failure.
- Change threads between externally serialized waits.

## Then

- Timeout reports completed-prefix failures and the earliest pending task;
  unfinished and later outcomes remain collectible (REQ AC-2, AC-3, AC-8).
- Collected failures are not reported twice, and a target-only wait leaves
  earlier failures uncollected (REQ AC-2).
- One deadline bounds readiness; outcome allocation/collection may take additional
  time. Infinite retry can execute unstarted jobs and reaches a terminal result
  (REQ AC-8, AC-9).

## Automated coverage

- `WorkerStep.WaitAllTimeoutPrefix` in `tests/c_unittests/test_worker_step.cpp`.
- `ExecutorTasks.WaitAllStopsAtTheEarliestUnfinishedJob` and
  `WorkerWaits.FiniteWaitsAndPollsNeverRunJobs` in `tests/c_unittests/test_executor_tasks.cpp`.
- `PhysXTestFixture.PhysXAsync_PendingSimulationTimeoutsDoNotBlockOrConsume`
  in `tests/c_unittests/test_physx_async.cpp`.
- `PhysXTestFixture.TaskWaitAndErrorLookupMayMoveAcrossSerializedThreads`
  in `tests/c_unittests/test_task_completion.cpp`.

## Coverage limits

The final ready-at-deadline predicate check is source-inspected in
`InstanceExecutor::waitLocked`; there is no injected-clock boundary test.
