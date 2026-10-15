<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-TASK-002
maps_to: REQ-CAPI-TASK-001
type: unit
---

## Scenario

A wait-all readiness timeout returns partial failures and preserves unfinished
and later tasks for retry.

## Given

- A real failed clone A, a callback-gated PhysX simulation B, and a later
  completed idempotent release C.
- Finite, polling and infinite wait modes.

## When

- Collect C directly, poll wait-all and retain its partial output; unblock B and retry.
- Advance an injected clock against one shared readiness budget in the helper;
  exercise public finite waits against the gated simulation.
- Change the caller thread between externally serialized waits.

## Then

- C completes independently while B is pending. Timeout returns A in
  `error_tasks` and B in `pending_task` (REQ AC-2, AC-3).
- Earlier collected A is not reported again (REQ AC-2).
- One deadline applies across tasks; readiness observed at the boundary wins,
  while finalization may exceed the readiness timeout (REQ AC-8).
- Polling never consumes unfinished tasks, and infinite retry reaches a terminal
  result (REQ AC-8).

## Automated cases

- [test_physx_async.cpp](../../../tests/c_unittests/test_physx_async.cpp):
  - `PhysXAsync.TaskWaitAllUsesOneReadinessDeadline` advances a fake clock against
    the shared budget helper (AC-3, AC-8).
  - `PhysXAsync.ReadinessObservedAtDeadlineWins` checks the readiness boundary
    with an injected clock and sleep function (AC-8).
  - `PhysXTestFixture.PhysXAsync_PendingSimulationTimeoutsDoNotBlockOrConsume`
    gates real PhysX execution, polls, waits finitely, releases the gate and
    retries indefinitely. Earlier failures appear once in partial wait-all output;
    earlier and later ready tasks remain individually collectable (AC-2, AC-3, AC-8).
- [test_task_completion.cpp](../../../tests/c_unittests/test_task_completion.cpp),
  `PhysXTestFixture.TaskWaitAndErrorLookupMayMoveAcrossSerializedThreads`:
  collection on a joined worker thread followed by lookup on the caller thread
  (AC-8).

## Coverage limits

- Shared finite-budget arithmetic and ready-at-deadline behavior use helper
  tests. The public wait-all regression uses polling; it does not time a finite
  wait across several pending simulations: the implementation permits only one
  pending simulation per instance.
- Real simulation coverage verifies readiness timeout and retry. It does not
  impose or measure a bound on result finalization.
