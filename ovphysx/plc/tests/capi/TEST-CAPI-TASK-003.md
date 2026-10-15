<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-TASK-003
maps_to: REQ-CAPI-TASK-001
type: unit
---

## Coverage status

Ownership and error retention have automated coverage. Allocation-failure and
identity-exhaustion paths (AC-7) are assessed by source review; allocator failures
and exhausted counters are not injected.

## Scenario

Collected causes outlive unrelated waits and instance teardown.

## Given

- Real missing-source clone failures and live wait results, including a clone
  failure with logging disabled.

## When

- Read the cause; perform unrelated waits; destroy the instance before the result.
- Attempt nonempty output reuse and repeat destruction on a zeroed output.
- Move a C++ wait result and inspect native status and failed receipts.
- Collect a batch of clone failures with distinct long missing-source paths,
  move its owner and destroy the instance before inspecting every retained
  receipt and message.

## Then

- Borrowed causes stay valid until result destruction, and fresh lookup through
  a destroyed instance fails (REQ AC-5).
- Nonempty reuse leaves ownership intact; null and repeated destruction are safe
  (REQ AC-4, AC-5).
- The real clone cause survives disabled logging; retained causes survive C++
  ownership moves (REQ AC-5, AC-6).
- The exposed failure array excludes internal ownership metadata and retains
  every receipt's matching message through growth, moves and teardown (REQ AC-5).
- A collected failure with empty detail still provides a fallback cause (REQ AC-6).

## Automated cases

- [test_task_completion.cpp](../../../tests/c_unittests/test_task_completion.cpp), fixture
  `PhysXTestFixture`:
  - `TaskOutOfOrderCollectionPreservesEarlierOutcomesAndErrorText`,
    `TaskErrorLifetimeEndsAtResultDestruction` and
    `TaskErrorTextAndResultDestructionSurviveInstanceTeardown`: retained and
    borrowed text lifetime, including cleanup after teardown (AC-5).
  - `TaskWaitRejectsInvalidReceiptsAndPreservesOwnedOutput`: nonempty reuse and
    null/repeated destruction (AC-4, AC-5).
  - `TaskCloneFailureIsCollectedWithLoggingDisabled`: the specific missing-source
    clone cause remains available with logging disabled (AC-6).
  - `CppTaskWaitResultMovesStatusAndOwnership`: move-only ownership, native
    error status and receipt preservation (AC-5).
  - `TaskWaitAllArrayOwnershipSurvivesGrowthMoveAndTeardown`: batch array and
    distinct long diagnostic retention through growth, moves and teardown (AC-5).
- [test_physx_async.cpp](../../../tests/c_unittests/test_physx_async.cpp), fixture
  `PhysXTestFixture`:
  - `PhysXAsync_PendingSimulationTimeoutsDoNotBlockOrConsume`: timeout status and
    pending identity survive a move (AC-5).

## Coverage limits

- The static fallback for a collected failure without detail is source-reviewed
  in `ovphysx_get_last_task_error`; real clone failures supply diagnostics, and
  these tests do not inject an empty internal cause (AC-6).

- There is no automated allocation-failure coverage. The tests use the installed
  SDK, which has no allocation-failure switches. Source review checks that
  `TaskContext::create_task` reserves a failure slot before publishing an identity,
  and `wait_tasks` prepares result storage and any interval node before
  collecting outcomes (AC-7).
- `TaskContext::create_task` rejects task-index exhaustion before publishing a
  receipt. The public task producers catch admission exceptions and return
  `OVPHYSX_API_ERROR` with zero task identity. This exhaustion path is
  source-reviewed, not runtime-injected.
- Clone error preservation is exercised directly. GPU simulation-finalization
  cause preservation is covered by [TEST-CAPI-TASK-004](TEST-CAPI-TASK-004.md).
  Internal warmup failure handling remains source-reviewed.
