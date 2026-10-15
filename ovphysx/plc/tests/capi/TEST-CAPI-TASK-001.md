<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-TASK-001
maps_to: REQ-CAPI-TASK-001
type: unit
---

## Scenario

Individual waits preserve earlier failures and collect each outcome exactly once.

## Given

- A live instance accepting an inline failing clone A and successful reset B.
- Real missing-source clone failures and successful idempotent releases,
  including discarded original receipts and sparse out-of-order collection.

## When

- Wait directly on B, then collect remaining tasks with `ovphysx_wait_all`.
- Repeat direct and aggregate waits; collect tasks out of submission order.
- Submit invalid, wrong-owner, never-issued and destroyed-instance receipts.
- Execute real step and clone producers and inspect their receipts and waits.

## Then

- Accepted inline failure has successful admission with nonzero identity;
  rejected work has zero identity and an immediate error (REQ AC-1).
- B succeeds and A remains in the next aggregate result; repeated direct waits
  return `NOT_FOUND`, and repeated empty aggregate waits succeed (REQ AC-2, AC-3).
- Identity and output validation return the documented statuses (REQ AC-4).
- Recoverable earlier failure does not prevent otherwise-valid work; real
  simulation/clone ordering and error reporting remain correct (REQ AC-6).

## Automated cases

- [test_task_completion.cpp](../../../tests/c_unittests/test_task_completion.cpp), fixture
  `PhysXTestFixture`:
  - `TaskWaitReportsOnlyItsTargetAndWaitAllRetainsPriorFailure`: B-only reporting,
    retained A, repeated direct waits and empty wait-all (AC-2, AC-3).
  - `TaskWaitAllFindsDiscardedReceiptsInSubmissionOrder` and
    `TaskOutOfOrderCollectionPreservesEarlierOutcomesAndErrorText`: discarded
    receipts and out-of-order collection (AC-2, AC-3).
  - `TaskSparseCollectionKeepsHolesAndCompactsAtWaitAll`: interleaved direct waits
    preserve holes and aggregate collection acknowledges the remainder (AC-2, AC-3).
  - `TaskInlineFailureIsAcceptedButInvalidSubmissionHasNoIdentity`: admission
    versus execution failure (AC-1).
  - `TaskWaitRejectsInvalidReceiptsAndPreservesOwnedOutput` and
    `TaskReceiptFromAnotherLiveInstanceIsRejected`: identity, instance-first
    validation and output ownership (AC-4).
  - `StepWaitSucceedsWhilePriorCloneFailureRemainsUncollected`: a real step
    proceeds after recoverable failure (AC-6).
- [test_clone.cpp](../../../tests/c_unittests/test_clone.cpp),
  `CloneTest.ErrorSourceDoesNotExist` and
  `CloneTest.FailedCloneUnregistersReplicatorForReattach`: inline clone failure
  is collected through its receipt and survives reset/reload (AC-1, AC-6).
- [test_physx_async.cpp](../../../tests/c_unittests/test_physx_async.cpp),
  `PhysXTestFixture.PhysXAsync_ConsumedOpsRemainSingleUseAfterInternalSync` and
  `PhysXTestFixture.PhysXAsync_WaitAllConsumesInternallySynchronizedIndex`:
  execution retirement preserves single-use collection (AC-2, AC-6).
- [test_c_api_compatibility.c](../../../tests/c_unittests/test_c_api_compatibility.c):
  C11 checks the receipt and wait/error/destructor function signatures.
