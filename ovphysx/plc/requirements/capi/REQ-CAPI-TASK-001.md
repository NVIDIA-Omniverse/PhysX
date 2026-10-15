<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-TASK-001
title: Task completion and retained execution errors
status: implemented
owner: ovphysx
---

## Description

A task receipt identifies an accepted operation and its owning instance.
Execution-resource retirement is independent of explicit outcome collection.
Wait results own collected failure receipts and text.

## Acceptance Criteria

- **AC-1: Admission identity.** The public task producers include `ovphysx_step`,
  `ovphysx_clone`, `ovphysx_attach_ovstage`, `ovphysx_update_from_ovstage`,
  `ovphysx_detach_ovstage`, and the session/articulation operations
  specified by REQ-CAPI-ASYNC-002. Accepted operations return `ovphysx_enqueue_result_t`
  with `SUCCESS`, a nonzero issued index and their instance, even if execution
  fails inline. Rejected submissions zero both identity fields and expose an
  immediate cause through `ovphysx_get_last_error()`.
- **AC-2: Target-only collection.** `ovphysx_wait_task` collects only its target's
  terminal outcome. Waiting on successful B does not report or collect failed A.
  Completed outcomes are single-use, including success; repeat waits return
  `NOT_FOUND`. Discarding a receipt leaves its outcome collectible.
- **AC-3: Aggregate collection.** `ovphysx_wait_all` covers operations accepted at
  entry, collecting uncollected outcomes in submission order. It uses one
  readiness deadline and stops at the earliest unfinished task when it expires.
  `TIMEOUT` takes precedence while returning earlier failures and `pending_task`;
  later tasks stay uncollected. Repeated empty waits succeed.
- **AC-4: Validation.** Waits validate the instance first (`NOT_FOUND` when invalid),
  reject zero, rejected or wrong-owner receipts with `INVALID_ARGUMENT`, and
  return `NOT_FOUND` for never-issued or collected identities. Output is required
  and must be empty; nonempty reuse is rejected without overwriting ownership.
- **AC-5: Retained causes.** Caller-owned wait results expose failed receipts,
  their count and the pending task, with ownership bookkeeping kept internal.
  Live wait results own `error_tasks` and error strings; returned fields must
  remain unchanged until result destruction, without separately freeing the array.
  `ovphysx_get_last_task_error` reads without collecting; absent retained causes
  yield empty text, and invalid instance/owner sets an immediate diagnostic.
  Borrowed text survives unrelated waits and instance teardown until result
  destruction. Fresh lookup requires a live instance. Destruction accepts null
  and empty results, zeros results and remains safe after teardown. C++ results
  are move-only and preserve native return status.
- **AC-6: Execution retirement.** Internal drains preserve uncollected failures,
  while otherwise-valid later work may proceed. Task identity and failure
  fallback storage are reserved before accepted work starts. Each instance tracks
  at most one unfinished simulation, finalized through `fetchResults`; completed
  inline operations need no execution event.
  Failures retain specific execution/finalization causes independently of log
  filtering. Custom application code runs directly after the relevant wait;
  it is not submitted as a task callback.
- **AC-7: Allocation safety.** Failure to allocate tracking rejects before work
  starts. Failure to prepare a wait result leaves an empty destructible output
  and completed outcomes retryable. Collection commits only after result storage
  and retained-error ownership are ready. No C++ exception crosses these C APIs.
- **AC-8: Readiness timeout.** Zero polls, `UINT64_MAX` waits indefinitely, and a
  finite value bounds readiness, with ready-at-deadline winning. Finalization
  may take longer. Unfinished timed-out tasks stay retryable. Calls remain
  externally serialized, including when the calling thread changes.

## Test References

- [TEST-CAPI-TASK-001](../../tests/capi/TEST-CAPI-TASK-001.md)
- [TEST-CAPI-TASK-002](../../tests/capi/TEST-CAPI-TASK-002.md)
- [TEST-CAPI-TASK-003](../../tests/capi/TEST-CAPI-TASK-003.md)
- [TEST-CAPI-TASK-004](../../tests/capi/TEST-CAPI-TASK-004.md)

## Code References

- `ovphysx/include/ovphysx/ovphysx_types.h`: task and wait-result representation.
- `ovphysx/include/ovphysx/ovphysx.h`: public completion/error contract.
- `ovphysx/src/ovphysx/ovphysx.cpp`: admission, drains, collection and lookup.
- `ovphysx/src/include/internal/sdk/ovphysxSDK.hpp`: instance tracking and errors.
- `ovphysx/src/include/internal/sdk/TaskContext.hpp`: task creation and outcome recording.
- `ovphysx/src/include/internal/sdk/ovphysxAsyncWait.hpp`: readiness and shared-deadline helpers.
- `ovphysx/include/ovphysx/experimental/ovphysx.hpp` and
  `ovphysx/src/ovphysx/PhysXWrapper.cpp`: receipt and move-only result wrappers.
- `ovphysx/include/ovphysx/experimental/Helpers.hpp`: wait-result ownership and status.

## Dependencies

None.
