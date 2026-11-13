<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-TEARDOWN-001
title: Destroy and shutdown stop the instance worker
status: implemented
owner: ovphysx
---

## Description

An instance's worker thread may be running a job, and more jobs may be queued, when
the application destroys the instance or shuts the library down. Destroy decides
first whether it can proceed at all, without disturbing queued work. Only then does
it cancel the jobs that never started, join the worker and tear the instance down.

## Acceptance Criteria

- **AC-1: Refusals first.** `ovphysx_destroy_instance` returns
  `OVPHYSX_API_INVALID_STATE` before it waits or changes anything when called in
  callback context ([REQ-CAPI-REENTRY-001](REQ-CAPI-REENTRY-001.md)), on the
  instance's own worker thread, while a companion operation runs, during a
  companion transition, or with pending companion previews while a recording
  is active.
- **AC-2: Retryable with the queue intact.** Otherwise destroy holds companion
  transition admission, stops the worker from starting further jobs, waits only
  for the job that is running, and checks every condition that makes destruction
  retryable: native-access leases still using the instance, and previews with an
  active recording. While admission is held, later companion operations report
  busy. If a retryable condition holds, destroy returns
  `OVPHYSX_API_INVALID_STATE`, the worker resumes, and queued jobs run and report
  as if destroy had not been called.
- **AC-3: Cancel, join, then tear down.** On success, jobs that never started are
  cancelled without running, the worker thread is joined, and only then are the
  attachment, the output sessions, an owned recording and the instance registration
  torn down. Afterwards waits and `ovphysx_query_task_status` on the instance's
  receipts return `NOT_FOUND`.
- **AC-4: Lock order.** Destroy and shutdown hold no companion mutex, registry
  lock or lifecycle lock while waiting for a lane or a join; destroy may retain
  its nonblocking companion transition admission. Recording-owner destroy takes
  every instance lane in ascending handle order, potentially holding earlier
  lanes (including its own) while waiting for later ones, and takes the
  lifecycle lock only after the full ordered set.
- **AC-5: Shutdown.** `ovphysx_shutdown` with live instances cancels their queued
  jobs and stops every worker before it shuts down log-callback delivery; no
  instance worker runs when it returns. New work on surviving stopped handles is
  rejected; existing terminal outcomes remain collectable, and the handles still
  require destruction. Process exit without shutdown is covered by
  [REQ-CAPI-HOST-EXIT-001](REQ-CAPI-HOST-EXIT-001.md) AC-4.

## Test References

- [TEST-CAPI-TEARDOWN-001](../../tests/capi/TEST-CAPI-TEARDOWN-001.md)

## Code References

- `ovphysx/src/ovphysx/ovphysx.cpp`: `ovphysx_destroy_instance`,
  `omni_sdk_physx_destroy` and `ovphysx_shutdown` (AC-1 to AC-5).
- `ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp`: allocation-free output-session
  retirement after teardown commits (AC-3).
- `ovphysx/src/ovphysx/InstanceExecutor.cpp`: pausing dispatch, cancellation and join
  ([REQ-CAPI-EXECUTOR-001](REQ-CAPI-EXECUTOR-001.md) AC-1, AC-7).
- `ovphysx/src/include/internal/sdk/CompanionRuntime.hpp`: companion leases and
  transitions checked by destroy (AC-1, AC-2, AC-4).

## Dependencies

- [REQ-CAPI-EXECUTOR-001](REQ-CAPI-EXECUTOR-001.md)
- [REQ-CAPI-LANE-001](REQ-CAPI-LANE-001.md) AC-4 (taking every lane)
- [REQ-CAPI-ATTACH-OWNER-001](REQ-CAPI-ATTACH-OWNER-001.md) AC-3 (destroy releases the attachment)
