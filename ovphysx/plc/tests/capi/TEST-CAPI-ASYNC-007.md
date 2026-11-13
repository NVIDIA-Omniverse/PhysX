<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-ASYNC-007
maps_to: REQ-CAPI-ASYNC-002
type: integration
---

## Scenario

Pending handles and attachment generations let callers submit dependent work
without observing execution timing.

## Given

- `HandlesAtSubmit` and `AttachmentGenerations` in
  `tests/c_unittests/test_handles_at_submit.cpp`, a worker hold and two live instances.

## When

- Queue query/read/write creators and their consumers without intermediate waits.
- Fail a creator, use and release its handle; try foreign and unknown handles.
- Release a pending handle and submit another call naming it; exhaust the handle space.
- Queue attach/detach and dependent operations, including failing attach followed
  by work submitted before and after collection of that failure.

## Then

- Accepted creators return distinct nonzero handles before execution; dependent
  calls execute in order and failures name the failed producer (REQ AC-13).
- Foreign pending handles reject admission. Release is ordered behind creation,
  and later use is rejected immediately; handles are never reused (REQ AC-5, AC-13).
- Failed attach/detach invalidates the generation expected by dependent calls.
  Collecting the failed lifecycle task lets newly submitted work use the surviving
  attachment but does not alter already submitted work (REQ AC-14).

## Automated coverage

- `HandlesAtSubmit.QueuedProducersReturnHandlesThatLaterCallsUseAtOnce`,
  `ForeignHandlesAreRejectedAtSubmissionOnBothVenues`,
  `CallsThatNameAFailedProducersHandleFailTheirTask`,
  `ReleasesOfPendingAndFailedHandlesRunAfterTheirProducers`, `HandlesAreNeverReused`,
  `ExhaustedHandlesRejectProducersOnBothVenues`.
- `AttachmentGenerations.CallsRightAfterAnAttachUseItsAttachment`,
  `CallsAfterAFailedAttachFailUntilItsFailureIsCollected`, `CallsAfterADetachSeeNoStage`,
  `AFailedAttachLeavesTheEarlierAttachmentToLaterCalls`,
  `LongCallsAfterAFailedAttachFailUntilItsFailureIsCollected`,
  `CallingThreadCallsAfterAFailedAttachFailUntilItsFailureIsCollected`.
