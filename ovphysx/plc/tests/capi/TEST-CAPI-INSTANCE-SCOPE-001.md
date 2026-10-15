<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-INSTANCE-SCOPE-001
maps_to: REQ-CAPI-INSTANCE-SCOPE-001
type: integration
---

## Scenario (AC-1, AC-2, AC-3, AC-4, AC-5, AC-6)

Two live ovphysx instances isolate wrapper-owned resources. A second instance
cannot consume the first's query, subscription, retained task error, or debug-render
cache, and destroying one leaves the other usable. A second instance may also
be attached at the same time (TEST-CAPI-ATTACH-OWNER-001).

## Given

- Process lifecycle is initialized.
- Instance A and instance B are both created.
- Instance A attaches an ovstage Stage.
- Instance B remains unattached for the isolation cases. A separate case
  attaches B while A is still attached.

## When

- A opens an output query; B submits `ovphysx_read` and `ovphysx_release_query` with
  A's query.
- A subscribes to object changes; B unsubscribes that subscription id; A then
  unsubscribes successfully.
- A creates a C++ subscription whose callback captures owned state, destroys A,
  then destroys the subscription. Python destroys an instance with callback
  keepalives still registered.
- A collects a failed task; B calls `ovphysx_get_last_task_error` with A's
  receipt; A still reads the original error text while its wait result lives.
- A writes debug-render `bodyAxes` through an `OVPHYSX_QUERY_TARGET_DEBUG_RENDER`
  session; B writes a different value for the same attribute; A reads back its
  own cached value.
- A is destroyed while B still exists; B then attaches.

## Then

- B's read and release of A's query are rejected with `OVPHYSX_API_NOT_FOUND`, and A can
  still read from and release its query (REQ AC-2).
- B's unsubscribe of A's subscription returns `OVPHYSX_API_NOT_FOUND`; A's
  subsequent unsubscribe succeeds (REQ AC-4).
- C++ and Python callback captures are released after instance destruction has
  completed native subscription cleanup (REQ AC-4).
- `ovphysx_get_last_task_error(B, A's task)` is empty;
  `ovphysx_get_last_task_error(A, task)` still returns A's wait error (REQ AC-3).
- A's `bodyAxes` read returns the value A wrote (REQ AC-5).
- After A is destroyed, B can attach and step. While both are attached, each
  keeps its own attachment (REQ AC-6).
- Subscribe / unsubscribe / last-task-error take the instance handle (REQ AC-1).
- `test_multi_instance.cpp`: `ForeignQueryRejectedOnPeerInstance`,
  `ForeignSubscriptionUnsubscribeRejected`, `TaskErrorIsolatedPerInstance`,
  `DebugRenderCacheIsolatedPerInstance`,
  `DestroyOwnerLeavesPeerReusable`.
- `TwoInstancesRemainAttachedAndStepIndependently` attaches both instances and
  checks that resetting one leaves the other steppable.
- `DetachAndDestroyOneLiveInstanceLeavesPeerSteppable` detaches and destroys
  one live instance and checks that the peer stays attached and steppable.
- `test_object_change_callbacks.cpp`: subscribe/unsubscribe call sites pass
  the canonical instance handle; `CppSubscriptionReleasesStateAfterInstanceDestroy`
  covers callback-state cleanup after owner destruction.
- `lifecycle_tests/test_multi_instance.py`:
  `test_peer_unsubscribe_does_not_drop_owner_subscription`,
  `test_destroy_clears_object_change_callback_keepalive`.
