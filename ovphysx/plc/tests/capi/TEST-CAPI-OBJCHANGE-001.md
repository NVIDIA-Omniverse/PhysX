<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OBJCHANGE-001
maps_to: REQ-CAPI-OBJCHANGE-001
type: integration
---

## Scenario

Unsubscribe either waits out an in-flight callback or, when invoked from any
object-change callback on the same Carbonite task (native thread when the
callback is not a Carbonite task), refuses instead of hanging. Destroying
the experimental C++ handle from inside a callback closes it without waiting
and releases its state after that call unwinds, including when the callback
throws. Covers REQ AC-1, AC-2, and AC-3.

## Given

- An attached stage that has completed one step, so `ovphysx_detach_ovstage`
  delivers `on_all_objects_destroyed`.
- A subscription whose callback records entry, then spends about 200ms before
  writing a marker through `user_data` and recording completion.

## When

- Another thread calls `ovphysx_unsubscribe_object_changes` after the callback
  has entered and before it has finished, while the test thread is inside
  `ovphysx_detach_ovstage` (`UnsubscribeWaitsForInFlightCallback`).
- The callback itself calls `ovphysx_unsubscribe_object_changes` for its own
  subscription id, then the test thread unsubscribes that id again
  (`UnsubscribeFromOwnCallbackReturnsInvalidState`).
- One callback calls `ovphysx_unsubscribe_object_changes` for a second
  subscription, then the test thread unsubscribes that second id
  (`UnsubscribeFromOtherCallbackReturnsInvalidState`).
- An experimental `ObjectChangeSubscription` is move-assigned away from inside
  its own all-objects callback, then the stage is reattached and stepped before
  another detach (`CppSubscriptionDestroyedFromCallbackStopsDelivery`).
- `ovphysx_detach_ovstage` runs on a Carbonite task whose all-objects callback
  suspends on a task mutex. Other tasks unsubscribe an unknown id, which
  stops at the callback guard
  (`CallbackGuardFollowsCarboniteTaskNotWorker`).
- An experimental `ObjectChangeSubscription` is move-assigned away from inside
  its all-objects callback, which then throws, then the stage is reattached and
  stepped before another detach
  (`CppSubscriptionReleasesCaptureWhenCallbackThrows`).

## Then

- The cross-thread unsubscribe returns `OVPHYSX_API_SUCCESS` only after the
  callback has stored its completion flag and marker (REQ AC-1). The test joins
  the unsubscriber thread before it reads that thread's status and observed
  marker, joins it on every exit, and bounds how long it waits to observe
  callback entry.
- Each in-callback unsubscribe returns `OVPHYSX_API_INVALID_STATE` and leaves
  that subscription active. The following unsubscribe of the same id, from
  outside the callback, returns `OVPHYSX_API_SUCCESS` (REQ AC-2).
- After the C++ handle is move-assigned away inside its callback, a later detach
  of a reattached stage does not invoke that callback again, and its captured
  state has been released (REQ AC-3).
- While the callback task is suspended, unsubscribes of an unknown id from
  other tasks return `OVPHYSX_API_NOT_FOUND`, never
  `OVPHYSX_API_INVALID_STATE`. The callback's own unsubscribe, after the task
  resumes, still returns `OVPHYSX_API_INVALID_STATE` (REQ AC-2).
- After the throwing callback unwinds, the closure's captured `shared_ptr` has
  been released and a later detach of a reattached stage does not invoke that
  callback again (REQ AC-3).
