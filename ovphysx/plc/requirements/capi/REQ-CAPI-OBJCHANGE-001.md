<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OBJCHANGE-001
title: Object-Change Unsubscribe Drain
status: implemented
owner: ovphysx
---

## Description

`ovphysx_unsubscribe_object_changes` removes one instance-owned object-change
subscription and does not return `OVPHYSX_API_SUCCESS` while that subscription
can still invoke its callback. Instance destruction uses the same unsubscribe
path for subscriptions that instance still owns.

A dispatch snapshot may already hold the runtime callback when unsubscribe
runs. Success waits until that snapshot is gone and the user callback has
returned, so `user_data` is not touched again by this subscription.

Unsubscribe called from inside any object-change callback returns
`OVPHYSX_API_INVALID_STATE` without removing the subscription and without
waiting. The check follows the Carbonite task when the callback is running as
one, and the native thread otherwise. Another task on the same worker is not
inside the callback. The dispatch snapshot keeps every subscriber until that
callback returns, so waiting there would deadlock.

## Acceptance Criteria

- AC-1: `ovphysx_unsubscribe_object_changes` for a subscription whose callback
  is running on another thread does not return `OVPHYSX_API_SUCCESS` until that
  callback has returned. After that return, the callback's `user_data` has
  remained valid for the whole call.
- AC-2: `ovphysx_unsubscribe_object_changes` from inside any object-change
  callback returns `OVPHYSX_API_INVALID_STATE` and leaves the subscription
  active, whether or not it is the subscription whose callback is running.
  The guard follows the Carbonite task, so a different task on the worker
  that the callback suspended is not inside the callback. When the callback
  is not a Carbonite task, the guard stays on the native thread. A later
  unsubscribe of that same id from outside the callback returns
  `OVPHYSX_API_SUCCESS`.
- AC-3: Destroying or move-assigning an experimental `ObjectChangeSubscription`
  from inside an object-change callback on that Carbonite task, or native
  thread, does not wait. That subscription receives no later callback, and
  its callback state is freed after that call has unwound, including when the
  callback throws.

## Test References

- [TEST-CAPI-OBJCHANGE-001](../../tests/capi/TEST-CAPI-OBJCHANGE-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_unsubscribe_object_changes`)
- ovphysx/src/ovphysx/ovphysxObjectChangeCallbacks.cpp (`unsubscribeOwned`, `invokeUser`)
- ovphysx/src/ovphysxInternal/ovphysxInternalObjectChange.cpp (`ovphysx_internal_subscribe_object_changes`)
- ovphysx/src/include/internal/sidecar/ovphysxInternalObjectChange.h (`OvphysxObjectChangeKeep`)
- ovphysx/include/ovphysx/experimental/ovphysx.hpp (`ObjectChangeSubscription::unsubscribe`)
- ovphysx/tests/c_unittests/test_object_change_callbacks.cpp (`UnsubscribeWaitsForInFlightCallback`, `UnsubscribeFromOwnCallbackReturnsInvalidState`, `UnsubscribeFromOtherCallbackReturnsInvalidState`, `CallbackGuardFollowsCarboniteTaskNotWorker`, `CppSubscriptionDestroyedFromCallbackStopsDelivery`, `CppSubscriptionReleasesCaptureWhenCallbackThrows`)

## Dependencies

- REQ-CAPI-INSTANCE-SCOPE-001
- ADR-0038
- ADR-0047
