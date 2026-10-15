<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-INSTANCE-SCOPE-002
maps_to: REQ-CAPI-INSTANCE-SCOPE-001
type: integration
---

## Scenario

An instance subscription survives the retirement and replacement of its attached stage.

## Given

- An initialized instance without an attached stage.
- An all-objects-destroyed subscription registered on that instance.

## When

- Attach, step, and detach a scene twice without recreating the subscription.
- Unsubscribe with the original ID, then attach, step, and detach again.

## Then

- Each of the first two detach operations delivers an all-destroyed notification
  to the original subscription (REQ AC-4).
- Unsubscribe with the original ID succeeds; the final detach delivers no further
  callback (REQ AC-4).
- The same instance remains reusable after each detach (REQ AC-6).

Implemented by `ovphysx/tests/c_unittests/test_object_change_callbacks.cpp`,
`PhysXTestFixture.SubscriptionSurvivesDetachAndReattach`.
