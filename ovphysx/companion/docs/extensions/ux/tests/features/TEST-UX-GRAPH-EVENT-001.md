<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-GRAPH-EVENT-001
maps_to: REQ-UX-GRAPH-EVENT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Deliver contact and trigger lifecycle events with owned data. Verifies REQ-UX-GRAPH-EVENT-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A live provider advertising contact and trigger subscriptions, two identifiable colliders and an instanced collider; explicit watched-trigger and catch-all variants.

## When

- Move into contact/trigger, persist, then separate using normal runtime steps; overwrite borrowed provider event storage after delivery.

## Then

- Contact found/persist/lost retain correct actor/collider/instance identities after borrowed storage is reused. (REQ-UX-GRAPH-EVENT-001 AC-1).
- Trigger enter/leave follow the declared watched scope and completed order, with no duplicate events from another scope. (REQ-UX-GRAPH-EVENT-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-GRAPH-EVENT-001](../../requirements/features/REQ-UX-GRAPH-EVENT-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.


