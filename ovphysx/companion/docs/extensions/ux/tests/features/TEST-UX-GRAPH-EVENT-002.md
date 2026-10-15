<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-GRAPH-EVENT-002
maps_to: REQ-UX-GRAPH-EVENT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Report unavailable subscriptions and discard late events. Verifies REQ-UX-GRAPH-EVENT-001 AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A missing generic-trigger-capability fixture and a supported subscription with queued events.

## When

- Evaluate missing trigger capability; for the supported case Stop/delete node/change engine or stage, then deliver queued events.

## Then

- Missing trigger support is explicitly unavailable, distinguishable from an active stream with no events. (REQ-UX-GRAPH-EVENT-001 AC-2).
- Teardown unsubscribes once, clears validity and rejects every late event for the old subscription. (REQ-UX-GRAPH-EVENT-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-GRAPH-EVENT-001](../../requirements/features/REQ-UX-GRAPH-EVENT-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

