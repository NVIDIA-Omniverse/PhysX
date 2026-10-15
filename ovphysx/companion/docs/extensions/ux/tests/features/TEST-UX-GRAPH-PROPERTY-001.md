<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-GRAPH-PROPERTY-001
maps_to: REQ-UX-GRAPH-PROPERTY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Return query results only to the current body and generation. Verifies REQ-UX-GRAPH-PROPERTY-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Two bodies with different computed mass properties, an advertised query service and a missing-service variant.

## When

- Query A, delay completion, switch input to B and deliver A; complete B, then remove its node.

## Then

- B output contains its computed values with units/frames and status, distinct from authored overrides; absent capability is unavailable without force load. (REQ-UX-GRAPH-PROPERTY-001 AC-1).
- A cannot overwrite B, node removal invalidates its pending output and no old callback revives it. (REQ-UX-GRAPH-PROPERTY-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-GRAPH-PROPERTY-001](../../requirements/features/REQ-UX-GRAPH-PROPERTY-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.


