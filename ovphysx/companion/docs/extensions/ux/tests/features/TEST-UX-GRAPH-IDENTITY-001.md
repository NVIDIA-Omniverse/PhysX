<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-GRAPH-IDENTITY-001
maps_to: REQ-UX-GRAPH-IDENTITY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Reopen saved graph nodes without silent engine substitution. Verifies REQ-UX-GRAPH-IDENTITY-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Saved graphs using legacy physics node identifiers, an explicit compatible provider and a missing-provider variant.

## When

- Load graphs through the generic graph extension without the legacy PhysX graph extension; evaluate, change engine and delete nodes.

## Then

- Each identifier resolves unchanged or through a declared versioned migration; unavailable PhysX behavior is reported rather than silently routed to another engine. (REQ-UX-GRAPH-IDENTITY-001 AC-1).
- Generic components load independently; outputs/subscriptions carry engine/stage scope and invalidate on engine change or deletion. (REQ-UX-GRAPH-IDENTITY-001 AC-2).

- **Complete node inventory.** Every legacy graph/CCT descriptor shall map its serialized type/version and exact input/output pin names, data types, array/role/default metadata and execution behavior to a retained adapter or explicit migration error. Compare saved graphs and outputs, not only registration counts. (REQ AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-GRAPH-IDENTITY-001](../../requirements/features/REQ-UX-GRAPH-IDENTITY-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.


