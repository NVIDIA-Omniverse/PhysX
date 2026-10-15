<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-GRAPH-QUERY-002
maps_to: REQ-UX-GRAPH-QUERY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Reject stopped or stale queries without creating a session. Verifies REQ-UX-GRAPH-QUERY-001 AC-1, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A stopped remembered stage, an unsupported query mode and a delayed valid query as separate cases.

## When

- Evaluate stopped/missing modes; change query input/engine/stage before completing the delayed query.

## Then

- Absent session/mode reports unavailable with zero create/attach calls. (REQ-UX-GRAPH-QUERY-001 AC-1).
- Old-input/generation results cannot replace the current result; errors have explicit validity/status. (REQ-UX-GRAPH-QUERY-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-GRAPH-QUERY-001](../../requirements/features/REQ-UX-GRAPH-QUERY-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

