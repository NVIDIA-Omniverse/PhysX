<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-GRAPH-QUERY-001
maps_to: REQ-UX-GRAPH-QUERY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Preserve query geometry and copied hit results. Verifies REQ-UX-GRAPH-QUERY-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A playing query provider with a near box, far sphere and instancer IDs; known world-space ray/sweep/overlap geometry and distinct hit distances.

## When

- Parameterize advertised raycast/overlap/sweep shapes and any/closest/all modes; query hit/no-hit inputs and recycle borrowed result arrays.

## Then

- Calls reach the explicitly selected provider/mode without force load; supported mode returns its documented result selection. (REQ-UX-GRAPH-QUERY-001 AC-1).
- Closest is the minimum-distance hit; all contains the expected hit set without assuming unspecified order; normals/distances/body/shape/instance IDs match geometry and survive borrowed-array reuse. (REQ-UX-GRAPH-QUERY-001 AC-2).
- A valid no-hit result is success with zero hits, distinct from unsupported/error. (REQ-UX-GRAPH-QUERY-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-GRAPH-QUERY-001](../../requirements/features/REQ-UX-GRAPH-QUERY-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

