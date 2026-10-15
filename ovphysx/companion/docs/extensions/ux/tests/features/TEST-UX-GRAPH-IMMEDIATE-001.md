<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-GRAPH-IMMEDIATE-001
maps_to: REQ-UX-GRAPH-IMMEDIATE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Compute immediate geometry results without a scene session. Verifies REQ-UX-GRAPH-IMMEDIATE-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Explicit intersecting/disjoint triangle meshes and compound geometry, no loaded scene session and an advertised immediate-geometry service.

## When

- Run supported immediate intersection/contact operations, overwrite input buffers after completion, then exercise malformed input and absent capability.

## Then

- Valid operations use only supplied geometry, produce the expected intersecting/disjoint/contact result and make no runtime attach/step calls. (REQ-UX-GRAPH-IMMEDIATE-001 AC-1).
- Results have owned valid storage; invalid/unsupported cases are explicit, and generated contacts do not emit simulation-contact events. (REQ-UX-GRAPH-IMMEDIATE-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-GRAPH-IMMEDIATE-001](../../requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.


