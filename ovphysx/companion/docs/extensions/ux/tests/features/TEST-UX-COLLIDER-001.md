<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-COLLIDER-001
maps_to: REQ-UX-COLLIDER-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Create an explicit collider batch as one undo action. Verifies REQ-UX-COLLIDER-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Writable meshes with a sentinel existing collider, invalid selected prim and explicit cooking support; runtime counters zero.

## When

- Parameterize static Triangle Mesh/Mesh Simplification and dynamic Convex Hull/Decomposition/SDF actions, complete the batch, inspect per-prim results, then undo/redo.

## Then

- Valid targets receive CollisionAPI and only the chosen approximation/rigid-body APIs; invalid or unsupported targets have an explicit failure and unchanged data. One undo restores the exact pre-batch layer. (REQ-UX-COLLIDER-001 AC-1).
- The batch has a stable operation ID and progress/error; conflicting edits are blocked and no simulation session is created. (REQ-UX-COLLIDER-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-COLLIDER-001](../../requirements/features/REQ-UX-COLLIDER-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

