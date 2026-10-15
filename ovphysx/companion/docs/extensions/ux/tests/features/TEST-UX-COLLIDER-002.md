<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-COLLIDER-002
maps_to: REQ-UX-COLLIDER-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Discard an old-stage cooking completion. Verifies REQ-UX-COLLIDER-001 AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A batch for stage A paused before completion and a new writable stage B with identical prim paths.

## When

- Replace A with B, complete A cooking and inspect both authoring/undo ownership.

## Then

- A completion neither authors B nor creates a B undo entry; A operation is cancelled/retired and its progress is removed. (REQ-UX-COLLIDER-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-COLLIDER-001](../../requirements/features/REQ-UX-COLLIDER-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

