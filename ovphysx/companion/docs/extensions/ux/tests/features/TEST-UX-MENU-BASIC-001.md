<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MENU-BASIC-001
maps_to: REQ-UX-MENU-BASIC-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Create generic scene and ground with exact undo. Verifies REQ-UX-MENU-BASIC-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A writable empty stage parameterized over Y/Z up and metersPerUnit 1/0.01, unrelated sentinel prims and no engine.

## When

- In separate fixture cases create a scene or ground, repeat for unique naming, then undo/redo the last action.

## Then

- Scene creation produces distinct UsdPhysics.Scene prims without implicit PhysX APIs. (REQ-UX-MENU-BASIC-001 AC-1).
- Ground normal follows stage up and physical dimensions match unit conversion, with standard collision authoring. (REQ-UX-MENU-BASIC-001 AC-2).
- One undo removes only the second operation and redo restores its exact opinions; sentinels survive and no simulation backend is created. (REQ-UX-MENU-BASIC-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MENU-BASIC-001](../../requirements/features/REQ-UX-MENU-BASIC-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

