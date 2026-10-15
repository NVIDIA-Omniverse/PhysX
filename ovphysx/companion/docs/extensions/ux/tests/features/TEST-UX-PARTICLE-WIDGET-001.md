<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PARTICLE-WIDGET-001
maps_to: REQ-UX-PARTICLE-WIDGET-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Round-trip advanced particle offsets without panel side effects. Verifies REQ-UX-PARTICLE-WIDGET-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A PhysX particle-system API with distinct authored/fallback offsets, an unrelated prim and inactive-engine variant.

## When

- Open/rebuild/select unrelated prim; on the particle system edit contactOffset/restOffset/solidRestOffset/fluidRestOffset one at a time and undo/redo; submit nonfinite or schema-invalid combinations.

## Then

- The widget binds only to applicable selection; inspection preserves defaults/authored status and creates no particles. (REQ-UX-PARTICLE-WIDGET-001 AC-1).
- All four fields appear under Advanced; valid edits round-trip in stage units without changing other offsets. (REQ-UX-PARTICLE-WIDGET-001 AC-2).
- Invalid combinations show the provider/schema validation reason and leave opinions/history unchanged; inactive values are never applied to runtime. (REQ-UX-PARTICLE-WIDGET-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-PARTICLE-WIDGET-001](../../requirements/features/REQ-UX-PARTICLE-WIDGET-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

