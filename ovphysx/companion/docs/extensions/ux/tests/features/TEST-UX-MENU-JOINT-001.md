<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MENU-JOINT-001
maps_to: REQ-UX-MENU-JOINT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Preserve joint defaults and body-local frames. Verifies REQ-UX-MENU-JOINT-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Two rigid bodies with translated/rotated parents in a stage with explicit length units; no backend; a provider advertises gear/rack authoring only in the supported fixture.

## When

- Create D6/fixed/revolute/prismatic/spherical/distance and advertised specialized joints at a known world pivot; invoke compatible saved command names; undo/redo.

## Then

- Standard types work with no engine; gear/rack are offered only in the advertised fixture. (REQ-UX-MENU-JOINT-001 AC-1).
- Body relationships identify the intended bodies; composing each local joint frame with its body transform recovers the requested world pivot. Defaults match the schema and legacy command cases: no inappropriate limits on fixed/distance, six free D6 axes and angle values in degrees. One undo restores the prior layer. (REQ-UX-MENU-JOINT-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MENU-JOINT-001](../../requirements/features/REQ-UX-MENU-JOINT-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

