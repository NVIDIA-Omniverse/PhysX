<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-OVERLAY-001
maps_to: REQ-UX-OVERLAY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Filter actual collision and joint representations. Verifies REQ-UX-OVERLAY-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Two selected/unselected bodies whose render mesh differs from cooked collision geometry, two tendon instances and generic joint-limit overlays; an advertised geometry snapshot provider.

## When

- Set None/Selected/All, toggle normals, move camera across the distance threshold and toggle joint/tendon drawings.

## Then

- Visible body IDs match the filter; normals and AABB distance simplification use collision geometry. A schema fallback is labeled, and missing geometry is unavailable. (REQ-UX-OVERLAY-001 AC-1).
- Each joint limit is drawn by its single owner; tendon selection draws the requested instance without duplicating the generic joint overlay. (REQ-UX-OVERLAY-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-OVERLAY-001](../../requirements/features/REQ-UX-OVERLAY-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

