<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-TOOL-SELECTION-001
maps_to: REQ-UX-TOOL-SELECTION-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Select rigid ancestors and restore selection hooks. Verifies REQ-UX-TOOL-SELECTION-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Nested rigid parent/mesh child, a nonrigid sibling, viewport/Stage/Layers selections and pre-existing selection hooks.

## When

- Enable rigid-body selection, select descendants through each surface, change provider/settings, then disable and unload the mass contributor.

## Then

- Each surface resolves the same intended rigid ancestor; disable restores the original hooks. (REQ-UX-TOOL-SELECTION-001 AC-1).
- Toolbar applicability follows current selection, generic policy stays independent of cooking settings, and mass-contributor removal clears only its toggle/hooks. (REQ-UX-TOOL-SELECTION-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-TOOL-SELECTION-001](../../requirements/features/REQ-UX-TOOL-SELECTION-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

