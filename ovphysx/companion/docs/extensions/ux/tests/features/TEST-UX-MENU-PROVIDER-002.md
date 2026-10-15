<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MENU-PROVIDER-002
maps_to: REQ-UX-MENU-PROVIDER-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Reject missing services and invalid creation targets. Verifies REQ-UX-MENU-PROVIDER-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Separate cases with absent authoring service, invalid attachment endpoint and deleted tire-material target.

## When

- Request the corresponding specialized creation action.

## Then

- Missing particle creation support displays unavailable without fallback. (REQ-UX-MENU-PROVIDER-001 AC-1).
- Invalid/unsupported deformable work leaves layers unchanged and reports why. (REQ-UX-MENU-PROVIDER-001 AC-2).
- Vehicle/table requests with invalid targets or missing capability leave unrelated materials and history unchanged. (REQ-UX-MENU-PROVIDER-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MENU-PROVIDER-001](../../requirements/features/REQ-UX-MENU-PROVIDER-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

