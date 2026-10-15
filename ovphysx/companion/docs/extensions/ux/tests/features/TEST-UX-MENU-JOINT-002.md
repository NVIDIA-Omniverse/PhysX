<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MENU-JOINT-002
maps_to: REQ-UX-MENU-JOINT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Reject invalid joint selection before authoring. Verifies REQ-UX-MENU-JOINT-001 AC-1.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A selection containing an invalid endpoint or the same body twice, and a provider without specialized-joint support.

## When

- Request a joint and request a gear/rack joint.

## Then

- Invalid endpoints and missing capability are reported; no joint or undo entry is created. (REQ-UX-MENU-JOINT-001 AC-1).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MENU-JOINT-001](../../requirements/features/REQ-UX-MENU-JOINT-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

