<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-GIZMO-002
maps_to: REQ-UX-GIZMO-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Pair live manipulation begin with exactly one end or cancellation. Verifies REQ-UX-GIZMO-001 AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A playing body and an explicitly advertised live force/constraint service; separate fixture with that capability absent.

## When

- Begin/update a drag with a normalized rotation; parameterize normal end, selection change, target deletion and provider replacement. Attempt the same action with no service.

## Then

- Supported calls retain owner/body ID, world pivot/delta/quaternion and locks; the absent-service case is unavailable without USD teleportation. (REQ-UX-GIZMO-001 AC-2).
- Every successful begin receives one end or cancel, releasing constraints before gizmo removal and restoring input ownership. (REQ-UX-GIZMO-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-GIZMO-001](../../requirements/features/REQ-UX-GIZMO-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

