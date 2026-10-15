<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-VEHICLE-002
maps_to: REQ-UX-VEHICLE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Bind input and wheel results to the active vehicle generation. Verifies REQ-UX-VEHICLE-001 AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A live supported vehicle service with mouse/auto-reverse input policies and two vehicles with distinct IDs.

## When

- Enable input, deliver steering/throttle before the normal step and wheel/drive data after completion; delete the target or Stop/unload before a delayed result.

## Then

- Control and completed-state timing matches the sole runtime schedule and output is associated with the requested vehicle. (REQ-UX-VEHICLE-001 AC-2).
- Input and stale state are released, owned contributions removed on unload, and delayed results cannot target the other vehicle. (REQ-UX-VEHICLE-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-VEHICLE-001](../../requirements/features/REQ-UX-VEHICLE-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.
