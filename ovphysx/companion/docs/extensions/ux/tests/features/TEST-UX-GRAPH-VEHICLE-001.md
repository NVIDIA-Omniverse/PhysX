<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-GRAPH-VEHICLE-001
maps_to: REQ-UX-GRAPH-VEHICLE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Invalidate completed vehicle graph state on Stop and target removal. Verifies REQ-UX-GRAPH-VEHICLE-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Drive/Wheel nodes bound to a live supported vehicle with known completed wheel speed/drive state; invalid-path and absent-service variants.

## When

- Evaluate after completed output; Stop/remove vehicle/delete node and deliver a delayed prior-generation result.

## Then

- Supported outputs match the requested vehicle completed state; invalid path/stopped/absent capability reports invalid/unavailable rather than a zero vehicle. (REQ-UX-GRAPH-VEHICLE-001 AC-1).
- Teardown invalidates handles/results and old values never become current. (REQ-UX-GRAPH-VEHICLE-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-GRAPH-VEHICLE-001](../../requirements/features/REQ-UX-GRAPH-VEHICLE-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

