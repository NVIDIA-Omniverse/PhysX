<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-COLLIDER-003
maps_to: REQ-UX-COLLIDER-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Keep automatic collider generation opt-in and idempotent. Verifies REQ-UX-COLLIDER-001 AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Automatic generation initially disabled, one explicitly authored collider and one eligible new mesh.

## When

- Open the stage, enable auto generation, deliver duplicate notices, disable it and add another mesh.

## Then

- No work occurs before enablement; the eligible mesh is processed once, explicit collider settings are preserved, and the final mesh is not processed after disablement. (REQ-UX-COLLIDER-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-COLLIDER-001](../../requirements/features/REQ-UX-COLLIDER-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

