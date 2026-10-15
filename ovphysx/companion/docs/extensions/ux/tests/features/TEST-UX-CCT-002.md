<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-CCT-002
maps_to: REQ-UX-CCT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Consume scheduled controller state and release input ownership. Verifies REQ-UX-CCT-001 AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- An advertised controller service, a playing capsule above a plane and a saved active camera/input owner.

## When

- Enable first-person control and gravity, send a movement input, finish a normal scheduled step with a collision-down event, then disable or Stop/detach; deliver an old event.

## Then

- Inputs are delivered through the declared schedule and collision-down feedback matches the completed session; step count equals normal runtime updates only. (REQ-UX-CCT-001 AC-2).
- Controller handles and keyboard/mouse capture are released, the prior camera/input owner restored and the late callback ignored. (REQ-UX-CCT-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-CCT-001](../../requirements/features/REQ-UX-CCT-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

