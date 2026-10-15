<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-DEBUG-DATA-001
maps_to: REQ-UX-DEBUG-DATA-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Draw completed diagnostic values in their declared frames. Verifies REQ-UX-DEBUG-DATA-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A rotated moving body with nonzero linear/angular velocity, separately known contact and friction quantities, and capability-tagged pruner/joint/cull/MBP/SDF/spline/suspension data.

## When

- Toggle each diagnostic mode, deliver one completed snapshot, remove a body, deliver an old snapshot and repeat a mode with its capability absent.

## Then

- Axes and velocities match the known frame/unit conversion; missing values draw no default-origin artifact. (REQ-UX-DEBUG-DATA-001 AC-1).
- Contact and friction points/normals/error/impulse retain their separate units and IDs; unavailable samples are not zero measurements. (REQ-UX-DEBUG-DATA-001 AC-2).
- Only advertised geometry diagnostics draw, removed-body data disappears and stale snapshots cannot restore it. (REQ-UX-DEBUG-DATA-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-DEBUG-DATA-001](../../requirements/features/REQ-UX-DEBUG-DATA-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

