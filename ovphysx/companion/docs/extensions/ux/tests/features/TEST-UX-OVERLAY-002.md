<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-OVERLAY-002
maps_to: REQ-UX-OVERLAY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Draw deformable and particle data from the active completed snapshot. Verifies REQ-UX-OVERLAY-001 AC-3, AC-4, AC-5.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A provider fixture with distinct default/bind/rest deformable positions and particle authored positions different from completed ovstage output; delayed output completion.

## When

- Parameterize supported deformable simulation default/bind/rest, collision default/bind and attachment modes; particle/fluid-surface/diffuse modes; publish then Stop and deliver an old snapshot.

## Then

- Each deformable mode uses its designated data; unsupported modes are explicitly unavailable. (REQ-UX-OVERLAY-001 AC-3).
- Particle render coordinates equal completed ovstage output, not the deliberately different USD values. (REQ-UX-OVERLAY-001 AC-4).
- No incomplete snapshot is rendered; borrowed data is copied/released before invalidation, and Stop removes old drawings without reviving them on late completion. (REQ-UX-OVERLAY-001 AC-5).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-OVERLAY-001](../../requirements/features/REQ-UX-OVERLAY-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.


