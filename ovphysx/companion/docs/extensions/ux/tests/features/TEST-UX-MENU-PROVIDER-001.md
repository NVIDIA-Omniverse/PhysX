<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MENU-PROVIDER-001
maps_to: REQ-UX-MENU-PROVIDER-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Route specialized creation without a simulation instance. Verifies REQ-UX-MENU-PROVIDER-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A stopped stage with valid meshes, attachment endpoints and materials; explicit particle/deformable/vehicle authoring services, each with a call recorder.

## When

- Parameterize particle-system creation, deformable surface/volume/attachment creation, vehicle wizard and tire-friction-table creation; complete supported jobs and undo.

## Then

- Particle creation calls only its selected schema/command provider and authors its API. (REQ-UX-MENU-PROVIDER-001 AC-1).
- Deformable operations validate endpoints, report the operation outcome and author only the requested data. (REQ-UX-MENU-PROVIDER-001 AC-2).
- Vehicle creation opens the owning wizard; table creation retains the selected material relationships and undoes independently. None of these actions creates or steps a runtime session. (REQ-UX-MENU-PROVIDER-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MENU-PROVIDER-001](../../requirements/features/REQ-UX-MENU-PROVIDER-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.


