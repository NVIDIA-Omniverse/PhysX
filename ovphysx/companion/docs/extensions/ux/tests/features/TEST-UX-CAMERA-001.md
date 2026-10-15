<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-CAMERA-001
maps_to: REQ-UX-CAMERA-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Author camera behavior with undo while runtime support is absent. Verifies REQ-UX-CAMERA-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A valid body, existing camera optics and no runtime camera service.

## When

- Create Follow Look/Follow Velocity/Drone cameras in separate cases, edit behavior properties and undo/redo; attempt invalid subject and live enablement.

## Then

- Supported creation uses unique paths and one undo action; invalid subjects write nothing and existing optics remain exact. (REQ-UX-CAMERA-001 AC-1).
- Fields bind only to the selected camera API and remain inspectable/editable while runtime support shows unavailable. (REQ-UX-CAMERA-001 AC-2).
- No private update driver or stopped simulation is created. (REQ-UX-CAMERA-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-CAMERA-001](../../requirements/features/REQ-UX-CAMERA-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.
