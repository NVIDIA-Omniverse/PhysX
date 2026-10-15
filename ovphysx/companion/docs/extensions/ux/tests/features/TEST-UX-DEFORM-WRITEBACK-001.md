<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-DEFORM-WRITEBACK-001
maps_to: REQ-UX-DEFORM-WRITEBACK-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Publish completed deformable results through customer USD, including failure, source replacement and exact undo/teardown.

## Given

One tetrahedron with analytically chosen barycentric weights and a distinct visual mesh, a nonidentity parent transform, centimeter units and nonzero velocities. Owners A/B use different generations and steps. Prepare a CPU record and an equivalent delayed device readback.

## When

Read the completed selected step, write visual positions/velocities at time 2.5 to a dedicated bake layer, undo, then repeat with a mismatched owner/step/topology and a closed destination. Hold the device fence before Python access.

## Then

- Python receives owned CPU data only after completion/fence synchronization; mutating/releasing the producer cannot change the copied result. Unsupported device extraction is reported unexecuted, not passing parity. (REQ AC-1).
- Independently multiply weights and transform/unit conversions to compare every visual point/velocity at 1e-5. Mismatched domains, owners, steps and topology are rejected before writing. (REQ AC-2).
- Only the bake layer at time 2.5 changes; simulation output produces no authored-input feedback. Undo/cancel restores its prior opinions and leaves unrelated layers and owner B untouched. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-DEFORM-WRITEBACK-001](../../requirements/features/REQ-UX-DEFORM-WRITEBACK-001.md).
