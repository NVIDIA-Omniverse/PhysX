<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-DEFORM-HIERARCHY-001
maps_to: REQ-UX-DEFORM-HIERARCHY-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Preserve deformable hierarchy and pose authoring, including failure, source replacement and exact undo/teardown.

## Given

A closed tetrahedral volume and a triangulated surface, each with distinct simulation/collision/visual meshes, an unrelated material, authored bind/rest poses and an invalid role assignment. Include optional simplification and hexahedral settings.

## When

Create surface and volume hierarchies, bake/recook, remove each declared component separately and undo/redo. Delay cooking and replace the input topology before completion; also run without the required schema.

## Then

- Each valid hierarchy has the specified roles, pose data and relationships; invalid roles or unavailable schemas leave the entire layer unchanged. (REQ AC-1).
- Returned copied geometry matches a standalone cook fixture with declared units; the native boundary receives arrays and IDs but no USD object. Python applies schemas to the requested layer. (REQ AC-2).
- Each removal preserves unrelated meshes/materials and undo restores the original hierarchy. Delayed stale results do not overwrite the replacement mesh. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-DEFORM-HIERARCHY-001](../../requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md).
