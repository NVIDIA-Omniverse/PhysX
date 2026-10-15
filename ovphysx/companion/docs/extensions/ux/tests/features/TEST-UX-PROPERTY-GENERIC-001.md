<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PROPERTY-GENERIC-001
maps_to: REQ-UX-PROPERTY-GENERIC-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Preserve generic physics property models and schema contributions, including failure, source replacement and exact undo/teardown.

## Given

Prims with inherited and authored rigid-body/mass properties, mixed collider approximations, two drive instances, an invalid simulation-owner relationship and a read-only edit layer; an external builder is registered beside the physics builders.

## When

Build and rebuild the panel, select mixed prims, reset an authored property, remove one drive instance and attempt an invalid relationship edit. Replace the provider and selection, then close/reopen the panel.

## Then

- Opening/rebuilding leaves the layer byte-equivalent; displayed default/inherited/mixed values match independent composed USD reads for each catalog schema. (REQ AC-1).
- Reset removes only the selected authored opinion; undo restores it. Removing drive X preserves drive Y. Invalid and read-only edits leave the layer and undo stack unchanged. (REQ AC-2).
- Widget ordering and aliases match the registry fixture; the external builder survives shutdown. Subscription counts return to baseline and old-selection callbacks cannot update the new model. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-PROPERTY-GENERIC-001](../../requirements/features/REQ-UX-PROPERTY-GENERIC-001.md).
