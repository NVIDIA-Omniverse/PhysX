<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-EDITOR-JOINT-001
maps_to: REQ-UX-EDITOR-JOINT-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Preserve editor joint registration independently of solver plugins, including failure, source replacement and exact undo/teardown.

## Given

Two transformed rigid bodies, a joint with nonzero local frames, a second joint selected for a batch action, an external glyph provider and no native custom-joint plugin.

## When

Register glyph/info/manipulator contributions; create, swap bodies, align and edit attachment position/orientation, undo each, delete a target and unregister. Separately load a native plugin without editor registration.

## Then

- Editor glyphs and panels appear once without a solver plugin; unregister removes only owned resources and leaves the external provider visible. (REQ AC-1).
- Independently compose body and local transforms to check world anchors/orientations within 1e-5 (quaternion sign invariant). Undo restores exact opinions and deleted targets invalidate manipulation. (REQ AC-2).
- Native plugin loading alone adds no editor contribution. Python callback registration adds no solver/contact callback and retains no SDK pointer. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-EDITOR-JOINT-001](../../requirements/features/REQ-UX-EDITOR-JOINT-001.md).
