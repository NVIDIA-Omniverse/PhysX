<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-COLLISION-FILTER-001
maps_to: REQ-UX-COLLISION-FILTER-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Retain collision groups and pair-filter authoring, including failure, source replacement and exact undo/teardown.

## Given

Three groups G0/G1/G2, a collider collection using descendants plus an excluded child, a filtered-group relationship with a preexisting unrelated target, an inverted collection case and a selected instance proxy.

## When

Create a group with a colliding requested name, toggle G0/G1 in the matrix, add/remove a collider pair, undo/redo, attempt a batch with the proxy, then delete G1 and disable the panel.

## Then

- The new group has a unique path; matrix membership matches composed collection expansion and inversion for the named child cases. No native owner is created. (REQ AC-1).
- G0/G1 target changes match the specified relationship directions while G2 and preexisting targets are unchanged. Undo restores exact list operations; the proxy batch edits nothing. (REQ AC-2).
- Deleting G1 removes its row on the next notice without dereferencing it. Disable removes owned subscriptions while all authored filter opinions persist; replay uses normal stage input. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-COLLISION-FILTER-001](../../requirements/features/REQ-UX-COLLISION-FILTER-001.md).
