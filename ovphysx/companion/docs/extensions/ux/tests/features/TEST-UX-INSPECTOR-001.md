<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-INSPECTOR-001
maps_to: REQ-UX-INSPECTOR-001
type: integration
---

## Coverage status

Specification only — not implemented. Existing companion component tests verify the substrate, not this Kit scenario.

## Scenario

Edit articulation drive/state rows, explicitly simulate independent preview B, commit/discard poses and invalidate stale work while playback A is retained.

## Given

Two articulation joints (one revolute, one prismatic), authored drive limits/gains, two Inspector windows with different selection locks, playback A paused at a known pose and task generation, and a copied preview source B.

## When

Edit each automatic/state-position/drive-position/drive-velocity row and undo its drag. Enable preview B, step to a known changed pose, discard, then repeat and commit only the selected joint pose.

## Then

- Displayed rows bind the expected joint/DOF and distinguish angular and linear units. Rebuilding/windows do not author defaults; each drag has one undo item and exact undo restores limits/gains and absent opinions. (REQ AC-1).
- A retains its pose, epoch and pending tasks while B advances. Only PreviewDisplay receives transient output; discard restores its baseline. Commit changes only the selected fresh authored poses as one reversible patch. (REQ AC-2).

## Why not implemented

The migrated Kit feature and host integration fixture are not present. Component examples are integration starting points; they do not establish full mode, widget or command parity.

Mapped requirement: [REQ-UX-INSPECTOR-001](../../requirements/features/REQ-UX-INSPECTOR-001.md).
