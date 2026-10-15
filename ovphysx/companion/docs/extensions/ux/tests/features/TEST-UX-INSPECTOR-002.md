<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-INSPECTOR-002
maps_to: REQ-UX-INSPECTOR-001
type: integration
---

## Coverage status

Specification only — not implemented. Existing companion component tests verify the substrate, not this Kit scenario.

## Scenario

Edit articulation drive/state rows, explicitly simulate independent preview B, commit/discard poses and invalidate stale work while playback A is retained.

## Given

A retained playback A, isolated articulation preview B with nonzero initial pose/velocity, gravity on/off, fixed/free base and a separately declared quasi-static policy with an analytic equilibrium fixture. Hold a preview completion at a barrier.

## When

For each policy reset B to the identical baseline and step to its fixture bound. Repeat after undo, Play, changed selection/topology, stage close and provider unload; deliver the held obsolete result after invalidation.

## Then

- Old-generation output and commits are rejected; B jobs/resources and owned display opinions retire without collecting A tasks or changing its source. A can continue stepping after B teardown. (REQ AC-3).
- Gravity-off has no gravitational acceleration; gravity-on follows the fixture acceleration tolerance. A fixed base stays fixed while the free-base control moves. Quasi-static reaches the independently specified equilibrium tolerance and reset restores baseline. An absent policy is reported as an unexecuted parity case, never a positive pass. (REQ AC-4).

## Why not implemented

The migrated Kit feature and host integration fixture are not present. Component examples are integration starting points; they do not establish full mode, widget or command parity.

Mapped requirement: [REQ-UX-INSPECTOR-001](../../requirements/features/REQ-UX-INSPECTOR-001.md).
