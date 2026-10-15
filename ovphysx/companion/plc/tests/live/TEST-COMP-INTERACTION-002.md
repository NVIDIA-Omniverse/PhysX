<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-INTERACTION-002
maps_to: REQ-COMP-INTERACTION-001
type: integration
---

## Coverage status

Executed on Linux x86_64: the six native interaction cases pass 430 assertions across USDChange/OvstageChange, and all eight Python interaction cases pass against the installed SDK. Covers AC-5/6 of [REQ-COMP-INTERACTION-001](../../requirements/live/REQ-COMP-INTERACTION-001.md).

## Scenario

Pick and hold a body with legacy force grabbing, then cancel or retire its owner.

## Given

- CPU bodies with known mass/inertia, COM offset, linear/angular velocity and damping; gravity-enabled and disabled scenes.
- Two point instances, independent owners, a ray hit and a retained-acceleration actor.

## When

- Begin the default force grab, update targets/rays and advance actual substeps with different h.
- Try joint/force duplicates, invalid/busy/foreign commands, release without a step, delete/recreate and detach.

## Then

- Zero and live coefficient changes preserve legacy gravity/damping behavior, invalid/busy tuning preserves state, and a zero-substep host update applies no impulse (REQ AC-5/6).
- Measured linear/angular response matches the independent legacy impulse and gravity oracle; angular damping restores and no constraint is created (REQ AC-5).
- Default Python grabbing is force mode; explicitly selected joint mode retains its original behavior (REQ AC-5).
- Normalized rays retain the original distance and exact instance; only that owner moves and failed commands preserve state/output (REQ AC-6).
- Release cancels immediately, retained accelerations do not preserve grab impulses, and retirement reports inactive without touching replacements (REQ AC-6).

## Executable coverage

- [Force-picking integration](../../../tests_native/test_interaction.py).
- Python cases: `test_force_grab_default_pick_and_ray_updates`, `test_force_picking_abi_preserves_rejected_outputs`, `test_force_grab_exact_point_instance_and_peer`, `test_force_picking_zero_and_live_tuning`.
