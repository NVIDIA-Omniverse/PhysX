<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-DEBUG-DATA-001
title: Render diagnostic quantities with their actual meaning and lifetime
status: proposed
owner: physics-ux
---

## Description

Render diagnostic quantities with their actual meaning and lifetime. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Frames and motion.** Body/world/mass/actor axes and linear/angular velocities shall use documented frames, units and completed state; missing values shall be unavailable rather than drawn at a default origin.
- AC-2: **Contacts and friction.** Contact points/normals/error/impulse and friction points/normals/impulse shall preserve their distinct quantities and identity/lifetime; no-data shall not masquerade as zero measurement.
- AC-3: **Engine geometry.** Static/dynamic pruners, joint frames/limits, cull box/MBP/SDF/spline velocity and suspension diagnostics shall be PhysX capability-gated and remove stale object data.
- AC-4: **Logging.** Robotics and Scene Multi-GPU log toggles shall modify only the selected PhysX channels and not another provider's logging.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-DEBUG-DATA-001](../../tests/features/TEST-UX-DEBUG-DATA-001.md) — Draw completed diagnostic values in their declared frames.
- [TEST-UX-DEBUG-DATA-002](../../tests/features/TEST-UX-DEBUG-DATA-002.md) — Scope logging toggles to PhysX.

## Code References

- [debug](../../../../../python/ovphysx_companion/debug.py) — existing companion substrate; host integration remains proposed.
- [inspection](../../../../../python/ovphysx_companion/inspection.py) — existing companion substrate; host integration remains proposed.
- [events](../../../../../python/ovphysx_companion/events.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0008](../../adr/ADR-0008-debug-window.md).
- [REQ-RT-OUTPUT-001](../../../runtime/requirements/simulation/REQ-RT-OUTPUT-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
- [REQ-UX-OVERLAY-001](REQ-UX-OVERLAY-001.md).
