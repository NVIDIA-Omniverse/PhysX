<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-CAMERA-001
title: Author camera modes and follow only completed supported state
status: proposed
owner: physics-ux
---

## Description

Author camera modes and follow only completed supported state. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Creation.** Follow Look/Follow Velocity/Drone actions shall validate a supported subject, create a unique camera through undoable authoring and preserve unrelated camera optics.
- AC-2: **Properties.** Widgets shall bind to the selected PhysX camera API, edit only its behavior and show unsupported runtime service separately from inspectable authored lens/behavior values.
- AC-3: **Controller lifecycle.** When advertised, a camera service shall use completed state through the sole simulation schedule, freeze with Pause, follow mandatory Stop restoration and restore temporary active-camera/input ownership on target deletion/disable/detach; absent capability shall not install another driver.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-CAMERA-001](../../tests/features/TEST-UX-CAMERA-001.md) — Author camera behavior with undo while runtime support is absent.
- [TEST-UX-CAMERA-002](../../tests/features/TEST-UX-CAMERA-002.md) — Follow completed state and restore temporary camera ownership.

## Code References

- [camera](../../../../../python/ovphysx_companion/camera.py) — existing companion substrate; host integration remains proposed.
- [usd/camera](../../../../../python/ovphysx_companion/usd/camera.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0017](../../adr/ADR-0017-physx-camera.md).
- [REQ-RT-DRIVER-001](../../../runtime/requirements/simulation/REQ-RT-DRIVER-001.md).
- [REQ-RT-OUTPUT-001](../../../runtime/requirements/simulation/REQ-RT-OUTPUT-001.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
