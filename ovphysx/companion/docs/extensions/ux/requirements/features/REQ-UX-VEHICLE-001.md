<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-VEHICLE-001
title: Separate vehicle schema authoring from optional live controls
status: proposed
owner: physics-ux
---

## Description

Separate vehicle schema authoring from optional live controls. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Authoring.** Wizard/friction-table creation shall validate targets and undo exactly; vehicle/wheel/tire schema widgets shall have one PhysX registry owner and remain inspectable/undoable without live vehicle simulation.
- AC-2: **Live state.** Input enablement, mouse/auto-reverse and steering controls shall use an advertised vehicle service, with pre-step control and completed wheel/drive results; unsupported runtime tools shall be unavailable rather than create another update schedule.
- AC-3: **Cleanup.** Stop, provider removal or invalid/deleted vehicle shall release input and stale state, remove contributed menus/widgets and never direct a completion to another vehicle.

- AC-4: **Complete authoring.** Retain wizard geometry/axes/unit choices, wheel/suspension/tire/friction-table authoring, drive modes, nonlinear response tables and multi-wheel targeting. Suspension data helpers alone do not satisfy the wizard; every committed group must undo to exact prior opinions.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-VEHICLE-001](../../tests/features/TEST-UX-VEHICLE-001.md) — Keep vehicle authoring usable without live vehicle service.
- [TEST-UX-VEHICLE-002](../../tests/features/TEST-UX-VEHICLE-002.md) — Bind input and wheel results to the active vehicle generation.

## Code References

- [vehicle](../../../../../python/ovphysx_companion/vehicle.py) — existing companion substrate; host integration remains proposed.
- [usd/vehicle](../../../../../python/ovphysx_companion/usd/vehicle.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0018](../../adr/ADR-0018-physx-vehicle.md).
- [REQ-RT-DRIVER-001](../../../runtime/requirements/simulation/REQ-RT-DRIVER-001.md).
- [REQ-RT-OUTPUT-001](../../../runtime/requirements/simulation/REQ-RT-OUTPUT-001.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
