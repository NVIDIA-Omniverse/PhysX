<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-GRAPH-VEHICLE-001
title: Read vehicle graph state only from a valid completed session
status: proposed
owner: physics-ux
---

## Description

Read vehicle graph state only from a valid completed session. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Vehicle output.** Drive/Wheel nodes shall report provider-owned completed vehicle state, scoped to the requested vehicle and generation; invalid path, stopped session or absent capability shall report invalid/unavailable, never a fabricated zero state.
- AC-2: **Lifecycle.** Node deletion, vehicle removal and Stop/detach shall invalidate stored handles/results and prevent late values from a previous session.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-GRAPH-VEHICLE-001](../../tests/features/TEST-UX-GRAPH-VEHICLE-001.md) — Invalidate completed vehicle graph state on Stop and target removal.

## Code References

- [vehicle](../../../../../python/ovphysx_companion/vehicle.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0019](../../adr/ADR-0019-omnigraph-nodes.md).
- [REQ-RT-OUTPUT-001](../../../runtime/requirements/simulation/REQ-RT-OUTPUT-001.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-UX-GRAPH-IDENTITY-001](REQ-UX-GRAPH-IDENTITY-001.md).
- [REQ-UX-VEHICLE-001](REQ-UX-VEHICLE-001.md).
