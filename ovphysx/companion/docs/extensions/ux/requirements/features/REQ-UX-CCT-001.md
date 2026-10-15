<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-CCT-001
title: Gate character-controller behavior while preserving authoring
status: proposed
owner: physics-ux
---

## Description

Gate character-controller behavior while preserving authoring. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Property authoring.** Controller UI shall bind only to applicable controller API prims, validate movement/gravity/height settings and preserve undoable authored data; unrelated selection shall not acquire a controller.
- AC-2: **Runtime service.** When an explicit controller capability exists, UI/compatible OGN input shall use its scheduled phases and completed collision events, with gravity/first-person policy preserved and no independent physics step. Absent capability shall show unavailable.
- AC-3: **Ownership.** Disable, Stop or detach shall release keyboard/mouse capture, controller handles and temporary camera ownership; subsequent callbacks shall not target old generations.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-CCT-001](../../tests/features/TEST-UX-CCT-001.md) — Edit controller properties without acquiring live controls.
- [TEST-UX-CCT-002](../../tests/features/TEST-UX-CCT-002.md) — Consume scheduled controller state and release input ownership.

## Code References

- [cct](../../../../../python/ovphysx_companion/cct.py) — existing companion substrate; host integration remains proposed.
- [camera](../../../../../python/ovphysx_companion/camera.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0014](../../adr/ADR-0014-character-controller.md).
- [REQ-RT-DRIVER-001](../../../runtime/requirements/simulation/REQ-RT-DRIVER-001.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
