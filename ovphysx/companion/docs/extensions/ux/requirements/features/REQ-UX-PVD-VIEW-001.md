<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-PVD-VIEW-001
title: Inspect recording data without advancing live simulation
status: proposed
owner: physics-ux
---

## Description

Inspect recording data without advancing live simulation. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Playback identity.** Import/playback shall keep a recording identity/frame cursor distinct from live handles, preserve source files and operate with no live ovphysx session.
- AC-2: **Frame views.** Recorded-prim properties and Contact/CoM/Joints/Velocity views shall all refresh from the selected recording/frame and invalidate stale selection data on seek or recording change.
- AC-3: **Viewer cleanup.** Close shall unregister recording property/viewport contributions and subscriptions without affecting live or other recording ownership.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-PVD-VIEW-001](../../tests/features/TEST-UX-PVD-VIEW-001.md) — Seek recorded data with no live physics backend.

## Code References

- [ovd](../../../../../python/ovphysx_companion/ovd.py) — existing companion substrate; host integration remains proposed.
- [usd/ovd](../../../../../python/ovphysx_companion/usd/ovd.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0016](../../adr/ADR-0016-omnipvd.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
