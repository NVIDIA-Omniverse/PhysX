<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-MASS-001
title: Separate computed mass display from undoable authored overrides
status: proposed
owner: physics-ux
---

## Description

Separate computed mass display from undoable authored overrides. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Query identity.** Computed mass/COM/inertia shall come only from an explicit supported query and be labeled separately from MassAPI authored values; results/errors shall target the current selected body, engine and stage, with correct display frames/units.
- AC-2: **Editing.** Mass/COM/diagonal inertia/principal-axis edits shall reject nonfinite or schema-invalid values before writing, group a drag into one undo command and preserve unrelated properties.
- AC-3: **Unavailable and cleanup.** Missing computed-query capability shall leave authored inspection/editing available without force load. Selection deletion, stage change or provider removal shall discard pending results and remove manipulators/helpers.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-MASS-001](../../tests/features/TEST-UX-MASS-001.md) — Show computed mass separately and undo authored overrides.
- [TEST-UX-MASS-002](../../tests/features/TEST-UX-MASS-002.md) — Retain authoring when computed data is unavailable or stale.

## Code References

- [mass](../../../../../python/ovphysx_companion/mass.py) — existing companion substrate; host integration remains proposed.
- [usd/mass](../../../../../python/ovphysx_companion/usd/mass.py) — existing companion substrate; host integration remains proposed.
- [jobs](../../../../../python/ovphysx_companion/jobs.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0003](../../adr/ADR-0003-debug-visualization.md).
- [ADR-0008](../../adr/ADR-0008-debug-window.md).
- [ADR-0013](../../adr/ADR-0013-mass-manipulator.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
