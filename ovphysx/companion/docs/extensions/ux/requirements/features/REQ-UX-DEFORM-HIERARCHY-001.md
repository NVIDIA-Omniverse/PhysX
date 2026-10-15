<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-DEFORM-HIERARCHY-001
title: Preserve deformable hierarchy and pose authoring
status: proposed
owner: physics-ux
---

## Description

Preserve deformable hierarchy and pose authoring. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Roles.** Surface/volume and auto-hierarchy dialogs shall retain separate simulation, collision and visual meshes, simplification and hexahedral options, default/bind/rest pose data, material and simulation-owner relationships. Invalid role combinations and missing schema support fail before mutation.
- AC-2: **Cook and author.** Capture topology/units/source revisions in Python, use companion native cooking/embedding on copied buffers, and author returned geometry/schemas through usd.deformables. Native C++ never receives a USD stage or authors schema opinions.
- AC-3: **Remove and restore.** Add/remove body, surface/volume simulation, pose, auto-body/hexahedral and simplification components shall preserve unrelated meshes/APIs. Bake/recook/removal is explicitly initiated and grouped for exact undo; stale/cancelled results never replace newer geometry.

## Test References

Specification only — not implemented.

- [TEST-UX-DEFORM-HIERARCHY-001](../../tests/features/TEST-UX-DEFORM-HIERARCHY-001.md)

## Code References

- [deformable_cooking](../../../../../python/ovphysx_companion/deformable_cooking.py) — existing companion substrate; host integration remains proposed.
- [usd/deformables](../../../../../python/ovphysx_companion/usd/deformables.py) — existing companion substrate; host integration remains proposed.
- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0021-deformable-properties](../../adr/ADR-0021-deformable-properties.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
