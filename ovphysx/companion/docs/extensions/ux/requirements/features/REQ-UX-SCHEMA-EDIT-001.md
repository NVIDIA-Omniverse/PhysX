<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-SCHEMA-EDIT-001
title: Apply presets and clear only the selected physics scope
status: proposed
owner: physics-ux
---

## Description

Apply presets and clear only the selected physics scope. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Preset scope.** Apply/remove presets shall validate current selection, show applicable provider contributions, and author/remove only the intended APIs; another provider's APIs shall survive.
- AC-2: **Clear scope.** Clear Physics and selected PhysicsSchema/PhysxSchema removal shall identify selection/subtree scope and perform the requested generic/provider removal as one undo group, preserving geometry and unrelated APIs.
- AC-3: **Composed undo.** Undo/redo shall restore exact composed opinions, including referenced and multiple-apply API instances; stale or read-only targets shall not be partially edited.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-SCHEMA-EDIT-001](../../tests/features/TEST-UX-SCHEMA-EDIT-001.md) — Apply or clear an explicitly scoped physics preset with exact undo.

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [migration](../../../../../python/ovphysx_companion/migration.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0001](../../adr/ADR-0001-physics-menu.md).
- [ADR-0008](../../adr/ADR-0008-debug-window.md).
- [ADR-0010](../../adr/ADR-0010-authoring-toolbar.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
