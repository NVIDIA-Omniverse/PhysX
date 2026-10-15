<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-MENU-BASIC-001
title: Create standard scene and ground prims through undoable generic commands
status: proposed
owner: physics-ux
---

## Description

Create standard scene and ground prims through undoable generic commands. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Scene.** Create Physics Scene shall author a uniquely named UsdPhysics.Scene in the active writable edit target, respecting stage units/up axis and without implicitly applying engine APIs.
- AC-2: **Ground.** Create Ground Plane shall author correctly oriented geometry and standard collision data in stage units; an engine-specific representation shall require its advertised contribution.
- AC-3: **Undo.** Each creation shall be one undoable action; undo/redo shall restore exact authored opinions and unrelated prims. No-stage/read-only/inapplicable requests shall author nothing.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-MENU-BASIC-001](../../tests/features/TEST-UX-MENU-BASIC-001.md) — Create generic scene and ground with exact undo.
- [TEST-UX-MENU-BASIC-002](../../tests/features/TEST-UX-MENU-BASIC-002.md) — Reject creation without a writable stage.

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [migration](../../../../../python/ovphysx_companion/migration.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0001](../../adr/ADR-0001-physics-menu.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
