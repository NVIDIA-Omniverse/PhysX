<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-MENU-MATERIAL-001
title: Create only selected supported material schemas
status: proposed
owner: physics-ux
---

## Description

Create only selected supported material schemas. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Supported selection.** The material dialog shall expose the standard rigid material and provider-declared deformable, surface-deformable and PBD particle types, rejecting unsupported combinations with a reason.
- AC-2: **Atomic commit.** Confirm shall create a unique material and exactly the selected supported APIs in one undo group; cancel/invalid input shall author nothing and undo shall remove only that action.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-MENU-MATERIAL-001](../../tests/features/TEST-UX-MENU-MATERIAL-001.md) — Commit only selected material APIs.

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [migration](../../../../../python/ovphysx_companion/migration.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0001](../../adr/ADR-0001-physics-menu.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
