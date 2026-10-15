<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-COLLISION-FILTER-001
title: Retain collision groups and pair-filter authoring
status: proposed
owner: physics-ux
---

## Description

Retain collision groups and pair-filter authoring. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Groups.** Create uniquely named UsdPhysics.CollisionGroup prims, edit collider collection membership and filtered-group relationships with scene/path validation. Rebuild the group matrix from composed customer USD without initializing physics.
- AC-2: **Pairs.** Add/remove selected pair filters and symmetric matrix edits shall preserve relationship directionality, collection expansion/inversion semantics and unrelated authored targets. One edit is one undo action; invalid/deleted/instance-proxy/read-only selections cause no partial edits.
- AC-3: **Lifetime.** Selection, stage notices and provider changes shall refresh only current models and remove owned subscriptions on close. Existing filters survive UI disable; runtime filtering consumes the normal sealed authored input rather than a private filter interface.

## Test References

Specification only — not implemented.

- [TEST-UX-COLLISION-FILTER-001](../../tests/features/TEST-UX-COLLISION-FILTER-001.md)

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [migration](../../../../../python/ovphysx_companion/migration.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0001-physics-menu](../../adr/ADR-0001-physics-menu.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
