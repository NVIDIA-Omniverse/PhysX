<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-MENU-JOINT-001
title: Author valid joint relationships and local frames
status: proposed
owner: physics-ux
---

## Description

Author valid joint relationships and local frames. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Types and applicability.** D6/fixed/revolute/prismatic/spherical/distance authoring shall work generically; gear/rack-and-pinion shall require a provider. Invalid body selection shall be rejected before authoring.
- AC-2: **Joint command.** The command shall author body relationships, body-relative local frames and schema-appropriate defaults as one undo group, preserve stage-unit/angle conventions, and retain compatible saved command identifiers.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-MENU-JOINT-001](../../tests/features/TEST-UX-MENU-JOINT-001.md) — Preserve joint defaults and body-local frames.
- [TEST-UX-MENU-JOINT-002](../../tests/features/TEST-UX-MENU-JOINT-002.md) — Reject invalid joint selection before authoring.

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [migration](../../../../../python/ovphysx_companion/migration.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0001](../../adr/ADR-0001-physics-menu.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
