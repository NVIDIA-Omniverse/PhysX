<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-EDITOR-JOINT-001
title: Preserve editor joint registration independently of solver plugins
status: proposed
owner: physics-ux
---

## Description

Preserve editor joint registration independently of solver plugins. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Registration.** The host shall register custom joint glyphs, info boxes, applicability and property/limit manipulators by stable type/provider identity, without requiring a native solver plugin. Removing a registration removes only its editor resources.
- AC-2: **Editing.** Joint creation, body swap/alignment, attachment-position/orientation editing and multi-selection actions shall use exact body-relative frames and one customer-USD undo transaction; selection/target deletion invalidates handles.
- AC-3: **Native distinction.** Custom joint/geometry simulation requires the separate build-matched native SDK and capability checks. Python editor callbacks must never become solver/contact callbacks or retain SDK pointers; a loaded native plugin does not imply an editor contribution.

## Test References

Specification only — not implemented.

- [TEST-UX-EDITOR-JOINT-001](../../tests/features/TEST-UX-EDITOR-JOINT-001.md)

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [schemas](../../../../../python/ovphysx_companion/schemas.py) — existing companion substrate; host integration remains proposed.
- [plugins](../../../../../python/ovphysx_companion/plugins.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0001-physics-menu](../../adr/ADR-0001-physics-menu.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
