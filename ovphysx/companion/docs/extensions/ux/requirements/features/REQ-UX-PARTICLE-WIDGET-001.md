<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-PARTICLE-WIDGET-001
title: Preserve particle defaults and validate explicit offset edits
status: proposed
owner: physics-ux
---

## Description

Preserve particle defaults and validate explicit offset edits. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Applicability.** The particle widget shall bind only to a selected PhysX particle-system API and preserve schema defaults, units and authored-versus-fallback status; open/rebuild shall write nothing or instantiate particles.
- AC-2: **Advanced fields.** contactOffset, restOffset, solidRestOffset and fluidRestOffset shall appear in the advanced group and round-trip through USD/undo/redo without changing untargeted offsets.
- AC-3: **Validation.** Invalid/nonfinite combinations shall be rejected with an explanation before commit using provider/schema rules; inactive-engine values shall remain inspectable without being applied.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-PARTICLE-WIDGET-001](../../tests/features/TEST-UX-PARTICLE-WIDGET-001.md) — Round-trip advanced particle offsets without panel side effects.

## Code References

- [usd/particles](../../../../../python/ovphysx_companion/usd/particles.py) — existing companion substrate; host integration remains proposed.
- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0023](../../adr/ADR-0023-particle-properties.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
