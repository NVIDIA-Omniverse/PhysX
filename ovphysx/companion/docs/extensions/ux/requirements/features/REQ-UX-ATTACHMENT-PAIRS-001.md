<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-ATTACHMENT-PAIRS-001
title: Preserve attachment discovery and element-filter workflows
status: proposed
owner: physics-ux
---

## Description

Preserve attachment discovery and element-filter workflows. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Pair discovery.** Capture selected deformables/actors and transforms in customer USD; preserve tolerance-based overlap candidates, duplicate/existing-pair exclusion, validity checks, island grouping and selected-pair add/remove confirmation. Tolerance converts from stage units to declared world meters before computation.
- AC-2: **Typed computation.** Use copied vertex/triangle/tetrahedral/transform endpoint data and companion AuthoringService proposals with topology/revision identity. Distinguish auto attachment creation, anchor mapping and element collision filters; unsupported curves or endpoint/backend combinations remain declared parity gaps without partial authoring.
- AC-3: **Apply and cancel.** Apply selected fresh proposals through usd.attachments into the approved layer as one undo patch, preserving unselected attachments. Tolerance/selection/topology changes invalidate work; cancellation, stale completion or computation failure changes no USD and restores only owned preview state.

## Test References

Specification only — not implemented.

- [TEST-UX-ATTACHMENT-PAIRS-001](../../tests/features/TEST-UX-ATTACHMENT-PAIRS-001.md)

## Code References

- [attachments](../../../../../python/ovphysx_companion/attachments.py) — existing companion substrate; host integration remains proposed.
- [usd/attachments](../../../../../python/ovphysx_companion/usd/attachments.py) — existing companion substrate; host integration remains proposed.
- [usd/meshes](../../../../../python/ovphysx_companion/usd/meshes.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0021-deformable-properties](../../adr/ADR-0021-deformable-properties.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
