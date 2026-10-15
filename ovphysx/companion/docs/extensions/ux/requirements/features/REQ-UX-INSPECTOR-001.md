<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-INSPECTOR-001
title: Inspect and author articulations with independent companion preview
status: proposed
owner: physics-ux
---

## Description

Inspect and author articulations with independent companion preview. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Authoring model.** The Inspector shall retain hierarchy, articulation selection, selection lock/sync, multiple windows, automatic/state-position/drive-position/drive-velocity controls and limits/gains. Bind each row to exact joint/DOF identity, authored-versus-live value and stage units; one edit/drag shall have one undo action.
- AC-2: **Preview ownership.** Explicit preview shall use PreviewSession B with independent ovstage input and completed output while playback A is unaffected. PreviewDisplay owns a dedicated transient layer; discard clears only its output, while commit_preview validates captured revisions and commits selected poses as one undo patch. No second live facade attachment is permitted.
- AC-3: **Cleanup and modes.** Close, Play, stage/selection/topology change, undo or unload shall cancel stale preview/model work and release only B resources. Gravity, fixed-base and quasi-static options each need a verified preview policy and an explicit error if unavailable; absence of those policies remains a parity gap, not a claim that the full Inspector has migrated.

- AC-4: **Mode parity.** Gravity, fixed-base and quasi-static preview shall each retain their declared effect on isolated B and reset deterministically; supported controls must have positive motion/state assertions, while an unavailable mode remains an unclosed parity item.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-INSPECTOR-001](../../tests/features/TEST-UX-INSPECTOR-001.md) — Author drive targets without force-loading a preview.
- [TEST-UX-INSPECTOR-002](../../tests/features/TEST-UX-INSPECTOR-002.md) — Do not reopen the inspector from stale background work.

## Code References

- [preview](../../../../../python/ovphysx_companion/preview.py) — existing companion substrate; host integration remains proposed.
- [inspection](../../../../../python/ovphysx_companion/inspection.py) — existing companion substrate; host integration remains proposed.
- [usd/preview](../../../../../python/ovphysx_companion/usd/preview.py) — existing companion substrate; host integration remains proposed.
- [descriptions](../../../../../python/ovphysx_companion/descriptions.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0011](../../adr/ADR-0011-physics-inspector.md).
- [REQ-RT-STAGE-001](../../../runtime/requirements/simulation/REQ-RT-STAGE-001.md).
- [REQ-RT-PLAY-001](../../../runtime/requirements/simulation/REQ-RT-PLAY-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
