<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-GIZMO-001
title: Manipulate authored transforms or use an explicit live capability
status: proposed
owner: physics-ux
---

## Description

Manipulate authored transforms or use an explicit live capability. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Stopped edits.** Outside simulation, the generic gizmo shall author valid selected USD transforms with one undoable drag, correct world-pivot/local conversion and axis locks, without loading physics.
- AC-2: **Live manipulation.** During Play, begin/update/end/cancel shall carry owner/body identity, world pivot, translation delta, normalized quaternion and lock policy to an advertised provider force/constraint service; absent capability shall disable live manipulation rather than teleport via USD.
- AC-3: **Pairing.** Every successful begin shall have exactly one end or cancel; target/selection/provider changes shall release constraints and restore input ownership before removing the gizmo.

- AC-4: **Rotation and locks.** Retained rotational/translation gizmos shall implement their declared world pivot, snapping and axis-lock behavior under positive motion/orientation tests; unsupported rotational targets shall remain an explicit gap rather than silently using point dragging.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-GIZMO-001](../../tests/features/TEST-UX-GIZMO-001.md) — Author a stopped transform drag in the correct local frame.
- [TEST-UX-GIZMO-002](../../tests/features/TEST-UX-GIZMO-002.md) — Pair live manipulation begin with exactly one end or cancellation.

## Code References

- [interaction](../../../../../python/ovphysx_companion/interaction.py) — existing companion substrate; host integration remains proposed.
- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0012](../../adr/ADR-0012-rigid-body-manipulator.md).
- [REQ-RT-PICK-001](../../../runtime/requirements/simulation/REQ-RT-PICK-001.md).
- [REQ-RT-DETACH-001](../../../runtime/requirements/simulation/REQ-RT-DETACH-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
