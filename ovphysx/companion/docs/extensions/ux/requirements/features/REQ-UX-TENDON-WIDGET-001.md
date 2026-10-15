<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-TENDON-WIDGET-001
title: Keep multiple-apply tendon instances independent
status: proposed
owner: physics-ux
---

## Description

Keep multiple-apply tendon instances independent. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Instance binding.** Fixed/spatial tendon controls shall preserve API inheritance and multiple-apply instance identity when editing stiffness/damping/limits/attachments; an edit shall target only the selected instance.
- AC-2: **Optional helpers.** Missing PhysX visualization shall disable only selection/viewport helpers; valid schema inspection/editing/undo shall remain available without runtime loading.
- AC-3: **Helper lifetime.** Deselect/unload shall remove only the owned tendon filters/helpers and discard stale callbacks.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-TENDON-WIDGET-001](../../tests/features/TEST-UX-TENDON-WIDGET-001.md) — Edit one tendon instance and release only its helpers.

## Code References

- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.
- [descriptions](../../../../../python/ovphysx_companion/descriptions.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0022](../../adr/ADR-0022-tendon-properties.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
