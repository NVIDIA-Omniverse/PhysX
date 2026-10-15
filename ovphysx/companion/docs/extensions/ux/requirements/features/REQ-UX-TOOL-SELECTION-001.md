<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-TOOL-SELECTION-001
title: Manage authoring selection and contributed toolbar state
status: proposed
owner: physics-ux
---

## Description

Manage authoring selection and contributed toolbar state. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Rigid selection.** Rigid-body selection mode shall map descendants to their USD rigid ancestor consistently in viewport/Stage/Layers and restore previous hooks on disable.
- AC-2: **Contributed settings.** Toolbar applicability/settings shall track current selection/provider; generic visibility/selection policy shall remain distinct from provider cooking/manipulator settings. Removing the mass provider shall clear its toggle and selection hooks.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-TOOL-SELECTION-001](../../tests/features/TEST-UX-TOOL-SELECTION-001.md) — Select rigid ancestors and restore selection hooks.

## Code References

- [host](../../../../../python/ovphysx_companion/host.py) — existing companion substrate; host integration remains proposed.
- [usd/colliders](../../../../../python/ovphysx_companion/usd/colliders.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0010](../../adr/ADR-0010-authoring-toolbar.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
