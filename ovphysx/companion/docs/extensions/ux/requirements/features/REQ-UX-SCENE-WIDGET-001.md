<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-SCENE-WIDGET-001
title: Edit scene GPU dependencies only as deliberate undoable actions
status: proposed
owner: physics-ux
---

## Description

Edit scene GPU dependencies only as deliberate undoable actions. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Read-only display.** The PhysX scene widget shall inspect solver/broadphase/collision/advanced GPU attributes without authoring on open, rebuild or engine inactivity.
- AC-2: **Coupled edit.** A deliberate enable-GPU-dynamics change shall explain required GPU broadphase/PCM dependencies and commit the intended dependent attributes in the active edit target as one undo group.
- AC-3: **Effective state.** The widget shall distinguish authored values from runtime support and restart/reload/pending state; it shall not silently switch a live process device or apply inactive engine values.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-SCENE-WIDGET-001](../../tests/features/TEST-UX-SCENE-WIDGET-001.md) — Opening a scene widget does not repair or author GPU dependencies.
- [TEST-UX-SCENE-WIDGET-002](../../tests/features/TEST-UX-SCENE-WIDGET-002.md) — Group deliberate GPU dependencies in one undo action.

## Code References

- [configuration](../../../../../python/ovphysx_companion/configuration.py) — existing companion substrate; host integration remains proposed.
- [usd/configuration](../../../../../python/ovphysx_companion/usd/configuration.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0020](../../adr/ADR-0020-scene-properties.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
