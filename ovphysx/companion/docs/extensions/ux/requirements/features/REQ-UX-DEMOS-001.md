<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-DEMOS-001
title: Browse metadata and launch demos through normal stage lifecycle
status: proposed
owner: physics-ux
---

## Description

Browse metadata and launch demos through normal stage lifecycle. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Discovery.** An optional generic browser shall show provider title/category/description and required engine/capabilities without starting physics or enabling test extensions; unavailable demos shall state missing capabilities before launch.
- AC-2: **Launch.** Launch shall use Kit stage creation/switching and normal Play, respect unsaved-stage save/discard/cancel handling and require the reviewed single-scene runtime contract for runnable ovphysx demos.
- AC-3: **Close.** Demo close/failure shall restore only its temporary settings/subscriptions and release its scene through normal Stop/detach without leaving a hidden stepping task.

- AC-4: **Inventory.** Each legacy demo and retained internal/example scene shall have a recorded runnable, fixture-only, separate-backend or unsupported disposition with its service/backend/assets requirements; no unclassified scene may silently disappear from migration coverage.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-DEMOS-001](../../tests/features/TEST-UX-DEMOS-001.md) — Browse and launch through normal unsaved-stage and Play behavior.
- [TEST-UX-DEMOS-002](../../tests/features/TEST-UX-DEMOS-002.md) — Restore demo-owned state after close or launch failure.

## Code References

- [migration](../../../../../python/ovphysx_companion/migration.py) — existing companion substrate; host integration remains proposed.
- [compatibility](../../../../../python/ovphysx_companion/compatibility.py) — existing companion substrate; host integration remains proposed.
- [schemas](../../../../../python/ovphysx_companion/schemas.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0025](../../adr/ADR-0025-demo-browser.md).
- [REQ-RT-STAGE-001](../../../runtime/requirements/simulation/REQ-RT-STAGE-001.md).
- [REQ-RT-PLAY-001](../../../runtime/requirements/simulation/REQ-RT-PLAY-001.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-RT-DETACH-001](../../../runtime/requirements/simulation/REQ-RT-DETACH-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
