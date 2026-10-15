<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-PROGRESS-001
title: Display independently identified current-stage operations
status: proposed
owner: physics-ux
---

## Description

Display independently identified current-stage operations. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Identity.** Progress events shall carry engine, stage generation, operation identity, label and running/completed/failed/cancelled state; simultaneous operations shall not overwrite one another.
- AC-2: **Presentation.** The UI shall clamp finite fractions to [0,1], represent unknown progress as indeterminate and surface error text for failure rather than imply success.
- AC-3: **Termination.** Terminal events shall remove the matching active overlay after exposing failure feedback where applicable; stage close/provider unload shall remove its operations/listeners and ignore later stale events.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-PROGRESS-001](../../tests/features/TEST-UX-PROGRESS-001.md) — Track concurrent operation progress and terminal states.

## Code References

- [jobs](../../../../../python/ovphysx_companion/jobs.py) — existing companion substrate; host integration remains proposed.
- [host](../../../../../python/ovphysx_companion/host.py) — existing companion substrate; host integration remains proposed.
- [usd/colliders](../../../../../python/ovphysx_companion/usd/colliders.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0015](../../adr/ADR-0015-progress-overlay.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
