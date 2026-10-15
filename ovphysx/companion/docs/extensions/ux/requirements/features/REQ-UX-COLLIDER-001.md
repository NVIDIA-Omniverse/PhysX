<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-COLLIDER-001
title: Author collider batches without stale or implicit simulation work
status: proposed
owner: physics-ux
---

## Description

Author collider batches without stale or implicit simulation work. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Explicit batch.** Static Triangle Mesh/Mesh Simplification and dynamic Convex Hull/Decomposition/SDF actions shall validate selection, compatibility and provider capability before authoring CollisionAPI plus only chosen provider APIs. One batch shall be one undo action with per-prim failures reported.
- AC-2: **Async ownership.** A cooking batch shall expose operation identity/progress/error, disable conflicting actions, and discard old-stage completions; it shall not start the simulation runtime to perform authoring.
- AC-3: **Automatic policy.** Automatic collider generation shall be opt-in, process each eligible prim once per relevant change and skip existing explicit collision authoring; disabling shall stop future auto work and preserve existing schema.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-COLLIDER-001](../../tests/features/TEST-UX-COLLIDER-001.md) — Create an explicit collider batch as one undo action.
- [TEST-UX-COLLIDER-002](../../tests/features/TEST-UX-COLLIDER-002.md) — Discard an old-stage cooking completion.
- [TEST-UX-COLLIDER-003](../../tests/features/TEST-UX-COLLIDER-003.md) — Keep automatic collider generation opt-in and idempotent.

## Code References

- [usd/colliders](../../../../../python/ovphysx_companion/usd/colliders.py) — existing companion substrate; host integration remains proposed.
- [cooking](../../../../../python/ovphysx_companion/cooking.py) — existing companion substrate; host integration remains proposed.
- [host](../../../../../python/ovphysx_companion/host.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0010](../../adr/ADR-0010-authoring-toolbar.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
- [REQ-UX-PROGRESS-001](REQ-UX-PROGRESS-001.md).
