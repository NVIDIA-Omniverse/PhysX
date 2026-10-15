<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-GRAPH-IMMEDIATE-001
title: Keep immediate geometry queries independent from simulation events
status: proposed
owner: physics-ux
---

## Description

Keep immediate geometry queries independent from simulation events. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Stopped geometry operations.** Generate Contacts and immediate bounds/overlap/penetration/raycast/intersecting-face nodes shall retain the supplied-geometry contract without attach/step. These seven immediate families require a dedicated geometry provider; current live SceneQueries, collision cooking and triangle attachment filters do not establish parity.
- AC-2: **Ownership and validity.** Outputs shall preserve caller geometry and result ownership; malformed/unsupported inputs shall return an explicit error/unavailable state, not simulated contact events or stale results.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-GRAPH-IMMEDIATE-001](../../tests/features/TEST-UX-GRAPH-IMMEDIATE-001.md) — Compute immediate geometry results without a scene session.

## Code References

- [cooking](../../../../../python/ovphysx_companion/cooking.py) — existing companion substrate; host integration remains proposed.
- [attachments](../../../../../python/ovphysx_companion/attachments.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0019](../../adr/ADR-0019-omnigraph-nodes.md).
- [REQ-UX-GRAPH-IDENTITY-001](REQ-UX-GRAPH-IDENTITY-001.md).
