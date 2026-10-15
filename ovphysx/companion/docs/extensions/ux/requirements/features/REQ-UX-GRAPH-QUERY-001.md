<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-GRAPH-QUERY-001
title: Preserve portable scene-query result semantics
status: proposed
owner: physics-ux
---

## Description

Preserve portable scene-query result semantics. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Query routing.** Overlap/raycast/sweep nodes shall route explicitly through the selected query provider and retain supported any/closest/all and shape modes; a missing backend/session/mode shall report unavailable rather than zero hits or force-load simulation.
- AC-2: **Result data.** Results shall preserve distances, normals, shape/body/instance identities and query coordinate conventions, copying borrowed arrays before their validity ends.
- AC-3: **Invalidation.** Node input/engine/stage changes shall prevent old query results from appearing current; a valid zero-hit query shall be distinguishable from errors or unsupported queries.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-GRAPH-QUERY-001](../../tests/features/TEST-UX-GRAPH-QUERY-001.md) — Preserve query geometry and copied hit results.
- [TEST-UX-GRAPH-QUERY-002](../../tests/features/TEST-UX-GRAPH-QUERY-002.md) — Reject stopped or stale queries without creating a session.

## Code References

- [queries](../../../../../python/ovphysx_companion/queries.py) — existing companion substrate; host integration remains proposed.
- [inspection](../../../../../python/ovphysx_companion/inspection.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0019](../../adr/ADR-0019-omnigraph-nodes.md).
- [REQ-RT-PLAY-001](../../../runtime/requirements/simulation/REQ-RT-PLAY-001.md).
- [REQ-RT-DETACH-001](../../../runtime/requirements/simulation/REQ-RT-DETACH-001.md).
- [REQ-UX-GRAPH-IDENTITY-001](REQ-UX-GRAPH-IDENTITY-001.md).
