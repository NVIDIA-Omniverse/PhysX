<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-GRAPH-PROPERTY-001
title: Scope asynchronous rigid-body property queries to their request
status: proposed
owner: physics-ux
---

## Description

Scope asynchronous rigid-body property queries to their request. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Query result.** The PhysX property node shall use an explicit computed-query capability and preserve mass/COM/inertia completion/error and frame/unit semantics; absent capability shall not force-load the runtime.
- AC-2: **Stale result.** Reevaluation, target deletion or stage replacement shall invalidate old requests, so only the matching request publishes a valid result.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-GRAPH-PROPERTY-001](../../tests/features/TEST-UX-GRAPH-PROPERTY-001.md) — Return query results only to the current body and generation.

## Code References

- [mass](../../../../../python/ovphysx_companion/mass.py) — existing companion substrate; host integration remains proposed.
- [usd/mass](../../../../../python/ovphysx_companion/usd/mass.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0019](../../adr/ADR-0019-omnigraph-nodes.md).
- [REQ-UX-GRAPH-IDENTITY-001](REQ-UX-GRAPH-IDENTITY-001.md).
- [REQ-UX-MASS-001](REQ-UX-MASS-001.md).
