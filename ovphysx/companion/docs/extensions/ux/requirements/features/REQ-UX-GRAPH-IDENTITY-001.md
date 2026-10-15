<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-GRAPH-IDENTITY-001
title: Keep saved graph identity and engine dispatch explicit
status: proposed
owner: physics-ux
---

## Description

Keep saved graph identity and engine dispatch explicit. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Compatibility.** Saved physics node identifiers shall reopen through unchanged semantics or explicit versioned aliases/migration, without dropped nodes or silent rerouting of a PhysX node to a different engine.
- AC-2: **Scope.** Generic graph components shall load independently of the legacy PhysX graph extension, require explicit engine/stage capability scope and invalidate output/subscriptions on Stop, node deletion or provider/stage replacement.

- AC-3: **Complete node inventory.** Every legacy graph/CCT descriptor shall map its serialized type/version and exact input/output pin names, data types, array/role/default metadata and execution behavior to a retained adapter or explicit migration error. Compare saved graphs and outputs, not only registration counts.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-GRAPH-IDENTITY-001](../../tests/features/TEST-UX-GRAPH-IDENTITY-001.md) — Reopen saved graph nodes without silent engine substitution.

## Code References

- [compatibility](../../../../../python/ovphysx_companion/compatibility.py) — existing companion substrate; host integration remains proposed.
- [host](../../../../../python/ovphysx_companion/host.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0019](../../adr/ADR-0019-omnigraph-nodes.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-RT-DETACH-001](../../../runtime/requirements/simulation/REQ-RT-DETACH-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
