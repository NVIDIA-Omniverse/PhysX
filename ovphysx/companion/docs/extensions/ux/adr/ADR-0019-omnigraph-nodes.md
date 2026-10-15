<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0019 — Implement graph adapters against companion operation contracts
status: proposed
date: 2026-09-18
last_updated: 2026-10-01
supersedes: none
---

## Status

Proposed revision for companion-based extension integration. This does not assert that the Kit adapter is implemented or that the revision has been accepted.

## Context / Motivation

The legacy feature combines host presentation, USD authoring and PhysX services. The companion now exposes explicit native owners, copied data and optional customer-USD helpers. The extension must retain its workflows without importing legacy private interfaces or duplicating runtime ownership.

## Decision

Keep graph registration and scheduling in Kit. Bind contact/trigger nodes to copied `EventQueue`, scene queries to `SceneQueries`, property queries to stopped mass/description services and vehicle nodes to copied telemetry. Every node carries an explicit owner and validity; preserve serialized type/version, pins, arrays, any/closest/all and execution semantics.

Inventory each node, including CCT nodes and immediate geometry nodes. Immediate bounds/overlap/penetration/raycast/contact/intersecting-face algorithms need a stopped geometry service; live scene queries and attachment triangle filters are not equivalent. Existing mesh/plane/custom/compound query-shape exclusions remain parity gaps. No native callback invokes graph evaluation or Python.

**Host responsibility:** Node registration, serialized compatibility, execution pins and missing immediate providers.

**Out of scope:** changing playback's accepted single-scene lifecycle, embedding USD in native companion code, or claiming unimplemented UX parity from a successful service-table lookup.

## Alternatives considered

- Keep legacy interface calls behind the new widgets: retains hidden stage/runtime ownership and cannot satisfy the supported dependency boundary.
- Add the whole UX interface to core ovphysx: couples the simulation facade to host commands, USD and presentation.
- Treat an unavailable capability as completed feature parity: prevents unsafe fallback but leaves the required user workflow unimplemented.

## Consequences

- **Positive:** native computation, customer-USD edits and host presentation have separate explicit owners.
- **Costs:** adapt the feature's models/commands and verify its workflow-specific REQ/TEST scenarios; extend services where the recorded parity gap requires it.
- **Lifecycle:** cancel stale delivery and drain owned resources before stage/provider teardown; never collect another owner's tasks.

## References

- [REQ-UX-GRAPH-EVENT-001](../requirements/features/REQ-UX-GRAPH-EVENT-001.md)
- [REQ-UX-GRAPH-IDENTITY-001](../requirements/features/REQ-UX-GRAPH-IDENTITY-001.md)
- [REQ-UX-GRAPH-IMMEDIATE-001](../requirements/features/REQ-UX-GRAPH-IMMEDIATE-001.md)
- [REQ-UX-GRAPH-PROPERTY-001](../requirements/features/REQ-UX-GRAPH-PROPERTY-001.md)
- [REQ-UX-GRAPH-QUERY-001](../requirements/features/REQ-UX-GRAPH-QUERY-001.md)
- [REQ-UX-GRAPH-VEHICLE-001](../requirements/features/REQ-UX-GRAPH-VEHICLE-001.md)
- [REQ-UX-HOST-001](../requirements/features/REQ-UX-HOST-001.md)
- [REQ-COMP-EVENTS-001](../../../../plc/requirements/live/REQ-COMP-EVENTS-001.md)
- [REQ-COMP-QUERY-001](../../../../plc/requirements/live/REQ-COMP-QUERY-001.md)
- [REQ-COMP-MASS-001](../../../../plc/requirements/authoring/REQ-COMP-MASS-001.md)
- [REQ-COMP-VEHICLE-001](../../../../plc/requirements/controllers/REQ-COMP-VEHICLE-001.md)
- [REQ-COMP-CCT-001](../../../../plc/requirements/controllers/REQ-COMP-CCT-001.md)
- [REQ-COMP-JOBS-001](../../../../plc/requirements/exchange/REQ-COMP-JOBS-001.md)
