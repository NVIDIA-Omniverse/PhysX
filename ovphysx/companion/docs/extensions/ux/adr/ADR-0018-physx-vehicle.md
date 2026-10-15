<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0018 — Compose vehicle authoring and live companion control
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

Use companion vehicle computations and `usd.vehicle` for copied suspension/sprung-mass data and reversible application. Keep the complete vehicle wizard, tire friction table, nonlinear response widgets and schema presets in Python/host code; the supplied suspension helper is not a full wizard.

Route filtered steering/throttle/brake, mouse and auto-reverse policy through the explicit owner before one step; read completed wheel/drive telemetry afterward. Preserve graph node identity and units. Gate each backend and cancel input/handles on removal or Stop.

**Host responsibility:** Wizard, schema widget families, device input and graph adaptation.

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

- [REQ-UX-HOST-001](../requirements/features/REQ-UX-HOST-001.md)
- [REQ-UX-VEHICLE-001](../requirements/features/REQ-UX-VEHICLE-001.md)
- [REQ-COMP-VEHICLE-001](../../../../plc/requirements/controllers/REQ-COMP-VEHICLE-001.md)
- [REQ-COMP-QUERY-001](../../../../plc/requirements/live/REQ-COMP-QUERY-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)
