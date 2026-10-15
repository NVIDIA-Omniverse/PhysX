<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0002 — Host one capability-aware viewport physics menu
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

Keep one Physics display category keyed by engine and stable action identity. Share the effective visibility model across the viewport, toolbar and debug window. Use companion `DebugDraw`, stopped `collision_overlay` and copied descriptions as data sources; render and select on the Kit thread. Generic USD joint visibility needs no runtime.

Keep None/Selected/All, normals, distance and AABB policies separate from native debug flags. Report absent geometry modes explicitly. Provider unload removes only its contribution and cancels subscriptions before releasing data.

**Host responsibility:** Kit menu and viewport integration, selection filters and renderer resources.

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
- [REQ-UX-VIEWPORT-001](../requirements/features/REQ-UX-VIEWPORT-001.md)
- [REQ-COMP-DEBUG-001](../../../../plc/requirements/live/REQ-COMP-DEBUG-001.md)
- [REQ-COMP-DESCRIPTION-001](../../../../plc/requirements/authoring/REQ-COMP-DESCRIPTION-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)
