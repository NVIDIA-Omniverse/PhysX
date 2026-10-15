<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0012 — Use companion interaction for live rigid-body manipulators
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

Stopped gizmos author customer-USD transforms through one `EditSession` transaction with correct parent/pivot conversion. Live gizmos use explicit `Interaction` force/torque or D6 drag on an exact owner/object; they never teleport authored USD during playback.

Keep translation/rotation locks, snapping and selection policy in the host. The current D6 point drag does not itself implement arbitrary rotational gizmo targets; preserve that as a tested extension obligation. Begin/update/end/cancel pair exactly once and restore flags/velocity policy on retirement.

**Host responsibility:** Transform gizmos, rotation behavior, axis locks and host input ownership.

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

- [REQ-UX-GIZMO-001](../requirements/features/REQ-UX-GIZMO-001.md)
- [REQ-UX-HOST-001](../requirements/features/REQ-UX-HOST-001.md)
- [REQ-COMP-INTERACTION-001](../../../../plc/requirements/live/REQ-COMP-INTERACTION-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-IDENTITY-001](../../../../plc/requirements/exchange/REQ-COMP-IDENTITY-001.md)
