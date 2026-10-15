<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0009 — Route viewport interaction to the exact companion owner
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

The host owns hit selection, world rays, modifiers and feedback. Use `SceneQueries` and `Interaction` on the selected `ToolsSession`; include object generation and point-instance identity. Deliver queued actions at the elected completed/pre-step boundary.

Default grab to companion managed force picking, which owns the legacy impulse, point-velocity damping, gravity assistance and angular-damping restoration in native code. Route pointer rays through pick/move_ray or pass host-filtered query hits and explicit targets. Route settings changes to the grab's per-instance force coefficient, including zero; transient angular damping retains its initial press value. D6 dragging remains an explicit alternative. Keep push as its separate impulse workflow. Cancel on focus loss, Pause/Stop, input disable, target deletion or owner replacement and restore transient state before releasing the session.

**Host responsibility:** Input picking/filter and timeline/focus policy plus renderer feedback; force-grab equations stay native.

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
- [REQ-UX-INTERACTION-SETTINGS-001](../requirements/features/REQ-UX-INTERACTION-SETTINGS-001.md)
- [REQ-UX-MOUSE-001](../requirements/features/REQ-UX-MOUSE-001.md)
- [REQ-COMP-QUERY-001](../../../../plc/requirements/live/REQ-COMP-QUERY-001.md)
- [REQ-COMP-INTERACTION-001](../../../../plc/requirements/live/REQ-COMP-INTERACTION-001.md)
- [REQ-COMP-OWNER-001](../../../../plc/requirements/architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)
