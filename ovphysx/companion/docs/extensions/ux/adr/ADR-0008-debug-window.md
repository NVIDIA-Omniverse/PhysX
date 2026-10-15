<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0008 — Present companion diagnostics without changing playback lifecycle
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

Route Run/Pause/Stop to the runtime adapter. Keep playback force-load, private reset/release and arbitrary paused stepping unavailable. Dedicated Inspector preview is a separate explicit companion owner, never an exception inside the playback adapter.

Use copied `Inspector`, `DebugDraw`, `EventQueue` and statistics for diagnostic views, and stopped mass/cooking services where appropriate. Restore the Simulation Info workflow as a capability-aware view of actual owner/pipeline/output state and supported counters; unavailable counters are not zero. Core OVD recording is distinct from unsupported RepX export or legacy PVD connectivity. Do not import test-only interfaces.

**Host responsibility:** Diagnostic windows, log routing, labels and legacy field-by-field availability.

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

- [REQ-UX-DEBUG-CONTROL-001](../requirements/features/REQ-UX-DEBUG-CONTROL-001.md)
- [REQ-UX-DEBUG-DATA-001](../requirements/features/REQ-UX-DEBUG-DATA-001.md)
- [REQ-UX-HOST-001](../requirements/features/REQ-UX-HOST-001.md)
- [REQ-UX-MASS-001](../requirements/features/REQ-UX-MASS-001.md)
- [REQ-UX-OVERLAY-001](../requirements/features/REQ-UX-OVERLAY-001.md)
- [REQ-UX-PVD-RECORD-001](../requirements/features/REQ-UX-PVD-RECORD-001.md)
- [REQ-UX-SCHEMA-EDIT-001](../requirements/features/REQ-UX-SCHEMA-EDIT-001.md)
- [REQ-COMP-DEBUG-001](../../../../plc/requirements/live/REQ-COMP-DEBUG-001.md)
- [REQ-COMP-QUERY-001](../../../../plc/requirements/live/REQ-COMP-QUERY-001.md)
- [REQ-COMP-EVENTS-001](../../../../plc/requirements/live/REQ-COMP-EVENTS-001.md)
- [REQ-COMP-MASS-001](../../../../plc/requirements/authoring/REQ-COMP-MASS-001.md)
- [REQ-COMP-OVD-001](../../../../plc/requirements/diagnostics/REQ-COMP-OVD-001.md)
- [REQ-COMP-PREVIEW-001](../../../../plc/requirements/live/REQ-COMP-PREVIEW-001.md)

- [REQ-UX-SIM-INFO-001](../requirements/features/REQ-UX-SIM-INFO-001.md)
