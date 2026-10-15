<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0013 — Compute and author mass through the companion Python/native split
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

Capture shapes, material density and overrides with `usd.mass`; compute stopped mass/COM/inertia through `AuthoringService` on copied data. Use `Inspector` only for completed live values and label those separately from authored values. Do not force-load playback or call a test mass printer.

Apply validated mass, COM, diagonal inertia and principal axes through customer USD with one undo patch per drag. Preserve frame/unit conventions and reject stale geometry/material revisions before publication. Unsupported mass geometry is a visible gap, not an approximate value presented as exact.

**Host responsibility:** Mass visualization, handles, display units and drag-to-command grouping.

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
- [REQ-UX-MASS-001](../requirements/features/REQ-UX-MASS-001.md)
- [REQ-COMP-MASS-001](../../../../plc/requirements/authoring/REQ-COMP-MASS-001.md)
- [REQ-COMP-DESCRIPTION-001](../../../../plc/requirements/authoring/REQ-COMP-DESCRIPTION-001.md)
- [REQ-COMP-QUERY-001](../../../../plc/requirements/live/REQ-COMP-QUERY-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)
