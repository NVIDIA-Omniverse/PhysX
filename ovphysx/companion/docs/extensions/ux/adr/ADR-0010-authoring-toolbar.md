<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0010 — Build the authoring toolbar over companion jobs and edit transactions
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

Keep rigid-ancestor selection and tool contributions in the host. Explicit collider batches use `usd.colliders.ColliderBatch`: capture selection/revision, prepare bounded chunks, show progress, then commit one atomic undo patch. Dynamic-body preset creation/removal and automatic generation policy are additional Python/host work; the existing collider helper alone does not implement them.

Cooking uses `CookingService` jobs, not a hidden simulation attach. Cancel or reject stale results on stage/selection changes. Mass editing, Inspector and attachments use their own companion services and capabilities. Conflicting operations cannot publish into the same authored destination.

**Host responsibility:** Toolbar, auto-generation notice policy, dynamic-body presets and progress/undo integration.

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

- [REQ-UX-COLLIDER-001](../requirements/features/REQ-UX-COLLIDER-001.md)
- [REQ-UX-HOST-001](../requirements/features/REQ-UX-HOST-001.md)
- [REQ-UX-SCHEMA-EDIT-001](../requirements/features/REQ-UX-SCHEMA-EDIT-001.md)
- [REQ-UX-TOOL-SELECTION-001](../requirements/features/REQ-UX-TOOL-SELECTION-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)
- [REQ-COMP-JOBS-001](../../../../plc/requirements/exchange/REQ-COMP-JOBS-001.md)
- [REQ-COMP-COOK-001](../../../../plc/requirements/authoring/REQ-COMP-COOK-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-PREVIEW-001](../../../../plc/requirements/live/REQ-COMP-PREVIEW-001.md)
- [REQ-COMP-MASS-001](../../../../plc/requirements/authoring/REQ-COMP-MASS-001.md)
