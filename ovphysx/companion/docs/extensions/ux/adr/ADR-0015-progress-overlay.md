<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0015 — Present generation-scoped companion job progress
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

Adapt companion jobs, `ColliderBatch.progress` and `DeferredEdit` to one host operation model keyed by provider, stage revision and operation ID. Show unknown native progress as indeterminate rather than inventing percentages.

Cancellation suppresses delivery but waits for accepted native work to quiesce before resource release. Publish only through the initiating edit session. Error and cancellation are distinct terminal outcomes; remove subscriptions and stale overlays on close/unload without affecting other jobs.

**Host responsibility:** Progress presentation, UI scheduling and terminal error feedback.

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
- [REQ-UX-PROGRESS-001](../requirements/features/REQ-UX-PROGRESS-001.md)
- [REQ-COMP-JOBS-001](../../../../plc/requirements/exchange/REQ-COMP-JOBS-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)
- [REQ-COMP-IDENTITY-001](../../../../plc/requirements/exchange/REQ-COMP-IDENTITY-001.md)
