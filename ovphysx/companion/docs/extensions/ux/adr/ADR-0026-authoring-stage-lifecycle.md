<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0026 — Associate authoring sessions with host stages independently of playback
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

Expose a host authoring association keyed by stage identity/generation, with idempotent attach and paired detach. The association owns a Python `EditSession`, selections, notices and queued job references. Generic authoring needs no simulator registration, native attachment or ovphysx instance. Initialize native data services only when an explicit compute operation needs them.

Replacement/unregister cancels delivery, drains owned work and closes the old edit session before accepting a new generation. Use customer USD only on its approved thread. Use `DeferredEdit` to publish copied computation and `OutputNotices` to distinguish authored input from preview/bake output; ordinary authoring must still reach the sealed playback input path.

Play/Pause/Stop leave this association separate from the playback epoch. Inspector/validation may explicitly acquire their own companion attachment; teardown drains only that owner. No legacy simulation attach, disabled-update maintenance pump or authoring-global current stage is introduced.

**Host responsibility:** Umbrella/Kit association callbacks, stage generation and dispatcher wiring.

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

- [REQ-UX-AUTHOR-STAGE-001](../requirements/features/REQ-UX-AUTHOR-STAGE-001.md)
- [REQ-UX-HOST-001](../requirements/features/REQ-UX-HOST-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-JOBS-001](../../../../plc/requirements/exchange/REQ-COMP-JOBS-001.md)
- [REQ-COMP-OVSTAGE-001](../../../../plc/requirements/exchange/REQ-COMP-OVSTAGE-001.md)
- [REQ-COMP-PREVIEW-001](../../../../plc/requirements/live/REQ-COMP-PREVIEW-001.md)

- [REQ-UX-METRICS-001](../requirements/features/REQ-UX-METRICS-001.md)
