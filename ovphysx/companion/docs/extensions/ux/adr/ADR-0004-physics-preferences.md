<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0004 — Compose typed companion preferences before runtime bootstrap
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

Build preferences from the companion `configuration` catalog and `HostPreferences`, with requested, effective, pending and error states. Submit core creation/process settings through the elected owner before first bootstrap. Stopped native cooking can bootstrap shared services too; freeze initialization policy before either playback or authoring starts. Stop is not process reset.

Keep scan limits and feature toggles in the host. Use the audited typed mappings for devices, threads and NVTX. Cache actions distinguish `CookingService` result-cache invalidation from runtime disk-cache configuration; never recursively delete a user directory or call a legacy private cache service. Removed legacy keys remain inert, not silently translated.

**Host responsibility:** Preference widgets, persistence, user-visible restart state and configuration conflict policy.

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
- [REQ-UX-PREF-CONFIG-001](../requirements/features/REQ-UX-PREF-CONFIG-001.md)
- [REQ-UX-PREF-COOK-001](../requirements/features/REQ-UX-PREF-COOK-001.md)
- [REQ-UX-PREFERENCES-001](../requirements/features/REQ-UX-PREFERENCES-001.md)
- [REQ-COMP-CONFIG-001](../../../../plc/requirements/architecture/REQ-COMP-CONFIG-001.md)
- [REQ-COMP-COOK-001](../../../../plc/requirements/authoring/REQ-COMP-COOK-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)
