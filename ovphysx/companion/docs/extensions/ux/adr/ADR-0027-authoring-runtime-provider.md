<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0027 — Use the optional companion as the PhysX authoring and tooling provider
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

Select the supported `ovphysx_companion` package and its versioned service tables exported by the already-loaded ovphysx owner. Negotiate ABI/size and per-operation capability before resource acquisition. The pure Python package carries no second runtime; native joint/geometry modules additionally match the SDK fingerprint and retire before unloading.

C++ services remain USD-free. Python authoring uses the host/customer USD installation; when ovstage population requires an incompatible USD image, transfer copied data to a worker process. Do not pass USD objects, SDK pointers or raw device pointers across that boundary.

Replace the legacy-first coexistence experiment and conditional standalone-ovruntime package proposal. Missing companion capabilities are explicit development obligations, not permission to import private tables or hot-fallback to legacy omni.physx. Same-process legacy-provider coexistence is unqualified; an explicitly isolated worker may be used with a documented copied-data protocol.

**Host responsibility:** Dependency/capability manifests, host provider adapter and package compatibility checks.

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

- [REQ-UX-AUTHOR-PROVIDER-001](../requirements/features/REQ-UX-AUTHOR-PROVIDER-001.md)
- [REQ-UX-HOST-001](../requirements/features/REQ-UX-HOST-001.md)
- [REQ-COMP-BOUNDARY-001](../../../../plc/requirements/architecture/REQ-COMP-BOUNDARY-001.md)
- [REQ-COMP-API-001](../../../../plc/requirements/architecture/REQ-COMP-API-001.md)
- [REQ-COMP-OWNER-001](../../../../plc/requirements/architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-PYTHON-001](../../../../plc/requirements/python/REQ-COMP-PYTHON-001.md)
- [REQ-COMP-NATIVE-ABI-001](../../../../plc/requirements/native/REQ-COMP-NATIVE-ABI-001.md)
- [REQ-COMP-MIGRATION-001](../../../../plc/requirements/host/REQ-COMP-MIGRATION-001.md)

- [REQ-UX-READINESS-001](../requirements/features/REQ-UX-READINESS-001.md)

- [REQ-UX-VALIDATE-MIGRATE-001](../requirements/features/REQ-UX-VALIDATE-MIGRATE-001.md)

- [REQ-UX-VALIDATE-DATA-001](../requirements/features/REQ-UX-VALIDATE-DATA-001.md)

- [REQ-UX-VALIDATE-PREVIEW-001](../requirements/features/REQ-UX-VALIDATE-PREVIEW-001.md)
