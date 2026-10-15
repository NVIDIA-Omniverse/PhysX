<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0025 — Gate demo and example launching by companion capabilities
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

Keep discovery and launching in a host browser. Inventory every retained demo/example with exact backend, service, schema and asset prerequisites; no demo implicitly enables legacy simulation or test-only interfaces. Register custom native joint/geometry demos through the build-matched SDK, not Python callbacks.

Respect save/discard/cancel on stage replacement and use normal playback lifecycle. Restore only demo-owned settings/inputs on failure/close. Benchmark/test extensions remain fixtures, and separate-backend examples remain separate examples; neither proves companion parity.

**Host responsibility:** Demo manifests, stage switching and independent example/test ownership.

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

- [REQ-UX-DEMOS-001](../requirements/features/REQ-UX-DEMOS-001.md)
- [REQ-UX-HOST-001](../requirements/features/REQ-UX-HOST-001.md)
- [REQ-COMP-MIGRATION-001](../../../../plc/requirements/host/REQ-COMP-MIGRATION-001.md)
- [REQ-COMP-API-001](../../../../plc/requirements/architecture/REQ-COMP-API-001.md)
- [REQ-COMP-CONFIG-001](../../../../plc/requirements/architecture/REQ-COMP-CONFIG-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)
- [REQ-COMP-NATIVE-ABI-001](../../../../plc/requirements/native/REQ-COMP-NATIVE-ABI-001.md)

- [REQ-UX-PACKAGE-001](../requirements/features/REQ-UX-PACKAGE-001.md)
