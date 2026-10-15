<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0016 — Use companion OVD services and customer-USD conversion
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

Coordinate recording through the existing core owner and companion recording lease. A recording request does not create a playback scene. Flush/close recording before dependent resources disappear; shared recording owners cannot stop one another implicitly.

Use companion OVD decode and `usd.ovd` for stopped import; author through customer USD into an explicit destination. Host viewers own recording/frame identity, object selection and contact/COM/joint/velocity overlays. Decoder/converter support does not prove every legacy viewer diagnostic is available. Legacy PVD network/file APIs are separate unsupported capabilities.

**Host responsibility:** Recording controls, importer/viewer, seek/selection and unsupported diagnostic modes.

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
- [REQ-UX-PVD-RECORD-001](../requirements/features/REQ-UX-PVD-RECORD-001.md)
- [REQ-UX-PVD-VIEW-001](../requirements/features/REQ-UX-PVD-VIEW-001.md)
- [REQ-COMP-OVD-001](../../../../plc/requirements/diagnostics/REQ-COMP-OVD-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)
