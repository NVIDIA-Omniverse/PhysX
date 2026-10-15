<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0001 — Own physics creation and schema commands in the generic menu
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

Keep one menu/action registry in `omni.physics.ui`; PhysX supplies capability-scoped contributions. Generic scene, ground, material and joint authoring uses customer USD and the public `ovphysx.utils` helpers under an explicit `EditSession`. Preserve compatible command IDs through host aliases, not legacy module imports.

Route collider, attachment, deformable, particle and vehicle computation through the companion services; apply copied results through its Python USD utilities as one host undo command. Preview applicability without cooking. Restore collision-group creation and pair-filter authoring as Python workflows; the previous proposal to remove them is replaced by explicit filter requirements. Schema-only authoring stays available independently of solver support.

**Host responsibility:** Selection/subtree policy, unique naming, writable layers, command aliases and property/menu registration.

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
- [REQ-UX-MENU-BASIC-001](../requirements/features/REQ-UX-MENU-BASIC-001.md)
- [REQ-UX-MENU-JOINT-001](../requirements/features/REQ-UX-MENU-JOINT-001.md)
- [REQ-UX-MENU-MATERIAL-001](../requirements/features/REQ-UX-MENU-MATERIAL-001.md)
- [REQ-UX-MENU-PROVIDER-001](../requirements/features/REQ-UX-MENU-PROVIDER-001.md)
- [REQ-UX-SCHEMA-EDIT-001](../requirements/features/REQ-UX-SCHEMA-EDIT-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)
- [REQ-COMP-MIGRATION-001](../../../../plc/requirements/host/REQ-COMP-MIGRATION-001.md)
- [REQ-COMP-ATTACH-USD-001](../../../../plc/requirements/python/REQ-COMP-ATTACH-USD-001.md)
- [REQ-COMP-DEFORM-USD-001](../../../../plc/requirements/python/REQ-COMP-DEFORM-USD-001.md)

- [REQ-UX-COLLISION-FILTER-001](../requirements/features/REQ-UX-COLLISION-FILTER-001.md)

- [REQ-UX-COMMANDS-001](../requirements/features/REQ-UX-COMMANDS-001.md)

- [REQ-UX-EDITOR-JOINT-001](../requirements/features/REQ-UX-EDITOR-JOINT-001.md)
