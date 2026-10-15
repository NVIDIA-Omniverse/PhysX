<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0011 — Run Inspector preview in an independent companion attachment
status: proposed
date: 2026-09-18
last_updated: 2026-10-05
supersedes: none
---

## Status

Proposed revision for companion-based extension integration. This does not assert that the Kit adapter is implemented or that the revision has been accepted.

## Context / Motivation

The legacy feature combines host presentation, USD authoring and PhysX services. The companion now exposes explicit native owners, copied data and optional customer-USD helpers. The extension must retain its workflows without importing legacy private interfaces or duplicating runtime ownership.

## Decision

Use stopped descriptions and customer USD for hierarchy and joint authoring. On explicit preview activation, create `PreviewSession` with an independent ovstage/native attachment B; playback remains owner A. Use `Inspector` for B and serialize preview steps through its host coordinator. PreviewSession retains its native owner; independent public core attachments may coexist. Do not borrow playback tasks.

Capture source/selection revisions and an immutable preview baseline. Publish transient poses to a dedicated `PreviewDisplay` layer; discard clears only that output. Commit selected fresh poses through `commit_preview` as one undo command. A new edit, undo, Play transition, stage replacement or unsupported topology invalidates/rebuilds B before further publication. Closing B must not collect A tasks or restore stale authored input.

Retain joint state/drive position/velocity controls, limits/gains, hierarchy, selection lock/sync and multiple windows. Gravity, fixed-base and quasi-static controls require separate verified preview policies: the existing CPU preview is not proof of quasi-static parity. When host USD and population USD cannot share one image, use the copied snapshot/worker path; never pass a customer USD object to C++.

**Host responsibility:** Inspector models, per-window selection, joint controls and unimplemented preview mode policies.

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
- [REQ-UX-INSPECTOR-001](../requirements/features/REQ-UX-INSPECTOR-001.md)
- [REQ-COMP-PREVIEW-001](../../../../plc/requirements/live/REQ-COMP-PREVIEW-001.md)
- [REQ-COMP-DESCRIPTION-001](../../../../plc/requirements/authoring/REQ-COMP-DESCRIPTION-001.md)
- [REQ-COMP-QUERY-001](../../../../plc/requirements/live/REQ-COMP-QUERY-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-IDENTITY-001](../../../../plc/requirements/exchange/REQ-COMP-IDENTITY-001.md)
