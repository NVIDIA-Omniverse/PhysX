<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0021 — Split deformable and attachment UX between copied computation and Python USD
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

Keep schema widgets, hierarchy/relationship editing and dialogs on customer USD. Copy meshes, topology, frames, units and source identity to companion cooking, embedding and attachment services. Return typed proposals; apply them with `usd.deformables`/`usd.attachments` in one validated edit transaction.

Preserve surface/volume simulation/collision/visual roles, pose data, attachment endpoint domains, tolerance pair discovery, island selection and element-filter semantics. Explicit output maps completed CPU data through `usd.writeback`; GPU extraction and unsupported endpoint kinds need separate capabilities. Never move USD authoring or writeback into the C++ companion. Failures and stale results leave destination opinions intact.

**Host responsibility:** Dialogs, hierarchy presets, endpoint selection, specialized schemas and live extraction gaps.

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

- [REQ-UX-DEFORM-WIDGET-001](../requirements/features/REQ-UX-DEFORM-WIDGET-001.md)
- [REQ-UX-HOST-001](../requirements/features/REQ-UX-HOST-001.md)
- [REQ-COMP-DEFORM-COOK-001](../../../../plc/requirements/authoring/REQ-COMP-DEFORM-COOK-001.md)
- [REQ-COMP-DEFORM-USD-001](../../../../plc/requirements/python/REQ-COMP-DEFORM-USD-001.md)
- [REQ-COMP-ATTACH-COMPUTE-001](../../../../plc/requirements/authoring/REQ-COMP-ATTACH-COMPUTE-001.md)
- [REQ-COMP-ATTACH-USD-001](../../../../plc/requirements/python/REQ-COMP-ATTACH-USD-001.md)
- [REQ-COMP-DEFORM-WRITE-001](../../../../plc/requirements/python/REQ-COMP-DEFORM-WRITE-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)

- [REQ-UX-DEFORM-WRITEBACK-001](../requirements/features/REQ-UX-DEFORM-WRITEBACK-001.md)

- [REQ-UX-DEFORM-HIERARCHY-001](../requirements/features/REQ-UX-DEFORM-HIERARCHY-001.md)

- [REQ-UX-TET-IMPORT-001](../requirements/features/REQ-UX-TET-IMPORT-001.md)

- [REQ-UX-ATTACHMENT-PAIRS-001](../requirements/features/REQ-UX-ATTACHMENT-PAIRS-001.md)
