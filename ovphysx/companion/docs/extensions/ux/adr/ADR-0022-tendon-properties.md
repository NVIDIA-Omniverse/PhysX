<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0022 — Keep tendon property and overlay identity in the Python host
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

Edit fixed/spatial tendon multiple-apply instances through customer USD and `EditSession`, retaining inheritance, attachment-parent and root/leaf identity. Copy stopped descriptions for inspection; live `Inspector` only supplies fields its contract actually exposes.

Host selection filters and overlay handles are optional consumers. Validate reparenting/cycles and remove the selected instance only. No tendon widget acquires a private native parser or starts simulation; unsupported tendon-specific live visualization remains separately tracked.

**Host responsibility:** Multiple-apply widgets, tendon graph validation and specialized overlays.

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
- [REQ-UX-TENDON-WIDGET-001](../requirements/features/REQ-UX-TENDON-WIDGET-001.md)
- [REQ-COMP-DESCRIPTION-001](../../../../plc/requirements/authoring/REQ-COMP-DESCRIPTION-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-DEBUG-001](../../../../plc/requirements/live/REQ-COMP-DEBUG-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)
