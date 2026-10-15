<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0005 — Separate stage policy from explicit companion output destinations
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

Preserve root-layer physics settings and unknown metadata. Translate only catalogued options through `configuration`/`usd.configuration`; keep catch-up, pointer input and visualization distance in the host. Stage changes compose against explicit defaults, not the previous stage request.

Playback publishes completed ovstage output through its existing adapter. `StageFlow` excludes output ordinals from authored input. Optional customer-USD preview/bake is a separate explicit writer using `OutputNotices` and `usd.writeback`, not restoration of the old USD/Fabric toggle set. Never let an output notice become a solver input or a second writer claim the same destination.

**Host responsibility:** Metadata persistence, timeline policy, output-writer election and notice wiring.

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
- [REQ-UX-INTERACTION-SETTINGS-001](../requirements/features/REQ-UX-INTERACTION-SETTINGS-001.md)
- [REQ-UX-OVERLAY-001](../requirements/features/REQ-UX-OVERLAY-001.md)
- [REQ-UX-STAGE-POLICY-001](../requirements/features/REQ-UX-STAGE-POLICY-001.md)
- [REQ-UX-STAGE-SETTINGS-001](../requirements/features/REQ-UX-STAGE-SETTINGS-001.md)
- [REQ-COMP-CONFIG-001](../../../../plc/requirements/architecture/REQ-COMP-CONFIG-001.md)
- [REQ-COMP-OVSTAGE-001](../../../../plc/requirements/exchange/REQ-COMP-OVSTAGE-001.md)
- [REQ-COMP-DEFORM-WRITE-001](../../../../plc/requirements/python/REQ-COMP-DEFORM-WRITE-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)
