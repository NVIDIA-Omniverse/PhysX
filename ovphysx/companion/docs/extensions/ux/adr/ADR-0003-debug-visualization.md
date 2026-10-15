<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0003 — Render copied companion diagnostics and explicit USD overlays
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

Use completed `DebugDraw` frames and `Inspector` records for live bodies, joints and statistics. Use cooked triangle representations plus `collision_overlay` for stopped collision geometry and normals. Every cache key includes source/topology revision and owner generation; copied frames declare units and coordinate frame.

Customer-USD joint/tendon/attachment overlays are Python-owned. Deformable bind/rest/simulation/collision views and particle/fluid/diffuse modes each require their own data provider. A triangle overlay or CPU buffer is not proof of all those modes. Keep missing legacy visualization modes as explicit parity gaps. No drawing callback steps physics or writes authored input.

**Host responsibility:** Renderer, selection/distance culling, per-shape color, helper handles and undeveloped specialized overlays.

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
- [REQ-UX-MASS-001](../requirements/features/REQ-UX-MASS-001.md)
- [REQ-UX-OVERLAY-001](../requirements/features/REQ-UX-OVERLAY-001.md)
- [REQ-COMP-DEBUG-001](../../../../plc/requirements/live/REQ-COMP-DEBUG-001.md)
- [REQ-COMP-DESCRIPTION-001](../../../../plc/requirements/authoring/REQ-COMP-DESCRIPTION-001.md)
- [REQ-COMP-COOK-001](../../../../plc/requirements/authoring/REQ-COMP-COOK-001.md)
- [REQ-COMP-IDENTITY-001](../../../../plc/requirements/exchange/REQ-COMP-IDENTITY-001.md)
- [REQ-COMP-BUFFERS-001](../../../../plc/requirements/exchange/REQ-COMP-BUFFERS-001.md)
