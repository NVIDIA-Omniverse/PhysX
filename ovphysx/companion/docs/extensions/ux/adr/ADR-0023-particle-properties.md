<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0023 — Preserve particle authoring while separating optional compute providers
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

Inspect and edit particle-system offsets/materials through customer USD with explicit validation and undo. Use companion `SamplingService` for supported stopped surface/volume sampling and `usd.particles` for copied results.

Keep particle-set, sampling, diffuse, smoothing, anisotropy and isosurface actions individually inventoried. CPU sampling is not their common implementation. Missing postprocess or GPU output providers must be explicit parity gaps and cannot publish fabricated results; schema authoring may remain usable. Panel refresh never samples or instantiates particles.

**Host responsibility:** Particle widgets/presets, resampling notices and missing postprocess/output providers.

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
- [REQ-UX-PARTICLE-WIDGET-001](../requirements/features/REQ-UX-PARTICLE-WIDGET-001.md)
- [REQ-COMP-PARTICLES-001](../../../../plc/requirements/authoring/REQ-COMP-PARTICLES-001.md)
- [REQ-COMP-USD-EDIT-001](../../../../plc/requirements/python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-BUFFERS-001](../../../../plc/requirements/exchange/REQ-COMP-BUFFERS-001.md)
- [REQ-COMP-HOST-001](../../../../plc/requirements/host/REQ-COMP-HOST-001.md)

- [REQ-UX-PARTICLE-ACTIONS-001](../requirements/features/REQ-UX-PARTICLE-ACTIONS-001.md)
