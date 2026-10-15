<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0002 — Withdraw the broad runtime-services proposal
status: rejected
date: 2026-09-18
last_updated: 2026-09-18
supersedes: none
---

## Status

Rejected following runtime scope review; retained to explain existing references.

## Context / Motivation

The initial proposal bundled simulation with editor maintenance, forced loading, object release/reset, preview, cooking, controllers and diagnostics. It also inferred missing live-stage adoption from an older API surface. Review established a narrower simulation-only runtime and confirmed that ovphysx already has ovstage attach and output utilities.

## Decision

Withdraw this proposal. It establishes no runtime service contract. Use ADR-0001 for the simulation boundary and the linked operation decisions for implementation behavior. In particular, force load, reset simulation, release objects and editor updates with `enableUpdate=false` are unsupported; stage open only remembers the stage, Play loads simulation, and Stop restores initial state and destroys the play session.

Existing UX references to this record identify assumptions requiring subsequent UX review. They do not authorize adding those services to the runtime simulation adapter.

## Alternatives considered

- Keep this proposal active alongside the detailed records: would leave contradictory requirements and misleading API-gap claims.

## Consequences

- **Positive:** old links remain resolvable without preserving the rejected service scope.
- **Cost:** reconcile affected UX proposals during their review before generating REQ/TEST records.
- **Lifecycle:** attach/detach, state capture/restoration, stepping, interaction and output belong to the dedicated records.

## References

- [Simulation boundary](ADR-0001-ovphysx-stageupdate-lifecycle.md).
- [Stage-open bookkeeping](ADR-0003-ovstage-attach.md).
- [Play-time ovstage loading](ADR-0004-start-resume-state-capture.md).
- [Existing ovstage output utilities](ADR-0010-ovstage-simulation-output.md).
