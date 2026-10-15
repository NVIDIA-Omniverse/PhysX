<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0002 - Ovstage as scene data with explicit output publication
status: accepted
date: 2026-09-30
last_updated: 2026-10-02
supersedes: none
---

## Status

Accepted by the project owner on 2026-09-30.

## Context / Motivation

Authoring and simulation both change scene state. Treating solver output as newly authored input causes feedback; writing ovstage does not itself author the customer USD stage.

## Decision

1. The application owns ovstage, population and sealed input revisions. Runtime attachment borrows that stage and consumes explicit input updates.
   Companion population adapters and examples use `PopulationDomain.ALL` for
   OVStage 0.3 to resolve nested world transforms. A host selecting PHYSICS-only
   must remove its local world-matrix seeds before sealing, as described by core
   `REQ-CAPI-OVSTAGE-XFORM-001`; explicit world-matrix producers retain their
   authored values. This does not widen the physics snapshot capture contract.
2. Read simulation output through core sessions and publish selected data through a host-owned adapter at a separate output ordinal. Never submit that ordinal as authored input. On the shared ovstage, output
   columns use the reserved `companion:` namespace: publishing solver-input
   properties is rejected because a later same-object reparse reads their current
   values even when the output ordinal itself was skipped.
3. Make customer-USD writeback a separate optional Python operation with explicit destination and notice suppression. Ovstage revision filtering alone does not suppress USD notices.
4. Allow detached authoring computations to consume copied data or a retained stable scene revision without taking over the live attachment.

**Out of scope**

- Replacing ovstage population internals or claiming they contain no USD.
- Automatic persistence of simulation results.

## Alternatives considered

- Let every step implicitly write all destinations: obscures output ownership and introduces feedback.
- Use customer USD objects as the native data interface: binds the runtime to a USD ABI.
- Treat ovstage as a second simulation controller: conflates data lifetime and stepping ownership.

## Consequences

- **Positive:** The same scene-data contract supports headless simulation, UI output and snapshot computation.
- **Costs:** The host must manage revisions, stage lifetime, output selection and a separate USD-notice policy.
- **Lifecycle:** Implementation status belongs in requirements and executable test evidence, not the ADR status.

## References

- [REQ-COMP-OVSTAGE-001](../requirements/exchange/REQ-COMP-OVSTAGE-001.md)
- [REQ-COMP-IDENTITY-001](../requirements/exchange/REQ-COMP-IDENTITY-001.md)
- [REQ-COMP-DEFORM-WRITE-001](../requirements/python/REQ-COMP-DEFORM-WRITE-001.md)
