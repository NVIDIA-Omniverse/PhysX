<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0003 - Data-oriented authoring and live service contracts
status: accepted
date: 2026-09-30
last_updated: 2026-09-30
supersedes: none
---

## Status

Accepted by the project owner on 2026-09-30.

## Context / Motivation

Legacy authoring interfaces often combine USD access, private parser descriptors, caches and implicit timeline progress. Direct forwarding cannot work in a USD-free production runtime.

## Decision

1. Provide stopped authoring computations over immutable snapshots or raw buffers. Return versioned data plus identity, revision and diagnostics, never mutations to an authored stage.
2. Use explicit job submission, cancellation and drain for authoring. Live operations reuse the owning core task admission, execution order and completion semantics.
3. Return owned records or leases with release, layout, units, frame and CPU/GPU readiness metadata. Python views retain their owner.
4. Keep property derivation, cooking and physics descriptions distinct from live reads; advertise missing operations rather than exposing nonfunctional legacy tables.

**Out of scope**

- Private test hooks and direct cache/context destruction controls.
- A new competing queue for core simulation work.

## Alternatives considered

- Re-export private descriptor/table layouts: freezes allocator and implementation details.
- Drive all authoring through timeline callbacks: prevents standalone and stopped-stage use.
- Use arbitrary live attach to answer property queries: interferes with the actual simulation owner.

## Consequences

- **Positive:** Native clients, Python and host UI share computations without sharing USD objects.
- **Costs:** Implement immutable inputs, cancellation/teardown, typed error handling and device-aware buffers; port absent legacy computations explicitly.
- **Lifecycle:** Implementation status belongs in requirements and executable test evidence, not the ADR status.

### Multiple-attachment amendment (2026-09-30)

Decision 2 applies core task ordering to operations on a borrowed core owner.
An owned companion attachment has a distinct scheduler and exact native handle;
it does not enqueue Inspector simulation into playback's task stream. Native
owners have independent admission and can retain simultaneous pending intervals.
Same-owner conflicts and forbidden callback reentry report Busy without a
process-wide simulation lock. The companion must not infer public-facade thread
safety from the native contract. See
[REQ-COMP-JOBS-001](../requirements/exchange/REQ-COMP-JOBS-001.md) and
[Companion ADR-0007](ADR-0007-preview-ownership.md).

## References

- [REQ-COMP-JOBS-001](../requirements/exchange/REQ-COMP-JOBS-001.md)
- [REQ-COMP-BUFFERS-001](../requirements/exchange/REQ-COMP-BUFFERS-001.md)
- [REQ-COMP-MASS-001](../requirements/authoring/REQ-COMP-MASS-001.md)
- [REQ-COMP-COOK-001](../requirements/authoring/REQ-COMP-COOK-001.md)
- [REQ-COMP-DESCRIPTION-001](../requirements/authoring/REQ-COMP-DESCRIPTION-001.md)
- [REQ-COMP-PARTICLES-001](../requirements/authoring/REQ-COMP-PARTICLES-001.md)
