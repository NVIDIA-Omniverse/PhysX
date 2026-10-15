<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
title: ADR-0005 - Attachment and deformable computation separated from USD edits
status: accepted
date: 2026-09-30
last_updated: 2026-09-30
supersedes: none
---

## Status

Accepted by the project owner on 2026-09-30.

## Context / Motivation

Attachment construction and deformable output need geometric correspondence, not just generic arrays. Simulation, collision and visual meshes can have different topology and frames.

## Decision

1. Compute matching, barycentric coordinates, filter proposals, tetrahedral meshes and embedding in USD-free native services. Return mesh-role identities, topology versions and complete mapping data.
2. Extract authoring inputs and apply attachment/deformable schemas, relationships and rest data in shared customer-USD Python helpers.
3. Separate topology/rest authoring from dynamic simulation writeback. Dynamic output uses a validated mapping and writes only selected points/velocities at the explicit preview or bake time.
4. Transfer GPU data to CPU only when USD application requires it, after completion; keep GPU/ovstage consumers independent of that transfer.
5. Reject stale mappings before edits and manage only helper-owned generated content. Host undo and notice filtering wrap the same Python utilities.

**Out of scope**

- Guaranteeing arbitrary mesh correspondence without an embedding method.
- Python callbacks inside per-vertex native interpolation or solver execution.

## Alternatives considered

- Native auto-attachment USD mutation: restores the forbidden native USD dependency.
- Write simulation points directly to every visual mesh: fails when topology, frames or units differ.
- Recook/rewrite rest state on each frame: changes authored physics and destroys the rest/dynamic distinction.

## Consequences

- **Positive:** Attachment and deformable workflows become usable headlessly and in Kit with the same data contracts.
- **Costs:** Define mapping schemas, topology validation, managed-child ownership and separate authoring/writeback tests.
- **Lifecycle:** Implementation status belongs in requirements and executable test evidence, not the ADR status.

## References

- [REQ-COMP-ATTACH-COMPUTE-001](../requirements/authoring/REQ-COMP-ATTACH-COMPUTE-001.md)
- [REQ-COMP-ATTACH-USD-001](../requirements/python/REQ-COMP-ATTACH-USD-001.md)
- [REQ-COMP-DEFORM-COOK-001](../requirements/authoring/REQ-COMP-DEFORM-COOK-001.md)
- [REQ-COMP-DEFORM-USD-001](../requirements/python/REQ-COMP-DEFORM-USD-001.md)
- [REQ-COMP-DEFORM-WRITE-001](../requirements/python/REQ-COMP-DEFORM-WRITE-001.md)
