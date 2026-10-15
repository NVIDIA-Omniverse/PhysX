<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-AUTHOR-STAGE-002
maps_to: REQ-UX-AUTHOR-STAGE-001
type: integration
---

## Coverage status

Specification only — not implemented. Existing companion component tests verify the substrate, not this Kit scenario.

## Scenario

Associate two host stage generations, run delayed computation, replace/close the stage and drain only the old authoring/preview resources.

## Given

EditSession for S1, a pending copied cooking result and PreviewDisplay lease; S2 has a distinct generation and sentinel opinions. Hold completion before host delivery and inject an association failure for S2 in a separate run.

## When

Replace S1 with S2, release the obsolete result, retry after association failure, then author a transform and publish a completed preview frame.

## Then

- Teardown order is cancel delivery, drain jobs, close preview/output leases, close old EditSession, then associate S2. Failure releases only newly acquired resources and leaves generic USD editing usable. (REQ AC-2).
- The obsolete cook authors no S2 data or undo item. A current authored transform reaches the sealed input stream once; preview output notices do not return as authored input. Publication runs through the host pump even while the playback stage-update callback is disabled. (REQ AC-4).

## Why not implemented

The migrated Kit feature and host integration fixture are not present. Component examples are integration starting points; they do not establish full mode, widget or command parity.

Mapped requirement: [REQ-UX-AUTHOR-STAGE-001](../../requirements/features/REQ-UX-AUTHOR-STAGE-001.md).
