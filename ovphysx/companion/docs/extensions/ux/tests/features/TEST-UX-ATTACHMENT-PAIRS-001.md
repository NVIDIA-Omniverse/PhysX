<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-ATTACHMENT-PAIRS-001
maps_to: REQ-UX-ATTACHMENT-PAIRS-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Preserve attachment discovery and element-filter workflows, including failure, source replacement and exact undo/teardown.

## Given

Two disconnected tetrahedral islands, an overlapping rigid endpoint, an already attached pair and a nonoverlapping endpoint; equivalent stages use metersPerUnit 1 and 0.01. Store independent expected candidate pairs and anchor barycentric weights for simple tetrahedra.

## When

Compute candidates at a fixed physical tolerance, select one island pair, compute anchors and element filters separately, then apply/undo. Hold another job before completion and change tolerance, topology or selection; repeat with a curve endpoint and read-only target.

## Then

- Both unit variants yield the same physical candidates and island grouping; the existing pair is excluded and the nonoverlapping endpoint remains absent. (REQ AC-1).
- Computed anchors reconstruct the fixture positions within 1e-5; filter indices address the declared endpoint domains. Unsupported curves yield no partial attachment. (REQ AC-2).
- Only the selected fresh pair is authored as one undo item; exact undo restores baseline. Cancelled, stale and read-only variants author no opinions and leave unrelated preview state intact. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-ATTACHMENT-PAIRS-001](../../requirements/features/REQ-UX-ATTACHMENT-PAIRS-001.md).
