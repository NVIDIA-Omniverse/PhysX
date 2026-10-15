<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-VALIDATE-PREVIEW-001
maps_to: REQ-UX-VALIDATE-PREVIEW-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Run simulation-dependent validation in an isolated companion owner, including failure, source replacement and exact undo/teardown.

## Given

Playback A paused at a known pose and epoch, one stable joint asset and one asset with a known joint-state violation; each validation fixture declares tolerance and a finite maximum of 120 steps. A controllable worker supplies Busy, timeout and failure cases.

## When

Explicitly request each validation using a copied source B; run the stable/violating fixtures, cancel one outstanding run and replace the initiating stage before another completes.

## Then

- Only B is created and stepped for validation; its source is distinct. A retains pose, step count, task queue and attachment identity. (REQ AC-1).
- Stable and violating fixtures produce their expected completed-state decisions at the declared tolerance. Busy, timeout and execution failure have separate non-clean outcomes and bounded work. (REQ AC-2).
- Cancelled/stale runs produce no repair or USD edit and retire B resources. A remains usable afterward; a fresh selected repair is applied only through its initiating edit transaction. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-VALIDATE-PREVIEW-001](../../requirements/features/REQ-UX-VALIDATE-PREVIEW-001.md).
