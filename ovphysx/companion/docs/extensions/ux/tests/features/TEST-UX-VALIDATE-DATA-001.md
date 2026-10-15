<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-VALIDATE-DATA-001
maps_to: REQ-UX-VALIDATE-DATA-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Retain static physics asset checks and selected repairs, including failure, source replacement and exact undo/teardown.

## Given

An asset per static/cooking rule in the validator catalog: both a clean control and a single injected defect, including conflicting APIs, an invalid articulation joint, displaced joint frames, negative mass, wrong simulation owner, malformed deformable roles and an invalid collision approximation.

## When

Run all checks without applying repairs; select one repair, apply and undo. Hold another proposal and edit its source before applying; repeat with a read-only layer and one absent rule provider.

## Then

- Each defect yields the expected stable code, severity and exact path; clean controls yield no defect. USD-only checks create zero native attachments. (REQ AC-1).
- Validation does not change any layer. A selected fresh repair changes only its target, fixes its defect on rerun and is exactly undoable; stale/read-only repairs edit nothing. (REQ AC-2).
- The absent rule is listed as incomplete and the overall result is not all-valid. Each catalog rule has a positive fixture, rather than inheriting coverage from a generic check. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-VALIDATE-DATA-001](../../requirements/features/REQ-UX-VALIDATE-DATA-001.md).
