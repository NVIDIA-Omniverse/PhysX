<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-VALIDATE-MIGRATE-001
maps_to: REQ-UX-VALIDATE-MIGRATE-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Retain explicit schema migration and compatibility repairs, including failure, source replacement and exact undo/teardown.

## Given

Assets with deprecated PhysX attributes, convertible Newton joint/drive attributes and mimic relationships, plus ambiguous multiple-DOF and missing-target cases. Preserve authored reference list-ops and a sentinel unrelated schema.

## When

Detect all three migration families, approve one proposal at a time, rerun, undo/redo; replace the stage before applying another proposal. Repeat with a missing target codeless schema.

## Then

- Detection identifies exact old properties/API instances and never changes backend selection or the layer. Ambiguous cases are unresolved rather than clean. (REQ AC-1).
- Converted values and reference targets match independent fixture expectations and registered target schemas. Missing schemas and ambiguous constraints cause zero edits; the sentinel survives. (REQ AC-2).
- A second run yields no additional edits; undo/redo reproduces the old/new snapshots exactly. A stale proposal cannot write into the replacement stage. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-VALIDATE-MIGRATE-001](../../requirements/features/REQ-UX-VALIDATE-MIGRATE-001.md).
