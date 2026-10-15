<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-TET-IMPORT-001
maps_to: REQ-UX-TET-IMPORT-001
type: integration
---

## Coverage status

Specification only — not implemented. Companion component tests do not execute this complete Kit workflow.

## Scenario

Retain TetGen import as a Python authoring workflow, including failure, source replacement and exact undo/teardown.

## Given

Tiny .node/.ele files for one tetrahedron in zero- and one-based indexing, optional four-face .face input, an existing prim at the requested path, and separate missing-node, nonfinite-coordinate, bad-count and out-of-range-index files.

## When

Import each valid pair with and without faces, then each invalid asset; undo/redo successes. Replace the stage while file parsing is held and unregister the importer.

## Then

- Valid data resolve identical tetrahedral connectivity after index normalization; each malformed input reports its asset-specific error and produces no partial prim. (REQ AC-1).
- Successful paths are unique; optional surface faces match the requested file, units and orientation. Solver creation/step counts remain zero. (REQ AC-2).
- Only successful prims enter selection. Undo removes exactly imported opinions, redo reconstructs them, and stale parses publish nothing. Unregister leaves no import delegate or job. (REQ AC-3).

## Why not implemented

The Kit adapter and complete retained-feature fixtures have not been implemented. Use the mapped source audit and companion component tests to build this scenario; a capability-error test alone is not positive workflow coverage.

Mapped requirement: [REQ-UX-TET-IMPORT-001](../../requirements/features/REQ-UX-TET-IMPORT-001.md).
