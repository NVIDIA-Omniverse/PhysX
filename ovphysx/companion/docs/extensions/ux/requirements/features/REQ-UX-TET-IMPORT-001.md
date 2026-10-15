<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-TET-IMPORT-001
title: Retain TetGen import as a Python authoring workflow
status: proposed
owner: physics-ux
---

## Description

Retain TetGen import as a Python authoring workflow. This source-derived extension obligation supplements the original feature-sheet requirements. Existing companion helpers are substrate only; the complete Kit workflow remains to be implemented and verified.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Files.** The importer shall accept .ele with matching .node and optional .face files, validating headers, index bases, counts, finite positions and element/face references before authoring. Missing or malformed required files produce a per-asset error with no partial mesh.
- AC-2: **Authoring.** Create unique tetrahedral and optional surface-mesh paths with explicit units/orientation and schema availability in customer USD. Importing existing tetrahedra is not a cooking request and shall initialize no native solver.
- AC-3: **Selection and undo.** Successful import selects only created prims and forms one reversible edit per asset/batch policy; failed items never select nonexistent prims. Cancel, stage replacement and importer unregister remove jobs/delegates and discard stale results.

## Test References

Specification only — not implemented.

- [TEST-UX-TET-IMPORT-001](../../tests/features/TEST-UX-TET-IMPORT-001.md)

## Code References

- [usd/deformables](../../../../../python/ovphysx_companion/usd/deformables.py) — existing companion substrate; host integration remains proposed.
- [usd/meshes](../../../../../python/ovphysx_companion/usd/meshes.py) — existing companion substrate; host integration remains proposed.
- [usd/session](../../../../../python/ovphysx_companion/usd/session.py) — existing companion substrate; host integration remains proposed.

Companion integration anchors are listed below; the host extension implementation is not present here.


## Dependencies

- [ADR-0021-deformable-properties](../../adr/ADR-0021-deformable-properties.md)
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md)
