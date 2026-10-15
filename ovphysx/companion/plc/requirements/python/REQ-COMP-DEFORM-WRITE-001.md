<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-DEFORM-WRITE-001
title: "Deformable simulation output to customer USD"
status: implemented
owner: ovphysx
---

## Description

Write completed dynamic deformable data to customer USD through Python, independently of ovstage output and rest-state authoring.

## Acceptance Criteria

- AC-1: **Destination mapping.** write_simulation_output validates body/mesh identity and topology, converts frames/units and applies the declared simulation-to-collision/visual mapping before writing points or supported velocities.
- AC-2: **Ready arrays.** GPU completion and explicit CPU transfer precede USD access; arrays are applied in batches per mesh, without Python per-vertex loops or mandatory CPU copies for non-USD consumers.
- AC-3: **Time policy.** The caller selects preview-layer/default-value output or a bake time sample; each frame carries simulation step/time and writes neither rest/bind values nor unchanged topology.
- AC-4: **No feedback.** Stale mappings fail before edits, and the host output-notice policy prevents USD writeback from becoming a new physics input; ovstage and USD outputs can be selected independently.

## Implementation scope

`capture_deformable_output` copies every ragged prim returned by core reads, with explicit consent for GPU-to-CPU transfer. `map_deformable_output` uses native interpolation and converts simulation-local points and world velocities into destination-local stage units. `write_simulation_output` validates polygon/TetMesh topology and publishes a preflighted batch at explicit default/sample times with reversible edits and output-notice guards. Triangle normal-offset velocity interpolation includes the current normal derivative. Native mapping and the read transport contract are tested; actual GPU deformable simulation/extraction is not qualified on this host.

## Test References

- [TEST-COMP-DEFORM-WRITE-002](../../tests/python/TEST-COMP-DEFORM-WRITE-002.md) - executed Python subset; see its scenario and limitations.
- [TEST-COMP-DEFORM-WRITE-001](../../tests/python/TEST-COMP-DEFORM-WRITE-001.md) - executed supported-contract scenario.

## Code References

Companion implementation:

- [Output capture](../../../python/ovphysx_companion/output.py)
- [Native interpolation](../../../python/ovphysx_companion/mesh.py)
- [Mapped authoring tests](../../../tests/test_deformable_authoring.py)
- [Read transport tests](../../../tests_native/test_output.py)

- [usd/writeback.py](../../../python/ovphysx_companion/usd/writeback.py)
- [usd/meshes.py](../../../python/ovphysx_companion/usd/meshes.py)
- [usd/session.py](../../../python/ovphysx_companion/usd/session.py)
- [Executable subset tests](../../../tests/test_usd_helpers.py)

Existing integration anchors; these do not establish native implementation:

- [ovphysx/python/ovphysx/utils/simulation.py](../../../../python/ovphysx/utils/simulation.py)
- [ovphysx/python/ovphysx/utils/deformable.py](../../../../python/ovphysx/utils/deformable.py)
- [ovphysx/python/ovphysx/api.py](../../../../python/ovphysx/api.py)

## Dependencies

- [REQ-COMP-DEFORM-COOK-001](../authoring/REQ-COMP-DEFORM-COOK-001.md)
- [REQ-COMP-USD-EDIT-001](REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-OVSTAGE-001](../exchange/REQ-COMP-OVSTAGE-001.md)
- [Companion ADR-0005](../../adr/ADR-0005-attachment-and-deformable-split.md)
