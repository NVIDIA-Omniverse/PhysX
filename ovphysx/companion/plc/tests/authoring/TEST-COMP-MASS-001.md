<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-MASS-001
maps_to: REQ-COMP-MASS-001
type: integration
---

## Coverage status

Implemented and executed. Covers [REQ-COMP-MASS-001](../../requirements/authoring/REQ-COMP-MASS-001.md) AC-1 through AC-4.

## Scenario

Compute mass for a transformed box while stopped and reject its result after an authoring edit.

## Given

- A uniform box of side lengths 2, 4 and 6 meters at density 10 kg/m^3, plus a translated/rotated copy and an explicit mass-override variant.
- No attached simulation and a result-application hook.

## When

- Compute all variants; edit geometry before applying one result; submit a degenerate collider.

## Then

- All computations complete with zero attach/step calls (REQ AC-1).
- The untransformed box has mass 480 kg and principal inertia (2080, 1600, 800) kg*m^2 within declared numerical tolerance, with correct bounds and metadata (REQ AC-2).
- Transforms/units/overrides change the appropriate values and the degenerate collider is diagnosed by identity (REQ AC-3).
- The stale result cannot author mass opinions; a fresh result applies through the shared Python helper (REQ AC-4).

## Executable coverage

- [Native general-mass tests](../../../tests_native/test_general_mass.py): analytic mesh/box values, winding, rigid transforms, compound inertia, authored overrides, copied jobs, topology diagnostics, native output canaries and library-only worker.
- [Customer USD tests](../../../tests/test_mass_usd.py): centimeter/mass unit conversion, density precedence/materials, nested-body exclusion, collider/body overrides, stale/cancel/delete handling, atomic undo/redo and unsupported geometry diagnostics.
- [Stopped mass editor sample](../../../samples/stopped_mass_editor.py): selection capture, copied worker computation and reversible mass authoring.

Executed: nine native general-mass tests and six customer-USD tests pass. This pure CPU service requires no GPU qualification.

## Review regressions

Customer-USD tests also exercise inactive MassAPI opinions, sampled primitive and
bound material inputs, instanced collider groups, and child capsule diagonal-only
and axes-only tensors. Native worker computations verify source-frame inertia.


Round-two customer regressions ignore leftover material density after MaterialAPI
removal, preserve authored/computed body principal axes distinctly from collider
rules, and weight a body's own collider before aggregate mass normalization.
Zero-sample spline geometry is rejected before mass computation.

Round-three `test_inactive_material_density_is_ignored` pairs an active
density-100 material with the same material deactivated; stopped mass is 100 kg
and 1000 kg respectively, each matching the native oracle.
