<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-MASS-001
title: "Stopped mass and inertia computation"
status: implemented
owner: ovphysx
---

## Description

Replace USD-stage-dependent property queries with a source-neutral computation for mass manipulators and property-query consumers. Live mass reads remain core reads.

## Acceptance Criteria

- AC-1: **Stopped computation.** compute_mass_properties consumes mesh/shape data, transforms, density/material and authored mass overrides without attaching or stepping a simulation.
- AC-2: **Complete result.** The result includes mass, center of mass, diagonal inertia, principal axes, collider bounds/volume, input revision, units and frames.
- AC-3: **Authored semantics.** The computation honors authored mass/inertia overrides, collider transforms and stage units; invalid or degenerate inputs return object-specific diagnostics.
- AC-4: **Stale application.** Mass manipulators apply only a matching-revision result through Python USD edit helpers; cancellation, deletion or undo cannot trigger a late edit.

## Implementation scope

Stopped native primitive and closed triangle mesh aggregation supports copied SI
inputs, body/collider overrides and exact bounds. Optional customer-USD capture
resolves collider/body/material density precedence and converts stage units.
A bound material contributes density only if it is populated at runtime: it
must carry MaterialAPI and pass USD's default traversal (active, loaded,
defined, non-abstract); otherwise its collider uses the default density.
Meshes require a connected oriented closed non-self-intersecting triangular
boundary. Unsupported approximations, shear/reflection, unequal sphere/radial
capsule scales and animated inputs are explicit errors; no implicit repair or
shape approximation occurs. Python application validates source revision and
returns an atomic undoable layer patch. The nine native and six customer-USD focused tests pass.

## Test References

- [TEST-COMP-MASS-002](../../tests/authoring/TEST-COMP-MASS-002.md) - executed native/Python subset.

- [TEST-COMP-MASS-001](../../tests/authoring/TEST-COMP-MASS-001.md) - general mass and USD application scenario.

## Code References

Implemented companion subset (native/core changes retain their own PLC ownership):

- [include/ovphysx_companion/companion.h](../../../include/ovphysx_companion/companion.h)
- [src/Companion.cpp](../../../src/Companion.cpp)
- [python/ovphysx_companion/native.py](../../../python/ovphysx_companion/native.py)
- [samples/mass_properties.py](../../../samples/mass_properties.py)

General mass implementation:

- [ovphysx/ovruntime/source/omni.physx/plugins/usdBridge/PropertyQueryBridge.cpp](../../../../ovruntime/source/omni.physx/plugins/usdBridge/PropertyQueryBridge.cpp)
- [ovphysx/ovruntime/include/omni/physx/IPhysxCookingService.h](../../../../ovruntime/include/omni/physx/IPhysxCookingService.h)

- [C table](../../../include/ovphysx_companion/mass.h)
- [Bridge](../../../src/Mass.cpp)
- [Copied requests/results](../../../python/ovphysx_companion/mass.py)
- [Customer USD capture/application](../../../python/ovphysx_companion/usd/mass.py)
- [Stopped editor sample](../../../samples/stopped_mass_editor.py)

## Dependencies

- [REQ-COMP-JOBS-001](../exchange/REQ-COMP-JOBS-001.md)
- [REQ-COMP-USD-EDIT-001](../python/REQ-COMP-USD-EDIT-001.md)
- [Companion ADR-0003](../../adr/ADR-0003-data-services-and-jobs.md)

## Capture correctness

Body/collider mass, density, center and inertia opinions require applied MassAPI;
leftover attributes after API removal are inactive. Bound material density remains
a separate source. Traverse instance proxies and reject an instanced collider
instead of returning a partial compound result; continue pruning nested bodies.
Every consumed primitive, geometry, mass and bound-density attribute must be
static. Collider inertia is authored in its USD source frame: convert capsule axes
for an authored diagonal; an axes-only opinion does not rotate computed inertia.
