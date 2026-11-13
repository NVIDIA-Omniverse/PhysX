<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-POPULATION-DESC-001
maps_to: REQ-CAPI-POPULATION-DESC-001
type: integration
---

## Scenario

The physics population description ovphysx publishes is the one generated from the population contract,
in C and in Python, and populates a stage the two ways its documentation names.

## Given

- The generator's normalized dump `tests/data/physics_population_desc.json`.
- An inline stage with `metersPerUnit = 0.01` and `upAxis = "Y"`: under a `/World` translated by (10, 0, 0),
  a PhysicsScene, a Cube with PhysicsRigidBodyAPI and PhysicsCollisionAPI translated locally by (1, 2, 3), a
  Plane with PhysicsCollisionAPI and a Sphere with no physics schema and `radius = 3`.
- Cylinder and Cone colliders with the custom float `physxConvexGeometry:margin`
  absent, authored as zero, or authored as 0.1, and a visual-only prim of the same type.
- Transitional, until the host adopts the published description (ADR-0062 §4): a frozen copy of the host's
  description (Kit `OvphysxPopulation.h` at `f912cf4e5b1`), and a fixture corpus of every composition's usda,
  authored and with every column absent (generated `CompositionCorpus.inl`), plus scene-graph instancing, a
  PointInstancer, collider GeomSubsets and material bindings, collision groups and filtering, joint
  relationship targets with multiple-apply schemas, and the families ADR-0061 open question 1 leaves out.

## When

- C, compiled as C11: `ovphysx_population.h` included on its own, and after `ovphysx.h`.
- C++: the accessor called twice, and from eight threads; its stage metadata paths and derived properties
  inspected.
- Python: the C accessor called through the bindings and its struct walked into normalized form;
  `physics_population_desc()` called twice and its `Desc` walked into the same form through the ctypes
  structs OVStage builds from it; `ovphysx.population` imported in a fresh interpreter.
- Python: the stage populated through the Python description alone, then beside
  `Desc(domains=PopulationDomain.RENDERING)`, and columns read back.
- C++, transitional: each corpus fixture populated twice, with the generated description alone and with the
  host's alone, and the two compared column by column through the physics input snapshot fingerprint (rows,
  paths, tokens and values), then prim by prim where a column differs.

## Then

- Both translation units compile, each naming `ovphysx_get_physics_population_desc` with its signature, and
  the header alone naming `OVSTAGE_POPULATION_DOMAIN_RENDERING` (AC-1).
- Every call returns the same non-NULL pointer, with `domains` NONE, the stage metadata paths
  `metersPerUnit`, `kilogramsPerUnit` and `upAxis`, and a value on every derived property, among them
  `physics:gravityMagnitude` (AC-1, AC-2).
- The C walk and the Python walk each equal the dump byte for byte; independent Python calls return isolated
  objects whose `domains` is NONE, including after a caller mutates a nested dtype (AC-2, AC-3).
- The fresh import leaves the description unbuilt and imports neither `ovstage` nor `warp` (AC-3).
- Alone: gravity magnitude 981, the cube's sleep threshold 50, the plane's contact offset 2, no contact offset
  on the cube and no `radius` on the sphere. Beside RENDERING: the same gravity and the sphere's `radius` 3.
  Both: after the seal the cube's `omni:fabric:worldMatrix` translation is the composed (11, 2, 3) (AC-2).
- In both description compositions, Cylinder and Cone collider margins equal the
  authored value, including explicit zero; absent margins default to zero. Alone,
  the description publishes no margin on the visual-only prim (AC-2, AC-3).
- `gen_contract.py --check` finds the generated header, Python functions, dump and reference page in sync, and
  the generator tests build, emit and dump the description (AC-4).
- The two populations of every fixture have the same columns, rows and values apart from the reviewed
  allowlist, each entry of which still matches a difference. Parser-consumed Newton inputs are never
  allowlisted; authored convex margins are preserved even where the frozen host loses them (AC-2).

## Test locations

- ovphysx/tests/c_unittests/test_physics_population_desc_header_only.c and
  test_physics_population_desc_after_primary.c (the `c_api_compatibility_test` target)
- ovphysx/tests/c_unittests/test_physics_population_desc.cpp (`PhysicsPopulationDesc.*`)
- ovphysx/tests/python_tests/test_physics_population_desc.py, with the walker
  ovphysx/tests/python_tests/population_desc_dump.py
- ovphysx/ovruntime/physics_population_domain/tools/test_gen_contract.py (`Description`,
  `CompositionCorpus`) and `gen_contract.py --check`
- ovphysx/ovruntime/physics_population_domain/tests/TestHostDescriptionParity.cpp, with
  tests/host/OvphysxPopulation.h and the generated tests/CompositionCorpus.inl
  (`ovruntime.population.test.unit`; transitional, deleted once the host adopts the description)

## Authored-input regressions

`test_physics_population_desc.py` registers external schemas before any population.
Physics-only population preserves the GPU scene environment-bit setting, particle-sampling
mesh geometry without CollisionAPI, cooking-source winding and holes, a material-bound
spline target's points/topology, and explicit vehicle reference-frame metadata (AC-2).
