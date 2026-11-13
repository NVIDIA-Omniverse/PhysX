<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-POPULATION-DESC-001
title: Physics Population Description Published by ovphysx
status: proposed
owner: ovphysx
---

## Description

OVStage populates physics data from USD only through population descriptions passed to its
`*_with_desc` entry points; it has no physics domain of its own. ovphysx publishes the description of
everything its parser reads
([ADR-0061](../../../ovruntime/plc/adr/ADR-0061-ovphysx-publishes-the-physics-population-description.md)),
generated from the population contract
([ADR-0062](../../../ovruntime/plc/adr/ADR-0062-population-description-generated-from-the-contract.md),
REQ-POPULATION-CONTRACT-001 in the ovruntime tree): `ovphysx_get_physics_population_desc` in C and
`ovphysx.population.physics_population_desc` in Python.

## Acceptance Criteria

- AC-1: **C accessor.** `const ovstage_population_desc_t* ovphysx_get_physics_population_desc(void)` is
  declared in `ovphysx/ovphysx_population.h`, which compiles on its own and alongside `ovphysx.h`;
  `ovphysx.h` does not include it. It returns the same non-NULL pointer on every call, from any thread,
  without `ovphysx_initialize`, an instance or an attached stage. The description is immutable static
  data, valid while the library stays loaded, built once on first use.
- AC-2: **Content.** The description is the one the generator builds from the population contract: its
  `domains` is `OVSTAGE_POPULATION_DOMAIN_NONE`, and walked into normalized form it equals the generated
  dump `tests/data/physics_population_desc.json`. Passed alone to an OVStage `*_with_desc` entry point it
  populates the physics prims, their properties and the derived values the contract's producer rules state
  (for example unit-scaled gravity and thresholds, a Plane collider's contact offset) and no render-only
  prim; passed beside a description whose `domains` is `OVSTAGE_POPULATION_DOMAIN_RENDERING` it populates
  both. Either way, once the population ordinal is sealed, `omni:fabric:worldMatrix` holds composed world
  transforms, so a body under a transformed parent needs no RENDERING and no producer-side preparation.
- AC-3: **Python.** `ovphysx.population.physics_population_desc()` returns an `ovstage.population.Desc`
  whose normalized form equals the same dump. Each call builds an isolated graph, including nested dtype objects; importing `ovphysx.population` neither builds it nor imports `ovstage` or `warp`.
  `physics_population_descs(rendering=False)` returns the list to populate with: that description alone,
  or with `rendering` a `Desc(domains=PopulationDomain.RENDERING)` after it.
- AC-4: **Generated, not written.** The C++ header that builds the description (included by the C accessor
  and by the runtime's tests), the Python functions, the dump and the reference page
  `docs/population/physics-population-description.md` are emitted by `gen_contract.py` and covered by its
  `--check` drift test; no part of the description is maintained outside the contract directory.
- AC-5: **Documentation.** The header states the lifetime, threading and the composition with a RENDERING
  description; the C and Python API references name the function and link the reference page.

## Test References

- [TEST-CAPI-POPULATION-DESC-001](../../tests/capi/TEST-CAPI-POPULATION-DESC-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx_population.h, ovphysx/src/ovphysx/ovphysxPopulation.cpp: AC-1, AC-5
- ovphysx/ovruntime/include/omni/physics/population/PhysicsPopulationDesc.h (generated): AC-1, AC-2
- ovphysx/python/ovphysx/population.py (`physics_population_desc`, `physics_population_descs`, generated) and
  ovphysx/python/ovphysx/_bindings.py: AC-3
- ovphysx/ovruntime/physics_population_domain/tools/gen_contract.py (`build_description` and the
  description emitters), ovphysx/ovruntime/physics_population_domain/description.json: AC-2, AC-4
- ovphysx/docs/api.md, ovphysx/docs/python_api.rst, ovphysx/docs/population/physics-population-description.md
  (generated): AC-5

## Dependencies

- REQ-POPULATION-CONTRACT-001 (ovruntime: the contract the description is generated from)
- REQ-CAPI-POPULATION-001 (the population builders and reference shipped from the same contract)
