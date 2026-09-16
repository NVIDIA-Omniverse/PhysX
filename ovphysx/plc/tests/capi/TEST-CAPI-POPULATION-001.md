<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-POPULATION-001
maps_to: REQ-CAPI-POPULATION-001
type: integration
---

## Scenario

The ovphysx-shipped population artifacts are exactly what the runtime contract
generates, and the shipped header compiles for an SDK consumer. Covers AC-1
through AC-4. The behavior of what the builders write is verified by the
runtime's generated doctest suite (TEST-POPULATION-CONTRACT-001), which this
spec does not duplicate.

## Given

- A checkout with the committed generated artifacts and the contract under
  `ovruntime/physics_population_domain/`.
- The pinned ovstage headers, which the SDK header includes.

## When

- `gen_contract.py --check` and the generator's negative fixtures
  (`tools/test_gen_contract.py`) run in the `ovphysx-population-contract-check`
  CI job.
- `c_unittests` builds and runs `test_population_header.cpp`, a C++17
  translation unit that includes `ovphysx/population/Population.hpp` against
  the pinned ovstage include tree, creates an ovstage instance and drives the
  builder without PhysX.
- The ovphysx docs build runs with warnings as errors.
- The Python test suite runs `test_population_builder.py`: a fresh interpreter
  imports `ovphysx.population`, then the builder is driven against an ovstage
  Stage with warp present.

## Then

- The check reports every artifact in sync and no stale generated page or
  test source; each negative fixture is rejected with the documented message
  (AC-4).
- The header compiles and its namespace is `ovphysx::population`; the runtime
  copy differs only in namespace; resolved defaults follow the stage units,
  ancestor paths are derived once, an API on a type it does not apply to, a
  write missing a required API and an envelope on another drive instance
  throw `std::invalid_argument`, and a rigid Cube batch writes (AC-1, AC-2).
- The population pages and the findings page render and are linked from the
  index (AC-3).
- The import pulls neither warp nor ovstage; contract bool columns reach
  ovstage as `kDLBool`, prebuilt arrays of another dtype are rejected, the same
  three rule violations raise `ValueError`, an authored zero friction is
  written, and a helper write emits no `ResourceWarning` for its path list
  (AC-1, AC-2).
