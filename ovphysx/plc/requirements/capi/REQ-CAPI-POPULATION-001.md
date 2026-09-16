<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-POPULATION-001
title: Population Domain Helpers and Documentation Shipped by ovphysx
status: implemented
owner: ovphysx
---

## Description

Consumers that construct physics objects directly in an ovstage Stage, without
USD, need to know which columns the parser reads, what it assumes when a
column is absent, and where that differs from a Stage populated from USD. The
runtime records this as a machine-readable contract and generates every
artifact from it (`REQ-POPULATION-CONTRACT-001` in the ovruntime tree,
[ADR-0030](../../../ovruntime/plc/adr/ADR-0030-physics-population-domain-contract.md)).
ovphysx ships three of those artifacts: the SDK copy of the C++ builder
header, the Python builder module, and the reference documentation. This
requirement covers those three and the drift check that keeps them in sync
with the contract. Behavioral verification against PhysX is the runtime's
generated doctest suite, not an ovphysx test.

## Acceptance Criteria

- AC-1: **Shipped surfaces.** `include/ovphysx/population/Population.hpp`
  (C++17, header-only, namespace `ovphysx::population`) and
  `ovphysx.population` (Python, warp arrays, imports warp and ovstage on first
  use only) expose one `define<Type>` per prim type component, one
  `apply<API>` per API schema component, `write`, stage-unit authoring,
  ancestor path derivation, resolved-default tables and one helper per
  composition. The header ships in the SDK through the public-header install
  and the module in the wheel through the package glob.
- AC-2: **Same contract, same rules.** Both surfaces are generated from the
  runtime contract by the same emitters as the runtime's own header, differing
  from it only in namespace, and enforce the same rules: an API is rejected on
  a prim type it does not apply to, a write is rejected when a required
  co-applied API is missing (same instance when both are multi-apply), every
  Xformable prim receives the local transform and reset-stack columns, the
  type and schema-list columns are written last, contract `bool` columns are
  published as DLPack `kDLBool`, and every write releases the path list and
  query it created. A value equal to a divergent raw schema fallback is not
  vetoed (a value does not encode authoredness); the divergence table is
  exposed for documentation.
- AC-3: **Documentation.** `docs/population/` holds one generated page per
  component with the raw fallback, the producer value and the parser default
  per column, plus interactions and known divergences, and is reachable from
  the docs index; the docs build passes with warnings as errors.
- AC-4: **Drift check.** `gen_contract.py --check` covers the three ovphysx
  artifacts together with the runtime ones, and the `ovphysx-population-contract-check`
  CI job runs it on every merge request.

## Test References

- [TEST-CAPI-POPULATION-001](../../tests/capi/TEST-CAPI-POPULATION-001.md)
- Behavioral coverage: TEST-POPULATION-CONTRACT-001 in the ovruntime tree.

## Code References

- ovphysx/include/ovphysx/population/Population.hpp (generated) (AC-1, AC-2)
- ovphysx/python/ovphysx/population.py (generated) (AC-1, AC-2)
- ovphysx/docs/population/ (generated), ovphysx/docs/index.md (toctree) (AC-3)
- ovphysx/ovruntime/physics_population_domain/tools/gen_contract.py (`emit_cpp`, `emit_python`, `emit_component_md`, `render_all`) (AC-1 to AC-4)
- ovphysx CI pipeline definition, job `ovphysx-population-contract-check` (AC-4; the file itself is not part of the open-source tree)
- ovphysx/tests/c_unittests/test_population_header.cpp (C++17 compile and rule smoke of the shipped header) (AC-1, AC-2)
- ovphysx/tests/python_tests/test_population_builder.py (lazy import, DLPack dtypes, rules, path-list ownership) (AC-1, AC-2)

## Dependencies

- REQ-POPULATION-CONTRACT-001 (ovruntime tree) — the contract, generator and verification suite these artifacts derive from
- [ADR-0030](../../../ovruntime/plc/adr/ADR-0030-physics-population-domain-contract.md)
