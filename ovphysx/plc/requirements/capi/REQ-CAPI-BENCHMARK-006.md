<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BENCHMARK-006
title: Output Write Benchmark Validity
status: implemented
owner: ovphysx
---

## Description

`OutputWrite` rows measure completed writes against populated scenes. This
baseline covers the fixture and setup failures found in the normal benchmark
suite, including binding comparators that previously failed before measurement.

## Acceptance Criteria

- AC-1: **Fixtures.** Every normal CPU OutputWrite row resolves its committed
  scene dependencies. Its scene explicitly selects CPU dynamics and MBP.
- AC-2: **Valid payload.** Readable write columns use owned snapshots captured
  before warm-up, with borrowed read groups released before writing. Pose
  binding inputs preserve the loaded body positions and valid orientations.
  Force and wrench inputs contain valid zero forces. A session fills every
  tensor and dimension it commits, including ARRAY and per-shape columns.
- AC-3: **Binding coverage.** Binding warm-up queries the same object type and
  attribute as its session counterpart. Coverage is nonempty and equal in
  scalar count, except pose bindings, whose seven-float rows must match the
  three-float position session in object count.
- AC-4: **Failure visibility.** Setup, payload preparation, layout and write
  failures suppress the affected row and make the process fail. Successful
  selected CPU rows emit positive timings; missing fixture data cannot pass.
- AC-5: **Instancer coverage.** Instancer rows author every environment instead
  of using the replicator. Warm-up requires exactly the selected environment
  count in ARRAY groups, each with one prim and twenty three-float instances;
  standalone rigid-body groups cannot publish as an instancer measurement.
  The authored scene is retained across measured runs and released with the
  benchmark row; each run still prepares valid input and warms its sessions.

## Test References

- [TEST-CAPI-BENCHMARK-006](../../tests/capi/TEST-CAPI-BENCHMARK-006.md): focused
  CI contract, full 33-row CPU acceptance, and zero-input source inspection.

## Code References

- ovphysx/tests/benchmarks/benchmarks/OutputWrite.cpp
- ovphysx/tests/benchmarks/data/cubes20_write_cpu.usda
- ovphysx/tests/benchmarks/data/cartpole_write_cpu.usda
- ovphysx/tests/benchmarks/data/gen_cubes20_compound.py
- ovphysx/tests/benchmarks/data/gen_cubes20_instancer.py
- ovphysx/tests/benchmarks/data/gen_cartpole_tendon_probe.py
- ovphysx/scripts/test_output_write_contract.cmake

## Dependencies

- REQ-CAPI-BENCHMARK-001: shared exact-row failure suppression and baseline transaction.
