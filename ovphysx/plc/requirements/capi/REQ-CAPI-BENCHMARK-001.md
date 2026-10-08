<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BENCHMARK-001
title: Hidden In-Process Authoring Benchmark Contract
status: implemented
owner: ovphysx
---

## Description

The C++ benchmark suite provides a frozen set of synthetic, in-process L1B
producer-path measurements. These rows start at public population, OVStage, or
tensor calls and stop after the associated physics step. They do not include a
container, gRPC, serialization, transport, client scheduling, or an application
loop and therefore are not end-to-end customer measurements.

The dedicated CPU contract contains the following twelve rows:

| Row | Role | Measured operation |
|---|---|---|
| `Authoring.population_add_drip_cpu` | primary KPI | 80 one-body population add, apply, seal/drain, and step cycles |
| `Authoring.population_add_packed_cpu` | diagnostic | 80 bodies in five 16-body population add, apply, seal/drain, and step cycles |
| `Authoring.runtime_write_spawn_cpu` | diagnostic | 80 bodies written through OVStage columns in five drained and stepped batches |
| `Authoring.population_churn_cpu` | diagnostic | same-frame population create, drain, remove, drain, and step churn |
| `Authoring.population_scale_cpu` | diagnostic | one-body population add, apply, seal/drain, and step into an accumulating scene; reports per-spawn cost drift against scene size |
| `Authoring.teleport_ovstage_cpu` | diagnostic | existing-body `omni:xform` write, seal/drain, and step |
| `Authoring.teleport_tensor_cpu` | diagnostic | existing-body pose tensor write and step |
| `Authoring.drain_ovstage_cpu` | diagnostic | velocity OVStage write, seal/drain, and step |
| `Authoring.drain_tensor_cpu` | diagnostic | velocity tensor write and step |
| `Authoring.drain_step_only_cpu` | diagnostic | step only |
| `WriteScaling.velocity_ovstage_4096_cpu` | primary KPI | one 4,096-body velocity OVStage write, scoped floor, update, and step |
| `WriteScaling.velocity_tensor_4096_cpu` | comparator | one persistent 4,096-body velocity tensor write and step |

The six matching `Authoring.*_gpu` population-add, runtime-write-spawn,
population-churn, and teleport rows are retained only as requested-GPU
diagnostics. All eighteen rows are hidden.

## Acceptance Criteria

- AC-1: The pinned producer inventory contains exactly the twelve CPU rows and
  six requested-GPU rows described above. Every row is registered hidden and
  is absent from normal wildcard execution unless `--hidden` is explicit. The
  CMake driver propagates `BENCHMARK_HIDDEN=1` to every enabled pass, rejects an
  empty pass selection, and accepts `BENCHMARK_EXPECT_ROWS=N` only for exactly
  one enabled pass; that pass fails unless its fresh report contains exactly
  `N` positive data rows.
- AC-2: Only `Authoring.population_add_drip_cpu` and
  `WriteScaling.velocity_ovstage_4096_cpu` are lower-is-better absolute-latency
  primary KPIs. Every other row is a diagnostic or comparator and no derived
  ratio, delta, or slope is a KPI. For each step index, the harness sorts its
  `R` measured run times and takes the mean over the inclusive zero-based range
  `[max(1, floor(0.2*R)), max(min(floor(0.7*R), R-2), 1)]`; the row result is
  the mean of those per-step trimmed means. Both primary rows use `R=5` and
  retain three runs per step. Population-add-drip has one measured step.
  WriteScaling has twenty measured step indices. Reported `std_dev` is the
  mean of the per-step population standard deviations over each step's retained
  runs, not variance between benchmark invocations.
- AC-3: The population-add-drip timed step comprises all 80 one-body
  population-reference add/wait, apply/wait, write-floor seal/drain, and
  1/60-second synchronous-step cycles. Each WriteScaling OVStage timed step
  comprises its 4,096-body velocity query/write and completion wait, query
  release, velocity-scoped floor and completion wait, OVStage update, then an
  asynchronous 1/60-second step and `waitAll`; its tensor comparator uses a
  persistent `[4096,6]` velocity binding, write completion, and the same
  step/wait shape. Scene construction, population/attachment, same-stage
  warmup, and semantic validation remain outside these measured windows.
- AC-4: Setup, enqueue, wait, release, update, step, readback, and teardown
  failures are recorded against the exact row. Every wait these rows perform is
  bounded and every status they can observe is inspected, including the wait
  call's own status alongside its per-operation error list, so a timeout or an
  invalid handle is a failure rather than a silent success; a bounded wait that
  expires records the failure and then waits for final completion so teardown
  never releases state a pending operation still references. A row with an
  operational or semantic failure publishes no timing record and the benchmark
  process exits nonzero. A row that never executed -- because it was filtered
  out, device-gated, or failed before its first timed sample -- publishes no
  record at all rather than a zero-valued one, so `--regenerate` cannot write a
  zero baseline entry that disables later comparison for that row. For the same
  reason, a verification run that invokes `--regenerate` under a narrow filter
  must leave any pre-existing developer `_baseline.txt` exactly as it found it,
  and must not leave a narrow one where none existed; that restoration is
  verified rather than assumed, and an interrupted run leaves recoverable state
  the next run completes before it can overwrite anything. Positive
  post-timing gates prove the expected body count for growth rows, exact-path
  creation and removal plus return to the body-count floor for churn, pose or
  velocity effects for write rows, and the 4,096-body written velocity for both
  WriteScaling lanes. The driver converts a nonzero process, missing report, or
  requested exact-row mismatch into a hard failure. Before converting a
  benchmark-process result into that hard failure, it writes a fresh JSON
  status sidecar containing the exact process return value; it writes the same
  sidecar for success. This lets an external smoke classifier distinguish a
  loader failure, crash, and normal exit without scraping human-readable logs.
- AC-5: CPU is the canonical L1B device policy. CPU Authoring rows select
  fixtures that explicitly author CPU dynamics and MBP, and WriteScaling
  verifies equivalent authoring on its generated PhysicsScene. Requested-GPU
  rows remain hidden, unscheduled diagnostics and are never compared with CPU.
  Their callback, active across attachment and warm-up, recognizes CPU-fallback
  and GPU contact-capacity warnings, either of which fails the requested-GPU
  row. Silence in that window is only negative fallback and capacity evidence
  because ovphysx exposes no realized-device query. This pass/fail rule applies
  to all six hidden GPU diagnostics without changing their frozen names or the
  CPU KPI contract. All Authoring rows call the current `ovphysx_warmup()` API
  after attachment.

## Test References

- TEST-CAPI-BENCHMARK-001

## Code References

- ovphysx/scripts/benchmark_driver_common.cmake
- ovphysx/scripts/test_benchmarks_cpp.cmake
- ovphysx/scripts/test_benchmark_contract.cmake
- ovphysx/scripts/benchmark_baseline_txn.cmake
- ovphysx/scripts/benchmark_baseline_txn_cli.cmake
- ovphysx/scripts/test_benchmark_baseline_txn.cmake
- ovphysx/tests/benchmarks/Harness.cpp
- ovphysx/tests/benchmarks/BenchmarkFailure.cpp
- ovphysx/tests/benchmarks/framework/BmGlobals.h
- ovphysx/tests/benchmarks/framework/BmGlobals.cpp
- ovphysx/tests/benchmarks/framework/BmOutput.cpp
- ovphysx/tests/benchmarks/benchmarks/AuthoringCommon.h
- ovphysx/tests/benchmarks/benchmarks/AuthoringPopulationAdd.cpp
- ovphysx/tests/benchmarks/benchmarks/AuthoringTeleport.cpp
- ovphysx/tests/benchmarks/benchmarks/AuthoringDrain.cpp
- ovphysx/tests/benchmarks/benchmarks/AuthoringPopulationChurn.cpp
- ovphysx/tests/benchmarks/benchmarks/AuthoringRuntimeWriteSpawn.cpp
- ovphysx/tests/benchmarks/benchmarks/WriteScaling.cpp
- ovphysx/tests/benchmarks/tests/producer_inventory.json
- ovphysx/tests/benchmarks/tests/verify_inventory.py
- ovphysx/tests/benchmarks/tests/test_benchmark_driver.py
- ovphysx/tests/benchmarks/tests/benchmark_driver_probe.cmake

Two files are deliberately absent.
`ovphysx/tests/benchmarks/OvstageLoad.h`: its unbounded-wait convenience
helpers serve only the always-on rows outside this requirement, and no row
named above reaches them. `ovphysx/tests/benchmarks/BenchmarkFailure.h`: it
declares the suppression API but defines none of it, so listing it alongside
`BenchmarkFailure.cpp` would add a second reference to one implementation
rather than a second implementation.

## Dependencies

- None.
