<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BENCHMARK-005
title: Hidden High-Scale Write and Collider Ingestion Diagnostics
status: implemented
owner: ovphysx
---

## Description

The C++ benchmark suite provides five hidden, lower-is-better, CPU-only
diagnostic rows that extend the frozen L1B producer contract
([REQ-CAPI-BENCHMARK-001](REQ-CAPI-BENCHMARK-001.md)) without changing it:

| Row | Measured operation |
|---|---|
| `WriteScalingHighN.velocity_ovstage_8192_cpu` | the `WriteScaling` OVStage velocity sequence at 8,192 bodies |
| `WriteScalingHighN.velocity_tensor_8192_cpu` | the `WriteScaling` tensor velocity sequence at 8,192 bodies |
| `WriteScalingHighN.velocity_ovstage_16384_cpu` | the `WriteScaling` OVStage velocity sequence at 16,384 bodies |
| `WriteScalingHighN.velocity_tensor_16384_cpu` | the `WriteScaling` tensor velocity sequence at 16,384 bodies |
| `RuntimeSpawnScaling.collider_heavy_1280_cpu` | seal, structural drain, and first step of one runtime-spawned body into 1,280 existing static colliders |

The two families exist because the 4,096-body L1B point sits before the observed
OVStage write slowdown becomes obvious, and because nothing in L1B measures how
the drain that realizes one new body scales with the colliders already in the
scene. Both are registered under family prefixes the L1B selection
(`Authoring.*`, `WriteScaling.*`) cannot match, so the pinned twelve-row L1B
contract, its inventory, and its scheduled filter are unchanged. Every row is a
non-paging diagnostic; no ratio, slope, or delta derived from them is a KPI,
and no GPU variant exists.

## Acceptance Criteria

- AC-1: **Exact separate hidden inventory.** `high_scale_inventory.json`
  contains exactly the five CPU rows above, every one registered with
  `Register<T, true>` and therefore absent from normal wildcard execution
  unless `--hidden` is explicit. No row matches the L1B family filter
  `Authoring.*:WriteScaling.*` or the scheduled L1B filter
  `Authoring.*_cpu:WriteScaling.*_cpu`, and the filtered L1B `--list` still
  equals `producer_inventory.json` exactly. The union of all four committed
  inventories (L1B, OutputRead, high-scale, ContactReport) has no duplicate
  or substring-colliding registered names, and no registered name matches
  inside another row's record once the harness has appended its `_<N>T` or
  `_GPU` postfix.
- AC-2: **High-N writes reuse the frozen 4,096-body semantics.** Each
  `WriteScalingHighN.*` row is the `WriteScalingBase` fixture at 8,192 or
  16,384 bodies with no other change: scene generation, population and
  attachment, path-list creation, token interning, persistent tensor-binding
  creation, per-step value fill, and the correctness gate stay outside the
  timer; each timed step is the row's lane write plus one 1/60-second step
  exactly as in `WriteScaling.*`; the statistic is 20 measured step indices
  over five runs with the harness trim of REQ-CAPI-BENCHMARK-001 AC-2. The
  generated scene authors the same CPU dynamics and MBP lines, verified before
  population. After timing, every one of the N bodies must read back the
  distinctive written velocity.
- AC-3: **Collider ingestion boundary and gates.** The generated scene carries
  exactly 1,280 static Cube colliders (`PhysicsCollisionAPI`, no rigid-body
  schema), zero gravity, and the CPU scene lines, all verified in the text
  before population. The scene is populated, attached, and warmed, its
  zero-rigid-body seed state is read back, and the one runtime body is
  authored through the unchanged `Authoring.runtime_write_spawn` column writes
  (thirteen waited `OVSTAGE_PRIM_MODE_UPSERT` writes) plus one waited
  `omni:xform` write carrying the same pose, all outside the timer. The
  `omni:xform` write is required because ovphysx resolves a created prim's
  local transform from the data plane's `omni:xform` matrix, not from the
  `xformOp:*` columns, so the spawn columns alone realize the body at the
  identity pose. The timed sample is exactly `ovstage_advance_write_floor(ordinal)` plus its
  completion wait, `ovphysx_update_from_ovstage([prev+1, ordinal])`, and one
  `ovphysx_step_sync(1/60 s)`. Afterwards, outside the timer, the scene must
  hold exactly one dynamic rigid body, `ovphysx_get_object_type()` must
  classify `/World/Runtime/Body_0000` as `OVPHYSX_OBJECT_TYPE_RIGID_BODY`, and
  the pose readback of that path must return one match at the authored height;
  a dropped or deferred drain fails the row.
- AC-4: **Fail-closed rows and contract.** Every setup, enqueue, wait, release,
  drain, step, readback, and teardown failure is recorded against the exact
  row through `bmRecordFailure()`, every wait is bounded as in
  REQ-CAPI-BENCHMARK-001 AC-4, a failed or unexecuted row publishes no record,
  and the process exits nonzero. The focused contract verifies the
  collision-free inventory union, the exact hidden five-row `--list`, and
  no-row execution without `--hidden` on every run, plus, when
  `OVPHYSX_BENCHMARK_CONTRACT_HIGH_SCALE=1` opts in, a positive
  `--hidden --regenerate` run with reduced `--steps`/`--runs` that publishes
  exactly the five rows once each; a run recording any benchmark failure fails
  the contract. No job in this repository's CI executes the five rows: the
  per-MR benchmark job keeps the L1B filter, which cannot select these
  families, and does not opt into the positive run. Execution ownership sits
  outside this repository, in the FrameCore `ovphysx` project's
  `high_scale_hidden` suite defined in the FrameCore/fpp repository
  (`pipelines/ovphysx`, `OVPHYSX_SUITE=high_scale_hidden`) and its recurring
  schedule on the L40S/EPYC 7313P cohort; that run produces the default
  statistic and validates the exact five-row report.
- AC-5: **Diagnostic, CPU-only, nonpaging.** All five rows are CPU by
  construction from their authored scenes and are skipped under `--forceGpu`.
  They carry no primary-KPI status; any threshold attached to them by a
  consumer is an absolute, lower-is-better, non-paging diagnostic fence, and
  no consumer may compare them against the L1B `WriteScaling.*` rows as the
  same series.

## Known limitations

- The public API exposes no query for realized static shapes, so the
  1,280-collider count is established from the generated text rather than
  from a physics-side readback; the physics-side gates cover the spawned rigid
  body only.
- The 16,384-body scene text is several megabytes and is generated in memory,
  and populating and attaching it costs minutes per run on the current
  runtime; the rows are therefore heavier than any L1B row, are run only
  through their own explicit selection, and their positive contract run is
  opt-in rather than part of every CI job.

## Test References

- [TEST-CAPI-BENCHMARK-005](../../tests/capi/TEST-CAPI-BENCHMARK-005.md)

## Code References

- AC-1: `ovphysx/tests/benchmarks/tests/high_scale_inventory.json`,
  `ovphysx/tests/benchmarks/tests/verify_inventory.py`,
  `ovphysx/tests/benchmarks/tests/test_verify_inventory.py`, and
  `ovphysx/scripts/test_benchmark_contract.cmake`
- AC-2: `ovphysx/tests/benchmarks/benchmarks/WriteScaling.cpp`
- AC-3: `ovphysx/tests/benchmarks/benchmarks/AuthoringRuntimeWriteSpawn.cpp`
  and `ovphysx/tests/benchmarks/benchmarks/AuthoringCommon.h`
- AC-4: `ovphysx/tests/benchmarks/benchmarks/WriteScaling.cpp`,
  `ovphysx/tests/benchmarks/benchmarks/AuthoringRuntimeWriteSpawn.cpp`,
  `ovphysx/tests/benchmarks/BenchmarkFailure.cpp`, and
  `ovphysx/scripts/test_benchmark_contract.cmake`
- AC-5: `ovphysx/tests/benchmarks/benchmarks/WriteScaling.cpp` and
  `ovphysx/tests/benchmarks/benchmarks/AuthoringRuntimeWriteSpawn.cpp`

## Dependencies

- [REQ-CAPI-BENCHMARK-001](REQ-CAPI-BENCHMARK-001.md) defines the frozen L1B
  contract these rows extend, the bounded-wait rule, and the trimmed statistic.
