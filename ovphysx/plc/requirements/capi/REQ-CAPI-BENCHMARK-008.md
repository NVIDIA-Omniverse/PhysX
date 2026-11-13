<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BENCHMARK-008
title: CPU Caller Query Retention Benchmarks
status: implemented
owner: ovphysx
---

## Description

Two hidden `QueryRetention.*` diagnostics measure the effect of reusing an
OVStage query in the caller. They preserve their corresponding authoring,
drain and simulation work. The original fresh-query rows remain unchanged.

## Acceptance Criteria

- AC-1: **Separate inventory.** `query_retention_inventory.json` contains
  exactly `QueryRetention.runtime_write_spawn_cpu` and
  `QueryRetention.velocity_ovstage_4096_cpu`. Both are hidden and CPU-only,
  cannot match existing family filters, and have no plain or decorated-name
  collisions with existing inventories. Omitting `--hidden` or selecting
  `--forceGpu` publishes no retention row.
- AC-2: **Spawn lifetime.** The spawn row authors the same 80 body/collider
  pairs, 1,040 separate writes, five batches, floors, drains and synchronous
  steps as `Authoring.runtime_write_spawn_cpu`. One singleton query serves
  each body's eight writes and one serves its collider's five writes. Each
  singleton query is retired immediately after its final attribute write;
  the body query is retired before the collider query is created. All
  160 query/path-list constructions and query retirements remain inside the
  timed spawn step; no query survives its body/collider authoring call. The
  original row retains its 1,040 query lifecycles.
- AC-3: **Dense lifetime.** The velocity row uses the same 4,096-body fixture,
  path list, payload generation, write completion, velocity-scoped floor,
  update and asynchronous step/wait as `WriteScaling.velocity_ovstage_4096_cpu`.
  It creates one query per run before the same-stage warmup and retires it
  before path-list release and stage teardown. With the default 20 measured
  steps, that query serves 22 writes: one warmup, twenty measured and one
  distinctive verification write. Query setup/retirement are outside timing;
  default timing is twenty steps over five measured runs. No query crosses
  an attachment lifetime. Existing WriteScaling, HighN and WriteOnly rows
  retain their original query lifetime and timing boundaries.
- AC-4: **Ownership and correctness.** Failed creation, write, wait or
  retirement records a row failure. Each owned query receives one checked
  retirement attempt before its owner stage is cleared, including failure
  paths; an ambiguous failed retirement is not retried. Retention does not
  remove completion/error checks, ordinal advancement or payload lifetime
  protection. Untimed gates require every spawned rigid body and the
  distinctive written velocity on all 4,096 bodies. An operational, lifecycle
  or semantic failure suppresses that row and fails the process.
- AC-5: **Interpretation and validation.** Retention rows are diagnostics,
  not replacements for existing KPIs. Their comparison measures caller query
  lifetime plus its downstream effect, not just query-construction time.
  Performance comparisons use the same OVStage and Physics/dependency
  binaries, fixtures and execution policy. The focused contract checks both
  positive rows with shortened counts and restores the developer baseline.
  Shortened contract runs establish behavior, not a speedup threshold.

## Test References

- [TEST-CAPI-BENCHMARK-008](../../tests/capi/TEST-CAPI-BENCHMARK-008.md)

## Code References

- AC-1: `ovphysx/tests/benchmarks/tests/query_retention_inventory.json`,
  `ovphysx/tests/benchmarks/tests/test_verify_inventory.py`
- AC-2, AC-4: `ovphysx/tests/benchmarks/benchmarks/AuthoringRuntimeWriteSpawn.cpp`
- AC-3, AC-4: `ovphysx/tests/benchmarks/benchmarks/WriteScaling.cpp`
- AC-4: `ovphysx/tests/benchmarks/benchmarks/AuthoringCommon.h`,
  `ovphysx/tests/benchmarks/tests/BenchmarkFailureTest.cpp`
- AC-1, AC-5: `ovphysx/scripts/test_query_retention_contract.cmake`,
  `ovphysx/scripts/test_benchmark_contract.cmake`

## Dependencies

- [REQ-CAPI-BENCHMARK-001](REQ-CAPI-BENCHMARK-001.md)
- [REQ-CAPI-BENCHMARK-002](REQ-CAPI-BENCHMARK-002.md)
