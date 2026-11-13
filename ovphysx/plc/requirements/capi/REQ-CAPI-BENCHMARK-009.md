<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BENCHMARK-009
title: CPU Batched Attribute Spawn Diagnostic
status: implemented
owner: ovphysx
---

## Description

The hidden `BatchedWrite.runtime_write_spawn_cpu` row measures grouping a
body's eight attributes and its collider's five attributes into two completed
OVStage operations. Its control is caller query retention with separate
attribute writes. The original authoring row and its fixture remain available.

## Acceptance Criteria

- AC-1: **Separate row.** `batched_write_inventory.json` contains exactly
  `BatchedWrite.runtime_write_spawn_cpu`, hidden and CPU-only. It has no plain
  or decorated-name collision with existing inventories and cannot enter the
  existing Authoring, WriteScaling or QueryRetention family selections.
- AC-2: **Timed work.** One timed step creates the same 80 body/collider pairs
  with the same thirteen attribute values, types, shapes, semantics and
  ordinals as the retained control. It creates and retires 160 singleton
  query/path-list lifecycles and completes 160 `write_attributes` operations:
  eight entries for each body, then five for its collider. Body completion
  and checked query retirement precede collider query creation. The five
  original floor, update and synchronous Physics step sequences remain inside
  the timer. Query setup, payload construction and completion remain timed.
- AC-3: **Ownership and failure.** Every batch entry supplies a distinct,
  nonnull borrowed `DLTensor`, with `managed_tensors` null, and independent
  payload/shape storage that remains alive through completion or the existing failure drain. Each acquired query
  receives checked retirement within its stage lifetime. Failed preparation,
  enqueue, wait or retirement records a row failure; no partially completed
  batch is retried. The caller does not assume descriptor execution order or
  transactional rollback. Payload ownership remains with the benchmark.
- AC-4: **Untimed correctness.** Retained and batched rows use the same
  untimed verification of all thirteen authored attributes, including prim
  type and schema metadata, with exact type/shape/semantic and payload checks.
  Verification reads the final sealed authored input stage and supplements
  its 80-actor gate. The added 1,040 attribute reads and their query lifecycles
  are outside timing. Verification failure suppresses the row and
  fails the process. No verification is inserted into the timed step.
- AC-5: **Interpretation.** A batching comparison uses the same benchmark
  executable, OVStage/Physics binaries, dependency closure, fixture and wait
  policy for both rows. It measures grouping and its downstream consequences,
  not only entry-point overhead. Default measurements retain five timed spawn
  steps per process. Shortened correctness checks establish no speed threshold.

## Known limitations

The original thirteen-attribute fixture is preserved. This diagnostic does not
replace it with a corrected transform fixture or establish equivalent
intermediate per-attribute visibility. OVStage batches have one completion
result, permit incremental publication, and do not guarantee descriptor order.

## Test References

- [TEST-CAPI-BENCHMARK-009](../../tests/capi/TEST-CAPI-BENCHMARK-009.md)

## Code References

- AC-1: `ovphysx/tests/benchmarks/tests/batched_write_inventory.json`,
  `ovphysx/tests/benchmarks/tests/test_verify_inventory.py`
- AC-2, AC-3, AC-4: `ovphysx/tests/benchmarks/benchmarks/AuthoringRuntimeWriteSpawn.cpp`,
  `ovphysx/tests/benchmarks/benchmarks/AuthoringSpawnBatch.h`
- AC-3: `ovphysx/tests/benchmarks/benchmarks/AuthoringCommon.h`,
  `ovphysx/tests/benchmarks/tests/BenchmarkFailureTest.cpp`
- AC-5: `ovphysx/tests/benchmarks/README.md`

## Dependencies

- [REQ-CAPI-BENCHMARK-008](REQ-CAPI-BENCHMARK-008.md)
