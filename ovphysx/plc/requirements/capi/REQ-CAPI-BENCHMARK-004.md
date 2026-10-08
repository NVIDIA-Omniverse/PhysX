<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BENCHMARK-004
title: Persistent Contact Report Step and Read Benchmark
status: implemented
owner: ovphysx
---

## Description

The C++ benchmark suite provides two hidden, lower-is-better latency rows for
the steady-state raw contact-report path:

- `ContactReport.persistent_pairs_512_step_read_cpu`
- `ContactReport.persistent_pairs_512_step_read_gpu`

The GPU process decorates the second report command as
`ContactReport.persistent_pairs_512_step_read_gpu_GPU`. Each row advances one
stable scene step and pulls the public borrowed contact report. These rows do
not measure scene attachment, contact-report parameter parsing, contact
solving, DirectGPU, Contact Binding, dashboard behavior, or an application
loop.

The fixture deliberately covers one contact point per pair/header, zero
friction anchors under `solveContact=false`, and one default-material case. It
does not cover multi-point manifolds, friction-anchor report payloads, or
material diversity.

## Acceptance Criteria

- AC-1: **Exact separate inventory.** The contact inventory contains exactly
  the CPU and GPU registrations above, both hidden. It remains separate from
  the frozen seventeen-row Authoring/WriteScaling L1B inventory. The union of
  the two inventories has no duplicate or substring-colliding raw registration
  names and no duplicate or substring-colliding canonical emitted names. The
  CPU command is emitted without a harness postfix; the GPU command is emitted
  with exactly one `_GPU` postfix.
- AC-2: **Stable isolated workload and device policy.** The generated stage
  contains 512 isolated cells. Each cell has one awake dynamic reporting sphere
  overlapping one static sphere, with enough spacing to prevent cross-cell
  pairs. Gravity and sleeping are disabled. The reporter explicitly applies
  `PhysxContactReportAPI` with threshold zero and disables contact solving so
  persistent overlap and report cardinality do not depend on solver motion.
  The CPU row explicitly requests CPU dynamics and MBP. The GPU row explicitly
  requests GPU dynamics and GPU broadphase, pins its authored capacities to the
  exact schema defaults (524288 rigid contacts, 81920 rigid patches, and 262144
  found/lost pairs), and runs with `--forceGpu` but without `--directGpu`.
  DirectGPU is rejected. A known fallback or contact-capacity warning fails the
  GPU row; warning silence is negative evidence only because the public API
  does not expose the realized simulation device.
- AC-3: **Measured boundary and statistics.** Each row uses twenty measured
  step indices and five runs. Before timing, the same attached scene advances
  until three consecutive structurally valid, all-persistent reports contain
  exactly 512 headers, 512 points, and zero friction anchors, bounded by
  thirty-two warm-up steps. Each timed sample starts
  immediately before `ovphysx_step_sync(instance, 1.0f / 60.0f)` and stops
  immediately after `ovphysx_get_contact_report()`. Stage generation,
  population, attachment, warm-up, expected-identity construction, semantic
  validation, and teardown are outside the timer. Validation consumes every
  borrowed report view before another simulation step can invalidate it. The
  ordinary harness report remains the KPI source. Its emitted `std_dev / avg`
  ratio is the qualification noise check; the all-sample diagnostics sidecar
  is used only to prove that all 100 timed calls occurred.
- AC-4: **Fail-closed semantic gate.** Every measured report has exactly 512
  headers and 512 points, contains only persistent events, carries the current
  nonzero attach handle, and covers exactly the authored Reporter/Static pair
  set once. The header identity fields are opaque `ObjectKey` handles
  ([ADR-0019](../../../ovruntime/plc/adr/ADR-0019-public-api-object-identity.md)),
  so the expected set is established once during warm-up by resolving the
  reported handles through `ovphysx_scene_query_get_paths_from_ids` and proving
  they name the authored `Reporter_N`/`Static_N` prims pairwise; no client-side
  path bit-cast reproduces them. Every header owns exactly one contact point. Actor/collider
  identities, prototype sentinels, a complete non-overlapping point partition,
  exactly zero global friction anchors, zero per-header friction-anchor offsets
  and counts, and finite point position, normal, impulse, and separation values
  with sane normal length and separation range are validated outside the timer.
  Null buffers with positive counts, API failures, cardinality or identity
  drift, incomplete or overlapping point slices, non-finite data, warning evidence,
  and teardown failures are recorded against the exact row. A failed or
  unexecuted row publishes no metric, and an operational or semantic failure
  makes the process exit nonzero.
- AC-5: **Executable and CI contract.** The focused contract verifies the exact
  two-row hidden list, no-row execution without `--hidden`, one positive CPU
  row, one positive conventional-GPU row, exactly 100 diagnostics samples per
  positive row, reciprocal wrong-device suppression without zero records, and
  DirectGPU refusal without output. The shared report verifier rejects missing,
  extra, duplicate, zero, malformed, and wrongly decorated reports. The
  existing GPU-capable benchmark CI job runs this contract while the pinned
  twelve-row CPU L1B command remains unchanged.

## Test References

- [TEST-CAPI-BENCHMARK-004](../../tests/capi/TEST-CAPI-BENCHMARK-004.md)

## Code References

- AC-1: `ovphysx/tests/benchmarks/BenchmarkList.cpp`,
  `ovphysx/tests/benchmarks/tests/contact_report_inventory.json`,
  `ovphysx/tests/benchmarks/tests/verify_contact_report_inventory.py`, and
  `ovphysx/tests/benchmarks/tests/test_verify_contact_report_inventory.py`
- AC-2: `ovphysx/tests/benchmarks/benchmarks/ContactReport.cpp` and
  `ovphysx/tests/benchmarks/benchmarks/AuthoringCommon.h`
- AC-3: `ovphysx/tests/benchmarks/benchmarks/ContactReport.cpp` and
  `ovphysx/tests/benchmarks/Harness.cpp`
- AC-4: `ovphysx/tests/benchmarks/benchmarks/ContactReport.cpp`,
  `ovphysx/tests/benchmarks/benchmarks/AuthoringCommon.h`, and
  `ovphysx/tests/benchmarks/Harness.cpp`
- AC-5: `ovphysx/scripts/test_benchmark_contract.cmake` and
  `ovphysx/tests/benchmarks/CMakeLists.txt`

## Dependencies

- [REQ-CAPI-BENCHMARK-002](REQ-CAPI-BENCHMARK-002.md) defines the all-sample
  diagnostics sidecar used to prove the 100-call execution count.
