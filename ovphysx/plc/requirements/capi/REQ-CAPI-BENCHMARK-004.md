<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BENCHMARK-004
title: Contact-Sensor Net-Force Read Benchmark
status: implemented
owner: ovphysx
---

## Description

The C++ benchmark suite provides two hidden, lower-is-better latency rows for
the steady-state contact-sensor net-force read through the public session read
API, replacing the old raw-contact benchmark rows:

- `ContactReport.net_force_512_read_cpu`
- `ContactReport.net_force_512_read_gpu`

The GPU process decorates the second command as
`ContactReport.net_force_512_read_gpu_GPU`. Each row reads the net contact
forces for 512 steady contact sensors through
`ovphysx_query(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR)` plus
`ovphysx_read(netNormalContactForce, netFrictionContactForce)` and the drain.
The simulation step is advanced outside the timed region, so the row measures
the read path only. These rows do not measure scene attachment, the step,
contact solving, the deprecated contact-binding tensor reads, per-sensor
filtered forces, or an application loop.

The fixture deliberately covers the two net-force attributes over a
single-material resting-box scene. It does not cover per-contact-point geometry,
friction anchors, filtered force matrices, or material diversity.
Raw event/detail consumers of the removed `ovphysx_get_contact_report()`
migrate through the session contact-event/detail attributes described by
ADR-0044 (contact reads through the session query). The two aggregated
net-force columns do not replace event headers, contact points, or friction
anchors.

## Acceptance Criteria

- AC-1: **Exact separate inventory.** The contact inventory contains exactly
  the CPU and GPU registrations above, both hidden. It remains separate from
  the frozen Authoring/WriteScaling L1B inventory. The union of the two
  inventories has no duplicate or substring-colliding raw registration names and
  no duplicate or substring-colliding canonical emitted names. The CPU command
  is emitted without a harness postfix; the GPU command is emitted with exactly
  one `_GPU` postfix.
- AC-2: **Stable isolated workload and device policy.** The generated stage
  rests 512 dynamic boxes on one static ground slab, spaced so no box touches a
  neighbour. Gravity is enabled. Each box applies `PhysxContactReportAPI` with
  threshold zero and `physxRigidBody:sleepThreshold = 0` so a settled body keeps
  reporting a steady net contact force; the ground carries no contact-report
  schema and is not a sensor. The CPU row requests CPU dynamics and MBP and
  reads host-backed groups. The GPU row requests GPU dynamics and GPU
  broadphase, pins its authored capacities to the exact schema defaults (524288
  rigid contacts, 81920 rigid patches, and 262144 found/lost pairs), and reads
  device-resident net-force columns. Because those columns are device-resident
  only on a DirectGPU scene, the GPU row runs with `--forceGpu` **and**
  `--directGpu` and is otherwise selected and skipped without a metric; a
  benchmark build without CUDA skips the GPU row.
- AC-3: **Measured boundary and statistics.** Each row uses twenty measured
  step indices and five runs. Before timing, the same attached scene advances
  until three consecutive reads return the full 512-sensor set and -- on the
  host CPU read -- a live net normal contact force, bounded by forty-eight
  warm-up steps. The DirectGPU warm-up validates CUDA residency and 512 rows;
  it does not validate the force values. The
  simulation step
  (`ovphysx_step` plus the task wait) runs in the harness `preStep()` hook,
  outside the timed region. Each timed sample covers one
  `ovphysx_query(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_SCOPE_ALL)`, one
  `ovphysx_read()` of the two net-force attributes, the fetch/release drain of
  every group, and the device sync honoured per group on the DirectGPU row.
  Stage generation, population, attachment, warm-up, and teardown are outside
  the timer. The ordinary harness report remains the KPI source; its emitted
  `std_dev / avg` ratio is the qualification noise check, and the all-sample
  diagnostics sidecar is used only to prove that all 100 timed calls occurred.
- AC-4: **Fail-closed semantic gate.** Every warm-up and pre-step task must
  complete successfully before reading contacts. Every measured read enqueues and
  completes each query, read, fetch, release-group, release-read and
  release-query task, drains to the end-of-iteration sentinel, and observes every
  net-force group carrying exactly 512 contact-sensor rows on the expected
  device (`kDLCPU` for the CPU row, `kDLCUDA` for the GPU row). A rejected or failed
  task, a short or non-terminating drain, a read that returns no group, a row
  count other than 512 in any group, or a teardown failure is recorded against
  the exact row. On the CPU row the warm-up additionally fails if the host
  net-force column is all zeros (dead scene); the DirectGPU warm-up checks
  CUDA residency and 512 rows without validating force values, per AC-3.
  Warm-up retries successful reads whose contact set is empty, incomplete, or
  not yet live within its bounded step budget. Task, synchronization, device,
  and cleanup failures stop stabilization immediately and remain in the row
  failure ledger even if later calls could succeed. Measured reads with empty
  or incomplete output fail immediately.
  A failed or unexecuted row publishes no metric, and an operational or
  semantic failure makes the process exit nonzero.
- AC-5: **Executable and CI contract.** The focused contract verifies the exact
  two-row hidden list, no-row execution without `--hidden`, one positive CPU
  row, one positive DirectGPU row (`--forceGpu --directGpu`), exactly 100
  diagnostics samples per positive row, reciprocal wrong-device suppression
  without zero records, and the GPU row's DirectGPU requirement -- under
  `--forceGpu` alone the GPU row is selected on the GPU pass, skipped, and
  publishes no metric. The shared report verifier rejects missing, extra,
  duplicate, zero, malformed, and wrongly decorated reports. The existing
  GPU-capable benchmark CI job runs this contract while the frozen
  Authoring/WriteScaling L1B command remains unchanged.

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
- AC-4: `ovphysx/tests/benchmarks/benchmarks/ContactReport.cpp` and
  `ovphysx/tests/benchmarks/Harness.cpp`
- AC-5: `ovphysx/scripts/test_benchmark_contract.cmake` and
  `ovphysx/tests/benchmarks/CMakeLists.txt`

## Dependencies

- [REQ-CAPI-BENCHMARK-002](REQ-CAPI-BENCHMARK-002.md) defines the all-sample
  diagnostics sidecar used to prove the 100-call execution count.
