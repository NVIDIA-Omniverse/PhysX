<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BENCHMARK-004
maps_to: REQ-CAPI-BENCHMARK-004
type: integration
---

## Scenario

The hidden contact-sensor net-force CPU/GPU pair measures the supported session
read path, publishes only validated positive results, and fails closed when
inventory, device, output, or read semantics drift.

## Given

- An ovphysx build configured with `OVPHYSX_BUILD_BENCHMARKS=ON`, followed by
  SDK installation.
- The separate two-row `contact_report_inventory.json` and the frozen L1B
  `producer_inventory.json`.
- A CPU process and a CUDA-capable process that supports DirectGPU.
- The generated 512-box resting contact-sensor stage (dynamic boxes with
  `PhysxContactReportAPI` on a schemaless static ground).

## When

- The contact verifier unit tests mutate inventory cardinality and names, list
  visibility and membership, raw/emitted union collisions, no-metric
  suppression, and CPU/GPU decoration.
- The focused contract lists `ContactReport.*`, runs the family without
  `--hidden`, runs each row on its matching device, runs each row on the wrong
  device, and runs the GPU row with `--forceGpu` but without `--directGpu`.
- Positive CPU (`--forceGpu`-free) and DirectGPU (`--forceGpu --directGpu`)
  processes use the row defaults and write both the ordinary report and the
  all-sample diagnostics sidecar.
- Before MR submission, temporary unpushed mutations remove
  `PhysxContactReportAPI` from the sensors, perturb the generated box count,
  skip the `ovphysx_read()` call, drop the drain before end-of-iteration, assert
  a wrong expected row count, zero the host net-force column, and swap device
  authoring. Inject both step-admission and step-execution failures during
  warm-up and before a timed read; return host tensors to the GPU row. Each
  source mutation is restored before the next case and before any commit.
- Inject a single rejected contact query during warm-up, then restore successful
  query behavior. Repeat for read/fetch/release task failures and CUDA waits,
  including a cleanup failure that retires its session. As a positive control,
  make one successful warm-up read return no groups, an incomplete sensor set,
  or no live contacts before allowing subsequent complete, live reads.

## Then

- The executable list contains exactly the two raw names, both hidden, and the
  frozen L1B inventory remains byte-for-byte unchanged (REQ AC-1).
- The raw registration union and canonical emitted-name union contain no exact
  duplicate or substring collision. The CPU report has no device postfix and
  the GPU report has exactly one `_GPU` postfix (REQ AC-1).
- The CPU and DirectGPU positive runs each publish exactly one positive metric
  and one diagnostics object with `all_steps_count=100`. The diagnostics count
  proves execution cardinality only; qualification noise is calculated from the
  ordinary report's emitted `std_dev / avg` values (REQ AC-3, AC-5).
- Running without `--hidden` publishes nothing. Each wrong-device row is
  selected and skipped exactly once, emits no metric, and exits cleanly. The GPU
  row run with `--forceGpu` but without `--directGpu` is selected on the GPU
  pass, skipped, emits no metric, and exits cleanly (REQ AC-2, AC-5).
- Every semantic mutation suppresses the affected row and makes its process
  fail. Step failures prevent the contact read, and host tensors cannot publish
  a GPU metric. A restored positive run again proves every step/query/read/fetch/release task
  completes, the drain reaches the end-of-iteration sentinel, every net-force
  group carries exactly 512 contact-sensor rows on the expected device, and the
  CPU warm-up host read shows a live net normal force (REQ AC-4).
- Missing, extra, duplicate, zero, malformed, undecorated GPU, decorated CPU,
  and double-decorated GPU reports are rejected (REQ AC-5).
- A warm-up task, synchronization, device, or cleanup failure immediately stops
  stabilization, publishes no metric, and exits nonzero, even if subsequent
  calls would succeed. A successful warm-up read with an empty, incomplete, or
  not-yet-live contact set retries and permits a positive 100-call result after
  three consecutive complete, live reads. Empty or incomplete measured reads
  fail immediately (REQ AC-4).

## Coverage Limitations

- The GPU row checks that each net-force tensor has device type `kDLCUDA`.
  These columns are device-resident only under
  `PxSceneFlag::eENABLE_DIRECT_GPU_API`, so a successful CUDA-backed read is
  positive evidence of the realized device.
- This benchmark does not exercise the step, contact solving, the deprecated
  contact-binding tensor reads, per-sensor filtered force matrices, or dashboard
  routing.
- Its read shape is the two net-force attributes over one default-material
  resting-box scene. It does not cover per-contact-point geometry, friction
  anchors, filtered force matrices, or material diversity.
