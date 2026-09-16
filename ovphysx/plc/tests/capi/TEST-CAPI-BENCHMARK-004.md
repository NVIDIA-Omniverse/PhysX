<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BENCHMARK-004
maps_to: REQ-CAPI-BENCHMARK-004
type: integration
---

## Scenario

The hidden persistent-contact CPU/GPU pair measures the intended public
step-and-report path, publishes only validated positive results, and fails
closed when inventory, device, output, or report semantics drift.

## Given

- An ovphysx build configured with `OVPHYSX_BUILD_BENCHMARKS=ON`, followed by
  SDK installation.
- The separate two-row `contact_report_inventory.json` and the frozen L1B
  `producer_inventory.json`.
- A CPU process and a CUDA-capable process that supports conventional GPU
  dynamics without DirectGPU.
- The generated 512-cell dynamic-reporter/static-collider stage.

## When

- The contact verifier unit tests mutate inventory cardinality and names, list
  visibility and membership, raw/emitted union collisions, report membership,
  positive values, syntax, and CPU/GPU decoration.
- The focused contract lists `ContactReport.*`, runs the family without
  `--hidden`, runs each row on its matching device, runs each row on the wrong
  device, and invokes the GPU row with DirectGPU.
- Positive CPU and conventional-GPU processes use the row defaults and write
  both the ordinary report and all-sample diagnostics sidecar.
- Before MR submission, temporary unpushed mutations remove
  `PhysxContactReportAPI`, perturb generated cardinality, skip the getter,
  corrupt a point slice, inject a non-finite point, swap device authoring, and
  inject known fallback and capacity warnings. Each source mutation is restored
  before the next case and before any commit.

## Then

- The executable list contains exactly the two raw names, both hidden, and the
  frozen L1B inventory remains byte-for-byte unchanged (REQ AC-1).
- The raw registration union and canonical emitted-name union contain no exact
  duplicate or substring collision. The CPU report has no device postfix and
  the GPU report has exactly one `_GPU` postfix (REQ AC-1).
- The CPU and conventional-GPU positive runs each publish exactly one positive
  metric and one diagnostics object with `all_steps_count=100`. The diagnostics
  count proves execution cardinality only; qualification noise is calculated
  from the ordinary report's emitted `std_dev / avg` values (REQ AC-3, AC-5).
- Running without `--hidden` publishes nothing. Each wrong-device row is
  selected and skipped exactly once, emits no metric, and exits cleanly.
  DirectGPU records the exact mode failure, emits no row, and exits nonzero
  (REQ AC-2, AC-5).
- Every semantic mutation suppresses the affected row and makes its process
  fail. A restored positive run again proves exact persistent cardinality,
  pair identity -- resolved from the reported opaque `ObjectKey` handles via
  `ovphysx_scene_query_get_paths_from_ids`, not a client-side path bit-cast --
  attach handle, a complete non-overlapping point partition,
  zero global and per-header friction anchors, finite point data with sane
  normal length and separation range, and no recognized GPU fallback or
  capacity warning (REQ AC-4).
- The matcher unit accepts the source-backed GPU overflow/loss diagnostics and
  rejects benign warnings that only mention capacity-setting names (REQ AC-4).
- Missing, extra, duplicate, zero, malformed, undecorated GPU,
  decorated CPU, and double-decorated GPU reports are rejected (REQ AC-5).

## Coverage Limitations

- The public API cannot positively identify the realized GPU device. A
  successful requested-GPU run proves the authored/requested path and absence
  of recognized fallback warnings, not positive device realization.
- This benchmark does not exercise attach-time contact parameter parsing,
  contact solving, DirectGPU reporting, Contact Binding, or dashboard routing.
- Its report shape is exactly one contact point per pair/header, zero friction
  anchors under `solveContact=false`, and one default-material case. It does
  not cover multi-point manifolds, friction-anchor payloads, or material
  diversity.
