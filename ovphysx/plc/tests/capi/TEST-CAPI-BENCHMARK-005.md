<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BENCHMARK-005
maps_to: REQ-CAPI-BENCHMARK-005
type: integration
---

## Scenario

The five hidden high-scale diagnostic rows are selected only by their own
explicit filter, run their real generated scenes through public APIs, publish
exactly five validated positive CPU rows, and leave the frozen L1B contract
untouched.

## Given

- An ovphysx build configured with `OVPHYSX_BUILD_BENCHMARKS=ON`, followed by
  SDK installation.
- The separate five-row `high_scale_inventory.json`, the frozen
  `producer_inventory.json`, and the two-row `contact_report_inventory.json`.
- A CPU benchmark process; no fixture file is needed because every row
  generates its scene in memory.

## When

- The inventory unit tests load the committed high-scale inventory, check its
  five names against the L1B family globs, and validate the union of all four
  committed inventories, plus synthetic duplicate, substring, and
  harness-decoration (`_<N>T`, `_GPU`) collisions.
- The focused contract runs `check-union` over the four inventories, lists
  `WriteScalingHighN.*:RuntimeSpawnScaling.*`, runs that family filter without
  `--hidden`, and, when `OVPHYSX_BENCHMARK_CONTRACT_HIGH_SCALE=1` opts in,
  runs the exact five-row filter with `--hidden --regenerate --steps=1
  --runs=3` inside the shared baseline transaction. The reduced counts keep
  the contract a producer check; the default 20-by-5 statistic is produced by
  the scheduled FrameCore run.
- The contract's unchanged step 1 lists the frozen L1B family filter.

## Then

- The committed inventory holds exactly the five CPU rows, none matches
  `Authoring.*`, `WriteScaling.*`, `Authoring.*_cpu`, or `WriteScaling.*_cpu`,
  and the union check passes for the real inventories and fails for
  duplicate, substring, and emitted-name collisions (REQ AC-1).
- The filtered executable list contains exactly the five hidden names, the
  frozen L1B list still equals `producer_inventory.json`, and the family run
  without `--hidden` publishes nothing (REQ AC-1).
- The positive run exits zero, records no benchmark failure, and publishes
  exactly one positive metric per row: four `WriteScalingHighN.*` rows whose
  gates read the distinctive velocity back from all 8,192 or 16,384 bodies,
  and one `RuntimeSpawnScaling.collider_heavy_1280_cpu` row whose gates find
  exactly one rigid body, classify the authored path as a rigid body, and read
  its pose back at the authored height (REQ AC-2, AC-3, AC-4).
- At the row defaults, which the contract does not override anywhere but its
  positive run, each `WriteScalingHighN.*` row reports twenty measured step
  indices over five runs through the same `WriteScalingBase` timed step as the
  4,096-body pair, and the collider row reports one measured step over five
  runs whose timer brackets only the seal wait, the drain, and one synchronous
  step (REQ AC-2, AC-3).
- Every row is skipped under `--forceGpu` and publishes no record there; a
  failed or unexecuted row publishes no record and exits nonzero (REQ AC-4,
  AC-5).

## Coverage Limitations

- The positive five-row run is opt-in and is not part of the per-MR CI job:
  populating and attaching a 16,384-body scene costs minutes per run on the
  current runtime. The exact five-row publication at the row defaults is
  proven by the scheduled FrameCore run and by a local opt-in run before
  changes to these rows.
- The 1,280-collider count is verified in the generated text; the public API
  has no query for realized static shapes, so no physics-side collider count is
  asserted.
- The wrong-device skip is established by the shared `isValid()` gate these
  rows copy from `WriteScaling`; the focused contract does not run the family
  under `--forceGpu`.
