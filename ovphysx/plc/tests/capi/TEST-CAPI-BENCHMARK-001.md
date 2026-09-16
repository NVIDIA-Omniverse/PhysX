<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BENCHMARK-001
maps_to: REQ-CAPI-BENCHMARK-001
type: integration
---

## Scenario

The hidden L1B benchmark inventory is selected only by explicit opt-in, runs
the frozen CPU producer contract through its real fixtures and public APIs,
publishes exactly twelve positive CPU rows, and fails closed when selection,
operations, semantic gates, or row cardinality are wrong.

## Given

- An ovphysx build configured with `OVPHYSX_BUILD_BENCHMARKS=ON`, followed by
  SDK installation.
- The pinned `producer_inventory.json` containing eighteen unique,
  substring-collision-free names: twelve CPU and six requested-GPU rows.
- The four shipped Authoring fixture dependencies:
  `empty_dynamic_boxes.usda`, `empty_dynamic_boxes_cpu.usda`,
  `simple_physics_scene.usda`, and `simple_physics_scene_cpu.usda`.
- A CPU benchmark process selected with
  `--hidden --filter=Authoring.*_cpu:WriteScaling.*_cpu --regenerate`.

## When

- The inventory unit tests parse the filtered `--list` output, which includes
  hidden registrations by design, and compare it against the frozen producer
  inventory. A separate family execution without `--hidden` checks isolation.
- The benchmark contract script runs focused empty-output, list-mismatch,
  exact-row-failure, operation-failure, positive CPU-contract, and
  device-gated-row cases. The device-gated case runs `--regenerate` with a
  requested-GPU filter in a CPU pass -- no `--forceGpu`, which is what makes
  `isValid()` false for those rows regardless of host hardware -- and asserts
  through `verify_inventory.py check-skipped` that each row was selected and
  gated exactly once, then through `check-unpublished` that it emitted no
  metric line at all.
- Every contract step that invokes `--regenerate` wraps the developer
  `_baseline.txt` in the crash-recoverable transaction implemented by
  `scripts/benchmark_baseline_txn.cmake`, and step 0 runs
  `scripts/test_benchmark_baseline_txn.cmake`, which exercises that protocol
  against a scratch directory: normal round trips with and without a
  pre-existing baseline, hand-seeded interrupted transactions in both states,
  idempotent recovery, and the four refusals.
- `BenchmarkFailureTest` records exact-row and decorated-row failures, checks
  that family-prefix collisions do not suppress neighboring rows, accepts
  source-backed GPU capacity-failure diagnostics, and rejects benign mentions
  of capacity-setting names.
- The focused benchmark-driver tests exercise hidden-argument propagation,
  empty pass selection, exact-row configuration with zero, one, or multiple
  passes, invalid counts, missing positive rows, extra/duplicate rows, and
  skipped zero-time rows. They also write success and Windows loader-failure
  status sidecars through the same helper used by the real driver and parse
  them as JSON.
- The positive CPU contract executes every selected row, including its untimed
  correctness gate, and writes a regenerated report.

## Then

- Filtered `--list` output contains all eighteen exact hidden names and the
  CPU/GPU split is twelve/six, while family execution without `--hidden`
  publishes zero rows (REQ AC-1).
- `BENCHMARK_HIDDEN=1` produces `--hidden` for a pass. A selection with no
  enabled pass fails. `BENCHMARK_EXPECT_ROWS` rejects non-positive or
  non-integer input and every selection other than exactly one enabled pass
  (REQ AC-1).
- The positive CPU report contains exactly twelve positive data rows, each
  exact pinned CPU name appears once, and no requested-GPU row appears.
  Empty, missing, duplicate, skipped, or wrong-cardinality reports fail
  (REQ AC-1, AC-5).
- The primary rows run with five measured runs and retain the middle three
  sorted samples per step. Population-add-drip contributes one per-run timed
  sample containing its 80 cycles; WriteScaling contributes twenty per-run
  timed samples and the report aggregates their trimmed means. Validation
  remains outside the timed samples (REQ AC-2, AC-3).
- A failed operation or correctness gate is associated only with its exact row
  (plus a documented harness device/thread postfix), its record is omitted,
  and the benchmark process exits nonzero. A baseline comparison failure may
  already have printed its failed line before the nonzero exit; the dedicated
  contract uses regenerate mode and does not compare a baseline (REQ AC-4).
- A row that never executed emits no metric line, so `--regenerate` writes no
  entry for it and no zero baseline is recorded. A zero-valued line for such a
  row is a failure, not a pass (REQ AC-4).
- Every benchmark pass replaces its prior status sidecar with valid JSON that
  preserves the exact process return value for both success and failure. An
  external classifier can consume that value without parsing the driver log
  (REQ AC-4).
- A failed ovphysx bootstrap leaves no instance for any row to measure, so the
  harness records the fault, runs no row, publishes nothing, and exits nonzero
  rather than emitting a report of zeroes. There is no supported way to make
  `ovphysx_initialize()` or `PhysX::create()` fail from inside the contract, so
  this path shares the unexecuted-row suppression the device-gated case
  exercises and its own trigger is not automated (REQ AC-4).
- A contract run leaves any developer `_baseline.txt` byte-identical to what it
  found, so running the focused contract does not silently narrow a broader
  baseline to the contract's own filtered rows. The transaction machine-checks
  this: `baseline_txn_begin()` records the SHA-256 of what it found and
  `baseline_txn_commit()` aborts unless the restored file hashes the same. A
  baseline the contract created where none existed is removed, again verified.
  A run killed mid-transaction leaves recoverable on-disk state, and the next
  invocation completes it before it snapshots anything; the protocol test
  proves recovery from hand-seeded interrupted state, since a SIGKILL runs no
  cleanup of its own (REQ AC-4).
- Release and teardown calls -- query, read session, read group, tensor
  binding, path-list reference, stage reset/detach/destroy -- are status-checked
  at every call site, and a failure in any of them routes through
  `bmRecordFailure()`, which suppresses the row's record; the same holds for a
  wait that returns a non-success status even when its per-operation error list
  is empty. **Coverage limitation:** this is established by inspection of the
  call sites, not by execution. The runtime exposes no fault injection for
  these paths, so nothing in the suite makes a real release, destroy, or
  status-only wait failure occur; what the contract does execute is the
  suppression mechanism itself, through the operation-failure case, which
  proves a `bmRecordFailure()` call on any of these paths omits the record and
  exits nonzero. Two `queryPath()` rules are unit-tested in
  `BenchmarkFailureTest`: `queryPathMustDiscardQuery()`, so a query created
  behind a failed path-list release is retired rather than leaked to a caller
  that reads `false` as "no handle"; and `queryPathListIsUsable()`, so a
  path-list construction fault -- including a success status carrying the
  invalid sentinel -- is recorded against the row instead of being mistaken for
  the benign "that prim is already gone" result the churn remove loop relies on
  (REQ AC-4).
- The successful CPU run verifies growth counts, churn create/remove identity
  and floor, pose/velocity effects, and all 4,096 WriteScaling velocities.
  CPU authoring checks pass, while requested-GPU execution is not required and
  no realized-GPU claim is inferred from absent fallback or capacity output in
  the attachment and warm-up window. **Coverage limitation:** all six
  requested-GPU Authoring rows reuse the same detector by inspection; this
  contract does not separately inject a capacity warning into each row.
  TEST-CAPI-BENCHMARK-004's temporary warning mutations exercise the shared
  detector's fallback and capacity paths (REQ AC-4, AC-5).
