<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BENCHMARK-008
maps_to: REQ-CAPI-BENCHMARK-008
type: integration
---

## Coverage status

Inventory validation, the native ownership unit and the CPU benchmark
contract cover the scenarios below. The contract checks positive rows,
hidden-row vacuity, GPU rejection and baseline restoration. Its shortened
runs validate behavior; they do not establish a performance result.

## Scenario

The separate caller-retention rows preserve the original physics work while
retiring each reused query exactly once within its owner stage lifetime.

## Given

- An installed SDK built with benchmarks enabled.
- The two-row retention inventory and the unchanged existing inventories.
- The CPU Authoring fixtures and generated 4,096-body CPU velocity scene.

## When

- `test_verify_inventory.py` validates the retention names, device policy,
  family isolation and collision-free union with all existing inventories.
- `test_benchmark_contract.cmake` includes
  `test_query_retention_contract.cmake`: exact `QueryRetention.*` listing,
  execution without `--hidden`, device rejection under `--forceGpu`, and a
  positive CPU run using `--hidden --regenerate --steps=1 --runs=3`.
- `benchmark-failure-unit` runs `verifyScopedQueryOwnership()` with an
  injected release callback: repeated borrowing, explicit close, early
  failure after an acquisition, exception unwinding with two owners, and
  a failed explicit close without a second retirement attempt.
- The positive run executes the retained spawn and dense paths, their
  lifecycle checks, semantic gates and checked teardown.
- Source review follows acquisition, reuse and retirement, including failed
  query creation and failed writes, against the original fresh-query paths.

## Then

- Exactly the two named hidden CPU rows are listed, and the unchanged
  producer/high-scale/write-only filters cannot select either. No-hidden
  and GPU-rejection runs publish no retention metric (REQ AC-1).
- Each successful spawn step creates and retires two singleton queries per
  body/collider pair while performing the same 1,040 writes and five
  floor/drain/step sequences. Each singleton query is retired immediately
  after its final attribute write: the body query after eight writes and
  before collider query creation, then the collider query after five.
  Its actor-count gate passes (REQ AC-2, AC-4).
- A dense run uses one query for warmup, all measured steps and the distinct
  verification write; the shortened contract uses three writes per query and
  the default run uses 22. The query is retired before path-list/stage
  destruction, and every body's verified velocity matches (REQ AC-3, AC-4).
- Twenty-two borrows of one handle retain the same owner without an
  early release. Explicit close, early return and exception unwinding each
  retire every acquired owner once against the correct stage. A failed
  checked close reports failure and destruction does not retry it
  (REQ AC-4).
- The positive run exits zero and emits exactly two positive rows. Failed
  lifecycle or semantic checks cannot publish a successful timing, and the
  baseline transaction restores any developer baseline byte-for-byte
  (REQ AC-4, AC-5).
- Existing fresh-query rows retain their behavior; performance conclusions
  compare contemporaneous process pairs with identical runtime dependencies,
  rather than treating the shortened contract as a timing study (REQ AC-5).

- The shared spawn payload verifier accepts packed `[N]` vectors with
  `dtype.lanes = C` and expanded scalar `[N, C]` tensors, preserving exact type,
  semantic, identity and value checks. `verifySpawnTensorReadback()` verifies
  selected rows, byte offsets, padded row strides and non-unit component strides,
  plus packed/expanded quaternion arrays. Incorrect type/shape, out-of-range rows,
  non-positive strides and device tensors are rejected (REQ AC-4).

## Coverage limitations

- Source-level operation counts are not a trace of all runtime-internal
  query activity. Physics can maintain additional queries independently.
- The ownership unit substitutes a release callback; it does not inject
  native OVStage failures or establish runtime-internal cleanup behavior.
- The shared runtime exposes no general release/wait failure injection.
  Real error-path propagation is reviewed separately from successful native
  semantic gates; do not infer exhaustive failure coverage from a smoke run.
