<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BENCHMARK-009
maps_to: REQ-CAPI-BENCHMARK-009
type: integration
---

## Coverage

The inventory suite, native `benchmark-failure-unit` payload/ownership checks
and successful CPU benchmark executions cover the checks below. Both matched
rows pass the same untimed all-attribute and actor-count gates. Operation counts
are source-backed; the correctness checks do not establish a speed threshold.

## Scenario

Grouping the retained caller's attributes preserves authored data and query
ownership without changing the Physics workload or hiding failed operations.

## Given

- An SDK built with the hidden CPU benchmark rows enabled.
- The original CPU empty-dynamic-boxes fixture and the separate one-row batch
  inventory, alongside all existing inventories.
- Independent per-entry payload and borrowed-tensor storage for eight entries.

## When

- Inventory tests validate the exact name, hidden CPU policy, family isolation
  and collision-free union with existing names.
- `benchmark-failure-unit` runs `verifySpawnBatchPayloadOwnership()` for
  payload independence, scalar/vector/token shapes and capacity rejection; existing `ScopedQuery` tests exercise checked
  close, early return, exception unwind and a failed close without retry.
- `verifySpawnBatchAttributeLengths()` supplies a bounded name view over a
  non-null-terminated buffer to float, double and token writes, and checks
  rejection of null views without appending entries.
- Retained and batched spawn rows run with the same executable and runtime.
  Their untimed gate reads all authored attributes from the final sealed input
  stage, and their actor-count gates run after the timer.
- Source review follows all enqueue, wait and cleanup paths and the boundaries
  around the five floor/update/step sequences.

## Then

- Exactly one hidden CPU batch row is declared; existing family inventories
  are unchanged and cannot select it (REQ AC-1).
- Each batched step completes eighty eight-entry body operations and eighty
  five-entry collider operations, with 160 query lifecycles; the retained
  control still completes 1,040 separate writes. Body retirement precedes
  collider creation, and all five Physics boundaries remain timed (REQ AC-2).
- Each descriptor and tensor points at its own live data and shape. Rejected
  preparation cannot submit an incomplete batch. Enqueue/wait failures cannot
  publish a successful benchmark, and cleanup does not retry ambiguous
  retirement or reuse borrowed payloads before completion (REQ AC-3).
- Attribute descriptors preserve the supplied name pointer and exact length;
  bytes beyond the view are not part of the name (REQ AC-3).
- All thirteen authored fields match their expected types, shapes, semantics
  and values, including prim-type/schema tokens, and the measured stage has
  all eighty actors. Failure of either gate suppresses timing publication.
  These checks are outside the measured step (REQ AC-4).
- Performance analysis uses contemporaneous matched process blocks, retains
  every measured sample, and reports uncertainty independently of shortened
  correctness runs (REQ AC-5).

- The shared spawn payload verifier accepts packed `[N]` vectors with
  `dtype.lanes = C` and expanded scalar `[N, C]` tensors, preserving exact type,
  semantic, identity and value checks. `verifySpawnTensorReadback()` verifies
  selected rows, byte offsets, padded row strides and non-unit component strides,
  plus packed/expanded quaternion arrays. Incorrect type/shape, out-of-range rows,
  non-positive strides and device tensors are rejected (REQ AC-4).

## Coverage limitations

- `ScopedQuery` ownership tests substitute release callbacks. They do not
  inject every native enqueue, wait, retirement or partial-publication failure.
- Successful mixed-type batches and readbacks do not prove rollback or atomic
  publication. The API remains nontransactional and exposes one result per
  batch; source review verifies failure propagation and no automatic retry.
- Native rejection/failure-injection coverage is not claimed by this test
  record. Operation counts exclude the additional untimed verification queries
  and all runtime-internal queries.
- The original thirteen-attribute fixture remains unchanged. The input-stage
  payload gate validates authored data, not a corrected-transform fixture or
  the physical placement of every collider.
