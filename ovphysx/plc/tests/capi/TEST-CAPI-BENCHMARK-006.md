<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BENCHMARK-006
maps_to: REQ-CAPI-BENCHMARK-006
type: integration
---

## Scenario

The OutputWrite contract runs the failed binding and fixture families through
real public APIs, then checks that unavailable scene data fails closed.
Full-suite acceptance and source inspection cover the broader fixture and
zero-input requirements separately from this focused CI check.

## Given

- A benchmark-enabled build and installed SDK.
- The committed rigid, articulation, compound, instancer and tendon scenes.
- `scripts/test_output_write_contract.cmake`, also included by the normal
  benchmark contract entrypoint.

## When

- The script runs eight CPU rows with three runs and two measured steps:
  rigid pose, inertia, articulation root and DOF bindings; compound shape and
  instancer writes; fixed and spatial tendon writes.
- The script reruns the fixed tendon row with a nonexistent data directory.
- The script replaces the instancer fixture with an ordinary rigid-body scene.
- Regenerate calls use the existing baseline transaction and restore it.
- Full-suite acceptance runs all 33 normal `OutputWrite.*_cpu` rows with the
  default run and step counts, including force and wrench rows.
- Inspect force/wrench payload initialization in `OutputWriteBase::fillGroup`
  and `TensorBindingWriteBase::bind`, including the paths that use those
  buffers before committing them.

## Then

- Every positive selected row loads, completes and emits one positive timing;
  the process exits zero and reports no contact-capacity overflow (REQ AC-1,
  AC-4).
- Full-suite acceptance emits exactly 33 positive CPU timings with no failed
  rows or contact-capacity overflow. This is the all-row fixture check; the
  eight-row CI check alone does not establish that coverage (REQ AC-1, AC-4).
- Binding setup accepts nonempty matching coverage, including the pose versus
  position width difference and joint-array payload (REQ AC-3).
- The missing-data process exits nonzero and emits no timing for its selected
  row (REQ AC-4).
- The instancer row requires one ARRAY group per authored environment with
  twenty instances each; its ordinary-body replacement fails without a timing
  (REQ AC-5).
  Its scene is loaded once and released when the benchmark row ends, while
  all requested runs and measured steps still execute.
- Source inspection confirms snapshots own their storage after read release,
  all committed tensors are copied in full, and pose bindings read their
  initial values. Force/wrench session buffers and their binding input storage are
  zero-initialized and not changed before copying or writing. This verifies
  the zero-input construction by inspection; the runs do not independently
  assert every input value or inject a partial copy (REQ AC-2).
