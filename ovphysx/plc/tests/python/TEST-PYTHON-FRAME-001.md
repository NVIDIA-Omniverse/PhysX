<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-FRAME-001
maps_to: REQ-PYTHON-FRAME-001
type: integration
---

## Scenario

`ovphysx.utils.OvStageOutputCache` and
`step_and_write_to_ovstage` are exercised through the real OVStage Python API
with synthetic PhysX read groups. The focused cases cover preflight, cache
lifetime, fixed-pose composition, point-instancer reconstruction, CPU/CUDA
residency, direct DLPack capture, native storage types, synchronization, and
buffer reuse. The
runnable `tests/python_samples/output_read.py` covers the complete native
control-input and physics-output loop.

## Given

- A real `ovstage.Stage`, shared `PathDictionary`, and sealed input ordinal.
- Fixed rigid-body position/orientation groups in deliberately reversed return
  order, with mirrored and sheared source world matrices and mixed authored
  reset-stack values.
- A populated point instancer whose authored `positions` and half-precision
  `orientations` contain three rows, plus a malformed case where their lengths
  differ.
- A point-instancer physics read containing two rows: a live body at world
  position zero and an absent slot marked by an all-zero orientation. The
  authored third row is therefore a trailing baseline row.
- Both CPU and CUDA Warp arrays when CUDA is available.

## When

- An output cache is created before the frame loop and passed to two consecutive
  helper calls.
- The helper is also called repeatedly without a cache.
- Fixed poses are composed and written at two increasing output ordinals.
- Point-instancer local pose arrays are merged with cached OVStage snapshots
  and written at two increasing output ordinals.
- A lone pose component, missing source world matrix, changed attachment handle,
  closed cache, and both pose and non-pose write failures are used in the
  negative cases.

## Then

- The helper stays outside the main `PhysX` class and top-level `ovphysx`
  exports.
- Invalid configuration fails before `step_sync`.
- Each fixed pose pair produces exactly one float64, 16-lane MATRIX
  `omni:fabric:worldMatrix` write. No helper write targets `omni:xform` or
  `omni:resetXformStack`, and the previously authored reset values remain
  unchanged.
- The matrix translation matches PhysX, and scale matches ovphysx's matrix
  decomposition. Mirrored scale reconstructs the original handedness, and
  shear is dropped in the same way as ovphysx.
- Point-instancer output writes native float32x3 POINT `positions` and
  float16x4 QUATERNION `orientations`. A live zero position is updated, the
  absent row retains its authored values, and the authored trailing row is
  preserved. Mismatched baseline array lengths are rejected instead of padded.
  When the read grows beyond the baseline, new absent rows receive zero
  positions and identity orientations.
- Non-pose array output retains the existing lane-folded `sim:<name>` behavior.
- A missing fixed-body source world matrix is detected before any regular or
  pose attribute from that object type is written, and the partial ordinal is
  not sealed.
- Without a cache, current OVStage transform state is read on each call and no
  borrowed view survives into the write phase. Repeated writes normalize the
  PhysX quaternion before composition, so scale error does not compound, and a
  later source-matrix change is observed on the next call. The default full-group
  fixed and point-instancer paths do not clone source arrays.
- OVStage CUDA completion events are ordered before DLPack-backed Warp reads.
  A cache captures scale and point-instancer snapshots into owned Warp storage;
  the default path uses borrowed views only during the call. Neither path
  requests NumPy views.
- The second stable-topology call reuses the same fixed matrix or instancer
  output storage. CUDA calls reuse the same completion-event handle and pass no
  CUDA stream; CPU calls pass neither event nor stream.
- Cache reuse after detach/reattach and after `close()` fails. Repeated
  `close()` is harmless; cache construction requires no caller-provided scale or
  input ordinal.
- The full sample uses the preferred application-owned cache, authors and drains
  only control ordinals, writes physics output at never-drained ordinals, and
  reads back current world matrices.

Implemented in:

- `ovphysx/tests/python_tests/test_step_and_write_to_ovstage.py`
- `ovphysx/tests/python_samples/output_read.py`
