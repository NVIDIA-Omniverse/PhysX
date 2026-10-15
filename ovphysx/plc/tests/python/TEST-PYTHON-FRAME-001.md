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
- Call recording wraps methods on the actual `ovstage.Stage` owner, which remains valid for
  owner-aware `PathDictionary` construction and query/list lifetimes.

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
- The same fixed-pose groups are published through the private writer and the
  public wrapper, recording simulation steps and stage write-floor calls.
- Fixed poses are composed and written at two increasing output ordinals.
- Matrix and point-instancer baselines are read using OVStage's scalar-lane
  component axes, through both cached and uncached helper paths.
- Point-instancer local pose arrays are merged with cached OVStage snapshots
  and written at two increasing output ordinals.
- A lone pose component, missing source world matrix, invalidated native query,
  closed cache, and both pose and non-pose write failures are used in the
  negative cases.

## Then

- The helper stays outside the main `PhysX` class and top-level `ovphysx`
  exports.
- Invalid configuration fails before `step_sync`.
- The private writer performs no step or sealing call. The public wrapper
  steps exactly once before writing and seals exactly once after all writes
  complete (REQ AC-2, REQ AC-4).
- Each fixed pose pair produces exactly one float64, 16-lane MATRIX
  `omni:fabric:worldMatrix` write. No helper write targets `omni:xform` or
  `omni:resetXformStack`, and the previously authored reset values remain
  unchanged.
- The matrix translation matches PhysX, and scale matches ovphysx's matrix
  decomposition. Mirrored scale reconstructs the original handedness, and
  shear is dropped in the same way as ovphysx.
- Scalar-lane `[N, 4, 4]` matrix and `[N, 3]`/`[N, 4]` instancer read layouts
  preserve the same pose, signed scale and absent-slot baseline values (REQ AC-3).
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
  input ordinal. Reuse checks the retained query through the private boolean
  helper, while dictionary lookup uses the Stage. Neither
  `PhysX.query_shared_dictionary()` nor a public query validity method is needed
  (REQ AC-1).
- The full sample uses the preferred application-owned cache, authors and drains
  only control ordinals, writes physics output at never-drained ordinals, and
  reads back current world matrices. Its verifier preserves the leading data-row
  axis of scalar-lane `[N, 4, 4]` reads, checks all 16 matrix components per body,
  and completes all five frames with translations matching PhysX (REQ AC-3).

Implemented in:

- `ovphysx/tests/python_tests/test_step_and_write_to_ovstage.py`, including
  `test_fixed_pose_writes_only_world_matrix_and_reuses_buffers` in its
  `step-and-write` and `write-only` modes.
- `ovphysx/tests/python_samples/output_read.py`

The native-transition regression in `test_lifecycle.py` detaches and reattaches
the same Stage through ctypes, bypassing Python lifecycle bookkeeping. The old
cache fails and a fresh cache succeeds. Unit cases in
`test_step_and_write_to_ovstage.py` reject a changed query lifetime without
calling an attachment-identity getter and verify exactly-once query release,
rejection of a different native Stage despite a stale Python keepalive, and
deferred query release when the cache is collected.
