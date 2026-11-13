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
- Fixed vehicle-wheel position/orientation groups in deliberately reversed
  return order, with mirrored and sheared source world matrices and mixed
  authored reset-stack values.
- Fixed rigid-body and articulation-link position/orientation/`worldScale`
  groups carrying mirrored and rotated stored scales, over a stage whose world
  matrices carry a different scale.
- A populated point instancer whose authored `positions` and half-precision
  `orientations` contain three rows, plus a malformed case where their lengths
  differ.
- A point-instancer physics read containing two rows: a live body at world
  position zero and an absent slot marked by an all-zero orientation. The
  authored third row is therefore a trailing baseline row.
- Both CPU and CUDA Warp arrays when CUDA is available.
- A real CPU simulation with a stationary character controller for each capsule
  axis X/Y/Z, at unit and nonuniform `(2, 3, 4)` scale, with zero gravity.
- For the default set, a recording fake whose read returns either one
  `DEFORMABLE_SKIN_MESH` `points` group or no group at all. With CUDA, a real
  GPU session attached to `tests/data/volume_deformable_skinned.usda`, a volume
  deformable cube that carries one skin mesh, populated once with the physics
  population description and RENDERING beside it and once with the description alone.

## When

- An output cache is created before the frame loop and passed to two consecutive
  helper calls.
- The helper is also called repeatedly without a cache.
- The real controller is published twice at increasing output ordinals, with and
  without a cache.
- The same fixed-pose groups are published through the private writer and the
  public wrapper, recording simulation steps and stage write-floor calls.
- Fixed poses are composed and written at two increasing output ordinals.
- Matrix and point-instancer baselines are read using OVStage's scalar-lane
  component axes, through both cached and uncached helper paths.
- Point-instancer local pose arrays are merged with cached OVStage snapshots
  and written at two increasing output ordinals.
- Rigid-body and link poses are published with and without a cache, first
  without and then with `worldScale` in the caller's selection.
- A lone pose component, missing wheel source world matrix, missing body
  `worldScale` group, invalidated native query, closed cache, and both pose and
  non-pose write failures are used in the negative cases.
- The helper is called with `outputs=None` against the recording fake and the
  real skinned-volume session.

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
  unchanged. The published matrices read back unchanged after the hierarchy
  computation that precedes the seal (REQ AC-7).
- Each stationary controller keeps its initial world matrix, including its
  orientation and nonuniform scale, after both publications (REQ AC-3).
- The matrix translation matches PhysX. Wheel scale matches ovphysx's matrix
  decomposition; mirrored scale reconstructs the original handedness, and
  shear is dropped in the same way as ovphysx (REQ AC-3).
- Rigid-body and link matrices compose the read `worldScale` with no OVStage
  read at all, so the stage's different scale never reaches the output.
  `worldScale` appears exactly once in each `read_sync` request, is never
  requested for wheels, and is written to `sim:worldScale` only when selected
  (REQ AC-3).
- Scalar-lane `[N, 4, 4]` matrix and `[N, 3]`/`[N, 4]` instancer read layouts
  preserve the same pose, signed scale and absent-slot baseline values (REQ AC-3).
- Point-instancer output writes native float32x3 POINT `positions` and
  float16x4 QUATERNION `orientations`. A live zero position is updated, the
  absent row retains its authored values, and the authored trailing row is
  preserved. Mismatched baseline array lengths are rejected instead of padded.
  When the read grows beyond the baseline, new absent rows receive zero
  positions and identity orientations.
- Non-pose array output retains the existing lane-folded `sim:<name>` behavior.
- With `outputs=None` the helper reads exactly the documented default set, every
  type with its attribute names and in order, plus `worldScale` for rigid bodies
  and links, ending with `DEFORMABLE_SKIN_MESH` `points` (REQ AC-2). A
  skin-mesh group is written to the skin prim's `sim:points`, and no group
  writes nothing.
- On the real session, the volume's own `sim:points` matches its `points` read
  in both populations. With RENDERING the skin prim's `sim:points` matches
  the `DEFORMABLE_SKIN_MESH` read. With the description alone the skin read returns
  no rows and the skin prim receives no write.
- A missing wheel source world matrix, or a missing body `worldScale` group, is
  detected before any regular or pose attribute from that object type is
  written, and the partial ordinal is not sealed (REQ AC-1).
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
  CUDA stream; CPU calls pass neither event nor stream. For one prim set and
  device, a cache keeps body storage, which holds no scales, apart from wheel
  storage, which holds the captured stage scales.
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
- For OMPE-120176, a real session publishes a kinematic body under a parent Xform, under a
  translated, rotated and scaled ancestor, and then translates the parent's `omni:xform` at the
  next ordinal. The parent's world translation becomes `(42, 6, 0)`; the body follows it to world
  `(42, 6, 8)`, keeps its authored local translation `(0, 0, 4)`, and after the drain and a step
  its physics position is also `(42, 6, 8)` (REQ AC-7).
- On the humanoid fixture, whose joints have one, two and three axes, joint position and velocity
  are published as one `sim:<name>` array per joint prim, each holding that joint's rows of the
  joint group in order, read back joint by joint. A joint's `physxPointer` is published as one
  value per joint prim, not as an array, equal to the pointer the read returns (REQ AC-6).

Implemented in:

- `ovphysx/tests/python_tests/test_step_and_write_to_ovstage.py`, including
  `test_wheel_pose_writes_only_world_matrix_and_reuses_buffers` in its
  `step-and-write` and `write-only` modes,
  `test_body_pose_composes_stored_world_scale_without_stage_read`,
  `test_cache_keeps_stored_scale_and_wheel_pose_buffers_apart`,
  `test_body_pose_without_world_scale_fails_without_writing_or_sealing`,
  `test_default_set_reads_every_type_and_writes_skin_points`, and
  `test_default_set_publishes_skin_points_only_with_rendering_domain` (CUDA
  only, skipped without it), `test_joint_output_is_published_as_one_array_per_joint` and
  `test_joint_physx_pointer_is_published_as_one_value_per_joint`,
  `test_parent_edit_after_publication_moves_kinematic_child`
  (fixture `ovphysx/tests/data/ovstage_output_parent_edit.usda`).
- `ovphysx/tests/python_samples/output_read.py`
- `ovphysx/tests/python_tests/cpu_tests/test_character_controller_read_write.py`:
  `test_character_controller_publishes_prim_frame`.

The native-transition regression in `test_lifecycle.py` detaches and reattaches
the same Stage through ctypes, bypassing Python lifecycle bookkeeping. The old
cache fails and a fresh cache succeeds. Unit cases in
`test_step_and_write_to_ovstage.py` reject a changed query lifetime without
calling an attachment-identity getter and verify exactly-once query release,
rejection of a different native Stage despite a stale Python keepalive, and
deferred query release when the cache is collected.
