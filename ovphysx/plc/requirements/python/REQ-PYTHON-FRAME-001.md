<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-FRAME-001
title: Step and Write Physics Output to OVStage
status: implemented
owner: ovphysx
---

## Description

`ovphysx.utils.step_and_write_to_ovstage(physx, *, dt, output_ordinal,
cache=None, outputs=None)` composes the three calls a producer otherwise writes by hand --
one `PhysX.step_sync`, the matching `PhysX.read` output selections, and the
OVStage write-back -- into one opt-in utility call. It is not a method on the
core `PhysX` API and is not re-exported from the top-level `ovphysx` package.
It adds no native surface: it forwards each output read group's interned
`prim_list` through an OVStage query into `Stage.write_attribute` (the group
shape ADR-0007 defines), then seals the write with one
`Stage.advance_write_floor`. `outputs` selects which
simulated object types and attribute names to read; `None` reads the utility's
default dynamic set for rigid bodies, articulation roots and links,
articulation joints, vehicle wheels, deformable volumes and surfaces, and
particle sets. Fixed rigid-body, articulation-link, and vehicle-wheel
position/orientation pairs are composed with signed scale derived from current
float64 `omni:fabric:worldMatrix` values, without changing local transform or
reset-stack state. Rigid-body point-instancer pose pairs write the native
instancer-local `positions` and half-precision `orientations` arrays while
preserving authored holes. Other selected output remains under its shadow
`sim:<attribute>` name. Scale and point-instancer baselines are application or
OVStage state, not PhysX input or output.

By default, the utility reads the current scale and point-instancer arrays from
OVStage for each call, uses borrowed DLPack views to prepare final output
buffers, releases every read view, and then writes. The normal full-group path
does not clone source arrays or convert them through NumPy. Passing an
application-owned `OvStageOutputCache` opts into retaining owned source copies
plus reusable Warp buffers and CUDA events across calls.
The cache binds to the exact attachment; `refresh()` discards its snapshots after
transform, point-instancer array, or topology changes, and `close()` is
idempotent. Neither mode retains borrowed OVStage views across a write.

The one caller obligation the utility cannot enforce is the ovstage ordinal
lane split (see the ovstage Integration guide): `output_ordinal` must never
appear in a range passed to `PhysX.update_from_ovstage`, or physics would
re-ingest its own output as an authored edit. The utility never calls
`update_from_ovstage` itself.

## Acceptance Criteria

- AC-1: Before any step or write, the utility requires an attached Python
  `ovstage.Stage`; a raw-handle attachment is rejected with `RuntimeError`. If
  supplied, `cache` must be a live `OvStageOutputCache` owned by the same
  `PhysX` instance and exact attachment; a closed cache, different instance,
  or detach/reattach cycle is rejected with `RuntimeError`. The
  function and cache are defined in `ovphysx.utils`; they are absent from the
  `PhysX` runtime class and stub and from
  the top-level `ovphysx` exports. It validates that `outputs` (or the documented
  default set when `outputs=None`) is a mapping whose keys are `SimObjectType`
  values and whose values are non-string sequences containing only
  attribute-name strings. It does not itself validate `output_ordinal` or
  whether each attribute name is recognised: ordinal type/range checking is
  `Stage.write_attribute`/`advance_write_floor`'s responsibility, and
  attribute-name semantics are `PhysX.read`'s. An invalid mapping, key, cache,
  or value type raises `TypeError`; an invalid pose selection raises
  `ValueError`. Any helper preflight failure calls
  `step_sync`, `read`, or a stage write method zero times. For
  transform-producing object types, `position` and `orientation` must be
  selected together. A missing or malformed source `omni:fabric:worldMatrix`
  fails after the physics read identifies that prim and before that type writes.
- AC-2: On success the utility calls `physx.step_sync(dt)` exactly once, then
  calls `PhysX.read` once per selected object type -- the caller's `outputs`
  if given, else the documented per-type default set -- in mapping-iteration
  order, and never calls `update_from_ovstage`.
- AC-3: For each read result, groups with `is_delete` set or with no tensors
  are skipped. Attribute names and stable prim-path-handle tuples resolve
  through an `ovstage.PathDictionary`. Fixed `position` and
  `orientation` groups for rigid bodies, articulation links, and vehicle wheels
  are paired by path tuple, validated for compatible shape and residency, and
  combined with signed scale derived from the current source world matrix using
  the same scale decomposition as ovphysx. Shear is dropped by that
  decomposition. Each pair writes one
  float64 row-vector `omni:fabric:worldMatrix` per prim with MATRIX semantic;
  neither `omni:xform` nor `omni:resetXformStack` is written. The direct world
  matrix is consumer-facing output and does not author local state or schedule
  hierarchy propagation.

  A rigid-body point instancer emits an array `positions`/`orientations` pair
  keyed by its single prim. The utility preserves slots whose read orientation
  is the documented all-zero absent marker from the current OVStage baseline,
  rejects baseline position and orientation arrays with different lengths,
  copies live local poses, preserves a baseline tail longer than the read, and
  grows new rows with zero position and identity orientation. It writes native
  POINT float32x3 `positions` and QUATERNION float16x4 `orientations`, leaving
  all other instancer arrays unchanged. Every other group
  targets `sim:<emitted_name>`; its `prim_list`, fixed tensor or lane-folded
  array tensors, semantic, index map/count, and CUDA synchronization metadata
  forward unchanged. All OVStage source views are released before output
  writes begin. Fixed and instancer pose work runs on the relevant CPU or CUDA
  devices. An optional cache reuses Warp buffers and CUDA events; without one,
  temporary state lasts for one call and work that consumes borrowed data is
  completed before its view is released. Cached CUDA output hands OVStage an
  event recorded after the kernel and no stream. Each successful `write_attribute`
  is counted, so a fixed pose pair contributes one and an instancer pair
  contributes two. Writes and matching `release_query` operations are waited
  on. The return value is the
  number of OVStage attributes successfully written, not the number of groups
  read.
- AC-4: After every selected group is written, the utility calls
  `Stage.advance_write_floor(ordinal=output_ordinal)` exactly once, and that
  is the only ordinal-sealing call it makes. `output_ordinal` is used only as
  the write and floor ordinal; the utility has no code path that could place
  it in a drain range, and a caller-owned mock whose `update_from_ovstage`
  raises must never observe a call.
- AC-5: A failure from `write_attribute` or `release_query` propagates
  unchanged; the query that was already opened for that group is still
  released before the exception propagates, no compensating write or
  rollback is attempted, and `advance_write_floor` is not called, so the
  output ordinal is never sealed on a partial write. The utility adds no new
  native entry point: `step_sync`, `read`, `attach_ovstage`, and
  `update_from_ovstage` are unchanged and remain independently usable before
  and after a call.

## Test References

- TEST-PYTHON-FRAME-001

## Code References

- ovphysx/python/ovphysx/utils/simulation.py (`OvStageOutputCache`, `step_and_write_to_ovstage`)
- ovphysx/python/ovphysx/_utils_kernels.py
- ovphysx/python/ovphysx/utils/simulation.pyi
- ovphysx/tests/python_tests/test_step_and_write_to_ovstage.py
- ovphysx/tests/python_tests/test_type_stubs.py
- ovphysx/tests/python_samples/output_read.py

## Dependencies

- None
