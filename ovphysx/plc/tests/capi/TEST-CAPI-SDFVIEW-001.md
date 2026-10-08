<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-SDFVIEW-001
maps_to: REQ-CAPI-SDFVIEW-001
type: integration
---

## Scenario

An SDF view that outlives its stage is invalidated rather than fatal. Both stage
teardown routes -- `reset_stage()` and `detach_ovstage()` -- are exercised with an
undisposed view, and each is followed by an `evaluate()` on the stale handle. The
stale call must raise a catchable `RuntimeError` and the process must survive.

Regression origin: NVBug 6473872 (stale `evaluate()` after `reset_stage()` SIGSEGVs,
exit 139) and NVBug 6533106, which reported the detach half as still broken. The
detach case was in fact already fixed, but the shipped detach test could not have
detected it: it called `detach_ovstage()` and ended without evaluating again, so it
never entered the faulting path. Every teardown case in this specification therefore
ends in a stale `evaluate()`.

## Given

- A GPU PhysX instance with DirectGPU enabled (`/physics/suppressReadback`), since
  SDF evaluation is GPU-only.
- The `sdf_cube.usda` fixture attached through ovstage, warmed up.
- An `SdfView` over `/World/Cube` with `max_query_points=2`, plus CUDA-resident
  query `[1, 2, 3]` and output `[1, 2, 4]` float32 buffers.
- A baseline `evaluate()` returning the expected signed distances (negative inside,
  positive outside), establishing that the view was live before teardown.

## When

- The view is left undisposed and `reset_stage()` is called, then `evaluate()` is
  called on the stale handle.
- The view is left undisposed and `detach_ovstage()` is called, then `evaluate()`
  is called on the stale handle.
- The stage is reset and reloaded, and `evaluate()` is called on a handle created
  before the reset.
- `destroy()` is called twice on a handle whose native view was already released by
  teardown.
- A fresh view is created after a reset and reload, and evaluated.

## Then

- Every stale `evaluate()` raises `RuntimeError` whose message identifies the
  invalidation (`"SDF view invalidated (stage changed); recreate view"` or
  `"SDF view not found"`); it does not return successfully (REQ AC-2).
- The pytest process exits normally in every case; no SIGSEGV / exit 139 on either
  the reset or the detach route (REQ AC-3). Cases are run in separate processes when
  isolating a crash, since a SIGSEGV takes the whole session down.
- The stale `destroy()` calls return without raising (REQ AC-5).
- The freshly created post-reset view evaluates and returns the expected signed
  distances, so invalidation does not leave the instance unusable (REQ AC-1).
- Not asserted here: that the AC-4 ordering rejects the handle *before* the
  auto-warmup runs. Both orderings raise the same error, and the difference is an
  internal side effect (a suppressed `simulate()` / `fetchResults()` pass) with no
  observable at the public API surface. It is enforced by code structure and review.
