<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-SCENEQUERY-001
maps_to: REQ-CAPI-SCENEQUERY-001
type: integration
---

## Scenario

A raycast hit's opaque `collision` / `rigid_body` identity fields round-trip
through `ovphysx_scene_query_get_paths_from_ids()` back to the physics-object
path they name; the resolver's argument-validation, truncation, liveness, and
pointer-lifetime contract holds independent of any real hit.

## Given

- `SceneQueryRigidBodyTest`, the existing raycast/sweep/overlap fixture
  (`ovphysx/tests/c_unittests/test_scene_query.cpp`), which loads
  `tests/data/boxes_falling_on_groundplane.usda` and steps once before each
  case.
- `PhysXTestFixture`, the base fixture with no scene loaded, for the
  no-active-attach case.
- A plain `TEST` with no fixture, for the unknown-instance-handle case (no
  live instance is needed to exercise handle-lookup failure).

## When

- **Round trip (AC-1).** `RaycastHitResolvesToGroundPlanePath` raycasts
  straight down from above the ground plane, takes the single CLOSEST hit's
  `collision` and `rigid_body` fields as input ids, and calls
  `ovphysx_scene_query_get_paths_from_ids` with `max_paths == id_count == 2`.
- **Unresolvable id (AC-2).** `ResolveUnknownIdYieldsEmptyPath` passes the
  `0` sentinel id with no corresponding object.
- **Removed object, same live attach (AC-2).**
  `ResolveRemovedObjectYieldsEmptyPath` resolves the ground plane collision
  shape's live `collision` id, tombstones that exact prim in the *same*
  still-attached ovstage instance (`remove_ovstage_prim`:
  `ovstage_delete_attributes` with an empty attribute list -- mirroring
  `OvstageChangeTemplate::deletePrim`, the ovruntime test harness's own
  structural-removal primitive -- then `ovphysx_update_from_ovstage`, no
  detach/reattach), then resolves the same id again.
- **No active attach (AC-2).** `ResolvePathsFromIdsNoActiveAttach`
  (`PhysXTestFixture`, no stage loaded) resolves an arbitrary nonzero id.
- **Argument validation (AC-3).**
  `ResolvePathsFromIdsNullIdsWithNonzeroCount` passes `ids == NULL` with
  `id_count == 1`; `ResolvePathsFromIdsNullOutCount` passes `out_count ==
  NULL` with `out_paths` pre-seeded with sentinel bit patterns;
  `ResolvePathsFromIdsNullOutPathsWithNonzeroMax` passes `out_paths == NULL`
  with `max_paths == 1`; `SceneQueryHandleValidation.
  ResolvePathsFromIdsUnknownInstanceHandle` passes a handle
  (`0xDEADBEEF`) that `ovphysx_create_instance()` never returned.
- **Truncation (AC-1).** `ResolvePathsFromIdsTruncatesToMaxPaths` passes
  `id_count == 2` with `max_paths == 1`.
- **Pointer lifetime (AC-4).** `ResolveAfterDetachReattachGivesFreshAnswer`
  resolves a live id and records its path text, detaches and re-attaches the
  identical scene from scratch (a fresh source, fresh `ObjectKey`
  generation), then resolves fresh vs. stale ids under the new attach. It
  does not dereference the pre-detach pointer after the detach -- that would
  be UB -- only the documented, observable behavior is checked.

## Then

- `RaycastHitResolvesToGroundPlanePath`: the call returns
  `OVPHYSX_API_SUCCESS`, `*out_count == 2`, and both resolved path strings
  contain `/World/GroundPlane` -- confirming `objectKeyToPath` reached the
  real collision shape and rigid body the raycast hit, not an empty or
  unrelated path (REQ AC-1).
- `ResolveUnknownIdYieldsEmptyPath`: the call still returns
  `OVPHYSX_API_SUCCESS` and `*out_count == 1`, but the resolved entry has
  `length == 0` (REQ AC-2) -- an unresolvable id is not an error.
- `ResolveRemovedObjectYieldsEmptyPath`: the baseline resolve (before
  removal) is non-empty; after the same-attach repopulation removes the
  object, `*out_count == 1` and the resolved entry has `length == 0` (REQ
  AC-2) -- a key does not keep resolving to a stale path once its object is
  gone from the still-live attach.
- `ResolvePathsFromIdsNoActiveAttach`: the call returns
  `OVPHYSX_API_SUCCESS`, `*out_count == 1`, and the resolved entry has
  `length == 0` (REQ AC-2).
- The four argument-validation cases each fail cleanly: the first three
  return `OVPHYSX_API_INVALID_ARGUMENT` (REQ AC-3); `ResolvePathsFromIdsNull
  OutCount` additionally asserts `out_paths[0]` still holds its seeded
  sentinel `ptr`/`length` (the no-touch guarantee, REQ AC-3); the unknown-
  handle case asserts a non-`OVPHYSX_API_SUCCESS` status and `*out_count ==
  0`, not a crash (REQ AC-3).
- `ResolvePathsFromIdsTruncatesToMaxPaths`: the call returns
  `OVPHYSX_API_SUCCESS` with `*out_count == 2` (the full id count) even
  though only 1 entry fit in `out_paths`, confirming the "total needed, not
  written" counting contract (REQ AC-1).
- `ResolveAfterDetachReattachGivesFreshAnswer`: the fresh attach's own
  resolve of the same physics object returns the identical path text (via a
  new pointer) the pre-detach resolve did; the id captured before the detach
  does not resolve (`length == 0`) under the new attach (REQ AC-4).

This suite now exercises AC-4's pointer-lifetime contract as narrowed
(`ResolveAfterDetachReattachGivesFreshAnswer`) -- not by reading a freed
pointer, which is not observable without UB, but by confirming the documented
invalidation boundary (detach/re-attach) through its effect on subsequent
resolves. The `ObjectKey` generation-uniqueness mechanism this pointer-
lifetime and liveness behavior is built on (including the cross-backend case)
is pinned directly at the ovruntime layer by REQ-PUBLICAPI-003 /
TEST-PUBLICAPI-003's `TestObjectKeyMinting.cpp` cases, not re-proven here.
