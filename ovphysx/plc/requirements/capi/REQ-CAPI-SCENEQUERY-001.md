<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-SCENEQUERY-001
title: Scene-Query Hit Identity Path Resolution
status: implemented
owner: ovphysx
---

## Description

`ovphysx_raycast()`, `ovphysx_sweep()`, and `ovphysx_overlap()` report a hit's
`collision`, `rigid_body`, and `material` fields as an opaque
`omni::physics::parse::ObjectKey.handle` rather than a uint64-encoded `SdfPath`
(ADR-0019, landed for the scene-query surface alongside the `IPhysx` migration
covered by REQ-CAPI-OBJECTKEY-001). That migration left scene-query callers with
no way to turn a hit's identity fields back into a path -- unlike contact
bindings, whose `ovphysx_contact_binding_get_other_actor_paths_from_ids()`
already solves the same problem for a column of `ovphysx_read_raw_contact_data()`'s
`actor_ids_tensor`.

This requirement adds the scene-query counterpart:
`ovphysx_scene_query_get_paths_from_ids()` (Python:
`PhysX.get_scene_query_paths_from_ids()`). It resolves an array of
`ObjectKey.handle` values -- taken directly from hit-result identity fields --
to physics-object path strings, using the same `IPhysx::objectKeyToPath`
mechanism the runtime already exposes as one of the two public functions
allowed to cross a path string at the `IPhysx` boundary (ADR-0019 decision 3).
Unlike the contact-binding resolver, this function needs no per-call cache:
each returned pointer stays valid past the call that produced it and past
later calls to this same function.

`objectKeyToPath` gates on liveness before rendering a key (mirroring
`IPhysx::resolveObjectKey`'s own `source->exists(key)` check): a key whose
object is no longer live under the *current* attach -- including one removed
since it was minted, with no detach/reattach in between -- resolves to empty
rather than a stale cached path (round-3 review, MR !8217). Its returned
pointer is owned by the currently attached `IPhysicsSource` and is valid only
until the next detach or source rebuild, not for "the runtime's lifetime"
unqualified; a caller that needs a path to outlive that must copy it. The
underlying `ObjectKey` generation-tagging that makes a stale key fail to
resolve rather than alias a live one -- including across a USD-to-ovstage (or
reverse) source switch -- is REQ-PUBLICAPI-003's guarantee, which this
resolver inherits rather than re-implements.

## Acceptance Criteria

- AC-1: `ovphysx_scene_query_get_paths_from_ids(handle, ids, id_count,
  out_paths, max_paths, out_count)` resolves each entry of `ids` through
  `IPhysx::objectKeyToPath` and writes the result to the matching index of
  `out_paths`, in order. `*out_count` is always set to `id_count` (the total
  number of ids given), matching
  `ovphysx_contact_binding_get_other_actor_paths_from_ids()`'s "total needed,
  not written" convention; only `min(id_count, max_paths)` entries are
  actually written to `out_paths`, so a caller detects truncation by
  comparing `*out_count` against `max_paths`.

- AC-2: An id that does not resolve to a live object -- the zero/invalid
  sentinel, an id from an object removed since the query (whether or not the
  attach has since been detached and re-attached), or any id when there is no
  active attach or source -- yields an empty (non-NULL, zero-length)
  `ovphysx_string_t` entry rather than an error for that entry. The call as a
  whole still returns `OVPHYSX_API_SUCCESS`.

- AC-3: `out_count == NULL` returns `OVPHYSX_API_INVALID_ARGUMENT` without
  touching `out_paths` (its prior contents are unmodified). `ids == NULL`
  with `id_count > 0` returns `OVPHYSX_API_INVALID_ARGUMENT`. `out_paths ==
  NULL` with `max_paths > 0` returns `OVPHYSX_API_INVALID_ARGUMENT`. An
  unknown instance `handle` -- including one that was never returned by
  `ovphysx_create_instance()`, not only the zero sentinel -- returns a
  non-success status without crashing.

- AC-4: A pointer written to `out_paths[i]` by one call remains valid after a
  later, unrelated call to this same function -- there is no per-call cache to
  refill, unlike `ovphysx_contact_binding_get_other_actor_paths_from_ids()`'s
  per-binding cache which invalidates prior pointers on refill. That pointer's
  validity is bounded, though: it is owned by the currently attached
  `IPhysicsSource` and becomes invalid at the next detach/re-attach. A fresh
  resolve after a detach/re-attach against the same or an equivalent scene
  returns the same path text (via a new pointer); an id captured before the
  detach does not resolve under the new attach (AC-2).

## Test References

- TEST-CAPI-SCENEQUERY-001

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_scene_query_get_paths_from_ids` declaration, `ovphysx_scene_query` group)
- ovphysx/src/ovphysx/ovphysxSceneQuery.cpp (`ovphysx_scene_query_get_paths_from_ids` implementation)
- ovphysx/ovruntime/include/omni/physx/IPhysx.h (`objectKeyToPath` declaration/contract)
- ovphysx/ovruntime/source/omni.physx/plugins/PhysX.cpp (`objectKeyToPath` implementation, liveness gate)
- ovphysx/python/ovphysx/_bindings.py (ctypes prototype)
- ovphysx/python/ovphysx/api.py (`PhysX.get_scene_query_paths_from_ids`)
- ovphysx/python/ovphysx/api.pyi (stub)
- ovphysx/tests/c_unittests/test_scene_query.cpp (`SceneQueryRigidBodyTest.RaycastHitResolvesToGroundPlanePath` and the surrounding `ovphysx_scene_query_get_paths_from_ids` cases)
- ovphysx/docs/developer_guide.md ("Path Encoding" section under Scene Queries)

## Dependencies

- [ADR-0019](../../../ovruntime/plc/adr/ADR-0019-public-api-object-identity.md) - retires SdfPath as omni.physx's public object identity in favor of ObjectKey
- [REQ-CAPI-OBJECTKEY-001](REQ-CAPI-OBJECTKEY-001.md) - the sidecar-level ObjectKey boundary this resolver also crosses through `IPhysx::objectKeyToPath`
- [REQ-PUBLICAPI-003](../../../ovruntime/plc/requirements/api/REQ-PUBLICAPI-003.md) - the `ObjectKey` generation-tagging guarantee (including the cross-backend case) that makes a stale/removed key fail to resolve here rather than alias a live one
