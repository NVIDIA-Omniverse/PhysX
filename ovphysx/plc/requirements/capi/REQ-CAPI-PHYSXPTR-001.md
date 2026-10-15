<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-PHYSXPTR-001
title: Native PhysX Pointers Through Reads
status: implemented
owner: ovphysx
---

## Description

Native PhysX addresses are the read-only `physxPointer` column
(`OVPHYSX_ATTR_PHYSX_POINTER`), a host `uint64` with one borrowed address
per matched object. Query type selects the kind. There is no dedicated
lookup and no compatibility wrapper.

`OVPHYSX_QUERY_TARGET_PHYSICS` is one row. Its path-list entry is `/`, the
absolute root, and that entry is not a USD prim. Prim-set selection
does not change that row. Every other former
`ovphysx_physx_type_t` has a query type that publishes the same address
the old lookup returned. `OVPHYSX_QUERY_TARGET_RIGID_BODY` stays dynamics-only.

The address is copied into session scratch at read time. Scratch follows
the existing read lifetime. The PhysX object stays borrowed. A read does
not grant ownership, extend lifetime, or permit an SDK call the simulation
or device mode already forbids. Do not `release()` the address. Do not
toggle `PxActorFlag::eDISABLE_SIMULATION`; use the `disableSimulation` write.

A missing object is absent, including an explicit selection that matches
nothing. A query type with no `physxPointer` attribute skips that name.
A destroyed object disappears from the next query. A previously read
`uint64` is not cleared in a live session.

A completed step does not invalidate the object. Detach, instance
destruction, and per-object destruction do. Cached addresses are
invalidated by the instance-scoped object-destroyed and
all-objects-destroyed callbacks. Clone does not emit creation; refresh
after the clone task completes. `ovphysx_physx_type_t` remains the
callback payload.

Python reads the same column through `PhysX.read` as a Warp `uint64` array.

## Acceptance Criteria

- AC-1: With an attached, initialized stage, an `OVPHYSX_QUERY_TARGET_PHYSICS`
  read of `physxPointer` returns one non-null host `uint64`. The row's
  path is `/`. A later completed step returns the same address.
- AC-2: For each former path-bound pointer kind, a read of `physxPointer`
  on the matching query type returns a non-null address for a live object.
  A path that matches nothing is absent.
- AC-3: A query type that does not publish `physxPointer` skips that name.
  A standalone joint is absent from `OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT`,
  and an articulation joint is absent from `OVPHYSX_QUERY_TARGET_JOINT`.
- AC-4: After successful `ovphysx_detach_ovstage` completion, a new read
  does not return the previous address. Before attach, the physics row is
  absent.
- AC-5: A completed step does not change a path-bound address. Distinct
  objects of the same type have distinct addresses.
- AC-6: The developer guide states that the column is borrowed, that
  `release()` is forbidden, that toggling `eDISABLE_SIMULATION` on a
  returned actor is unsupported, and that instance-scoped destroy
  callbacks are the invalidation signal.

## Test References

- TEST-CAPI-PHYSXPTR-001
- AC-6 is a documentation obligation, verified by inspecting
  `docs/developer_guide.md`.

## Code References

- ovphysx/include/ovphysx/ovphysx.h
- ovphysx/include/ovphysx/ovphysx_types.h (`ovphysx_query_target_t`)
- ovphysx/include/ovphysx/ovphysx_attributes.h (`OVPHYSX_ATTR_PHYSX_POINTER`)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (read-only writability rows)
- ovphysx/tests/c_unittests/test_physx_ptr.cpp
- ovphysx/docs/developer_guide.md (`PhysX Pointer Interop`)

## Dependencies

- [ADR-0044](../../../ovruntime/plc/adr/ADR-0044-native-pointer-through-reads.md)
- [REQ-READ-POINTER-001](../../../ovruntime/plc/requirements/output/REQ-READ-POINTER-001.md)
- [ADR-0027](../../../ovruntime/plc/adr/ADR-0027-rigid-body-disable-notification.md)
