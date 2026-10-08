<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OBJECTKEY-001
title: Internal Sidecar Object Identity via ObjectKey
status: implemented
owner: ovphysx
---

## Description

The internal sidecar (`ovphysx/src/ovphysxInternal/`) bridges ovphysx's C ABI to
`omni::physx::IPhysx`, the runtime interface ovphysx links against. `IPhysx` names
objects by `omni::physics::parse::ObjectKey`, an opaque per-source identity handle;
[ADR-0019](../../../ovruntime/plc/adr/ADR-0019-public-api-object-identity.md) retired
`SdfPath` as that public boundary's object identity and named `resolveObjectKey` /
`objectKeyToPath` as the only two `IPhysx` entry points still allowed to cross a
path string.

This requirement states the sidecar's obligation at that boundary: wherever it
crosses from a path string (received over its own C ABI, which keeps speaking
plain `const char*` prim paths to ITS callers -- that surface is unaffected) into
`IPhysx`, it resolves through `resolveObjectKey`; wherever `IPhysx` hands it an
`ObjectKey`, it renders back through `objectKeyToPath` before forwarding a path
string across its own C callback surface. No sidecar code constructs, stores, or
compares an `SdfPath` to satisfy an `IPhysx` call. The last `SdfPath` use, the
uint64-encoded hit-result path once exchanged with `IPhysxSceneQuery`
(`ovphysx_encode_sdf_path`, `sdfPathToInt`), left with the sidecar's USD link
(REQ-PACKAGING-NOUSD-001); scene-query results carry object keys.

The pathless `PxPhysics` lookup defined by
[REQ-CAPI-PHYSXPTR-001](REQ-CAPI-PHYSXPTR-001.md) is the sole pointer-lookup
exception. Its zero-length selector names no source object, so the sidecar passes
the invalid `ObjectKey{}` sentinel directly to `IPhysx::getPhysXPtr`; the runtime
maps that sentinel back to the empty path reserved for `ePTPhysics`.

## Acceptance Criteria

- AC-1: For path-bound types, `ovphysx_internal_get_physx_ptr` (backing the
  public `ovphysx_get_physx_ptr`) resolves its `prim_path` argument to an
  `ObjectKey` via `IPhysx::resolveObjectKey` and passes that key -- not a path --
  to `IPhysx::getPhysXPtr`. A path that does not resolve (`ObjectKey::valid() ==
  false`) returns `nullptr` without calling `getPhysXPtr`, matching the prior
  behavior's null-return contract for an unresolvable path. The pathless
  `ePTPhysics` case instead passes `ObjectKey{}` directly, as required by
  REQ-CAPI-PHYSXPTR-001; no path string is resolved for that case.

- AC-2: `ovphysx_internal_update_kinematic` (backing the public
  `ovphysx_articulation_update_kinematic`, called once per articulation prim path
  in the target binding) resolves the same way before calling
  `IPhysx::getPhysXPtr(key, ePTArticulation)`. An unresolvable path returns
  `false` for that call, preserved as the existing per-articulation failure signal
  the caller already aggregates into `OVPHYSX_API_ERROR`.

- AC-3: `ovphysx_internal_subscribe_object_changes`'s `objectCreationNotifyFn` and
  `objectDestructionNotifyFn` lambdas registered with
  `IPhysx::subscribeObjectChangeNotifications` accept the `ObjectKey` `IPhysx`'s
  `ObjectCreationNotificationFn` / `ObjectDestructionNotificationFn` signatures
  carry, and render it to a path string via `IPhysx::objectKeyToPath(key)` before
  invoking the sidecar's own `on_created` / `on_destroyed` C callback with that
  string. The C callback signature itself (`const char*` + length) is unchanged.

- AC-4: The sidecar's own C ABI (`ovphysx_internal_get_physx_ptr`,
  `ovphysx_internal_update_kinematic`, and the `on_created` / `on_destroyed`
  function-pointer parameters of `ovphysx_internal_subscribe_object_changes`) is
  unaffected by this requirement: every one of those entry points still takes or
  emits a plain path string. `ObjectKey` is exclusively an internal detail of how
  the sidecar talks to `IPhysx`; it never crosses the sidecar's own C boundary.

## Test References

- TEST-CAPI-OBJECTKEY-001

## Code References

- ovphysx/src/ovphysxInternal/ovphysxInternalInterop.cpp (`ovphysx_internal_get_physx_ptr`, `ovphysx_internal_update_kinematic`)
- ovphysx/src/ovphysxInternal/ovphysxInternalObjectChange.cpp (`ovphysx_internal_subscribe_object_changes`)
- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_get_physx_ptr`, `ovphysx_articulation_update_kinematic`, `ovphysx_subscribe_object_changes` -- the public entry points this sidecar backs; signatures unchanged per AC-4)
- ovphysx/tests/c_unittests/test_physx_ptr.cpp (primary AC-1 coverage: per-type happy path, type-mismatch, pointer-identity, and lifecycle cases against `ovphysx_get_physx_ptr`)
- ovphysx/tests/c_unittests/test_tensor_binding.cpp (`CpuArticulationUpdateKinematicPropagatesDofToLinks` -- AC-2 coverage)
- ovphysx/tests/c_unittests/test_object_change_callbacks.cpp (AC-3/AC-4 subscription lifecycle coverage)
- ovphysx/tests/c_samples/physx_interop_cpp/main.cpp (AC-1 coverage, C sample; see TEST-CAPI-OBJECTKEY-001's environment note)

## Dependencies

- [ADR-0019](../../../ovruntime/plc/adr/ADR-0019-public-api-object-identity.md) - retires SdfPath as omni.physx's public object identity in favor of ObjectKey
- [REQ-CAPI-PHYSXPTR-001](REQ-CAPI-PHYSXPTR-001.md) - defines the pathless `PxPhysics` exception
