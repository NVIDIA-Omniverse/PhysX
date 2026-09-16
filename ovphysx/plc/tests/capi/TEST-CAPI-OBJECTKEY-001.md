<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OBJECTKEY-001
maps_to: REQ-CAPI-OBJECTKEY-001
type: integration
---

## Scenario

The sidecar's three `IPhysx`-crossing entry points -- raw pointer lookup, kinematic
propagation, and object-change notification -- still resolve real objects and
report real content after being converted from `SdfPath` to `ObjectKey` at the
`IPhysx` boundary. Coverage is drawn from the existing C++ unit suites (and, where
buildable, the C interop sample); this requirement adds no new test binaries, only
the traceability record for behavior those suites already exercise end to end.

## Given

- A stepped `ovphysx` instance with an attached USD scene containing rigid bodies,
  an articulation with links and joints, and a standalone joint, loaded via
  ovstage (`ovphysx/tests/c_unittests/test_physx_ptr.cpp`'s `PhysxPtrRigidBodyTest`
  / `PhysxPtrArticulationTest` / `PhysxPtrStandaloneJointTest` fixtures).
- A subscriber registered through `ovphysx_subscribe_object_changes` before stage
  attach, recording every `on_object_created` / `on_object_destroyed` /
  `on_all_objects_destroyed` invocation it receives.

## When

- **`getPhysXPtr` (AC-1).** `ovphysx_get_physx_ptr` is called per `ovphysx_physx_type_t`
  (SCENE, ACTOR, ARTICULATION, LINK, LINK_JOINT, standalone JOINT) against real
  attached objects, plus argument validation for path-bound lookups
  (null/empty/embedded-NUL `prim_path`, null `out_ptr`, invalid instance handle),
  type-mismatch lookups (JOINT on an articulation joint, PARTICLE_* on a
  rigid-body scene), pointer-identity checks (stable across `ovphysx_step()`,
  distinct between distinct prims of the same type), a nonexistent-path lookup,
  and a lookup after `ovphysx_reset_stage()` (`ovphysx/tests/c_unittests/test_physx_ptr.cpp`,
  all `PhysxPtr*Test` fixtures and the `PhysXTestFixture.GetPhysxPtr*`
  argument-validation cases). The pathless PHYSICS cases in the same file verify
  that the zero-length selector reaches `getPhysXPtr(ObjectKey{}, ePTPhysics)`;
  their complete selector contract is mapped by TEST-CAPI-PHYSXPTR-001. The C
  interop sample additionally casts the returned pointer to `PxRigidDynamic*`,
  calls `setKinematicTarget` on it directly, steps again, and reads the body's
  pose back through a tensor binding
  (`ovphysx/tests/c_samples/physx_interop_cpp/main.cpp`) -- not run in this pass;
  see Known gap below.
- **`update_kinematic` (AC-2).** A DOF-position tensor write is applied to an
  articulation binding, `ovphysx_articulation_update_kinematic` is called with
  `OVPHYSX_ARTICULATION_KINEMATIC_POSITION`, and non-root link poses are read back
  without an intervening simulation step
  (`TensorBindingCpuTest.CpuArticulationUpdateKinematicPropagatesDofToLinks` in
  `ovphysx/tests/c_unittests/test_tensor_binding.cpp`).
- **Object-change subscription lifecycle (AC-3, AC-4).** Subscribe / unsubscribe
  roundtrip, argument validation (null callbacks, null out-id, all-null callback
  struct, unknown/invalid subscription ids), a stage attach + step while
  subscribed (initial population must stay silent), and `ovphysx_reset_stage()`
  while subscribed (must fire `on_all_objects_destroyed`, then stop firing after
  unsubscribe) -- all in
  `ovphysx/tests/c_unittests/test_object_change_callbacks.cpp`.

## Then

- Every `PhysxPtr*Test` case returns a non-null, correctly-typed pointer for a
  real object, `NonExistentPathReturnsNotFound` returns `OVPHYSX_API_NOT_FOUND`
  with a null `out_ptr` for a path that does not resolve, and
  `DistinctArticulationsHaveDistinctPointers` / `DistinctLinksHaveDistinctPointers`
  confirm each `ObjectKey` resolves to its own object rather than aliasing --
  `resolveObjectKey` + `getPhysXPtr(key, type)` reach the correct object and the
  AC-1 null-return contract holds for an unresolvable path. The PHYSICS cases
  return the same non-null pointer through the intentional `ObjectKey{}`
  exception (REQ AC-1). Where the C interop sample can be built and run (see
  Known gap), its tensor read-back position matches the kinematic target set
  through the returned pointer.
- The link poses read back after `ovphysx_articulation_update_kinematic` differ
  from their pre-update values in the direction implied by the DOF write, with no
  intervening step -- `getPhysXPtr(key, ePTArticulation)` reached the correct live
  `PxArticulationReducedCoordinate` and `updateKinematic` ran on it (REQ AC-2).
- Argument-validation cases each return the documented status
  (`OVPHYSX_API_INVALID_ARGUMENT` / `OVPHYSX_API_NOT_FOUND`) without constructing a
  subscription. The subscribe/unsubscribe roundtrip succeeds and a repeat
  unsubscribe reports `NOT_FOUND`. Initial stage population delivers zero
  `on_object_created` / `on_object_destroyed` calls. `ovphysx_reset_stage()` while
  subscribed increments the all-destroyed counter at least once; after unsubscribe
  it does not increment further (REQ AC-3, AC-4 -- these cases exercise the
  lambdas' construction and capture of `physx` without crashing or hanging; none
  of them drives a per-object `on_created` / `on_destroyed` call with live content
  to assert against).
- No case in any of the above suites terminates the process or hangs; all report
  through their normal `ovphysx_result_t` / GTest assertion path.

**Environment note:** the C interop sample (`physx_interop_cpp`) and the CI-pinned
`*GpuTest*` fixtures were not exercised in every verification pass -- the sample's
CMake `find_package(ovstage)` step failed against a locally-checked-out ovstage
dev build present on the verifying host (a local-checkout path-resolution gap
unrelated to this fix), and GPU passes need CUDA device access the verifying
container did not have. `test_physx_ptr.cpp` (CPU) content-verifies the same
`getPhysXPtr` boundary the sample exercises, so AC-1 is confirmed either way.

**Known gap, not closed by this requirement:** no ovphysx-level test drives a
per-object `on_object_created` / `on_object_destroyed` callback with a live prim
and asserts the delivered path string's content (the pre-existing gap the
`ovphysx_clone()` docstring in `ovphysx.h` and the comment above
`ObjectChangeIntegrationTest` already flag -- ovphysx has no supported stimulus
that recreates a single actor mid-simulation). AC-3's `objectKeyToPath` rendering
is exercised structurally (lambda compiles, captures `physx`, is registered and
invoked-or-not-invoked correctly by the scenarios above) but not
content-verified at the ovphysx layer. The runtime primitives it calls
(`IPhysx::resolveObjectKey`, `IPhysx::getPhysXPtr`, `IPhysx::objectKeyToPath`) are
independently content-verified by `ovphysx/ovruntime`'s own suite (e.g.
`TestObjectChangeNotifications.cpp`, `TestOvstageStageless.cpp`'s
resolveObjectKey/objectKeyToPath round-trip cases, `TestPhysXInterface.cpp`,
`TestSceneOperations.cpp`).
