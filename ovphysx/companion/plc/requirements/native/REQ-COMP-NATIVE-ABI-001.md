<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-NATIVE-ABI-001
title: "Build-matched native extension registration"
status: implemented
owner: ovphysx
---

## Description

Native clients may acquire the original runtime custom joint/geometry interfaces through an optional C++ lease; compiled-module plugins may use the separate loader convenience. Its C entry point does not make PhysX C++ types ABI-independent.

## Implementation scope

The separate loader validates exact table/build/compiler contracts before startup.
Native callback execution, runtime imports and module retention are exercised by
compiled plugins, including an in-flight callback retirement attempt. Post-create
invalid flags/alignment and invalid geometry bounds verify balanced rollback.
Qualification is Linux CPU; the package does not infer other platform or GPU
support from registration. See [compatibility](../../../docs/compatibility.md).

## Acceptance Criteria

- AC-1: **Compatibility.** Registration validates companion ABI size/version, exact supported PhysX build, platform, compiler/runtime ABI and calling convention before invoking plugin callbacks.
- AC-2: **Callback contract.** Tables define plugin context, per-object state, required/optional callbacks, alignment and allocation/release ownership; invalid or incomplete tables are rejected without partial registration.
- AC-3: **Lifetime.** Unregister returns Busy while any object, result lease or callback uses the registration; plugin code/type storage stays retained until objects are released and callbacks drained.
- AC-4: **Native execution.** Solver/contact hot callbacks run in compiled C++ without Python/USD access or blocking reentry into owner waits; non-inline PhysX utilities resolve through supported owner imports or a validated utility library, not a second owner.

- AC-5: **Direct interface boundary.** `custom_interfaces.h` exposes `IPhysxCustomJoint` and `IPhysxCustomGeometry` unchanged, negotiating the exact native build before retaining the existing runtime. Acquisition needs no stage, creates no attachment and preserves output on failure. The lease may outlive its bootstrap instance; native callers retire registrations before release. The main C/C++ headers compile without PhysX/Carbonite SDK includes.

## Test References

- [TEST-COMP-NATIVE-ABI-001](../../tests/native/TEST-COMP-NATIVE-ABI-001.md) - executed compiled-plugin and lifecycle cases; platform limits are explicit.

- [TEST-COMP-NATIVE-DIRECT-001](../../tests/native/TEST-COMP-NATIVE-DIRECT-001.md).

## Code References

- [Direct optional interfaces](../../../include/ovphysx_companion/custom_interfaces.h) and [owner lease](../../../src/CustomInterfaces.cpp).
- [Build compatibility](../../../include/ovphysx_companion/native_compatibility.h).
- [Direct registration sample](../../../samples/native/direct_custom.h).
- [Direct interface tests](../../../../tests/c_unittests/test_companion_custom.cpp).
- [Ordinary C++ header compile test](../../../../tests/c_unittests/test_public_header_no_deprecated.cpp).
- [C module/control table](../../../include/ovphysx_companion/native.h)
- [Build-matched C++ SDK](../../../include/ovphysx_companion/native_sdk.h)
- [Module owner and callback bridge](../../../src/Native.cpp)
- [Python control plane](../../../python/ovphysx_companion/plugins.py)
- [Compiled spring sample](../../../samples/native/spring.cpp)
- [Compiled ball sample](../../../samples/native/ball.cpp)
- [Native integration fixtures](../../../tests_native/test_plugins.py)

Runtime integration anchors:

- [ovphysx/ovruntime/include/omni/physx/IPhysxCustomJoint.h](../../../../ovruntime/include/omni/physx/IPhysxCustomJoint.h)
- [ovphysx/ovruntime/include/omni/physx/IPhysxCustomGeometry.h](../../../../ovruntime/include/omni/physx/IPhysxCustomGeometry.h)
- [ovphysx/ovruntime/include/omni/physx/PhysXRuntime.h](../../../../ovruntime/include/omni/physx/PhysXRuntime.h)

## Dependencies

- [REQ-COMP-API-001](../architecture/REQ-COMP-API-001.md)
- [REQ-COMP-OWNER-001](../architecture/REQ-COMP-OWNER-001.md)
- [Companion ADR-0008](../../adr/ADR-0008-native-extension-sdk.md)
