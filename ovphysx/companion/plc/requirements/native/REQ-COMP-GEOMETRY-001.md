<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-GEOMETRY-001
title: "Custom geometry callbacks and destruction"
status: implemented
owner: ovphysx
---

## Description

Custom geometry supports native contact behavior, mass and optional queries/debug output, with a reliable per-shape destruction contract.

## Implementation scope

Required callbacks are validated; separate capability bits describe optional
raycast/overlap/sweep/debug functions. The compiled ball demonstrates analytical
mass/bounds and CPU rigid contacts/queries/debug. SDK shape destruction releases
its exact callback state immediately, including live deletion and rollback.
Authored clones and unit-scale point instances have distinct states; unsupported
scale fails before create. Custom SDF fallback, mesh merge and GPU-only pairs are
unavailable independently of stopped SDF cooking.

## Acceptance Criteria

- AC-1: **Required callbacks.** Registration requires create, release, bounds, contacts, mass-properties and persistent-manifold callbacks; optional raycast/overlap/sweep/debug callbacks have explicit availability semantics.
- AC-2: **Functionality.** A geometry sample demonstrates dynamic mass, contact response, bounds and supported query/debug behavior through the owning runtime, not registration success alone.
- AC-3: **Destruction.** Every successful per-shape create receives exactly one release on deletion/detach or construction rollback; type storage and plugin code outlive all shapes/callbacks and unregister returns Busy while used.
- AC-4: **Mode matrix.** Capabilities state CPU custom processing, unsupported GPU-only deformable/particle interaction and DirectGPU collision-data limits; SDF fallback and specialized mesh-merge support are reported separately.

- AC-5: **Direct native access.** The installed optional C++ SDK exposes the original runtime interface and callback types through a build-matched lifetime lease from an existing instance, without creating an attachment or requiring the compiled-module adapter. Ordinary main headers require no PhysX or Carbonite SDK headers.

## Test References

- [TEST-COMP-GEOMETRY-001](../../tests/native/TEST-COMP-GEOMETRY-001.md) - executed compiled-plugin and lifecycle cases; platform limits are explicit.

## Code References

- [Direct optional interfaces](../../../include/ovphysx_companion/custom_interfaces.h) and [owner lease](../../../src/CustomInterfaces.cpp).
- [Direct registration sample](../../../samples/native/direct_custom.h).
- [C module/control table](../../../include/ovphysx_companion/native.h)
- [Build-matched C++ SDK](../../../include/ovphysx_companion/native_sdk.h)
- [Module owner and callback bridge](../../../src/Native.cpp)
- [Python control plane](../../../python/ovphysx_companion/plugins.py)
- [Compiled spring sample](../../../samples/native/spring.cpp)
- [Compiled ball sample](../../../samples/native/ball.cpp)
- [Native integration fixtures](../../../tests_native/test_plugins.py)

Runtime integration anchors:

- [ovphysx/ovruntime/include/omni/physx/IPhysxCustomGeometry.h](../../../../ovruntime/include/omni/physx/IPhysxCustomGeometry.h)
- [ovphysx/ovruntime/source/omni.physx/plugins/PhysXCustomGeometry.cpp](../../../../ovruntime/source/omni.physx/plugins/PhysXCustomGeometry.cpp)
- [ovphysx/ovruntime/source/omni.physx/plugins/PhysXCustomGeometry.h](../../../../ovruntime/source/omni.physx/plugins/PhysXCustomGeometry.h)

## Dependencies

- [REQ-COMP-NATIVE-SCHEMA-001](REQ-COMP-NATIVE-SCHEMA-001.md)
- [REQ-COMP-NATIVE-ABI-001](REQ-COMP-NATIVE-ABI-001.md)
- [Companion ADR-0008](../../adr/ADR-0008-native-extension-sdk.md)
