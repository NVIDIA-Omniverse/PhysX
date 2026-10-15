<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-JOINT-001
title: "Custom joint behavior and lifecycle"
status: implemented
owner: ovphysx
---

## Description

Expose registration, dirty marking and flags for native custom constraints. This is not a promise of reduced-coordinate articulation DOF or tensor integration.

## Implementation scope

The CPU native constraint supports immutable parameters, refresh, dirty/ALWAYS_UPDATE
controls, COM/origin frames and bounded aligned solver rows. Tests exercise live
delete, post-create rejection, owner teardown and callback Busy retirement.
Capabilities reject GPU/DirectGPU and articulation/tensor DOF integration;
articulation actors fail before the plugin create callback.

## Acceptance Criteria

- AC-1: **Callbacks.** Custom joint creation, release, prepare-data, center-of-mass shift, origin shift and constant-block callbacks plus solver prep preserve declared local frames, row limits, mass scaling, data size and alignment.
- AC-2: **Updates.** mark_joint_dirty refreshes prepared data at the documented safe phase; setting and clearing ALWAYS_UPDATE changes the live constraint flags and behavior.
- AC-3: **Release.** Every successfully created per-joint state is released exactly once on object deletion, failed later construction or detach; unregister remains Busy while the state or solver work is live.
- AC-4: **Modes.** Capability reporting distinguishes CPU/GPU solver behavior, DirectGPU limits and unsupported articulation integration; unsupported combinations fail before callback execution.

- AC-5: **Direct native access.** The installed optional C++ SDK exposes the original runtime interface and callback types through a build-matched lifetime lease from an existing instance, without creating an attachment or requiring the compiled-module adapter. Ordinary main headers require no PhysX or Carbonite SDK headers.

## Test References

- [TEST-COMP-JOINT-001](../../tests/native/TEST-COMP-JOINT-001.md) - executed compiled-plugin and lifecycle cases; platform limits are explicit.

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

- [ovphysx/ovruntime/include/omni/physx/IPhysxCustomJoint.h](../../../../ovruntime/include/omni/physx/IPhysxCustomJoint.h)
- [ovphysx/ovruntime/source/omni.physx/plugins/PhysXCustomJoint.cpp](../../../../ovruntime/source/omni.physx/plugins/PhysXCustomJoint.cpp)

## Dependencies

- [REQ-COMP-NATIVE-SCHEMA-001](REQ-COMP-NATIVE-SCHEMA-001.md)
- [REQ-COMP-NATIVE-ABI-001](REQ-COMP-NATIVE-ABI-001.md)
- [Companion ADR-0008](../../adr/ADR-0008-native-extension-sdk.md)
