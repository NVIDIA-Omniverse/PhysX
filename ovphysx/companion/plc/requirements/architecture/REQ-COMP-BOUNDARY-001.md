<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-BOUNDARY-001
title: "Optional companion and native dependency boundary"
status: implemented
owner: ovphysx
---

## Description

Expose authoring, live tooling and native extension services as an optional companion. Keep reusable native algorithms in ovruntime and compile the owner bridge into the existing owner. This contract covers companion integration, not a second runtime product.

## Acceptance Criteria

- AC-1: **Placement.** Companion public headers, facades, bindings and Python utilities live under ovphysx/companion; ovruntime depends on no companion public header, Python module or Kit service.
- AC-2: **Native closure.** Companion native code and compiled Python bindings include and link no USD or Kit libraries and accept no USD objects, opaque USD pointers, path-storage bits or stage-cache IDs for native dereferencing.
- AC-3: **Single owner.** The companion resolves simulation services through the loaded ovphysx owner; its packages and native plugins do not contain a second statically linked OvruntimePhysX/PhysX owner.
- AC-4: **Optionality.** Core initialize, attach, step, read and detach work with the companion package absent; the companion exposes editor functionality in dedicated headers rather than adding it to the main simulation header.

## Implementation scope

Native services are compiled into the existing runtime owner and contain no USD or
Kit dependency. The separate Python wheel has no bundled runtime. Isolated core
processes complete attach/step/read/detach both with no importable companion and
with the package enabled; the latter observes the exact core owner. Tests inspect
header direction and native dependency closure. Installed SDK validation separately
enforces the py-less/USD-free package lock.

## Test References

- [TEST-COMP-BOUNDARY-002](../../tests/architecture/TEST-COMP-BOUNDARY-002.md) - executed native/Python subset.

- [TEST-COMP-BOUNDARY-001](../../tests/architecture/TEST-COMP-BOUNDARY-001.md) - executed supported-contract scenario.

## Code References

Companion implementation (native/core changes retain their own PLC ownership):

- [include/ovphysx_companion/companion.h](../../../include/ovphysx_companion/companion.h)
- [src/Companion.cpp](../../../src/Companion.cpp)
- [python/ovphysx_companion/native.py](../../../python/ovphysx_companion/native.py)

Existing integration anchors for remaining scope:

- [ovphysx/include/ovphysx/ovphysx.h](../../../../include/ovphysx/ovphysx.h)
- [ovphysx/CMakeLists.txt](../../../../CMakeLists.txt)
- [ovphysx/ovruntime/source/omni.physx/CMakeLists.txt](../../../../ovruntime/source/omni.physx/CMakeLists.txt)

- [test_boundary.py](../../../tests_native/test_boundary.py)
- [compatibility.md](../../../docs/compatibility.md)

## Dependencies

- [Companion ADR-0001](../../adr/ADR-0001-optional-companion-and-owner-bridge.md)
