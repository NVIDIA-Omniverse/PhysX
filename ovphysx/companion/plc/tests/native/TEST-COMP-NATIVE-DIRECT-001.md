<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-NATIVE-DIRECT-001
maps_to: REQ-COMP-NATIVE-ABI-001
type: integration
---

## Scenario

Acquire and use the original custom interfaces through an existing runtime,
without requiring their SDK dependencies in ordinary headers.

## Given

- Two initialized ovphysx instances, no attachment, and matching native build metadata in both Release and Debug configurations.
- Original runtime joint/geometry callback structures and an output canary.

## When

- Acquire with an incompatible SDK build, the opposite Debug/Release mode, invalid size and missing instance.
- Acquire through both instances, destroy one bootstrap, register and retire native types through the original tables.
- Attach and step an ordinary scene after registration retirement.
- Compile the normal C/C++ public headers with no PhysX/Carbonite include path, and compile the optional native headers against the installed matching PhysX SDK.

## Then

- Rejected acquisition preserves the output canary, including a Debug/Release mode mismatch (REQ AC-1, AC-5).
- Matching Debug and Release consumers acquire successfully; the C++ test target defines `_DEBUG` in Debug just as the library does (REQ AC-1, AC-5).
- Both leases expose the same loaded runtime tables, with the original typed callbacks and no new attachment (REQ AC-5).
- Destroying the bootstrap leaves its lease usable; explicit shutdown refuses an outstanding lease (REQ AC-5).
- Original registration/unregistration succeeds, and releasing a retired lease twice reports NotFound (REQ AC-5).
- Ordinary headers need no PhysX/Carbonite SDK; optional custom interface headers compile using the installed SDK only (REQ AC-5).

## Executable coverage

- [test_companion_custom.cpp](../../../../tests/c_unittests/test_companion_custom.cpp): `CustomInterfacesTest.RejectedAcquisitionPreservesOutput` and `CustomInterfacesTest.OriginalTablesRegisterBeforeAttachAndRetainRuntime`.
- [test_companion_access.cpp](../../../../tests/c_unittests/test_companion_access.cpp): `CompanionNativeAccess.BuildNegotiationPreservesRejectedOutput` and `CompanionAccess.TypedBorrowRejectsRecursiveWaitAndStaleRecord`, including an opposite-mode output canary.
- [C/C++ header targets](../../../../tests/c_unittests/CMakeLists.txt): public header compilation and installed sample compilation.
- [Direct native consumer](../../../samples/native/direct_custom.h).

## Build-mode regression evidence

On 2026-10-02, the four native-access/custom-interface cases reproduced the
Linux Debug CI failures when the test target omitted `_DEBUG`. With the test
target aligned to the library, all four pass in Debug and all four pass in
Release, including rejection of the opposite mode without changing outputs.
The isolated Debug build and the changed Release test sources compile with
warnings treated as errors. Windows execution remains a CI gate.
The broader local Debug CPU group passes 501 cases with one existing MBP
visualization skip; OVD loopback tests require local socket access outside the
network sandbox. GPU qualification remains in CI.
