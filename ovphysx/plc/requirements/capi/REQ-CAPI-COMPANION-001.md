<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-COMPANION-001
title: Companion discovery and core owner lifetime admission
status: implemented
owner: ovphysx
---

## Description

The core library exports a separate companion acquisition entry point from the
same loaded runtime owner. Borrowed native sessions retain an explicit instance
and attachment generation. Owned preview sessions retain the same process services independently of their
bootstrap instance and attach through a separate companion table. They do not
create another public facade attachment.

## Acceptance Criteria

- AC-1: **Separate discovery.** `ovphysx_companion_acquire`, `ovphysx_companion_preview_acquire` `ovphysx_companion_mesh_acquire` and `ovphysx_companion_mesh_motion_acquire` and `ovphysx_companion_native_acquire` / `ovphysx_companion_cooking_acquire` / `ovphysx_companion_cct_acquire` / `ovphysx_companion_debug_acquire` / `ovphysx_companion_ovd_acquire` / `ovphysx_companion_vehicle_acquire` / `ovphysx_companion_configuration_acquire` / `ovphysx_companion_events_acquire` / `ovphysx_companion_mass_acquire` / `ovphysx_companion_inspection_acquire` / `ovphysx_companion_queries_acquire` negotiate exact versions/sizes of dedicated C tables, without initializing simulation or changing the primary simulation header. Rejected negotiation leaves output unchanged.
- AC-2: **Retryable lifetime.** Core destroy and shutdown reject live companion leases with `OVPHYSX_API_INVALID_STATE` before native teardown or consuming initialization. Python preserves its handle, stage keepalive and lifecycle token on that result, allowing session close followed by retry. The experimental C++ reset likewise
  retains its owner and throws on retryable rejection; move assignment swaps owners
  without fallible destruction. Its noexcept destructor requires a closed/idle owner
  and fails fast on retryable rejection rather than silently leaking an instance.
- AC-3: **Attachment qualification.** Borrowed sessions never attach/step; calls resolve paths through the captured owner, fail stale after detach/reattach, and release their lease on close. An in-flight operation keeps its lease until it returns, even if its handle is closed.

- AC-4: **Owned service lifetime.** Stopped cooking services, native modules and preview attachments retain shared runtime services independently of core instances. Shutdown rejects them before consuming initialization. Core recording transitions reject pending previews without collecting their work; preview retirement drains only its own attachment and leaves core recording and uncollected tasks intact. Destroying the last core instance preserves a live preview or module; module close retires the registration before releasing code/context/services. Process transition admission excludes executing preview operations without serializing independent preview steps. One SDK-owned companion coordinator owns all companion synchronization, reentry state and handle registries; distinct coordinator objects do not share admission or lease state. Pending work is read from the retained previews, so fetching or closing one preview cannot clear another preview's recording gate.

- AC-5: **Scoped native access.** The build-matched native-access table validates SDK/compiler compatibility before publishing callbacks. While a read-only native-access callback holds owner admission, core wait/collection calls return INVALID_STATE before waiting or collecting a receipt. No SDK pointer is exposed in Python.

## Test References

- [TEST-CAPI-COMPANION-001](../../tests/capi/TEST-CAPI-COMPANION-001.md)

## Code References

- [C11 header compilation](../../../tests/c_unittests/test_companion_header.c) checks the standalone companion header and fixed-width table prefix.

- [Dedicated header](../../../companion/include/ovphysx_companion/companion.h)
- [Companion bridge](../../../companion/src/Companion.cpp)
- [Core lifetime gates](../../../src/ovphysx/ovphysx.cpp)
- [Instance lease bookkeeping](../../../src/include/internal/sdk/ovphysxSDK.hpp)
- [Python destruction retry](../../../python/ovphysx/api.py)
- [C tests](../../../tests/c_unittests/test_companion.cpp)
- [Stopped cooking bridge](../../../companion/src/Cooking.cpp) and [cooking lifetime tests](../../../companion/tests_native/test_cooking.py).
- [Native module bridge](../../../companion/src/Native.cpp)
- [Compiled module lifetime tests](../../../companion/tests_native/test_plugins.py)
- [Owned preview bridge](../../../companion/src/Preview.cpp)
- [Stopped element mapping bridge](../../../companion/src/Mesh.cpp)
- [Process transition admission](../../../src/include/internal/sdk/CompanionRuntime.hpp)
- [Mixed lifetime tests](../../../companion/tests_native/test_preview.py)
- [Python native integration tests](../../../companion/tests_native/test_bridge.py)
- [Python failure injection](../../../tests/python_tests/test_lifecycle_failure_injection.py)

- [CCT discovery and operation bridge](../../../companion/src/Cct.cpp)
- [CCT discovery canaries](../../../companion/tests_native/test_cct.py)

- [Debug frame discovery/ownership](../../../companion/src/Debug.cpp)
- [Debug discovery and copy canaries](../../../companion/tests_native/test_debug.py)

- [Stopped OVD table](../../../companion/src/Ovd.cpp)
- [OVD ABI/output canaries](../../../companion/tests_native/test_ovd.py)

- [Vehicle table](../../../companion/src/Vehicle.cpp)
- [Vehicle ABI/output canaries](../../../companion/tests_native/test_vehicle.py)

- Configuration side table: [configuration.h](../../../companion/include/ovphysx_companion/configuration.h) and [Configuration.cpp](../../../companion/src/Configuration.cpp) retain exact-owner admission and separate version negotiation.

- Event queues: [events.h](../../../companion/include/ovphysx_companion/events.h) and [Events.cpp](../../../companion/src/Events.cpp) retain an exact owner runtime lease through cancellation and expose no callback to Python.
- General mass: [mass.h](../../../companion/include/ovphysx_companion/mass.h) and [Mass.cpp](../../../companion/src/Mass.cpp) validate versioned table/record sizes and preserve failed outputs without runtime initialization.

- [Typed inspection bridge](../../../companion/src/Inspection.cpp) and [query bridge](../../../companion/src/Queries.cpp) expose independently versioned tables; callers retain their selected existing owner. [Inspection](../../../companion/tests_native/test_inspection.py) and [query](../../../companion/tests_native/test_queries.py) tests verify sizes, status/output canaries and owner identity.

## Dependencies

- [REQ-CAPI-INSTANCE-SCOPE-001](REQ-CAPI-INSTANCE-SCOPE-001.md)
- [REQ-PYTHON-LIFECYCLE-001](../python/REQ-PYTHON-LIFECYCLE-001.md)
