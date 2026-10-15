<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-NATIVE-ABI-001
maps_to: REQ-COMP-NATIVE-ABI-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-NATIVE-ABI-001; executable evidence and qualification limits follow.

## Scenario

Load a native plugin, reject mismatches and attempt unload while callbacks and objects remain live.

## Given

- A small compiled plugin with allocation/callback counters, a compatible build descriptor and deliberately altered variants.
- One live object and one controllably in-flight callback.

## When

- Try invalid tables/builds; register the valid plugin, attempt unregister while in use, release objects, drain callbacks and unregister again.

## Then

- Mismatches invoke zero plugin callbacks and publish no registration (REQ AC-1).
- Missing required callbacks and invalid data alignment are rejected with no leaked registry state (REQ AC-2).
- Early unregister reports Busy and retains code/type state; final unregister succeeds only after all use ends (REQ AC-3).
- All hot callbacks are native and utility resolution neither imports USD/Python nor creates a second PhysX owner (REQ AC-4).

## Executable coverage

[test_plugins.py](../../../tests_native/test_plugins.py):

29 compiled-plugin cases cover descriptor mismatch before startup, required callbacks, in-flight callback retirement, post-create rollback, dual owners and utility imports. Native callbacks use no Python/USD. The six affected runtime shards also pass.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.

## Build configuration

The facade and native sample compile with _DEBUG in Debug configuration. Installed
native_build.h declares the build mode; compiled-consumer tests choose that exact
configuration rather than building Release consumers against a Debug owner.
Linux Release is tested locally; matched Debug loading remains a CI gate.

The installed native `direct_contract.cpp` consumer includes events.hpp in
addition to native SDK headers, verifying installation of the C++ event facade.
