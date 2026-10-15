<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-GEOMETRY-001
maps_to: REQ-COMP-GEOMETRY-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-GEOMETRY-001; executable evidence and qualification limits follow.

## Scenario

Exercise a custom shape against a rigid ground, query it and destroy it under a pending-unregister attempt.

## Given

- A custom sphere-like geometry plugin with analytical mass/bounds, contact generation and optional raycast/debug callbacks.
- A real external geometry API, callback counters, variants missing required/optional callbacks and a failure injected after per-shape state creation.

## When

- Try missing-callback registrations; create/step/query the valid variant, attempt unregister, destroy/detach, exercise construction failure, then unregister; probe unsupported mode combinations.

## Then

- Missing required callbacks are rejected and absent optional callbacks return their declared unsupported behavior (REQ AC-1).
- Bounds/mass match the analytical fixture, contact prevents penetration within declared tolerance, and advertised queries/debug output work (REQ AC-2).
- Release is called once per successful create including rollback, early unregister is Busy, and no callback/type pointer survives final unload (REQ AC-3).
- Unsupported combinations fail explicitly and SDF/mesh-merge capability is not inferred from generic registration (REQ AC-4).

- The optional direct C++ interface uses the original runtime callback types and retains the loaded runtime; ordinary headers need no SDK includes (REQ AC-5). The executable cases are in [test_companion_custom.cpp](../../../../tests/c_unittests/test_companion_custom.cpp).

## Executable coverage

[test_plugins.py](../../../tests_native/test_plugins.py):

Analytical mass/bounds, dynamic contacts and all optional query/debug callbacks execute. Missing callbacks, bad bounds rollback, immediate SDK-shape release, mixed instances, cloned/recreated parameters and absent-mode capability probes pass.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
