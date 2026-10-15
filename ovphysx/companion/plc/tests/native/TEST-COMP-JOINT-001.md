<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-JOINT-001
maps_to: REQ-COMP-JOINT-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-JOINT-001; executable evidence and qualification limits follow.

## Scenario

Simulate a real custom constraint, update its parameters/flags and tear it down.

## Given

- A build-matched plugin with all six lifecycle callbacks, a deterministic solver-prep function and guard bytes around aligned constant data.
- A real populated external joint schema with known actor frames and an injected failure after per-joint state creation.

## When

- Create and step, change parameters and mark dirty, toggle ALWAYS_UPDATE on and off, apply origin/COM changes, try unregister while live, then delete/detach. Exercise construction rollback and unsupported modes before final unregister.

## Then

- Callbacks observe expected frames/shifts, row limits and aligned data with guard bytes intact (REQ AC-1).
- The new parameter reaches prepare-data and both flag transitions affect the actual live constraint, catching a pass-by-value flag setter (REQ AC-2).
- Created/released state counts match exactly and unregister is Busy until solver work/state are gone (REQ AC-3).
- Supported modes execute the sample and unsupported DirectGPU/articulation requests fail before callbacks (REQ AC-4).

- The optional direct C++ interface uses the original runtime callback types and retains the loaded runtime; ordinary headers need no SDK includes (REQ AC-5). The executable cases are in [test_companion_custom.cpp](../../../../tests/c_unittests/test_companion_custom.cpp).

## Executable coverage

[test_plugins.py](../../../tests_native/test_plugins.py):

Compiled callback-contract checks cover row limits/alignment, COM/origin frames and mass scaling. Live controls, parameter refresh, articulation rejection before create, callback Busy retirement and delete/rollback/detach release are executed.
Registration after an already-live attachment also refreshes external type
families and discovers both existing prims and subsequent authored clones.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
