<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-ATTRIBUTE-NAMES-001
maps_to: REQ-CAPI-ATTRIBUTE-NAMES-001
type: unit
---

## Scenario

Intern one portable name through independent dictionaries, including a failing call.

## Given

- `AttributeNames` tests with two recording dictionary vtables and the public C11/C++ include-only targets, plus `tests/installed_headers` configured with only installed ovphysx and ovstage include directories.

## When

- Intern a generated position name repeatedly in both dictionaries, force a backend failure after it touches output, then try invalid arguments.

## Then

- The portable position and debug names carry invalid tokens and position retains its canonical string (REQ AC-1).
- A session read of `jacobianShape` beside `jacobian` on `tests/data/mixed_base_articulations.usda` reports the removed name as an unsupported attribute with an invalid token and no group, while `jacobian` is served as `[N, rows, cols]` with one lane (REQ AC-1): `SessionPhysicsCpuTest.SessionReadReportsRemovedJacobianShapeAsUnsupported` in `ovphysx/tests/c_unittests/test_session_physics.cpp`.
- Both dictionaries are called independently and return their own tokens; repeated calls do not reuse a global cached token (REQ AC-2).
- Failure clears output, preserves the backend error for caller release and rejects invalid inputs; both language header checks compile, including generated declarations and the token helper from the installed SDK (REQ AC-3).
