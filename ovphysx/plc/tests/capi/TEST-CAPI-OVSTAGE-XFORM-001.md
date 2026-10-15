<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-XFORM-001
maps_to: REQ-CAPI-OVSTAGE-XFORM-001
type: integration
---

## Scenario

Separated cart-pole environments retain their world poses after USD
population. Automated coverage is
`SessionPhysicsGpuTest.GpuMultiCartPoleSpacedProjectedJointForceMatchesActuation`
and `TensorBindingGpuTest.GpuMultiCartPoleSpacedProjectedJointForceMatchesActuation`
in `tests/c_unittests/test_session_physics.cpp` and `test_tensor_binding.cpp`.

## Given

- A CUDA-capable GPU and a fresh ovphysx instance.
- `tests/data/MultiCartPoleSpaced.usda`, with environments under translated parents.

## When

- Populate with ALL at ordinal 1, wait for population, seal it, and attach via `attach_usd_with_ovstage`.
- Warm up, write prismatic actuation 10 for each environment and step 0.001 s.
- Read projected joint forces through each test's session or tensor binding.

## Then

- The helper uses ALL population before sealing and attach (REQ AC-1).
- Every cart-prismatic joint reports projected force within 0.9 of 10 (REQ AC-2).
