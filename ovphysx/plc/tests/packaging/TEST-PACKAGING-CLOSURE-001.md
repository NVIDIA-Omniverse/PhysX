<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-CLOSURE-001
maps_to: REQ-PACKAGING-CLOSURE-001
type: integration
---

## Coverage Status

Automated on Linux by the full validation pipeline and focused packaging tests.
The Windows CI lane covers the corresponding DLL and runtime-path changes.

## Scenario

The ovphysx SDK and wheel omit the unused Fabric-era GPU plugins while CPU and
CUDA runtime behavior remains intact.

## Given

- A clean ovphysx checkout with supported build dependencies and a CUDA-capable
  runner for the GPU test lane.
- The normal install and wheel packaging flow.

## When

- `cmake -DSKIP_GLIBC_CHECK=ON -P scripts/validate_all.cmake` runs.
- The generated SDK and wheel trees and packaging locks are inspected.
- The focused `test_package_deps.py` and `test_verify_pyless_closure.py` tests
  exercise dependency selection and stale-artifact rejection.

## Then

- `_install/plugins/gpu` and wheel `ovphysx/plugins/gpu` do not exist.
- Neither artifact nor packaging lock contains `omni.cubric.plugin` or
  `omni.gpucompute-cuda.plugin`.
- CPU and CUDA C++ tests, Python runtime tests, C/C++ and Python samples, and
  wheel smoke tests pass without those files.
