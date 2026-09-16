<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-CLOSURE-001
title: Ship only GPU dependencies used by ovphysx
status: implemented
owner: ovphysx
---

## Description

The ovphysx SDK and wheel shall not ship Carbonite GPU plugins left over from
the retired Fabric data path. PhysX owns its CUDA context through the PhysX SDK;
GPU simulation and tensor operations do not require Cubric or Carbonite's CUDA
compute plugin.

## Acceptance Criteria

- AC-1: Dependency staging excludes the `omni.cubric` and `gpucompute`
  infrastructure directories and rejects stale `omni.cubric.plugin` or
  `omni.gpucompute-cuda.plugin` artifacts.
- AC-2: The runtime does not request those plugins, search a `plugins/gpu`
  directory, or publish runtime paths for that directory.
- AC-3: SDK and wheel verification rejects either plugin on every supported
  platform, and packaging locks contain neither plugin nor a `gpu/` entry.
- AC-4: CPU and CUDA simulation, tensor, sample, and wheel validation pass with
  no `plugins/gpu` directory in either artifact.

## Test References

- [TEST-PACKAGING-CLOSURE-001](../../tests/packaging/TEST-PACKAGING-CLOSURE-001.md)

## Code References

- `ovphysx/scripts/package_deps.py` (AC-1)
- `ovphysx/deps_manifest.toml` (AC-1)
- `ovphysx/src/CarboniteLoader/CarboniteLoader.cpp` (AC-2)
- `ovphysx/python/ovphysx/_bindings.py` (AC-2)
- `ovphysx/CMakeLists.txt` (AC-2)
- `ovphysx/scripts/verify_pyless_closure.py` (AC-3)
- `ovphysx/scripts/generate_packaging_lock.py` (AC-3)
- `ovphysx/tests/python_tests/test_package_deps.py` (AC-1, AC-2)
- `ovphysx/tests/python_tests/test_verify_pyless_closure.py` (AC-3)
- `ovphysx/scripts/validate_all.cmake` (AC-4)

## Dependencies

- `ovphysx/ovruntime/plc/requirements/fabric/REQ-FABRIC-RETIRE-001.md`
