<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-OVSTAGE-003
maps_to: REQ-PACKAGING-OVSTAGE-001
type: integration
---

## Scenario

The installed `basic-workflow` skill's C program builds and launches with the
documented CMake commands and Windows launcher paths (REQ AC-5).

## Given

- An installed SDK at `_install/`, its separate OVStage package, and the external
  PhysX schema provider prepared by `scripts/test_cpp_samples.cmake`.
- A Windows SDK build using Ninja or Visual Studio, with its CMake cache and
  toolchain available for the consumer build.
- `scripts/test_basic_workflow.py` and the installed
  `skills/basic-workflow/SKILL.md`.

## When

- Run `cmake -DBASIC_WORKFLOW_SMOKE=ON -DSAMPLES_BUILD_ONLY=ON -P scripts/test_cpp_samples.cmake`
  from the source root of ovphysx, for each generator.
- The driver extracts the installed skill's C and CMake snippets, copies the
  installed sample headers and scene into a separate consumer directory, and
  builds with the documented commands and the SDK build's generator/toolchain.
- Run the same command with `-DSAMPLES_RUN_ONLY=ON` instead of
  `-DSAMPLES_BUILD_ONLY=ON`, retaining the build tree and installed dependencies.
  The driver launches the documented path from the consumer root. Its initial
  Windows `PATH` contains only system directories and the CUDA toolkit when set;
  `PHYSX_USD_SCHEMAS_ROOT` identifies the separate provider.

## Then

- The consumer configures and builds with both generators; each documented
  launcher exists and returns success (REQ AC-5).
- The launcher supplies OVStage runtime paths without inherited SDK or OVStage
  directories on `PATH`; the copied headers and relative scene path resolve
  from the documented layout (REQ AC-5).

## Coverage Limits

- `tests/python_tests/test_basic_workflow_smoke.py` checks recipe extraction,
  toolchain arguments, runtime environment isolation, and split build/run behavior.
  Its offline Linux consumer uses a synthetic C program. It does not establish
  that the native OVPhysX example runs on Windows; that requires the installed-SDK
  integration above.
