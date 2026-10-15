<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-OVSTAGE-002
maps_to: REQ-PACKAGING-OVSTAGE-001
type: integration
---

## Scenario

The generated Windows launcher scopes its runtime search path and preserves the
child process's exit status (REQ AC-5).

## Given

- CMake and `tests/python_tests/test_cmake_package_config.py`.
- Temporary SDK and OVStage package configurations generated from the real
  `cmake/ovphysxConfig.cmake.in`, with an imported executable in a path with spaces.
- On Windows, a copy of the system command interpreter as the test executable.
  No compiler, native SDK, GPU, or download is required.

## When

- Configure the consumer and call `ovphysx_set_ovstage_runtime_env()`.
- Inspect the generated launcher on every host.
- On Windows, call the launcher twice from one parent batch script, with child
  exit codes 0 and 7 in separate test cases.

## Then

- The launcher sets all four resolved OVStage runtime directories before
  starting the quoted executable and forwards arguments (REQ AC-5).
- It scopes `PATH` before changing it and exits with the child's status
  (REQ AC-5).
- On Windows, each call returns the requested exit code and leaves the parent's
  `PATH` unchanged. Other hosts skip execution of the batch script (REQ AC-5).
