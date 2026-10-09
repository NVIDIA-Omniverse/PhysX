<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-OVSTAGEVERSION-001
maps_to: REQ-PACKAGING-OVSTAGEVERSION-001
type: integration
---

## Scenario

A native consumer resolves the OVStage release recorded by the SDK's
generated package configuration without consulting documentation.

## Given

- CMake on `PATH` and `tests/python_tests/test_cmake_package_config.py`.
- Temporary SDK targets and OVStage package configurations; no compiler,
  native library, GPU, or dependency download is needed.
- The real version reader and `ovphysxConfig.cmake.in` template.

## When

- Generate package configurations from two different pins and configure
  consumers with matching, older, newer-patch, and different-minor OVStage
  packages using `find_package(ovphysx REQUIRED CONFIG)`.
- Try a matching package without the schema-registration declaration.
- Generate metadata from missing, malformed, and duplicate assignments,
  or a missing pin source file.

## Then

- Matching releases configure successfully, expose the full pin and its
  three-component CMake version, and propagate `ovstage::ovstage` through
  the SDK target (REQ AC-1, AC-2).
- Mismatched releases fail configuration and identify the required
  version; a missing schema-registration API still fails (REQ AC-2).
- Invalid or missing pin inputs fail metadata generation (REQ AC-3).

The tests exercise regeneration explicitly. The main build's
`CMAKE_CONFIGURE_DEPENDS` registration is checked during code review;
automatic rebuild triggering is not exercised by this fixture.
