<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PACKAGING-DOCS-001
title: Portable Artifact Documentation
status: implemented
owner: ovphysx
---

## Description

The ovphysx Python wheel and C/C++ SDK provide a portable copy of the public
documentation alongside any rendered HTML. Portable documentation lets offline
consumers and tools inspect the same guides, simulation-setup material, Python
API source, and referenced images that are public in the source repository.

Internal publishing documentation and source-only test trees remain excluded
from both artifacts. Packaging documentation does not change runtime behavior.

## Acceptance Criteria

- AC-1: The Python wheel and C/C++ SDK preserve the source-relative path of
  every public `docs/**/*.md`, `docs/**/*.rst`, `docs/**/*.png`, and
  `docs/**/*.jpg` file. Markdown is converted to portable Markdown by resolving
  supported MyST constructs; RST and images are copied verbatim.

- AC-2: Neither artifact contains a public-doc path with an `internal/` segment.
  Every inline relative link in its portable Markdown resolves inside the
  artifact, except C/C++ header links that resolve only in the SDK; portable
  documentation must not link to source-only `tests/` paths.

## Test References

- TEST-PACKAGING-DOCS-001

## Code References

- ovphysx/scripts/preprocess_docs.cmake
- ovphysx/scripts/build_wheel.cmake
- ovphysx/scripts/install.cmake
- ovphysx/scripts/test_python_wheel.cmake
- ovphysx/tests/python_tests/wheel_documentation_smoke.py
- ovphysx/docs/developer_guide.md

## Dependencies

- None
