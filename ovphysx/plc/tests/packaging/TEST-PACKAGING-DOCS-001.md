<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PACKAGING-DOCS-001
maps_to: REQ-PACKAGING-DOCS-001
type: integration
---

## Scenario

The installed-wheel smoke suite checks the portable documentation in the exact
wheel users import and in the installed C/C++ SDK tree. The test compares both
artifacts with the repository's public documentation tree and crawls every
bundled Markdown file. It is invoked by `scripts/test_python_wheel.cmake` for
every supported Python version.

## Given

- A freshly built and installed ovphysx wheel and C/C++ SDK.
- The public `ovphysx/docs` source tree used to build that wheel.

## When

- `wheel_documentation_smoke.py` enumerates public Markdown, RST, PNG, and JPG
  source paths and checks the exact portable inventory in each artifact.
- The same test compares RST and image bytes, searches each artifact for any
  internal path, and resolves every inline artifact-scoped relative Markdown
  link.

## Then

- Every public portable-documentation source has a matching artifact file, with
  RST and image content preserved exactly (REQ AC-1).
- Internal paths are absent, no portable Markdown link targets source-only
  tests or escapes its package root, and every other inline artifact-scoped
  relative link resolves. C/C++ header links tracked by NVBug 6560076 remain
  explicitly outside the wheel check (REQ AC-2).

## Code References

- ovphysx/tests/python_tests/wheel_documentation_smoke.py
- ovphysx/tests/python_tests/test_wheel_documentation_smoke.py
- ovphysx/scripts/test_python_wheel.cmake
