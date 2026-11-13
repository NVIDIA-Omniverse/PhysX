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

Docs-only CI also invokes `scripts/test_documentation_package.py` with the
Sphinx artifact. It stages fresh SDK and wheel documentation layouts using the
same helper as native packaging, then runs the existing artifact smoke check
without a native build or importing the ovphysx package.

## Given

- A freshly built and installed ovphysx wheel and C/C++ SDK.
- The public `ovphysx/docs` source tree used to build that wheel.

## When

- `wheel_documentation_smoke.py` enumerates public Markdown, RST, PNG, and JPG
  source paths and checks the exact portable inventory in each artifact.
- The same test compares RST and image bytes, searches each artifact for any
  internal path, and resolves every inline artifact-scoped relative Markdown
  link.
- Decode every PNG/JPEG in portable and rendered documentation, checking its
  container integrity and that the detected format agrees with its extension.
  Install the test-only decoder from `scripts/documentation_requirements.txt`;
  wheel smoke tests first verify runtime import with only the wheel's own dependencies.
- Run with `--require-rendered-docs`, which CI sets through
  `OVPHYSX_REQUIRE_HTML_DOCS`, it additionally checks the rendered `docs/html`
  tree: its `index.html` and the downloadable C headers.
- The source-only gate requires `--html-docs` for complete validation. An
  explicit `--portable-only` local run checks only portable content and links.
- SDK header link targets use the same source directories and header patterns
  as the native CMake installation; samples use the actual artifact-specific
  staging rules. No placeholder link targets are manufactured.

## Then

- Every public portable-documentation source has a matching artifact file, with
  RST and image content preserved exactly (REQ AC-1).
- Git LFS pointers, truncated images, and images with the wrong format are
  rejected even when source and portable bytes match. A stale rendered pointer
  is rejected when the portable copy is valid. Unit fixtures cover PNG and JPEG
  in both layouts; the real CMake staging check rejects both pointer cases and
  accepts decodable images in SDK and wheel layouts (REQ AC-1, AC-3).
- Internal paths are absent, no portable Markdown link targets source-only
  tests or escapes its package root, and every other inline artifact-scoped
  relative link resolves. C/C++ header links tracked by NVBug 6560076 remain
  explicitly outside the wheel check (REQ AC-2).
- Link-like text inside backtick or tilde code fences is ignored; real links
  after the matching closing fence are checked. Unit tests include C++ lambda
  syntax and mismatched or shorter closing fences (REQ AC-2).
- Under `--require-rendered-docs`, an artifact missing the rendered tree, its
  index page, or any downloadable C header is rejected; without the flag such an
  artifact still passes, so a local build that never rendered the documentation
  stays testable (REQ AC-3). Unit tests in `test_wheel_documentation_smoke.py` cover
  both outcomes against the artifact shape the aarch64 wheel had in NVBug
  6717253: portable Markdown present, `docs/html` absent.
- The source-only staging gate rejects unsupported MyST constructs, broken
  catalog/skill links, links to unshipped files, and missing rendered header
  downloads. Its fixture checks preserve the distinct SDK C-sample and wheel
  Python-sample layouts and exclude generated sample caches.

## Code References

- ovphysx/tests/python_tests/wheel_documentation_smoke.py
- ovphysx/tests/python_tests/test_wheel_documentation_smoke.py
- ovphysx/scripts/test_python_wheel.cmake
- ovphysx/scripts/install.cmake
- ovphysx/scripts/build_wheel.cmake
- ovphysx/scripts/stage_documentation.cmake
- ovphysx/scripts/test_documentation_package.py
- ovphysx/cmake/public_headers.cmake
- ovphysx/tests/python_tests/test_documentation_package.py
