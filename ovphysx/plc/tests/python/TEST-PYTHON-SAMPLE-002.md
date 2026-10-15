<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-SAMPLE-002
maps_to: REQ-PYTHON-SAMPLE-002
type: unit
---

## Scenario

The four tutorial samples that open a bundled scene resolve it from the
installed package, with a checked-in data-directory fallback (REQ AC-1, AC-2).

## Given

- The profiling, contact-binding, kinematic-support, and ovstage-integration
  pages, each literal-including its Python sample.
- `test_tutorial_python_samples_resolve_bundled_data` in
  `ovphysx/tests/python_tests/test_documentation_contracts.py`.

## When

- That test reads each page and parses the sample it includes.

## Then

- Each page still literal-includes its sample (REQ AC-1).
- Each sample names `ovphysx.__file__` and the path components `samples` and
  `data`, so the installed package is the first location tried (REQ AC-1).
- No sample builds that path with `".." / "data"`. The fallback is the
  checked-in directory beside the sample, and a missing file raises
  `FileNotFoundError` naming that path (REQ AC-2).
