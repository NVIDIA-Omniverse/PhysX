<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-SAMPLE-002
title: Copied Tutorial Samples Open Bundled Scenes
status: implemented
owner: ovphysx
---

## Description

The Python samples included from the profiling, contact-binding, kinematic-support,
and ovstage-integration pages open a bundled USD scene. A copy of that included
block must find the scene in the installed package, then in the checked-in sample
data directory, rather than only beside the reader's copy of the script.

This requirement covers those four samples. Other samples that still build a
scene path from the script location are outside it.

## Acceptance Criteria

- AC-1: Each sample opens its scene from `samples/data` next to the installed
  `ovphysx` package when that file is present. Profiling uses
  `links_chain_sample.usda`, contact binding and ovstage output-read use
  `boxes_falling_on_groundplane.usda`, and kinematic support uses
  `kinematic_support.usda`.
- AC-2: When that package file is absent, the sample opens the same scene from
  the checked-in `data` directory beside the sample source. When neither
  location has the file, the sample raises `FileNotFoundError` naming the
  missing path.

## Test References

- TEST-PYTHON-SAMPLE-002

## Code References

- ovphysx/tests/python_samples/profiling.py
- ovphysx/tests/python_samples/contact_binding.py
- ovphysx/tests/python_samples/kinematic_support.py
- ovphysx/tests/python_samples/output_read.py
- ovphysx/tests/python_tests/test_documentation_contracts.py
- ovphysx/docs/tutorials/profiling.md
- ovphysx/docs/tutorials/contact_binding.md
- ovphysx/docs/simulation_setup/kinematic_support.md
- ovphysx/docs/ovstage_integration.md

## Dependencies

- None
