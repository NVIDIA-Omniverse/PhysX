<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-SAMPLE-001
title: Observable Tensor-Bindings Sample Readback
status: implemented
owner: ovphysx
---

## Description

The shipped Python tensor-bindings sample makes successful control, simulation,
and pose readback observable when it runs the fixed-base articulation-chain
fixture. Its reported pose identifies a moving link, and the sample reports
success only after that same displayed link has visibly moved.

## Acceptance Criteria

- AC-1: The sample selects and labels a non-root link whose pose changes when
  the authored articulation velocity targets are applied.
- AC-2: Every displayed pose component is finite at each periodic readback and,
  before reporting success, the sample verifies that the displayed link's
  maximum absolute position delta exceeds `1.0e-3`.

## Test References

- TEST-PYTHON-SAMPLE-001

## Code References

- ovphysx/tests/python_samples/tensor_bindings.py
- ovphysx/tests/data/links_chain_sample.usda
- ovphysx/scripts/test_python_samples.cmake

## Dependencies

- None
