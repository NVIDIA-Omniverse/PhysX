<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-KEYWORD-001
title: Keyword-Only Python Configuration Arguments
status: implemented
owner: ovphysx
---

## Description

Selected public Python entry points distinguish their primary operands from
configuration arguments with PEP 3102 keyword-only syntax. This makes call
sites self-describing. Positional configuration calls intentionally raise
`TypeError`; calls using the keyword form preserve native execution and
simulation results. The main `PhysX` constructor has no primary operand, so all
of its configuration arguments are keyword-only.

This is a measured conversion, not a uniform rewrite of the existing Python
surface. TensorBindings and `clone` retain their compatibility contracts, and
the removal-bound legacy scene queries remain unchanged. `PhysXConfig` remains
a data container outside this selected callable contract. Single-argument helpers
already satisfy the SDD's Rule 13 definition without `*`.

## Acceptance Criteria

- AC-1: The runtime signatures make `PhysX` constructor settings, operation
  wait timeouts, the initial ovstage read ordinal, output-read scope, contact
  report options, and Python logging severity/filter options keyword-only.
  Operation indices, stages, output object types/attributes, and the Python
  logger name remain positional-or-keyword primary operands.

- AC-2: `api.pyi` exposes the same positional and keyword-only parameter split
  as the runtime implementation for every entry point covered by AC-1.

## Test References

- TEST-PYTHON-KEYWORD-001

## Code References

- ovphysx/python/ovphysx/api.py
- ovphysx/python/ovphysx/api.pyi
- ovphysx/tests/python_tests/test_python_keyword_only.py

## Dependencies

- None
