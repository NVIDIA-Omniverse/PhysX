<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-WAIT-RANGE-001
title: Python Wait Integer Range
status: implemented
owner: ovphysx
---

## Description

Python operation waits validate integer arguments before native conversion.
This baseline covers the wait argument wrapping reported in NVBug 6856105.

## Acceptance Criteria

- AC-1: `PhysX.wait_op` rejects operation indices and explicit timeouts outside
  `[0, 2**64 - 1]` with `ValueError` before calling native code or consuming work.
  `PhysX.wait_all` applies the same timeout validation.
- AC-2: Wait arguments use the integer index protocol; non-integer values raise
  `TypeError` before calling native code.
- AC-3: In-range integers reach native code unchanged. `None` timeout maps to
  `2**64 - 1`; the positive wait-all and infinite-timeout sentinels remain valid.

## Test References

- [TEST-PYTHON-WAIT-RANGE-001](../../tests/python/TEST-PYTHON-WAIT-RANGE-001.md)

## Code References

- ovphysx/python/ovphysx/api.py: `PhysX.wait_op`, `PhysX.wait_all`
- ovphysx/tests/python_tests/cpu_tests/test_step_boundary_conditions.py

## Dependencies

- None
