<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-WAIT-RANGE-001
maps_to: REQ-PYTHON-WAIT-RANGE-001
type: integration
---

## Scenario

Wait arguments cannot wrap into another operation, wait-all, or timeout value.
Automated in `tests/python_tests/cpu_tests/test_step_boundary_conditions.py`.

## Given

- A valid CPU-mode PhysX instance and an unconsumed simulation operation.

## When

- `wait_op` receives negative, oversized, non-integer, and boundary arguments.
- `wait_all` receives negative and oversized timeouts.

## Then

- Out-of-range integers raise `ValueError` without calling native wait;
  rejected `wait_op` and `wait_all` calls leave the original operation
  consumable (REQ AC-1).
- Floating-point and string arguments raise `TypeError` without calling native
  wait (REQ AC-2).
- Zero and maximum uint64 arguments pass unchanged; default timeout maps to
  the maximum (REQ AC-3).
