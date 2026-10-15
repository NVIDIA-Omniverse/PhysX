<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-SDFVIEW-001
title: Destroyed-instance SDF view creation
status: implemented
owner: ovphysx
---

## Description

Bootstrapped from a missing destroyed-instance check. `PhysX.create_sdf_view`
rejects a destroyed instance with the same `RuntimeError` as
`create_tensor_binding` and `create_contact_binding`, before it reads the
cleared native handle.

## Acceptance Criteria

- **AC-1: Destroyed instance.** After `PhysX.destroy()`, `create_sdf_view`
  raises `RuntimeError` stating that the instance has been destroyed. The
  failure happens before the native SDF-view constructor.

## Test References

- TEST-PYTHON-SDFVIEW-001

## Code References

- ovphysx/python/ovphysx/api.py (`PhysX.create_sdf_view`)
- ovphysx/tests/python_tests/lifecycle_tests/test_destroy.py

## Dependencies

- [REQ-PYTHON-LIFECYCLE-001](REQ-PYTHON-LIFECYCLE-001.md)
