<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-SDFVIEW-001
maps_to: REQ-PYTHON-SDFVIEW-001
type: integration
---

## Scenario

Creating an SDF view on a destroyed `PhysX` instance fails with the
destroyed-instance error instead of an attribute error on the cleared handle
(REQ AC-1).

## Given

- A real `PhysX` instance that has been destroyed, in
  `test_destroy_is_checked_idempotent_and_refcounted`.

## When

- `create_sdf_view` is called on that instance.

## Then

- The call raises `RuntimeError` matching "has been destroyed" (REQ AC-1).
  The deprecation warning is still emitted, and the native constructor is not
  reached because the check runs first.
