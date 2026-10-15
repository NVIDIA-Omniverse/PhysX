<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-READ-CONTACT-002
maps_to: REQ-CAPI-READ-CONTACT-001
type: unit
---

## Scenario

`ovphysx_writability` classifies both of the contact sensor's net contact-force components, its five raw
contact names and its nineteen event names as read-only, and its type gate follows `OVPHYSX_QUERY_TARGET_COUNT`
(AC-3, AC-4, AC-5, AC-6). A bare process: no instance, scene or step.

## Given

- The ovphysx library loaded in a bare process (C unit test; Python test without a fixture).

## When

- `ovphysx_writability(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, ...)` is queried for `"netNormalContactForce"`,
  `"netFrictionContactForce"`, the five `rawContact*` names and the nineteen `contactEvent*` names, from C and
  from Python.
- `ovphysx_writability` is queried with `OVPHYSX_QUERY_TARGET_COUNT` (C) and with `max(SimObjectType) + 1`
  (Python).
- Every `OVPHYSX_ATTR_*` the header declares is classified against every type below
  `OVPHYSX_QUERY_TARGET_COUNT`.

## Then

- `netNormalContactForce` and `netFrictionContactForce` on the contact sensor are
  `OVPHYSX_WRITABILITY_READ_ONLY` (AC-3), and so are the five raw contact names (AC-5) and the nineteen event
  names (AC-6).
- The out-of-range type returns `OVPHYSX_API_INVALID_ARGUMENT` from C and raises an "unknown object
  type" `ValueError` from Python (AC-4).
- Every declared attribute, both contact-force components included, classifies as something other than
  `UNCLASSIFIED` for at least one type, which holds for those two only because the loop reaches the
  contact sensor (AC-3, AC-4).

## Test locations

- ovphysx/tests/c_unittests/test_writability.cpp (`Writability.ContactSensor`,
  `Writability.RejectsUnknownObjectType`, `Writability.EveryDeclaredAttributeIsClassifiedSomewhere`)
- ovphysx/tests/python_tests/test_writability.py (the `SimObjectType.CONTACT_SENSOR` cases of
  `test_writability_matches_the_write_api`, `test_unknown_object_type_is_rejected`)
