<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-READ-CONTACT-001
title: Contact-Sensor Net Contact Forces Through the ovphysx Read API
status: implemented
owner: ovphysx
---

## Description

The ovphysx physics-output read serves contact sensors: the bodies whose contacts PhysX reports, those
carrying `PhysxContactReportAPI` and the links of an articulation carrying it — and, on CPU simulation
and GPU dynamics with CPU readback, the colliders carrying it (host-only; a DirectGPU scene has no
separate child-collider rows).
`OVPHYSX_QUERY_TARGET_CONTACT_SENSOR` (C) and `SimObjectType.CONTACT_SENSOR` (Python) select them. Each
sensor's net contact force is read as two components: `OVPHYSX_ATTR_NET_NORMAL_CONTACT_FORCE`
(`"netNormalContactForce"`) and `OVPHYSX_ATTR_NET_FRICTION_CONTACT_FORCE` (`"netFrictionContactForce"`),
the same aggregates the contact binding's net normal and net friction reads return. Their sum is the
total contact force. `ovphysx_query` and `ovphysx_query_selection` forward the type to the runtime
read, which owns discovery, the reductions and their residency; this requirement owns the public
surface that must accept the type and forward it. Python uses completed `PhysX.query` and
`PhysX.read` operations, or the waiting `PhysX.read_sync` convenience.

The public type gates in `ovphysx_query`, `ovphysx_query_selection` and `ovphysx_writability`
bound the type by the `OVPHYSX_QUERY_TARGET_COUNT` sentinel rather than by the last type's name,
so a type appended to `ovphysx_query_target_t` is accepted without editing these gates.

## Acceptance Criteria

- AC-1: **The type is accepted and selects the sensors.** `ovphysx_query` and
  `ovphysx_query_selection` (Python `PhysX.query`, also used by `PhysX.read_sync`) accept
  `OVPHYSX_QUERY_TARGET_CONTACT_SENSOR` / `SimObjectType.CONTACT_SENSOR` (11). A read returns one row per
  selected contact-reporting body, including links of an articulation carrying `PhysxContactReportAPI`, including one that touches nothing, and no row
  for a body without it. On CPU simulation and GPU dynamics with CPU readback, a selected collider
  carrying the schema is additionally its own row, decomposing its body's contacts (host-only; a
  DirectGPU scene has no separate child-collider rows and a selection naming only those colliders
  opens an empty session). A body row and its collider rows overlap, so a consumer must not sum net
  forces across them. A selection
  matching no sensor opens a valid empty session; optional query/read summaries describe that selected
  membership.
- AC-2: **Each component is one row per sensor, keyed to its own prim.** Each row is a world-space
  `vec3` float32: a body resting on flat ground reads its own weight upward as its normal force and no
  friction, and an untouched sensor reads zero in both. Requested together, the two come back as one
  column each, in request order. A column is a CPU Warp array on a non-DirectGPU scene and a CUDA Warp
  array on a DirectGPU scene.
- AC-3: **Both are read-only.** `ovphysx_writability` returns `OVPHYSX_WRITABILITY_READ_ONLY` for
  `OVPHYSX_QUERY_TARGET_CONTACT_SENSOR` with `"netNormalContactForce"` and with `"netFrictionContactForce"`.
- AC-4: **All three type gates are bounded by `OVPHYSX_QUERY_TARGET_COUNT`.** `OVPHYSX_QUERY_TARGET_COUNT` is one
  past the last `ovphysx_query_target_t` value; `ovphysx_query`, `ovphysx_query_selection` and
  `ovphysx_writability` reject a type at or past it with `OVPHYSX_API_INVALID_ARGUMENT` and accept the
  last type.
- AC-5: **The raw contacts are public, read-only names on every contact-sensor query.**
  `OVPHYSX_ATTR_RAW_CONTACT_FORCE`, `_POINT`, `_NORMAL`, `_SEPARATION` and `_OTHER_BODY` name each sensor's
  contact points against any body: array groups, one tensor per sensor with at least one point this step (a
  list of Warp arrays in Python, shape `(n,)` or `(n, 3)`, CPU or CUDA as the scene runs). The other body is a
  uint64 prim-path handle the stage's path dictionary resolves (`PathDictionary.path_to_string` in Python).
  `ovphysx_writability` classifies all five `OVPHYSX_WRITABILITY_READ_ONLY` for
  `OVPHYSX_QUERY_TARGET_CONTACT_SENSOR`.
- AC-6: **The contact events are public, read-only names on every contact-sensor query.** The nineteen
  `OVPHYSX_ATTR_CONTACT_EVENT_*` names carry each sensor's contact-report pairs as seen from the sensor: array
  groups at one of three levels — per event (the point and anchor counts included), per contact point or per
  friction anchor — one tensor per sensor with at least one element at that level this step, the points and
  anchors split by the per-event counts (a sensor whose only event is a lost pair, which carries no point, has
  event-level tensors but no point or anchor one). Identities (colliders, bodies, materials) are
  uint64 prim-path handles the stage's path dictionary resolves; the event type keeps
  the found/lost/persist values (0, 1, 2). On CPU simulation and GPU dynamics with CPU readback they are
  CPU Warp arrays; a DirectGPU scene returns no event group and the read still succeeds.
  When metadata is requested, event rows report `OVPHYSX_READ_OMISSION_UNSUPPORTED_CONFIGURATION`.
  `ovphysx_writability` classifies all nineteen `OVPHYSX_WRITABILITY_READ_ONLY` for
  `OVPHYSX_QUERY_TARGET_CONTACT_SENSOR`.


## Test References

- [TEST-CAPI-READ-CONTACT-001](../../tests/capi/TEST-CAPI-READ-CONTACT-001.md) (the read: AC-1, AC-2, AC-5, AC-6, and AC-4's query gate)
- [TEST-CAPI-READ-CONTACT-002](../../tests/capi/TEST-CAPI-READ-CONTACT-002.md) (writability: AC-3, AC-5, AC-6, and AC-4's writability gate)

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h (`OVPHYSX_QUERY_TARGET_CONTACT_SENSOR`, `OVPHYSX_QUERY_TARGET_COUNT`)
- ovphysx/include/ovphysx/ovphysx_attributes.h (`OVPHYSX_ATTR_NET_NORMAL_CONTACT_FORCE`,
  `OVPHYSX_ATTR_NET_FRICTION_CONTACT_FORCE`; the `OVPHYSX_ATTR_RAW_CONTACT_*` names: AC-5; the
  `OVPHYSX_ATTR_CONTACT_EVENT_*` names: AC-6)
- ovphysx/python/ovphysx/types.py (`SimObjectType.CONTACT_SENSOR`)
- ovphysx/python/ovphysx/api.py (`PhysX.query`, `PhysX.read`, `PhysX.read_sync`: AC-1, AC-2)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (`ovphysx_query`'s and `ovphysx_query_selection`'s type
  gates: AC-1, AC-4;
  `ovphysx_writability`'s type gate and its contact-sensor `kWritabilityTable` rows: AC-3, AC-4, AC-5, AC-6)
- ovphysx/tests/python_tests/test_output_read_contacts.py and
  ovphysx/tests/python_tests/cpu_tests/test_output_read_contacts_cpu.py (the read, the raw contacts: AC-5, the
  events: AC-6),
  with the scene and read helpers in ovphysx/tests/python_tests/test_utils.py; ovphysx/tests/python_tests/test_contact_query.py
  and ovphysx/tests/python_tests/cpu_tests/test_contact_query_cpu.py (the pattern-selected read)
- ovphysx/tests/c_unittests/test_writability.cpp and ovphysx/tests/python_tests/test_writability.py
  (writability)

## Dependencies

- REQ-PYTHON-READ-001 (the Warp frontend)
- REQ-CAPI-SESSION-001 (completion, summaries and session ownership)
- REQ-CAPI-SELECT-001 (prim-set selection)

