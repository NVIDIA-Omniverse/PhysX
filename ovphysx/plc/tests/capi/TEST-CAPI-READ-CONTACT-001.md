<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-READ-CONTACT-001
maps_to: REQ-CAPI-READ-CONTACT-001
type: integration
---

## Scenario

Contact sensors are read through the public ovphysx read API on both residencies: the type is
accepted, the read returns exactly the contact-reporting bodies, and each resting sensor carries its
own weight as its normal force, with no friction, on its own row (AC-1, AC-2); each resting sensor's raw
contacts are against the ground and sum to its weight (AC-5), and its events are persisting pairs with the
ground on a CPU scene and absent on DirectGPU (AC-6). One past the last type is refused (AC-4).
CPU selected reads preserve sensor identity and an empty match. Both query entry points reject one past the last type (AC-4).

## Given

- A Z-up, meters, gravity 9.81 scene from `write_contact_sensor_usda` (test_utils.py): `/World/Light`
  (1 kg) and `/World/Heavy` (2 kg) rest on a static ground with `PhysxContactReportAPI`; `/World/Float`
  carries it but is kinematic and hovers; `/World/Quiet` rests on the ground without it. Sleep is
  disabled on the resting sensors, so they keep reporting once settled.
- The scene is attached to a CPU-mode instance (`physx_sdk_cpu`) and, separately, to a DirectGPU
  instance (`physx_sdk`), then stepped 60 times at 1/60 s.

## When

- `PhysX.read_sync(SimObjectType.CONTACT_SENSOR, ["netNormalContactForce", "netFrictionContactForce"],
  scope=ObjectScope.ALL)` is opened; each group's attribute token is resolved to its name, and its rows
  are mapped to prims through its `prim_list`, both with the stage's path dictionary.
- On the CPU instance, `PhysX.query(SimObjectType.CONTACT_SENSOR, pattern=path).wait()` selects
  `/World/Heavy`, `/World/Float` and `/World/Missing` in turn. Each completed query opens
  `PhysX.read(query, ["netFrictionContactForce", "netNormalContactForce"], metadata=True).wait()`;
  `fetch_next().wait()` drains it. Each selected read is followed by another whole-type read.
- On the CPU instance, `PhysX.read_sync` is also called with the type `max(SimObjectType) + 1`,
  as is `PhysX.query(max(SimObjectType) + 1, pattern="/World/Heavy").wait()`.
- On both instances, the contact sensors are read with `pattern="/World/Heavy"`, through
  `ovphysx_query_selection`.
- On both instances, `PhysX.read_sync(SimObjectType.CONTACT_SENSOR, ["rawContactForce", "rawContactNormal",
  "rawContactOtherBody"], scope=ObjectScope.ALL)` is opened, each other-body handle resolved with
  `PathDictionary.path_to_string`.
- On both instances, `PhysX.read_sync(SimObjectType.CONTACT_SENSOR, ["contactEventType", "contactEventOtherBody",
  "contactEventPointCount", "netNormalContactForce"], scope=ObjectScope.ALL)` is opened.

## Then

- The read opens: `ovphysx_query` accepts the contact-sensor type (AC-1).
- Exactly `/World/Light`, `/World/Heavy` and `/World/Float` come back, in both components;
  `/World/Quiet` and the ground do not (AC-1).
- Two `(3, 3)` columns, `netNormalContactForce` then `netFrictionContactForce`: CPU Warp arrays on the
  CPU instance and CUDA Warp arrays on the DirectGPU one (AC-2).
- `/World/Light` and `/World/Heavy` each read their own weight upward as their normal force (9.81 N and
  19.62 N within 5%, lateral within 5% of the weight), so a row assigned to the wrong prim fails, and
  every lane of their friction is below 5% of the weight; `/World/Float` reads zero in both (AC-2).
- Each selected group names exactly its selected prim and matches that prim's two components from
  the whole-type read. Heavy and Float have query/read membership counts of 1 and one group per
  request; Missing has counts of 0 and no groups. All requested summary rows have no omission
  flags, distinguishing an empty match from unsupported data. The whole-type read after each
  selection still returns all three sensors with their original values (AC-1, AC-2).
- The out-of-range type raises an "invalid object type" error for both query paths (AC-4).
- The pattern-selected read opens (`ovphysx_query_selection` admits the contact-sensor type) and returns
  `/World/Heavy` alone, at its weight (AC-1, AC-4).
- The raw read returns, in each group, one array per resting sensor (`/World/Light`, `/World/Heavy`; not the
  hovering `/World/Float`), CPU or CUDA as the instance runs, shape `(n,)` or `(n, 3)`: every other body is
  `/World/Ground`, and the sum of force × normal z is each sensor's weight within 5% (AC-5).
- The event read on the CPU instance returns, for `/World/Light` and `/World/Heavy` only, persisting events (2)
  with `/World/Ground` as the other body and at least one point each, then the net force; on the DirectGPU
  instance it returns the net force alone, without error (AC-6).

## Test locations

- ovphysx/tests/python_tests/test_output_read_contacts.py (`test_contact_sensor_net_force_read_device`,
  `test_contact_sensor_raw_contacts_device`, `test_contact_sensor_events_omitted_device`)
- ovphysx/tests/python_tests/cpu_tests/test_output_read_contacts_cpu.py
  (`test_cpu_contact_sensor_net_force_read`, `test_cpu_read_rejects_type_past_the_last`,
  `test_cpu_contact_sensor_raw_contacts`, `test_cpu_contact_sensor_events`)
- Contact-binding ports (ADR-0044) in `cpu_tests/test_output_read_contacts_cpu.py`:
  `test_cpu_contact_sensor_net_friction_after_sliding`, `test_cpu_contact_force_uses_the_step_dt`,
  `test_cpu_contact_sensors_discovered_on_runtime_clones`, `test_cpu_contact_sensor_before_first_step`,
  `test_cpu_contact_event_point_detail`, `test_cpu_contact_event_anchor_detail`.
- ovphysx/tests/python_tests/cpu_tests/test_contact_query_cpu.py (`test_cpu_pattern_selects_contact_sensors`)
  and ovphysx/tests/python_tests/test_contact_query.py (`test_contact_query_and_selection_device`)

## Coverage limits

- Nothing pushes a sensor sideways, so this public test checks the friction column for wiring
  (present, keyed, near zero), not for a non-zero value.
- The selected-query, empty-match and selected type-bound cases are CPU-only here; the DirectGPU
  case covers the whole-type read. Automated CPU coverage verifies selected membership, empty
  results, whole-type restoration and rejection of an out-of-range selected type.
- Reporting colliders forward through the same type (the C-API accepts `CONTACT_SENSOR` and does not
  filter rows by body-vs-collider, so a CPU scene returns a reporting collider as its own row and a
  DirectGPU scene only reporting bodies — AC-1), but this public test does not exercise them: their
  enumeration and per-collider decomposition are the runtime's, covered by
  [REQ-READ-CONTACT-001](../../../ovruntime/plc/requirements/output/REQ-READ-CONTACT-001.md) AC-12
  (host net/raw/events/force-matrix and the DirectGPU omission).
