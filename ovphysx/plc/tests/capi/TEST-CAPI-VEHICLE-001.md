<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-VEHICLE-001
maps_to: REQ-CAPI-VEHICLE-001
type: integration
---

## Scenario

The public C and Python session APIs expose selected vehicle controls and telemetry with the
correct identity, types, applicability and reset behavior.

## Given

- A CPU instance attached to `ovphysx/tests/data/vehicle_read_write.usda`, containing Basic,
  Standard, tank and raw-wheel vehicles with four wheel attachments each.
- The attached Stage's path dictionary and the public C/Python selectors.

## When

- Write a filtered accelerator column before any read or step, then step and read it.
- Query all vehicles, individual drive types and an empty selection; read mixed transmission data.
- Read every public vehicle/wheel column through Python and representative native DLTensor types
  through C; resolve valid ground shape/material identities.
- Write valid controls and requested gear boundary values, reject invalid values and incompatible
  drive types, then require TaskError for accepted-task failures and compare
  controls and moving-chassis pose/velocities after rejection. Reject a two-vehicle gear
  batch whose first row requests reverse and last row is invalid; step and require both
  vehicles to retain their settled manual current/target gear.
- Request a manual Standard gear and observe the completed shift; drive raw wheels and a tank
  for fixed simulation steps and read their wheel rotation and chassis motion.
- Reset a moving chassis, retry the spent group, discard an uncommitted reset, and write chassis
  velocity after the successful reset. Attempt reset values zero and one on a disabled chassis.

## Then

- Filtered writes affect only the selected car, chassis/wheel selections remain independent,
  discovery reflects drive applicability, and mixed read omissions are explicit (REQ AC-1).
- Columns have the documented fixed shapes, CPU residency, scalar/vector dtypes and Stage-owned
  ground identities; invalid ground contact fields are zero (REQ AC-2).
- Public writability matches access modes, write-only names do not read, and read-only or
  inapplicable writes fail (REQ AC-3).
- Controls persist, invalid controls fail without mutation, and reset preserves pose/controls,
  clears velocity once, rejects duplicate commit and is not applied by discard. Manual gear
  requests change engaged/internal gear, raw torque rotates wheels, and tank thrust produces
  chassis motion. Reset rejects disabled chassis even for zero-valued rows (REQ AC-4).

## Test locations

- ovphysx/tests/c_unittests/test_vehicle_read_write.cpp (`VehicleReadWriteTest`)
- ovphysx/tests/python_tests/cpu_tests/test_vehicle_read_write.py
- ovphysx/tests/c_unittests/test_writability.cpp (`Writability.Vehicle`)
- ovphysx/tests/python_tests/test_writability.py (vehicle cases)
