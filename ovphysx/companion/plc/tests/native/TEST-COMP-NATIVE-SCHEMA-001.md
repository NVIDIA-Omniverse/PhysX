<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-NATIVE-SCHEMA-001
maps_to: REQ-COMP-NATIVE-SCHEMA-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-NATIVE-SCHEMA-001; executable evidence and qualification limits follow.

## Scenario

Populate real external joint and geometry schemas and update parameters on one instance.

## Given

- A codeless test schema defining a new joint type and a new geometry API with numeric/custom attributes.
- Separate customer-USD and population schema registration contexts and two instances.
- Description-based population calls use the pinned OVStage API's `time` argument with the
  existing numeric or default-time values.

## When

- Omit each registration in turn, then register both, populate, create plugin objects and edit one instance parameter.

## Then

- PhysX provider discovery loads neither USD image and rejects missing packages or incomplete resource directories (REQ AC-1).
- Missing registration is diagnosed in the correct context rather than silently considered registered (REQ AC-1).
- Real custom type/API names and authored parameter values survive population and reach native creation (REQ AC-2).
- Callbacks read immutable validated values without USD references; omitted fields use declared defaults (REQ AC-3).
- The edited instance updates at a safe phase while the peer retains separate state; unknown data is diagnosed (REQ AC-4).

## Executable coverage

[test_plugins.py](../../../tests_native/test_plugins.py):

External schema population/refresh, six geometry recreations, mixed point-instance prototypes, independently parameterized authored clones and immediate external-type deletion pass. Customer registration/default/unknown-property and provider discovery/failure cases are in [test_external_schemas.py](../../../tests/test_external_schemas.py).

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
