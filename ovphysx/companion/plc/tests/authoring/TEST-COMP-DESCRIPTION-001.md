<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-DESCRIPTION-001
maps_to: REQ-COMP-DESCRIPTION-001
type: integration
---

## Coverage status

Three automated customer-USD cases pass; capture performs no runtime initialization or attach. Covers [REQ-COMP-DESCRIPTION-001](../../requirements/authoring/REQ-COMP-DESCRIPTION-001.md) AC-1 through AC-4.

## Scenario

Inspect a stopped articulation with a tendon and collision shapes, then mutate the source.

## Given

- An immutable snapshot with two links, one joint, one tendon and known shape transforms.
- An optional description family deliberately disabled.

## When

- Query descriptions, retain the result, edit the source and query the disabled family.

## Then

- Supported relationships are present and the disabled family reports unsupported (REQ AC-1).
- No attach, force-load or step occurs (REQ AC-2).
- The consumer reconstructs topology and transforms using only public records and result release (REQ AC-3).
- The retained result remains readable but is marked stale for current-source application (REQ AC-4).

## Executable coverage

- [test_descriptions.py](../../../tests/test_descriptions.py): `test_stopped_description_families_defaults_relationships_units_and_lifetime`, `test_stopped_descriptions_distinguish_instance_paths`, `test_description_time_and_copied_mesh_topology`.
