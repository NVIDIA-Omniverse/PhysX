<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-XFORM-003
maps_to: REQ-CAPI-OVSTAGE-XFORM-001
type: unit
---

## Scenario

The OVD investigation sample populates the physics description with RENDERING beside it before sealing
and attaching.
Automated coverage is `test_attach_scene_registers_schemas_populates_with_rendering_and_seals_before_attach`
in `tests/ovd_workflow/test_workflow.py`.

## Given

- The real `attach_scene` helper and isolated population, stage and instance doubles.
- Separate cases with and without the optional Newton schema package.
- Population records its actual `domains` argument; no native runtime is launched.

## When

- Call `attach_scene` in each package case and record schema registration, population and attach order.

## Then

- Population receives the physics description with RENDERING beside it, then the seal is awaited at ordinal 1 before attach reads ordinal 1 (REQ AC-1).
- This guard verifies the selected descriptions and order, not numerical world-transform resolution (REQ AC-1).
