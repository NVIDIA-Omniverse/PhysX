<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-OVSTAGE-XFORM-001
title: Resolve Physics Population Transforms Before Attach
status: implemented
owner: ovphysx
---

## Description

ovphysx's USD population helpers and samples own the producer's transform
resolution: they seal the population ordinal before handing it to ovphysx, and
the seal composes world matrices (OVStage runs its hierarchy computation when a
write-floor advance covers `omni:fabric:worldMatrix`), with or without RENDERING.
OVStage 0.3 left physics-only world matrices at their local values; this baseline
records the nested-environment force regression that exposed it.

## Acceptance Criteria

- AC-1: **Resolve before attach.** C and Python USD population helpers and
  samples default to the physics population description with a RENDERING
  description beside it (`physics_population_descs(rendering=True)` in Python),
  the compatibility path for native instancing, and seal the population ordinal
  before attach. The sealed ordinal holds composed world matrices with or without
  RENDERING, and they remain available for simulation output publication as well
  as the initial attach.
- AC-2: **Preserve separated environments.** Attaching populated cart-pole
  environments under different translated parents preserves their separation:
  after warmup, a prismatic actuation of 10 produces projected joint force
  within 0.9 of 10 in each environment, through both session and tensor APIs.

## Test References

- [TEST-CAPI-OVSTAGE-XFORM-001](../../tests/capi/TEST-CAPI-OVSTAGE-XFORM-001.md)
- [TEST-CAPI-OVSTAGE-XFORM-002](../../tests/capi/TEST-CAPI-OVSTAGE-XFORM-002.md)
- [TEST-CAPI-OVSTAGE-XFORM-003](../../tests/capi/TEST-CAPI-OVSTAGE-XFORM-003.md)

## Code References

- ovphysx/tests/c_unittests/test_utilities.h (`attach_usd_with_ovstage`, AC-1)
- ovphysx/tests/python_tests/test_utils.py (`attach_usd_with_ovstage`, AC-1)
- ovphysx/tests/c_samples/common/ovstage_sample.h (population helper, AC-1)
- ovphysx/tests/python_samples/ (population helpers, AC-1)
- ovphysx/tests/c_unittests/test_session_physics.cpp (AC-2)
- ovphysx/tests/c_unittests/test_tensor_binding.cpp (AC-2)
- ovphysx/tests/c_unittests/test_ovstage_output.cpp (world-column availability, AC-1)

## Dependencies

- REQ-CAPI-OVSTAGE-SCHEMA-001
