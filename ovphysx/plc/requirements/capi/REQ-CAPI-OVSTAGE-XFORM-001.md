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
resolution. Their ALL population includes rendering, which resolves world
matrices before handing the sealed ordinal to ovphysx. PHYSICS alone seeds
world matrices with local values in the pinned release.
This baseline records the nested-environment force regression exposed by the
OVStage 0.3 dependency update.

## Acceptance Criteria

- AC-1: **Resolve before sealing.** C and Python USD population helpers and
  samples default to `OVSTAGE_POPULATION_DOMAIN_ALL` / `PopulationDomain.ALL` so
  the rendering domain resolves world transforms before advancing the write
  floor. Resolved world matrices remain available for simulation output
  publication, as well as the initial attach.
- AC-2: **Preserve separated environments.** Attaching populated cart-pole
  environments under different translated parents preserves their separation:
  after warmup, a prismatic actuation of 10 produces projected joint force
  within 0.9 of 10 in each environment, through both session and tensor APIs.

## Test References

- [TEST-CAPI-OVSTAGE-XFORM-001](../../tests/capi/TEST-CAPI-OVSTAGE-XFORM-001.md)
- [TEST-CAPI-OVSTAGE-XFORM-002](../../tests/capi/TEST-CAPI-OVSTAGE-XFORM-002.md)

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
