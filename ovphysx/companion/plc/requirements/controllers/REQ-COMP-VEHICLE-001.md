<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-VEHICLE-001
title: "Optional vehicle computations control and telemetry"
status: implemented
owner: ovphysx
---

## Description

Vehicle authoring calculations and live controls are optional services. Host input devices and UI preferences remain outside the native controller.

## Acceptance Criteria

- AC-1: **Authoring.** Sprung masses, suspension-frame and steering geometry calculations accept typed source data and return data for Python authoring without a live attach.
- AC-2: **Inputs.** Live throttle/brake/steer and supported filtering/auto-reverse updates are applied at a declared pre-step phase from explicit host-provided input, not direct Kit input polling.
- AC-3: **Telemetry.** Vehicle and wheel telemetry includes the supported drive/wheel fields and step/object identities; unavailable fields are explicit rather than substituted with body poses.
- AC-4: **Lifecycle.** Object recreation, input disable, pause and owner close invalidate cached actor references and clear latched input according to the declared reset policy.

## Test References

- [TEST-COMP-VEHICLE-001](../../tests/controllers/TEST-COMP-VEHICLE-001.md).

## Code References

- [Separate C vehicle table](../../../include/ovphysx_companion/vehicle.h)
- [C bridge](../../../src/Vehicle.cpp)
- [Python controls and stopped math](../../../python/ovphysx_companion/vehicle.py)
- [Customer-USD authoring](../../../python/ovphysx_companion/usd/vehicle.py)
- [Extension-style sample](../../../samples/vehicle_control.py)

## Implementation scope

CPU managed direct/engine controls, bounded stopped math, immutable copied wheel
and drive telemetry, optional host filtering/auto-reverse and reversible customer
USD authoring. Host input polling and clock stay outside native code. Raw wheels,
tanks and GPU simulation are not advertised. Both source-backend replacement checks and all native/customer-USD cases pass.
The full integration gate is rerun after this feature.

## Dependencies

- [REQ-COMP-OWNER-001](../architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-JOBS-001](../exchange/REQ-COMP-JOBS-001.md)
- [Companion ADR-0006](../../adr/ADR-0006-host-services-and-ux-migration.md)
