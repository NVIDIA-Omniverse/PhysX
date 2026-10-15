<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-DRIVER-001
maps_to: REQ-RT-DRIVER-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Generic hooks are the only producer. Verifies REQ-RT-DRIVER-001 AC-1, AC-2.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One-scene Kit/ovstage fixture and provider-registration/node probes; legacy omni.physx.stageupdate is not an active driver.

## When

- Open the stage, Play, deliver one h update, Pause, Stop, and close through omni.physics.stageupdate.
- Inspect registrations and directly request each unsupported force-load/resetSimulation/release operation using the generic dispatcher.

## Then

- Exactly one generic driver feeds the adapter; one h update reaches one solver step and no late Fabric/app-update producer is registered. (REQ-RT-DRIVER-001 AC-1).
- Supported hooks execute; all three unsupported callbacks are null and their requests cause zero backend create/step/teardown calls. onReset still reaches Stop. (REQ-RT-DRIVER-001 AC-2).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-DRIVER-001](../../requirements/simulation/REQ-RT-DRIVER-001.md).
