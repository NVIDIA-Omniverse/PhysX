<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-VEHICLE-001
maps_to: REQ-COMP-VEHICLE-001
type: integration
---

## Coverage status

Executable CPU native, host-input and customer-USD coverage. Covers [REQ-COMP-VEHICLE-001](../../requirements/controllers/REQ-COMP-VEHICLE-001.md) AC-1 through AC-4.

## Scenario

Author and drive a simple vehicle, then recreate it while input is enabled.

## Given

- A symmetric four-wheel vehicle with known total mass and wheel positions.
- A deterministic input sequence and a telemetry recorder.

## When

- Compute authoring data, attach, submit controls and step, then disable/recreate the vehicle and close its service.

## Then

- Sprung masses sum to the specified mass and derived geometry is returned without attach (REQ AC-1).
- Control application precedes the intended step with no Kit dependency in native code (REQ AC-2).
- Drive/wheel outputs are available for their declared fields and completed step (REQ AC-3).
- Recreated objects get fresh references and old/latching input does not leak across reset or close (REQ AC-4).

## Executable coverage

- [Native tests](../../../tests_native/test_vehicle.py): six cases pass. Real direct-drive acceleration and wheel/ground telemetry, pre-step filtering and auto-reverse, pause/disable/close clearing, pending rejection without filter progress, independent owners, owner replacement, C ABI/output canaries, asymmetric sprung masses, suspension-frame/Ackermann math and a stopped worker with no initialized SDK.
- [Customer-USD tests](../../../tests/test_vehicle_usd.py): three cases pass. Explicit stage length/mass conversion, source/frame/scale validation, atomic edits, undo/redo and a native worker feeding the optional customer-USD helper.

The ovstage frame regression failed before the walker consumed explicitly
populated vehicle reference-frame metadata, then passed after the fix. CPU
managed direct/engine vehicles are advertised; GPU, tank and raw-wheel control
are explicitly outside this table. Core typed wheel control remains available.

## Review regression

`test_vehicle_forward_speed_uses_world_center_of_mass_frame` authors nonidentity
principal axes and forward world velocity, independently reconstructs the COM
forward vector from copied actor/COM state, and checks telemetry and auto-reverse.
