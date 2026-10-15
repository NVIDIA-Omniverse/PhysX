<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-CONFIG-001
title: "Scoped settings and initialization ownership"
status: implemented
owner: ovphysx
---

## Description

Classify settings as UI preference, authored scene data, instance configuration or process initialization. Keep device/context ownership inside the runtime.

## Acceptance Criteria

- AC-1: **Classification.** Every exposed setting declares its scope, default, mutability phase and persistence owner; UI preferences are not native solver state. `process.ovd_recording_capable` is a bool with default false and phase before-first-instance.
- AC-2: **Core reuse.** Existing typed core configuration is reused where equivalent; owned companion attachments use copied PhysxAttachOptions settings/resource overrides rather than temporary writes to process defaults. Scene-authoring settings flow through Python USD/ovstage edits. The OVD capability delegates to `GlobalConfigKey.OVD_RECORDING_CAPABLE`, shares the core's retained process setting and remains dormant through create/attach until explicit recording start.
- AC-3: **Conflicts.** A change prohibited after initialization or attach returns invalid-state/conflict and preserves the effective configuration rather than claiming success. OVD capability writes freeze after the first successful instance creation until full successful shutdown with no live instances.
- AC-4: **Device ownership.** Each native attachment acquires its dispatcher/CUDA resources through the runtime using captured options; supported thread/device choices may differ. SDK tolerance scale, recording-provider compatibility and hard CPU-only policy remain process constraints. Clients cannot manually release runtime-owned resources; a borrowed active CUDA context remains valid for the required SDK lifetime.

## Test References

- [TEST-COMP-CONFIG-001](../../tests/architecture/TEST-COMP-CONFIG-001.md).

## Code References

- [Scoped Python configuration](../../../python/ovphysx_companion/configuration.py)
- [Customer-USD scene edits](../../../python/ovphysx_companion/usd/configuration.py)
- [C table](../../../include/ovphysx_companion/configuration.h)
- [Core owner bridge](../../../src/Configuration.cpp)
- [Copied preview creation](../../../python/ovphysx_companion/preview.py)
- [Extension-style settings sample](../../../samples/scoped_settings.py)

## Implementation scope

Typed core globals and instance GPU selection are delegated, without arbitrary
Carbonite key exposure. Attachment snapshots label captured resource requests;
only contact processing mutates live owner state. Thread/device/solver/pipeline
choices are copied into preview construction. Host booleans use the creating
thread and a caller-owned persistence mapping. Scene gravity/solver/GPU edits
use customer USD, explicit revision/layer and atomic undo. Six native and three customer-USD cases pass, along with native resource/callback
and recording-provider conflict fixtures. Actual borrowed CUDA context execution
requires GPU hardware; snapshots do not assert that a requested GPU was realized.

## Dependencies

- [REQ-COMP-API-001](REQ-COMP-API-001.md)
- [REQ-COMP-USD-EDIT-001](../python/REQ-COMP-USD-EDIT-001.md)
- [Companion ADR-0001](../../adr/ADR-0001-optional-companion-and-owner-bridge.md)
