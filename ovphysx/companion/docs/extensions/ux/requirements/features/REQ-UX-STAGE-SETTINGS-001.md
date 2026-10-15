<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-STAGE-SETTINGS-001
title: Persist stage overrides without cross-stage leakage
status: proposed
owner: physics-ux
---

## Description

Persist stage overrides without cross-stage leakage. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Metadata model.** Stage settings shall preserve root-layer customLayerData.physicsSettings keys, unrelated metadata and unknown provider keys; opening/refreshing shall not author defaults or apply unknown keys as runtime settings.
- AC-2: **Edit and reset.** A persistent setting edit shall target the active writable stage; resetting one key shall remove that override and expose its default without removing other keys. No-stage/read-only edits shall be disabled.
- AC-3: **Stage switching.** Sequential stages shall display/apply their own overrides; listeners and delayed completions from the previous stage shall not mutate the next stage.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-STAGE-SETTINGS-001](../../tests/features/TEST-UX-STAGE-SETTINGS-001.md) — Round-trip stage overrides without leaking them to the next stage.

## Code References

- [configuration](../../../../../python/ovphysx_companion/configuration.py) — existing companion substrate; host integration remains proposed.
- [usd/configuration](../../../../../python/ovphysx_companion/usd/configuration.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.

- [Reviewed option mapping and API evidence](../../CONFIGURATION-MAPPING.md).
- [config.py](../../../../../../python/ovphysx/config.py) — PhysXConfig fields and typed mapping, current API reference.
- [ovphysx_types.h](../../../../../../include/ovphysx/ovphysx_types.h) — typed config enums and active_cuda_gpus contract, current API reference.
- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — global config, lifecycle and device constraints, current API reference.
- [ovphysx.cpp](../../../../../../src/ovphysx/ovphysx.cpp) — typed path mapping and blocked CUDA escape hatch, current API reference.
- [CarboniteLoader.cpp](../../../../../../src/CarboniteLoader/CarboniteLoader.cpp) — first-bootstrap cache configuration, current API reference.

## Dependencies

- [ADR-0005](../../adr/ADR-0005-stage-settings.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
