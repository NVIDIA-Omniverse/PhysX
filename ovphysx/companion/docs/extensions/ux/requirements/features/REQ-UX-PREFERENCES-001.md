<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-PREFERENCES-001
title: Store and reset only owned PhysX preferences
status: proposed
owner: physics-ux
---

## Description

Store and reset only owned PhysX preferences. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Audited model.** The PhysX preferences model shall expose only the reviewed mapping: supported ovphysx configuration, separately owned UX preferences, and read-only lifecycle facts. Obsolete unmapped switches shall be removed from active controls; preserve their stored values inertly without forwarding arbitrary Carbonite keys.
- AC-2: **Reset scope.** Reset shall clear supported owned preference overrides to declared defaults, including pending runtime requests, without applying unmapped legacy keys, mutating stage metadata, another provider or engine activation. Effective process settings shall not be reported reset until the required lifecycle boundary is reached.
- AC-3: **Mandatory Stop.** Remove the legacy reset-on-stop toggle; Stop restoration is mandatory. Stored false values shall be ignored and never change the runtime Stop contract.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-PREFERENCES-001](../../tests/features/TEST-UX-PREFERENCES-001.md) — Build and reset only the audited preference model.

## Code References

- [configuration](../../../../../python/ovphysx_companion/configuration.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.

- [Reviewed option mapping and API evidence](../../CONFIGURATION-MAPPING.md).
- [config.py](../../../../../../python/ovphysx/config.py) — PhysXConfig fields and typed mapping, current API reference.
- [ovphysx_types.h](../../../../../../include/ovphysx/ovphysx_types.h) — typed config enums and active_cuda_gpus contract, current API reference.
- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — global config, lifecycle and device constraints, current API reference.
- [ovphysx.cpp](../../../../../../src/ovphysx/ovphysx.cpp) — typed path mapping and blocked CUDA escape hatch, current API reference.
- [CarboniteLoader.cpp](../../../../../../src/CarboniteLoader/CarboniteLoader.cpp) — first-bootstrap cache configuration, current API reference.

## Dependencies

- [ADR-0004](../../adr/ADR-0004-physics-preferences.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
