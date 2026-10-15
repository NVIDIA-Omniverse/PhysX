<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-STAGE-POLICY-001
title: Map stage settings only to exposed ovphysx or owned adapter behavior
status: proposed
owner: physics-ux
---

## Description

Map stage settings only to exposed ovphysx or owned adapter behavior. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Removed output modes.** Remove Update to USD, XformCommonAPI, velocity-to-USD/local-space and particle-to-USD controls from the ovphysx stage panel. Preserve their metadata inertly; the runtime output contract is completed ovstage world/native/sim:* data, not a configurable legacy USD writer.
- AC-2: **Adapter catch-up.** Minimum simulation frame rate shall remain an adapter-owned catch-up policy, accepting 0 (uncapped) or 1–1000, independent of scene timeStepsPerSecond. Freeze it for each Play epoch; edits during Play/Pause remain pending until Stop/new Play. It shall not be forwarded as an assumed ovphysx typed setting.
- AC-3: **Collision configuration.** Remove the old convex-approximation switches as unverified mappings. Expose separate Use Custom Cone/Cylinder Geometry only after provider validation of PhysXConfig.collision_cone_custom_geometry / collision_cylinder_custom_geometry (matching OVPHYSX_GLOBAL_CONFIG_* keys). Preserve old approximation values inertly: neither a direct nor inverted migration is assumed. Stage overrides are requested policy translated by the runtime provider before Play attach, with no live recooking or replacement of authored geometry.
- AC-4: **Removed unmapped settings.** Remove joint-body transform tolerance, extended joint angles and USDRT traversal controls from the ovphysx stage panel; no reviewed exposed equivalent exists. Preserve stored metadata without applying private Carbonite paths or misrepresenting renderer traversal as ovstage simulation control.
- AC-5: **Scope and failure.** Compose only whitelisted overrides over explicit provider defaults for the selected stage, so absent values in stage B never inherit stage A configuration. Typed ovphysx settings are process-global, not per-stage handles; the runtime owner shall serialize/conflict-check application before attach and expose rejection/deferred state. A typed setter/getter alone does not prove a behavior is implemented; unvalidated geometry modes stay unavailable.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-STAGE-POLICY-001](../../tests/features/TEST-UX-STAGE-POLICY-001.md) — Retire unsupported stage controls without losing metadata.
- [TEST-UX-STAGE-POLICY-002](../../tests/features/TEST-UX-STAGE-POLICY-002.md) — Freeze adapter catch-up for one play epoch.
- [TEST-UX-STAGE-POLICY-003](../../tests/features/TEST-UX-STAGE-POLICY-003.md) — Translate only validated geometry settings without cross-stage leakage.

## Code References

- [configuration](../../../../../python/ovphysx_companion/configuration.py) — existing companion substrate; host integration remains proposed.
- [exchange](../../../../../python/ovphysx_companion/exchange.py) — existing companion substrate; host integration remains proposed.
- [usd/writeback](../../../../../python/ovphysx_companion/usd/writeback.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.

- [Reviewed option mapping and API evidence](../../CONFIGURATION-MAPPING.md).
- [config.py](../../../../../../python/ovphysx/config.py) — PhysXConfig fields and typed mapping, current API reference.
- [ovphysx_types.h](../../../../../../include/ovphysx/ovphysx_types.h) — typed config enums and active_cuda_gpus contract, current API reference.
- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — global config, lifecycle and device constraints, current API reference.
- [ovphysx.cpp](../../../../../../src/ovphysx/ovphysx.cpp) — typed path mapping and blocked CUDA escape hatch, current API reference.
- [CarboniteLoader.cpp](../../../../../../src/CarboniteLoader/CarboniteLoader.cpp) — first-bootstrap cache configuration, current API reference.

## Dependencies

- [ADR-0005](../../adr/ADR-0005-stage-settings.md).
- [REQ-RT-STEP-001](../../../runtime/requirements/simulation/REQ-RT-STEP-001.md).
- [REQ-RT-OUTPUT-001](../../../runtime/requirements/simulation/REQ-RT-OUTPUT-001.md).
- [REQ-UX-STAGE-SETTINGS-001](REQ-UX-STAGE-SETTINGS-001.md).
