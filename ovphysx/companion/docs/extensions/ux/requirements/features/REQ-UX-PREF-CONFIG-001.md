<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-PREF-CONFIG-001
title: Translate audited preferences into ovphysx runtime configuration
status: proposed
owner: physics-ux
---

## Description

Translate audited preferences into ovphysx runtime configuration. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Device mapping.** Replace Use Active CUDA Context with explicit CUDA Device selection mapped to create_args.active_cuda_gpus (empty=no override, -1=automatic, one valid ordinal). Do not translate the old boolean into a device choice. Remove the multi-GPU scene-distribution selector from this single-scene integration; do not forward stored modes. Device changes after GPU bootstrap shall report Requires Restart.
- AC-2: **UX-only preferences.** Selection/subtree limits (0 through uint32 max) and attachment/particle authoring toggles shall remain UX-owned and never be passed as ovphysx config; positive scan caps shall not disable generic commands. Authoring toggles shall state their authoring-provider reload boundary and preserve existing schemas.
- AC-3: **Engine activation.** Remove the legacy None/PhysX default-simulator preference; Umbrella engine activation is authoritative. Preserve its old key inertly and do not apply it to runtime or another provider.
- AC-4: **Typed runtime mapping.** Simulation Threads shall map to PhysXConfig.num_threads / OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, retaining the UI range 0–256 with 0=automatic. Replace the legacy SDK profiler toggle with explicitly named NVTX Profiling mapped to profiler_backend="nvtx" / OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND; do not silently migrate the old profiler boolean. Remove Error Stop Threshold, CPU Dispatcher, USD Prim Names and Simulate Empty Scene controls because no reviewed ovphysx product mapping exists.
- AC-5: **Runtime ownership.** Typed pending requests shall reach the elected runtime owner before first authoring or playback bootstrap. Initialization conflicts must be rejected/reported; preference storage is not proof of effective solver state. Reset and provider unload shall not reset shared process state. Native authoring services and preview must honor the same initialization policy.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-PREF-CONFIG-001](../../tests/features/TEST-UX-PREF-CONFIG-001.md) — Select a CUDA device without replaying legacy context or multi-GPU policy.
- [TEST-UX-PREF-CONFIG-002](../../tests/features/TEST-UX-PREF-CONFIG-002.md) — Keep authoring preferences local and retire default simulator selection.
- [TEST-UX-PREF-CONFIG-003](../../tests/features/TEST-UX-PREF-CONFIG-003.md) — Apply typed threads and NVTX at the runtime-owned boundary.

## Code References

- [configuration](../../../../../python/ovphysx_companion/configuration.py) — existing companion substrate; host integration remains proposed.
- [compatibility](../../../../../python/ovphysx_companion/compatibility.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.

- [Reviewed option mapping and API evidence](../../CONFIGURATION-MAPPING.md).
- [config.py](../../../../../../python/ovphysx/config.py) — PhysXConfig fields and typed mapping, current API reference.
- [ovphysx_types.h](../../../../../../include/ovphysx/ovphysx_types.h) — typed config enums and active_cuda_gpus contract, current API reference.
- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — global config, lifecycle and device constraints, current API reference.
- [ovphysx.cpp](../../../../../../src/ovphysx/ovphysx.cpp) — typed path mapping and blocked CUDA escape hatch, current API reference.
- [CarboniteLoader.cpp](../../../../../../src/CarboniteLoader/CarboniteLoader.cpp) — first-bootstrap cache configuration, current API reference.

## Dependencies

- [ADR-0004](../../adr/ADR-0004-physics-preferences.md).
- [REQ-UX-PREFERENCES-001](REQ-UX-PREFERENCES-001.md).
